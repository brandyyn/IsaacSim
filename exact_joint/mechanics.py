"""Cable-loaded, rigid-cap, statically condensed 3D laminate FEM.

Finite elements are assembled from the exact JSON panel surfaces. Only free
interior coordinates are eliminated; static condensation is algebraically exact
for this small-strain solid-element model. Cable forces, not joint position
targets, determine cap motion. Large-fold constitutive behavior is not validated.
"""

from __future__ import annotations

from dataclasses import asdict
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import splu
from scipy.spatial.transform import Rotation

from exact_joint.geometry import JointConfig, laminate_mesh
from exact_joint.elements import elevate,b_matrices,subdivide_surface
from exact_joint.mesh_utils import compute_tet_volumes,elasticity_matrix


def cable_anchors(width_m: float, height_m: float):
    """Four axial and eight crossed perimeter tendons with nonnegative tensions."""
    a = width_m/2
    corners = np.array([[-a,-a,0],[a,-a,0],[a,a,0],[-a,a,0]])
    pairs = [(i,i) for i in range(4)] + [(i,(i+1)%4) for i in range(4)] + [((i+1)%4,i) for i in range(4)]
    top = np.array([corners[i]+[0,0,height_m] for i,j in pairs])
    bottom = np.array([corners[j] for i,j in pairs])
    return top,bottom


def tension_pattern(mode: str, amplitude_n: float):
    """Return named, pull-only actuator experiments; coupling is not suppressed."""
    if not np.isfinite(amplitude_n) or amplitude_n<0:
        raise ValueError("Cable tension must be finite and nonnegative")
    tension = np.zeros(12)
    groups = {"Compression":[0,1,2,3],"Bend X+":[0,1],"Bend X-":[2,3],
              "Bend Y+":[1,2],"Bend Y-":[0,3],"Twist CW":[4,5,6,7],"Twist CCW":[8,9,10,11]}
    if mode not in groups:
        raise ValueError("Unknown cable pattern")
    tension[groups[mode]] = amplitude_n
    return tension


class CableFem:
    """Assemble a bonded two-material FEM with exact rigid roof transformations."""

    def __init__(self,source: dict,config: JointConfig):
        self.source,self.config = source,config
        pet,pla,pet_bond,pla_bond,self.coarse_map = laminate_mesh(source,config)
        self.pet,self.pla = pet,pla
        points = pet.local_points.tolist()
        self.pet_map = np.arange(len(points))
        self.pla_map = np.full(len(pla.local_points),-1,dtype=int)
        self.pla_map[pla_bond] = pet_bond
        for i,p in enumerate(pla.local_points):
            if self.pla_map[i]<0:
                self.pla_map[i]=len(points)
                points.append(p.tolist())
        self.points = np.asarray(points)
        self.tets = np.concatenate([pet.tets,self.pla_map[pla.tets]])
        self.ids = np.r_[np.zeros(len(pet.tets),dtype=int),np.ones(len(pla.tets),dtype=int)]
        self.points,self.tets,self.top_nodes,self.bottom_nodes,edges=elevate(
            self.points,self.tets,pet.top_nodes,pet.bottom_nodes)
        self.pet_surface,pet_face_map=subdivide_surface(pet.surface_triangles,edges)
        self.pla_surface,pla_face_map=subdivide_surface(self.pla_map[pla.surface_triangles],edges)
        self.surface_owners=np.r_[pet.surface_owners[pet_face_map],pla.surface_owners[pla_face_map]+len(pet.tets)]
        self.bmat = b_matrices(self.points,self.tets)
        self.dmat = np.stack([elasticity_matrix(config.pet_modulus_pa,config.pet_poisson),
                              elasticity_matrix(config.pla_modulus_pa,config.pla_poisson)])[self.ids]
        volume = compute_tet_volumes(self.points,self.tets[:,:4])
        element = np.einsum("egai,eab,egbj,e->eij",self.bmat,self.dmat,self.bmat,volume/4,optimize=True)
        self.dofs = (3*self.tets[:,:,None]+np.arange(3)).reshape(-1,30)
        rows = np.broadcast_to(self.dofs[:,:,None],element.shape).ravel()
        cols = np.broadcast_to(self.dofs[:,None,:],element.shape).ravel()
        size = len(self.points)*3
        self.stiffness = coo_matrix((element.ravel(),(rows,cols)),shape=(size,size)).tocsr()
        bottom = (self.bottom_nodes[:,None]*3+np.arange(3)).ravel()
        top = (self.top_nodes[:,None]*3+np.arange(3)).ravel()
        self.free = np.setdiff1d(np.arange(size),np.r_[top,bottom])
        self.boundary_dofs = bottom
        self.basis = np.zeros((size,len(bottom)))
        self.basis[bottom] = np.eye(len(bottom))
        lu = splu(self.stiffness[self.free][:,self.free].tocsc())
        self.basis[self.free] = lu.solve(-self.stiffness[self.free][:,bottom].toarray())
        self.kcap = self.basis.T @ (self.stiffness @ self.basis)
        self.kcap = (self.kcap+self.kcap.T)/2
        self.strain_basis = np.einsum("egij,ejk->egik",self.bmat,self.basis[self.dofs],optimize=True)
        self.top_anchors,self.bottom_anchors = cable_anchors(config.width_m,source["height_m"])
        self.last_q = np.zeros(6)
        # Rotation coordinates use a width scale in Newton's solve to condition
        # translations and rotations comparably. Exchanged data remain SI.
        self.qscale = np.r_[np.ones(3),np.ones(3)/config.width_m]
        self.k6 = self._hessian(np.zeros(6),np.zeros(12))

    def kinematics(self,q):
        rotation = Rotation.from_rotvec(q[3:]).as_matrix()
        bottom = self.points[self.bottom_nodes]
        ub = (bottom @ (rotation-np.eye(3)).T+q[:3]).ravel()
        moved = self.bottom_anchors @ rotation.T+q[:3]
        return ub,moved,rotation

    def gradient(self,q,tensions):
        ub,moved,_ = self.kinematics(q)
        jac = np.empty((len(ub),6))
        cable_jac = np.empty((12,3,6))
        for i in range(6):
            step = 1e-7 if i<3 else 1e-5
            delta = np.eye(6)[i]*step
            plus,pplus,_ = self.kinematics(q+delta)
            minus,pminus,_ = self.kinematics(q-delta)
            jac[:,i] = (plus-minus)/(2*step)
            cable_jac[:,:,i] = (pplus-pminus)/(2*step)
        spans = moved-self.top_anchors
        lengths = np.linalg.norm(spans,axis=1)
        if np.min(lengths)<1e-7:
            raise ValueError("Cable endpoints coincide")
        # The gradient of +T*length produces a pulling force -T*dL/dq.
        cable_gradient = np.einsum("n,nk,nkj->j",tensions,spans/lengths[:,None],cable_jac)
        return jac.T @ (self.kcap @ ub)+cable_gradient

    def _hessian(self,q,tensions):
        result = np.empty((6,6))
        for i in range(6):
            step = 1e-7 if i<3 else 1e-5
            delta = np.eye(6)[i]*step
            result[:,i]=(self.gradient(q+delta,tensions)-self.gradient(q-delta,tensions))/(2*step)
        return (result+result.T)/2

    def solve(self,tensions,*,initial=None):
        """Solve equilibrium for 12 ideal constant-tension, pull-only cables.

        Rigid caps use exact rotations. The underlying continuum is linear elastic
        about the supplied folded shape; >2 degree cap rotation or >1% principal
        strain is flagged and must not be accepted as calibrated folding mechanics.
        """
        tensions = np.asarray(tensions,dtype=float)
        if tensions.shape!=(12,) or not np.isfinite(tensions).all() or np.any(tensions<0):
            raise ValueError("Supply twelve finite nonnegative tensions in N")
        q = self.last_q.copy() if initial is None else np.asarray(initial,dtype=float).copy()
        steps = 0
        for steps in range(15):
            gradient = self.gradient(q,tensions)
            scaled = gradient*self.qscale
            if np.linalg.norm(scaled)<1e-7:
                break
            hessian = self._hessian(q,tensions)*self.qscale[:,None]*self.qscale[None,:]
            delta = np.linalg.solve(hessian,-scaled)*self.qscale
            # Bounded continuation avoids jumping onto a distant nonlinear branch.
            fraction = min(1,.0003/max(np.linalg.norm(delta[:3]),1e-15),.015/max(np.linalg.norm(delta[3:]),1e-15))
            q += delta*fraction
            if np.linalg.norm(q[3:])>.15 or np.linalg.norm(q[:3])>.003:
                raise ValueError("Load exceeds the exploratory small-deformation solve range")
        residual = float(np.linalg.norm(self.gradient(q,tensions)*self.qscale))
        if residual>1e-5:
            raise RuntimeError(f"Cable/FEM equilibrium did not converge: {residual:.3g} N equivalent")
        self.last_q = q.copy()
        ub,moved,rotation = self.kinematics(q)
        displacement = (self.basis @ ub).reshape(-1,3)
        strain = np.einsum("egik,k->egi",self.strain_basis,ub)
        stress = np.einsum("eij,egj->egi",self.dmat,strain)
        xx,yy,zz,xy,yz,xz = np.moveaxis(stress,-1,0)
        vm_gauss = np.sqrt(.5*((xx-yy)**2+(yy-zz)**2+(zz-xx)**2)+3*(xy*xy+yz*yz+xz*xz))
        vm = vm_gauss.max(axis=1)
        tensor = np.zeros((len(strain),4,3,3))
        tensor[:,:,0,0],tensor[:,:,1,1],tensor[:,:,2,2]=np.moveaxis(strain[:,:,:3],-1,0)
        for (i,j),component in (((0,1),3),((1,2),4),((0,2),5)):
            tensor[:,:,i,j]=tensor[:,:,j,i]=strain[:,:,component]/2
        principal = np.linalg.eigvalsh(tensor)
        max_strain = float(np.abs(principal).max())
        reaction = (self.kcap @ ub).reshape(-1,3)
        lengths = np.linalg.norm(moved-self.top_anchors,axis=1)
        net_cable_force = ((self.top_anchors-moved)/lengths[:,None]*tensions[:,None]).sum(axis=0)
        stats = {"cap_translation_m":q[:3].tolist(),"cap_rotation_vector_rad":q[3:].tolist(),
                 "compression_m":float(q[2]),"bend_xy_deg":np.rad2deg(q[3:5]).tolist(),
                 "twist_deg":float(np.rad2deg(q[5])),"tensions_n":tensions.tolist(),"lengths_m":lengths.tolist(),
                 "max_principal_strain":max_strain,"max_von_mises_pa":float(vm.max()),
                 "pet_peak_pa":float(vm[self.ids==0].max()),"pla_peak_pa":float(vm[self.ids==1].max()),
                 "cap_equilibrium_residual_n_equivalent":residual,
                 "force_balance_n":(reaction.sum(axis=0)-net_cable_force).tolist(),
                 "strain_energy_j":float(.5*ub @ self.kcap @ ub),"newton_iterations":steps+1,
                 "within_small_deformation_model":bool(max_strain<.01 and np.linalg.norm(q[3:])<np.deg2rad(2)),
                 "solver":"live cable-loaded statically condensed 3D TET10 linear-elastic FEM; four Gauss points; exact rigid cap transforms",
                 "scope":"quasistatic; no gravity/payload/inertia/contact/plasticity/fatigue/slack dynamics; unconverged mesh"}
        return {"q":q,"rotation":rotation,"displacements":displacement,"points":self.points+displacement,
                "von_mises":vm,"stats":stats,"tensions":tensions,"bottom_anchors":moved}

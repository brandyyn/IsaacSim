"""Exploratory finite-rotation shell with rigid perimeter frames only.

Triangular membrane elements and discrete bending/folding hinges replace the
small-strain TET10 reduction in this opt-in model. Effective crease stiffness is
an adjustable model parameter, not a measured PET constitutive or failure law.
Import after Isaac's physics startup has settled (torch initialization order).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
import torch
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from exact_joint.geometry import JointConfig
from exact_joint.mechanics import cable_anchors
from exact_joint.shell_contact import intersection_pairs


@dataclass(frozen=True)
class ShellConfig:
    subdivision: int = 1
    panel_to_crease_ratio: float = 100.0
    crease_twist_ratio: float = 0.1
    panel_bending_scale: float = 1.0
    membrane_scale: float = 1.0
    panel_strain_limit: float = 0.03
    minimum_height_fraction: float = 0.08
    max_iterations: int = 900

    def validate(self):
        if not np.isfinite(list(asdict(self).values())).all():
            raise ValueError("Shell parameters must be finite")
        if self.subdivision not in (1, 2):
            raise ValueError("Shell subdivision must be 1 or 2")
        if not 1 <= self.panel_to_crease_ratio <= 10000:
            raise ValueError("Panel/crease bending ratio must be 1-10000")
        if not 0 <= self.crease_twist_ratio <= 10:
            raise ValueError("Crease twist coupling must be 0-10")
        if not .01 <= self.panel_bending_scale <= 10 or not .01 <= self.membrane_scale <= 10:
            raise ValueError("Panel bending and membrane scales must be 0.01-10")
        if not .001 <= self.panel_strain_limit <= .1:
            raise ValueError("Exploratory membrane strain guard must be 0.1-10%")
        if not .02 <= self.minimum_height_fraction <= .5:
            raise ValueError("Minimum frame separation must be 2-50% of neutral height")
        if self.max_iterations != int(self.max_iterations) or not 10 <= self.max_iterations <= 5000:
            raise ValueError("Iteration limit must be an integer 10-5000")


def surface_mesh(source, subdivision=1, gap_m=0.0):
    """Subdivide original planar faces; keep every source vertex and crease chain.

No averaged extrusion normals are used, so refinement preserves the reference
midsurface. The two roof interiors receive free membrane nodes. Only nodes on
the eight original square perimeter edges are attached to rigid frames.
    """
    points = source["points"].tolist()
    chains, edge_chains = [], {}
    for line in source["lines"]:
        a, b = line["vertices"]
        chain = [a]
        for step in range(1, 2**subdivision):
            chain.append(len(points))
            points.append(((1-step/2**subdivision)*source["points"][a]
                           + step/2**subdivision*source["points"][b]).tolist())
        chain.append(b)
        chains.append(chain)
        edge_chains[a, b] = chain
        edge_chains[b, a] = chain[::-1]
    triangles, owners, laminate = [], [], []
    for owner, panel in enumerate(source["panels"]):
        ids = panel["vertices"]
        center = len(points)
        center_point = source["points"][ids].mean(axis=0)
        points.append(center_point.tolist())
        boundary = []
        for a, b in zip(ids, ids[1:]+ids[:1]):
            boundary.extend(edge_chains[a, b][:-1])
        faces, face_laminate = [], []
        if len(ids) == 4:
            # An inner roof ring gives the membrane independent bending/twist
            # nodes; none are included in the rigid perimeter-frame constraint.
            inner = []
            for vertex in boundary:
                inner.append(len(points))
                points.append(((np.asarray(points[vertex])+center_point)/2).tolist())
            for i in range(len(boundary)):
                j = (i+1) % len(boundary)
                a, b, c, d = boundary[i], boundary[j], inner[j], inner[i]
                faces.extend([[a, b, c], [a, c, d], [d, c, center]])
            face_laminate = [True]*len(faces)
        else:
            # Explicit PET-only strip, half the total exposed gap on each
            # adjacent face. Insetting about the incenter gives the specified
            # perpendicular setback even on the narrow original triangles.
            face = source["points"][ids]
            sides = np.linalg.norm(face[[1, 2, 0]]-face[[2, 0, 1]], axis=1)
            incenter = np.average(face, axis=0, weights=sides)
            radius = np.linalg.norm(np.cross(face[1]-face[0], face[2]-face[0]))/sides.sum()
            fraction = gap_m/(2*radius)
            if not 0 <= fraction < .8:
                raise ValueError("PET gap is too wide for an original panel")
            if gap_m:
                inner = []
                for vertex in boundary:
                    inner.append(len(points))
                    points.append(((1-fraction)*np.asarray(points[vertex])+fraction*incenter).tolist())
                points[center] = ((1-fraction)*center_point+fraction*incenter).tolist()
                for i in range(len(boundary)):
                    j = (i+1) % len(boundary)
                    a, b, c, d = boundary[i], boundary[j], inner[j], inner[i]
                    faces.extend([[a, b, c], [a, c, d]])
                    face_laminate.extend([False, False])
            else:
                inner = boundary
            faces.extend([[a, b, center] for a, b in zip(inner, inner[1:]+inner[:1])])
            face_laminate.extend([True]*len(inner))
        for face, has_pla in zip(faces, face_laminate):
            p = np.asarray(points)[face]
            normal = np.cross(p[1] - p[0], p[2] - p[0])
            outward = p.mean(axis=0) - [0, 0, source["height_m"] / 2]
            if normal @ outward < 0:
                face = [face[0], face[2], face[1]]
            triangles.append(face)
            owners.append(owner)
            laminate.append(has_pla)
    points, triangles, owners = np.asarray(points), np.asarray(triangles), np.asarray(owners)
    top, bottom = [], []
    frame_line_ids = []
    for line_id, chain in enumerate(chains):
        z = points[chain, 2]
        if np.allclose(z, source["height_m"], atol=1e-12):
            top.extend(chain)
            frame_line_ids.append(line_id)
        elif np.allclose(z, 0, atol=1e-12):
            bottom.extend(chain)
            frame_line_ids.append(line_id)
    edge_faces = {}
    for face_id, (a, b, c) in enumerate(triangles):
        for start, end, opposite in ((a, b, c), (b, c, a), (c, a, b)):
            edge_faces.setdefault(tuple(sorted((start, end))), []).append((face_id, start, end, opposite))
    crease_lookup = {tuple(sorted((a, b))): i for i, chain in enumerate(chains)
                     for a, b in zip(chain[:-1], chain[1:])}
    hinges, crease_ids, hinge_faces = [], [], []
    for edge, adjacent in edge_faces.items():
        if len(adjacent) != 2:
            raise ValueError("Original shell must remain a closed two-manifold surface")
        first, second = adjacent
        if first[1] != second[2] or first[2] != second[1]:
            raise ValueError("Inconsistent reference shell winding")
        hinges.append([first[1], first[2], first[3], second[3]])
        crease_ids.append(crease_lookup.get(edge, -1))
        hinge_faces.append([first[0], second[0]])
    return {"points": points, "triangles": triangles, "owners": owners,
            "top": np.unique(top), "bottom": np.unique(bottom), "chains": chains,
            "frame_line_ids": frame_line_ids, "hinges": np.asarray(hinges),
            "crease_ids": np.asarray(crease_ids), "laminate": np.asarray(laminate),
            "hinge_faces": np.asarray(hinge_faces)}


def rotation_matrix(vector):
    """Differentiable exact Rodrigues rotation, including a finite zero limit."""
    zero = vector[0] * 0
    x, y, z = vector.unbind()
    skew = torch.stack((zero, -z, y, z, zero, -x, -y, x, zero)).reshape(3, 3)
    angle = torch.linalg.vector_norm(vector)
    return (torch.eye(3, dtype=vector.dtype) + torch.sinc(angle / torch.pi) * skew
            + .5 * torch.sinc(angle / (2 * torch.pi))**2 * (skew @ skew))


class NonlinearShell:
    """Finite-rotation membrane/discrete-hinge energy with exact frame constraints."""

    def __init__(self, source, material=None, config=None):
        self.source = source
        self.material = material or JointConfig()
        self.config = config or ShellConfig()
        self.material.validate()
        self.config.validate()
        self.mesh = surface_mesh(source, self.config.subdivision, self.material.hinge_gap_m)
        self.points = self.mesh["points"]
        self.width = self.material.width_m
        self.height = source["height_m"]
        self.free_nodes = np.setdiff1d(np.arange(len(self.points)), np.r_[self.mesh["top"], self.mesh["bottom"]])
        self.frame_start = len(self.free_nodes) * 3
        self.ndof = self.frame_start + 12  # upper then lower: scaled translations, rotation vectors
        self.state = np.zeros(self.ndof)
        self._tensor = lambda value: torch.as_tensor(value, dtype=torch.float64)
        self.rest = self._tensor(self.points)
        self.faces = torch.as_tensor(self.mesh["triangles"], dtype=torch.long)
        self.hinges = torch.as_tensor(self.mesh["hinges"], dtype=torch.long)
        self.frame_centers = self._tensor([[0, 0, self.height], [0, 0, 0]])
        self.frame_indices = [torch.as_tensor(self.mesh[name], dtype=torch.long) for name in ("top", "bottom")]
        self.free_indices = torch.as_tensor(self.free_nodes, dtype=torch.long)
        p = self.points[self.mesh["triangles"]]
        e1, e2 = p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]
        length = np.linalg.norm(e1, axis=1)
        projection = np.sum(e1 * e2, axis=1) / length
        # Cross-product altitude avoids cancellation in narrow PET-strip cells.
        altitude = np.linalg.norm(np.cross(e1, e2), axis=1)/length
        reference = np.zeros((len(p), 2, 2))
        reference[:, 0, 0], reference[:, 0, 1], reference[:, 1, 1] = length, projection, altitude
        self.inverse_reference = self._tensor(np.linalg.inv(reference))
        self.area = self._tensor(length * altitude / 2)
        mat = self.material
        laminate = self.mesh["laminate"]
        self.nu = self._tensor(np.where(laminate, (mat.pet_poisson+mat.pla_poisson)/2, mat.pet_poisson))
        self.membrane_modulus = self._tensor((mat.pet_modulus_pa*mat.pet_thickness_m
                                             + laminate*mat.pla_modulus_pa*mat.pla_thickness_m)
                                            * self.config.membrane_scale)
        # Bonded laminate bending about its stiffness-weighted neutral axis.
        thickness = np.array([mat.pet_thickness_m, mat.pla_thickness_m])
        moduli = np.array([mat.pet_modulus_pa, mat.pla_modulus_pa])
        centers = np.cumsum(thickness) - thickness / 2
        neutral = np.sum(moduli * thickness * centers) / np.sum(moduli * thickness)
        self.panel_rigidity_nm = float(np.sum(moduli * (thickness**3 / 12 + thickness * (centers-neutral)**2)
                                             / (1-np.array([mat.pet_poisson, mat.pla_poisson])**2))
                                      * self.config.panel_bending_scale)
        self.pet_rigidity_nm = mat.pet_modulus_pa*mat.pet_thickness_m**3/(12*(1-mat.pet_poisson**2))
        # The ratio is an explicit effective bending assumption, not a change
        # to PET in-plane modulus. All PET strip hinges use this rigidity, so
        # widening the strip actually changes its folding/twisting compliance.
        self.effective_strip_rigidity_nm = self.panel_rigidity_nm/self.config.panel_to_crease_ratio
        h = self.points[self.mesh["hinges"]]
        edge = h[:, 1] - h[:, 0]
        edge_length = np.linalg.norm(edge, axis=1)
        h1 = np.linalg.norm(np.cross(edge, h[:, 2]-h[:, 0]), axis=1) / edge_length
        h2 = np.linalg.norm(np.cross(edge, h[:, 3]-h[:, 0]), axis=1) / edge_length
        face_rigidity = np.where(laminate, self.panel_rigidity_nm, self.effective_strip_rigidity_nm)
        adjacent = self.mesh["hinge_faces"]
        # Series compliance across the dual strip for dissimilar materials.
        stiffness = edge_length / (.5*h1/face_rigidity[adjacent[:, 0]]
                                    + .5*h2/face_rigidity[adjacent[:, 1]])
        crease_mask = self.mesh["crease_ids"] >= 0
        self.hinge_stiffness = self._tensor(stiffness)
        self.rest_angles = self.angles(self.rest).detach()
        self.crease_mask = torch.as_tensor(crease_mask)
        self.folding_mask = torch.as_tensor(~np.all(laminate[adjacent], axis=1))
        twist_pairs = []
        for i in range(len(self.mesh["chains"])):
            rows = np.flatnonzero(self.mesh["crease_ids"] == i)
            twist_pairs.extend(zip(rows[:-1], rows[1:]))
        self.twist_pairs = torch.as_tensor(np.asarray(twist_pairs), dtype=torch.long)
        self.twist_stiffness = self._tensor([min(stiffness[a], stiffness[b]) * self.config.crease_twist_ratio
                                           for a, b in twist_pairs])
        top, bottom = cable_anchors(self.width, self.height)
        self.cable_top, self.cable_bottom = self._tensor(top), self._tensor(bottom)
        self.last_report = None
        self.preconditioners = {}

    def reference_hessian(self):
        """Neutral Gauss-Newton stiffness for optimization scaling, not new physics."""
        if "hessian" not in self.preconditioners:
            self.preconditioners["hessian"] = self.gauss_newton_stiffness(np.zeros(self.ndof))
        return self.preconditioners["hessian"]

    def residual_vector(self, state):
        """Elastic residuals whose half squared norm equals elastic energy."""
        points = self.positions(state)
        strain = self.strains(points)
        weight = torch.sqrt(self.area*self.membrane_modulus/(1+self.nu))
        trace = strain[:, 0, 0]+strain[:, 1, 1]
        delta = self.angles(points)-self.rest_angles
        delta = torch.atan2(torch.sin(delta), torch.cos(delta))
        return torch.cat([(weight[:, None, None]*strain).ravel(),
                          weight*torch.sqrt(self.nu/(1-self.nu))*trace,
                          torch.sqrt(self.hinge_stiffness)*delta,
                          torch.sqrt(self.twist_stiffness)*(delta[self.twist_pairs[:, 0]]-delta[self.twist_pairs[:, 1]])])

    def gauss_newton_stiffness(self, state):
        """Current-geometry positive tangent approximation used ONLY as a preconditioner."""
        jacobian = torch.autograd.functional.jacobian(self.residual_vector, self._tensor(state), vectorize=True).numpy()
        return jacobian.T@jacobian

    @staticmethod
    def stiffness_scaling(hessian):
        """Build reusable optimization coordinates without changing the energy."""
        eigenvalues, vectors = np.linalg.eigh(hessian)
        floor = max(float(np.max(eigenvalues))*1e-10, 1e-9)
        return vectors / np.sqrt(np.maximum(eigenvalues, floor))[None, :]

    @staticmethod
    def minimize_scaled(objective, start, hessian, max_iterations, transform=None):
        """Whiten a reference stiffness; evaluate the actual nonlinear objective."""
        if transform is None:
            transform = NonlinearShell.stiffness_scaling(hessian)

        def scaled(value):
            energy, gradient = objective(start + transform @ value)
            return energy, transform.T @ gradient

        solution = minimize(scaled, np.zeros(len(start)), method="L-BFGS-B", jac=True,
                            options={"maxiter": max_iterations, "ftol": 1e-15,
                                     "gtol": 1e-9, "maxls": 40, "maxcor": 50})
        solution.x = start + transform @ solution.x
        solution.fun, solution.jac = objective(solution.x)
        return solution

    def positions(self, state):
        points = self.rest.clone()
        points[self.free_indices] = self.rest[self.free_indices] + self.width * state[:self.frame_start].reshape(-1, 3)
        for frame, indices in enumerate(self.frame_indices):
            q = state[self.frame_start+6*frame:self.frame_start+6*(frame+1)]
            rotation = rotation_matrix(q[3:])
            center = self.frame_centers[frame]
            points[indices] = (self.rest[indices]-center) @ rotation.T + center + self.width*q[:3]
        return points

    def frame_point(self, state, frame, points):
        q = state[self.frame_start+6*frame:self.frame_start+6*(frame+1)]
        center = self.frame_centers[frame]
        return (points-center) @ rotation_matrix(q[3:]).T + center + self.width*q[:3]

    def angles(self, points):
        a, b, c, d = points[self.hinges].unbind(dim=1)
        edge = b-a
        unit = edge / torch.linalg.vector_norm(edge, dim=1)[:, None].clamp_min(1e-14)
        n1 = torch.linalg.cross(edge, c-a)
        n2 = torch.linalg.cross(d-a, edge)
        n1 = n1 / torch.linalg.vector_norm(n1, dim=1)[:, None].clamp_min(1e-18)
        n2 = n2 / torch.linalg.vector_norm(n2, dim=1)[:, None].clamp_min(1e-18)
        return torch.atan2(torch.sum(torch.linalg.cross(n1, n2)*unit, dim=1), torch.sum(n1*n2, dim=1))

    def strains(self, points):
        p = points[self.faces]
        current = torch.stack((p[:, 1]-p[:, 0], p[:, 2]-p[:, 0]), dim=2)
        gradient = current @ self.inverse_reference
        return .5 * (gradient.transpose(1, 2) @ gradient - torch.eye(2, dtype=points.dtype))

    def elastic_terms(self, points):
        strain = self.strains(points)
        trace = strain[:, 0, 0] + strain[:, 1, 1]
        membrane = torch.sum(self.area * self.membrane_modulus / (2*(1+self.nu))
                             * (torch.sum(strain**2, dim=(1, 2)) + self.nu/(1-self.nu)*trace**2))
        delta = self.angles(points) - self.rest_angles
        delta = torch.atan2(torch.sin(delta), torch.cos(delta))
        bending = .5 * self.hinge_stiffness * delta**2
        pair_delta = delta[self.twist_pairs[:, 0]] - delta[self.twist_pairs[:, 1]]
        twist = .5 * torch.sum(self.twist_stiffness * pair_delta**2)
        return membrane, bending[~self.folding_mask].sum(), bending[self.folding_mask].sum(), twist

    def energy(self, state, tensions=None, nodal_loads=None):
        points = self.positions(state)
        energy = sum(self.elastic_terms(points))
        if tensions is not None:
            top = self.frame_point(state, 0, self.cable_top)
            bottom = self.frame_point(state, 1, self.cable_bottom)
            # Constant pull-only tension has potential +T*length.
            energy = energy + torch.sum(tensions * torch.linalg.vector_norm(top-bottom, dim=1))
        if nodal_loads is not None:
            energy = energy - torch.sum(nodal_loads * (points-self.rest))
        return energy

    def value_gradient(self, state, tensions=None, nodal_loads=None):
        value = self._tensor(state).clone().requires_grad_(True)
        tension_tensor = self._tensor(tensions) if tensions is not None else None
        load_tensor = self._tensor(nodal_loads) if nodal_loads is not None else None
        energy = self.energy(value, tension_tensor, load_tensor)
        gradient = torch.autograd.grad(energy, value)[0]
        return float(energy.detach()), gradient.detach().numpy()

    def diagnostics(self, state):
        with torch.no_grad():
            value = self._tensor(state)
            points = self.positions(value)
            strain = self.strains(points)
            principal = torch.linalg.eigvalsh(strain).abs().max(dim=1).values
            energies = self.elastic_terms(points)
            rotations = [Rotation.from_rotvec(state[self.frame_start+6*i+3:self.frame_start+6*i+6]) for i in range(2)]
            relative = (rotations[0].inv() * rotations[1]).as_rotvec()
            centers = np.array([[0, 0, self.height], [0, 0, 0]]) + self.width*np.stack([
                state[self.frame_start+6*i:self.frame_start+6*i+3] for i in range(2)])
            separation = rotations[0].inv().apply(centers[0]-centers[1])[2]
            angle_change = self.angles(points)-self.rest_angles
            angle_change = torch.atan2(torch.sin(angle_change), torch.cos(angle_change))
        return {"points": points.numpy(), "principal_membrane_strain": principal.numpy(),
                "max_membrane_strain": float(principal.max()), "relative_rotation_rad": relative,
                "max_laminate_membrane_strain": float(principal[self.mesh["laminate"]].max()),
                "max_pet_strip_membrane_strain": float(principal[~self.mesh["laminate"]].max()),
                "frame_separation_m": float(separation), "compression_fraction": float(1-separation/self.height),
                "energy_j": dict(zip(("membrane", "panel_bending", "folding", "crease_twist"), map(float, energies))),
                "hinge_angle_changes_rad": angle_change.numpy(),
                "scope": "Uncalibrated nonlinear membrane/discrete-hinge model; no solid stress or failure verdict"}

    def solve(self, tensions=None, lower_pose=None, nodal_loads=None, initial=None, lower_constraints=None):
        """Static continuation step; a requested pose is not a force prediction.

The upper frame is fixed. With lower_pose, both frames are prescribed and only
the panels relax; otherwise cable loads determine lower-frame motion. Returns
the candidate plus explicit convergence/strain checks without silently applying.
"""
        if tensions is None:
            tensions = np.zeros(12)
        tensions = np.asarray(tensions, dtype=float)
        if tensions.shape != (12,) or not np.isfinite(tensions).all() or np.min(tensions) < 0:
            raise ValueError("Use twelve finite nonnegative cable tensions")
        if nodal_loads is not None:
            nodal_loads = np.asarray(nodal_loads, dtype=float)
            if nodal_loads.shape != self.points.shape or not np.isfinite(nodal_loads).all():
                raise ValueError("Nodal forces need the mesh's (nodes, 3) shape and finite SI values")
        state = self.state.copy() if initial is None else np.asarray(initial, dtype=float).copy()
        if state.shape != (self.ndof,) or not np.isfinite(state).all():
            raise ValueError("Invalid shell state")
        state[self.frame_start:self.frame_start+6] = 0
        active = np.r_[np.arange(self.frame_start), np.arange(self.frame_start+6, self.ndof)]
        if lower_pose is not None:
            lower_pose = np.asarray(lower_pose, dtype=float)
            if lower_pose.shape != (6,) or not np.isfinite(lower_pose).all():
                raise ValueError("Lower frame pose must be six finite SI values")
            state[-6:] = lower_pose / np.r_[np.full(3, self.width), np.ones(3)]
            active = np.arange(self.frame_start)
        if lower_constraints is not None:
            if lower_pose is not None:
                raise ValueError("Choose complete pose or partial constraints, not both")
            for coordinate, value in lower_constraints.items():
                if coordinate not in range(6) or not np.isfinite(value):
                    raise ValueError("Partial frame constraints need coordinate 0-5 and finite SI value")
                state[self.ndof-6+coordinate] = value / (self.width if coordinate < 3 else 1)
                active = active[active != self.ndof-6+coordinate]
        initial_energy, _ = self.value_gradient(state, tensions, nodal_loads)

        def objective(value):
            candidate = state.copy()
            candidate[active] = value
            energy, gradient = self.value_gradient(candidate, tensions, nodal_loads)
            return energy, gradient[active]

        reference = self.reference_hessian()[np.ix_(active, active)]
        solution = self.minimize_scaled(objective, state[active], reference, min(250, self.config.max_iterations))
        total_iterations = int(solution.nit)
        # At large rotations the neutral-coordinate scaling becomes poor.
        # Recompute a current-geometry preconditioner, while keeping the same
        # exact nonlinear potential and force-residual acceptance criterion.
        while np.abs(solution.jac).max() >= 1e-5 and total_iterations < self.config.max_iterations:
            state[active] = solution.x
            reference = self.gauss_newton_stiffness(state)[np.ix_(active, active)]
            solution = self.minimize_scaled(objective, solution.x, reference,
                                           min(200, self.config.max_iterations-total_iterations))
            total_iterations += max(1, int(solution.nit))
        state[active] = solution.x
        report = self.diagnostics(state)
        gradient_max = float(np.abs(solution.jac).max())
        report.update({"converged": bool(np.isfinite(solution.fun) and gradient_max < 1e-5),
                       "optimizer_success": bool(solution.success),
                       "optimizer_message": str(solution.message), "iterations": total_iterations,
                       "gradient_max_j_per_scaled_coordinate": gradient_max,
                       "initial_potential_j": initial_energy, "final_potential_j": float(solution.fun),
                       "load_type": "prescribed_frame_pose" if lower_pose is not None or lower_constraints is not None else "cable_and_nodal_forces",
                       "tensions_n": tensions.tolist()})
        report["within_strain_guard"] = report["max_membrane_strain"] <= self.config.panel_strain_limit
        report["within_height_guard"] = report["frame_separation_m"] >= self.height*self.config.minimum_height_fraction
        report["surface_intersection_pairs"] = intersection_pairs(report["points"], self.mesh["triangles"]).tolist()
        report["accepted"] = (report["converged"] and report["within_strain_guard"] and report["within_height_guard"]
                              and not report["surface_intersection_pairs"])
        # Rotation-vector gradients are not Cartesian moments at finite angle.
        _, full_gradient = self.value_gradient(state, tensions, nodal_loads)
        reactions = []
        for frame in range(2):
            offset = self.frame_start+6*frame
            phi = state[offset+3:offset+6]
            angle = np.linalg.norm(phi)
            x, y, z = phi
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            coefficient = 1/12+angle**2/720 if angle < 1e-4 else (1/angle**2-np.cos(angle/2)/(2*angle*np.sin(angle/2)))
            inverse_left = np.eye(3)-.5*skew+coefficient*(skew@skew)
            reactions.append(np.r_[full_gradient[offset:offset+3]/self.width,
                                   inverse_left.T@full_gradient[offset+3:offset+6]].tolist())
        report["required_frame_reactions_world_n_nm"] = reactions
        return state, report

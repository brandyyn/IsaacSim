"""Ten-node quadratic tetrahedra with four-point volume integration.

Straight-sided reference elements preserve the source geometry. Quadratic
displacement interpolation reduces the bending locking of first-order thin tets.
Reference: FEBio Theory Manual 4.7, section 4.1.4 (TET10, four-point Gauss rule).
"""

import numpy as np

EDGE_PAIRS=((0,1),(1,2),(2,0),(0,3),(1,3),(2,3))
GAUSS=np.full((4,4),.1381966011250105)
np.fill_diagonal(GAUSS,.5854101966249685)


def elevate(points,tets,top,bottom):
    """Insert shared edge nodes, preserving bonded interfaces and rigid clamp sets."""
    values=points.tolist()
    registry={}
    quadratic=[]
    for tet in tets:
        row=list(tet)
        for a,b in EDGE_PAIRS:
            key=tuple(sorted([int(tet[a]),int(tet[b])]))
            if key not in registry:
                registry[key]=len(values)
                values.append(((points[key[0]]+points[key[1]])/2).tolist())
            row.append(registry[key])
        quadratic.append(row)
    def boundary(indices):
        selected=set(int(i) for i in indices)
        return np.array(list(indices)+[mid for (a,b),mid in registry.items() if a in selected and b in selected],dtype=int)
    return np.asarray(values),np.asarray(quadratic),boundary(top),boundary(bottom),registry


def b_matrices(points,tets):
    """Return B[e,gauss,engineering-strain-component,nodal-displacement]."""
    corners=points[tets[:,:4]]
    dm=np.stack([corners[:,i]-corners[:,0] for i in (1,2,3)],axis=-1)
    inverse=np.linalg.inv(dm)
    linear=np.concatenate([-inverse.sum(axis=1)[:,None,:],inverse],axis=1)
    gradients=np.zeros((len(tets),4,10,3))
    for i in range(4):
        gradients[:,:,i]=(4*GAUSS[:,i]-1)[None,:,None]*linear[:,None,i]
    for n,(i,j) in enumerate(EDGE_PAIRS):
        gradients[:,:,4+n]=4*(GAUSS[None,:,i,None]*linear[:,None,j]+GAUSS[None,:,j,None]*linear[:,None,i])
    matrices=np.zeros((len(tets),4,6,30))
    for i in range(10):
        x,y,z=np.moveaxis(gradients[:,:,i],-1,0)
        matrices[:,:,0,3*i]=x
        matrices[:,:,1,3*i+1]=y
        matrices[:,:,2,3*i+2]=z
        matrices[:,:,3,3*i],matrices[:,:,3,3*i+1]=y,x
        matrices[:,:,4,3*i+1],matrices[:,:,4,3*i+2]=z,y
        matrices[:,:,5,3*i],matrices[:,:,5,3*i+2]=z,x
    return matrices


def subdivide_surface(faces,edge_registry):
    """Render curved quadratic triangles as four subtriangles, with owner mapping."""
    result=[]
    owners=[]
    for index,(a,b,c) in enumerate(faces):
        ab=edge_registry[tuple(sorted([int(a),int(b)]))]
        bc=edge_registry[tuple(sorted([int(b),int(c)]))]
        ca=edge_registry[tuple(sorted([int(c),int(a)]))]
        result.extend([(a,ab,ca),(ab,b,bc),(ca,bc,c),(ab,bc,ca)])
        owners.extend([index]*4)
    return np.array(result),np.array(owners)

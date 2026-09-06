"""Small, solver-independent volume-mesh utilities for the exact joint."""

from dataclasses import dataclass
import numpy as np


@dataclass
class VolumeMesh:
    local_points: np.ndarray
    tets: np.ndarray
    top_nodes: np.ndarray
    bottom_nodes: np.ndarray
    surface_triangles: np.ndarray
    surface_owners: np.ndarray
    rest_volumes: np.ndarray


def compute_tet_volumes(points,tets):
    vertices=np.asarray(points,dtype=np.float64)[tets]
    matrices=np.stack([vertices[:,i]-vertices[:,0] for i in (1,2,3)],axis=-1)
    return np.linalg.det(matrices)/6


def extract_boundary(points,tets):
    faces={}
    for owner,tet in enumerate(tets):
        for opposite in range(4):
            triangle=[int(tet[i]) for i in range(4) if i!=opposite]
            vertices=points[triangle]
            normal=np.cross(vertices[1]-vertices[0],vertices[2]-vertices[0])
            if np.dot(normal,points[tet[opposite]]-vertices[0])>0:
                triangle[1],triangle[2]=triangle[2],triangle[1]
            faces.setdefault(tuple(sorted(triangle)),[]).append((triangle,owner))
    if any(len(entries)>2 for entries in faces.values()):
        raise ValueError("Nonmanifold volume mesh")
    boundary=[entries[0] for entries in faces.values() if len(entries)==1]
    return np.asarray([entry[0] for entry in boundary]),np.asarray([entry[1] for entry in boundary])


def volume_mesh(bottom,top,triangles,height):
    count=len(bottom)
    points=np.concatenate([bottom,top])
    tetrahedra=[]
    for face in triangles:
        a,b,c=sorted(face)
        aa,bb,cc=a+count,b+count,c+count
        tetrahedra.extend([(a,b,c,cc),(a,b,bb,cc),(a,aa,bb,cc)])
    tets=np.asarray(tetrahedra,dtype=np.int32)
    volumes=compute_tet_volumes(points,tets)
    reverse=np.flatnonzero(volumes<0)
    tets[reverse,:2]=tets[reverse,1::-1]
    volumes=compute_tet_volumes(points,tets)
    if volumes.min()<=1e-22:
        raise ValueError("Degenerate laminate element")
    surface,owners=extract_boundary(points,tets)
    upper=np.flatnonzero(np.isclose(bottom[:,2],height,atol=1e-12,rtol=0))
    lower=np.flatnonzero(np.isclose(bottom[:,2],0,atol=1e-12,rtol=0))
    return VolumeMesh(points,tets,np.r_[upper,upper+count],np.r_[lower,lower+count],surface,owners,volumes)


def elasticity_matrix(youngs,poisson):
    mu=youngs/(2*(1+poisson))
    lam=youngs*poisson/((1+poisson)*(1-2*poisson))
    matrix=np.zeros((6,6))
    matrix[:3,:3]=lam
    np.fill_diagonal(matrix[:3,:3],lam+2*mu)
    matrix[3:,3:]=np.eye(3)*mu
    return matrix

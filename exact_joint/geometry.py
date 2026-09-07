"""Preserve the user's exact panel topology while meshing its PLA/PET laminate."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from exact_joint.mesh_utils import volume_mesh


def parse_refinement(value: float) -> int:
    """Validate the UI value before conversion; never silently truncate a mesh level."""
    if not np.isfinite(value) or value != int(value) or not 1 <= value <= 4:
        raise ValueError("Mesh refinement must be a finite whole number from 1 to 4")
    return int(value)


@dataclass
class JointConfig:
    """SI geometry/material assumptions; original shape and proportions are fixed."""

    width_m: float = .030
    pet_thickness_m: float = .00008
    pla_thickness_m: float = .0004
    pet_modulus_pa: float = 3.5e9
    pla_modulus_pa: float = 2.2e9
    pet_poisson: float = .35
    pla_poisson: float = .35
    hinge_gap_m: float = .0002
    refinement: int = 1

    def validate(self):
        values = [v for v in self.__dict__.values()]
        if not np.isfinite(values).all():
            raise ValueError("All configuration values must be finite")
        if not .01 <= self.width_m <= .04:
            raise ValueError("Joint width must be 10..40 mm")
        if not 1 <= self.refinement <= 4 or self.refinement != int(self.refinement):
            raise ValueError("Mesh refinement must be an integer 1..4")
        if min(self.pet_thickness_m,self.pla_thickness_m,self.pet_modulus_pa,self.pla_modulus_pa,self.hinge_gap_m) <= 0:
            raise ValueError("Thickness, hinge gap and moduli must be positive")
        if not 0 <= self.pet_poisson < .49 or not 0 <= self.pla_poisson < .49:
            raise ValueError("Poisson ratios must be in [0, .49)")


def load_source(path: Path, width_m: float = .030) -> dict:
    """Import every original vertex, panel and edge with an explicit uniform mapping.

    Source coordinates are treated as the existing documented millimetre reference.
    Rotate +90 degrees about X and uniformly size the 250-unit roof to width_m.
    Center XY and place the lower roof at local Z=0. No shape-fitting or remeshing
    replaces the original coarse geometry.
    """
    raw = path.read_bytes()
    data = json.loads(raw)
    registry, points, panels = {}, [], []
    def vertex(value):
        coordinate = tuple(float(value[key]) for key in ("x","y","z"))
        if coordinate not in registry:
            registry[coordinate] = len(points)
            points.append(coordinate)
        return registry[coordinate]
    for panel in data["panels"]:
        panels.append({"name":panel["name"],"vertices":[vertex(p) for p in panel["points"]]})
    lines = [{"name":line["name"],"vertices":[vertex(line["start"]),vertex(line["end"])]}
             for line in data["lines"]]
    source_points = np.asarray(points)
    rotation = np.array([[1,0,0],[0,0,-1],[0,1,0]],dtype=float)
    scale = width_m/np.ptp(source_points[:,0])
    converted = source_points @ rotation.T*scale
    converted -= [0,0,converted[:,2].min()]
    height = np.ptp(converted[:,2])
    top = np.flatnonzero(np.isclose(converted[:,2],height,atol=1e-12))
    bottom = np.flatnonzero(np.isclose(converted[:,2],0,atol=1e-12))
    sides = [p for p in panels if len(p["vertices"]) == 3]
    roofs = [p for p in panels if len(p["vertices"]) == 4]
    derived = {}
    for panel in panels:
        ids = panel["vertices"]
        for a,b in zip(ids,ids[1:]+ids[:1]):
            key = tuple(sorted([a,b]))
            derived[key] = derived.get(key,0)+1
    registered = {tuple(sorted(line["vertices"])) for line in lines}
    if len(points)!=28 or len(panels)!=50 or len(lines)!=76 or len(sides)!=48 or len(roofs)!=2:
        raise ValueError("The supplied source topology differs from the agreed 28/50/76 joint")
    if set(derived)!=registered or any(count!=2 for count in derived.values()):
        raise ValueError("Source panel boundaries and line registry do not match")
    return {"points":converted,"source_points":source_points,"panels":panels,"sides":sides,
            "roofs":roofs,"lines":lines,"top":top,"bottom":bottom,"height_m":float(height),
            "sha256":hashlib.sha256(raw).hexdigest(),"source_units_to_m":float(scale),
            "rotation":rotation,"source_data":data}


def laminate_mesh(source: dict, config: JointConfig):
    """Mesh PET-only fold borders and separate PLA facets on the original surfaces.

    The laminate base surface exactly follows each JSON triangle; thickness is
    added outward. Inset each PLA edge by half the provisional exposed fold gap.
    Rigid quadrilateral roofs remain separate boundary bodies, not flexible panels.
    """
    config.validate()
    points,lookup,normals,triangles,pla_triangles = [],{},{},[],[]
    coarse_maps = {}

    def insert(p,n):
        key = tuple(np.round(p,13))
        if key not in lookup:
            lookup[key] = len(points)
            points.append(p)
            normals[key] = np.zeros(3)
        normals[key] += n
        return lookup[key]

    for panel in source["sides"]:
        face = source["points"][panel["vertices"]]
        normal = np.cross(face[1]-face[0],face[2]-face[0])
        # The hourglass is star-shaped about its axis; side outward normal has
        # positive radial projection, independent of the JSON winding order.
        if normal[:2] @ face.mean(axis=0)[:2] < 0:
            face = face[[0,2,1]]
            normal *= -1
        double_area = np.linalg.norm(normal)
        normal /= double_area
        edges = np.array([np.linalg.norm(face[1]-face[2]),np.linalg.norm(face[2]-face[0]),np.linalg.norm(face[0]-face[1])])
        incenter = np.average(face,axis=0,weights=edges)
        inradius = double_area/edges.sum()
        fraction = config.hinge_gap_m/(2*inradius)
        if fraction >= .8:
            raise ValueError("PET gap is too wide for the narrow source triangles; reduce hinge gap")
        inset = (1-fraction)*face+fraction*incenter
        patches = [(inset,True)]
        for i in range(3):
            j = (i+1)%3
            patches += [(np.array([face[i],face[j],inset[j]]),False),
                        (np.array([face[i],inset[j],inset[i]]),False)]
        for patch,is_pla in patches:
            a,b,c = patch
            r = config.refinement
            ids = {(i,j):insert(a+(b-a)*i/r+(c-a)*j/r,normal)
                   for i in range(r+1) for j in range(r+1-i)}
            for i in range(r):
                for j in range(r-i):
                    split = [[ids[i,j],ids[i+1,j],ids[i,j+1]]]
                    if i+j<r-1:
                        split += [[ids[i+1,j],ids[i+1,j+1],ids[i,j+1]]]
                    triangles += split
                    if is_pla:
                        pla_triangles += split
    base = np.asarray(points)
    for i,p in enumerate(source["points"]):
        coarse_maps[i] = lookup[tuple(np.round(p,13))]
    normal_array = np.asarray(list(normals.values()))
    normal_array /= np.linalg.norm(normal_array,axis=1)[:,None]
    outer = base+normal_array*config.pet_thickness_m
    pet = volume_mesh(base,outer,np.asarray(triangles),source["height_m"])
    selected = np.unique(pla_triangles)
    mapping = {int(old):i for i,old in enumerate(selected)}
    pla_faces = np.array([[mapping[int(v)] for v in f] for f in pla_triangles])
    pla = volume_mesh(outer[selected],outer[selected]+normal_array[selected]*config.pla_thickness_m,
                      pla_faces,source["height_m"])
    pla.top_nodes = pla.bottom_nodes = np.array([],dtype=np.int32)
    return pet,pla,selected+len(base),np.arange(len(selected)),coarse_maps

"""Discrete midsurface intersection guard, not a thickness/contact solver."""

import numpy as np


def intersection_pairs(points, triangles):
    """Find proper edge-through-triangle intersections using current geometry.

Shared vertices/edges are excluded by strict barycentric/segment interiors.
This discrete guard is not continuous collision detection; coplanar overlap and
finite laminate thickness still require a dedicated validated contact model.
"""
    triangles = np.asarray(triangles)
    faces = np.asarray(points)[triangles]
    first, second = np.triu_indices(len(faces), 1)
    minimum, maximum = faces.min(axis=1), faces.max(axis=1)
    active = np.all((minimum[first] <= maximum[second]+1e-12)
                    & (minimum[second] <= maximum[first]+1e-12), axis=1)
    # Incident triangles belong to the same local surface neighbourhood.
    # Near-coplanar roundoff must not count their shared vertex/edge as impact.
    active &= ~np.any(triangles[first, :, None] == triangles[second, None, :], axis=(1, 2))
    first, second = first[active], second[active]
    if not len(first):
        return np.empty((0, 2), dtype=int)
    hit = np.zeros(len(first), dtype=bool)
    for left, right in ((first, second), (second, first)):
        a, b, c = faces[right].transpose(1, 0, 2)
        e1, e2 = b-a, c-a
        for start, end in ((0, 1), (1, 2), (2, 0)):
            origin = faces[left, start]
            direction = faces[left, end]-origin
            cross = np.cross(direction, e2)
            determinant = np.sum(e1*cross, axis=1)
            scale = np.linalg.norm(e1, axis=1)*np.linalg.norm(direction, axis=1)*np.linalg.norm(e2, axis=1)
            valid = np.abs(determinant) > 1e-10*scale
            inverse = np.zeros_like(determinant)
            inverse[valid] = 1/determinant[valid]
            offset = origin-a
            u = inverse*np.sum(offset*cross, axis=1)
            q = np.cross(offset, e1)
            v = inverse*np.sum(direction*q, axis=1)
            t = inverse*np.sum(e2*q, axis=1)
            eps = 1e-7
            hit |= valid & (u > eps) & (v > eps) & (u+v < 1-eps) & (t > eps) & (t < 1-eps)
    return np.c_[first[hit], second[hit]]

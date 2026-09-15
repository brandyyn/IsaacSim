"""Infinitesimal rigid-facet audit, NOT a test of the deformable PET laminate.

Fix the upper square, treat the lower square as a rigid body, and linearize
the 76 original edge lengths. A null vector is a candidate first-order mechanism;
its existence would not prove a finite folding path. No null vector means that
motion requires deformation or additional geometric freedom under these ideal
constraints. Finite-width PET strips in the shell are deliberately not this model.
"""

import numpy as np


def rigid_facet_audit(source, width_m):
    p = source["points"]
    free = np.setdiff1d(np.arange(len(p)), np.r_[source["top"], source["bottom"]])
    maps = np.zeros((len(p), 3, len(free)*3+6))
    maps[free, :, :len(free)*3] = np.eye(len(free)*3).reshape(len(free), 3, -1)*width_m
    maps[source["bottom"], :, -6:-3] = np.eye(3)*width_m
    center = p[source["bottom"]].mean(axis=0)
    maps[source["bottom"], :, -3:] = np.cross(np.eye(3)[None, :, :],
                                            (p[source["bottom"]]-center)[:, None, :]).transpose(0, 2, 1)
    edges = np.asarray([line["vertices"] for line in source["lines"]])
    directions = p[edges[:, 1]]-p[edges[:, 0]]
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    jacobian = np.einsum("ei,eij->ej", directions, maps[edges[:, 1]]-maps[edges[:, 0]])/width_m
    singular = np.linalg.svd(jacobian, compute_uv=False)
    tolerance = 1e-9
    rank = int(np.count_nonzero(singular > tolerance))
    return {"scope": "Neutral first-order ideal rigid panels and zero-width hinges, not the deformable shell",
            "edge_constraints": len(edges), "coordinates": maps.shape[-1], "rank": rank,
            "nullity": maps.shape[-1]-rank, "rank_tolerance": tolerance,
            "dimensionless_singular_values": singular.tolist(),
            "interpretation": "No ideal hinge-only mechanism" if rank == maps.shape[-1] else "First-order candidates; finite mechanism not established"}

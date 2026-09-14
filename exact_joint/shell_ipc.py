"""Optional IPC midsurface self-contact, NOT a finite-thickness contact model.

IPC Toolkit supplies barrier derivatives and continuous collision detection.
The shell energy, materials, loads and equilibrium solver remain project-owned.
Install the pinned optional dependency with CONTACT_AND_REFINEMENT.md.
"""

import importlib
import sys
from pathlib import Path

import numpy as np
from scipy.sparse import csc_matrix


def load_ipc():
    try:
        module = importlib.import_module("ipctk")
    except ModuleNotFoundError:
        local = Path(__file__).resolve().parents[1]/"tmp/knee_contact_deps"
        if not local.is_dir():
            raise RuntimeError("IPC is not installed; see exact_joint/CONTACT_AND_REFINEMENT.md") from None
        sys.path.insert(0, str(local))
        module = importlib.import_module("ipctk")
    if module.__version__ != "1.6.0":
        raise RuntimeError("This contact implementation is tested with ipctk==1.6.0 only")
    return module


class MidsurfaceContact:
    def __init__(self, points, faces, distance_m=1e-6, energy_j=1e-6):
        self.ipc = load_ipc()
        faces = np.asarray(faces, dtype=np.int32)
        edges = np.unique(np.sort(faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1), axis=0)
        self.mesh = self.ipc.CollisionMesh(np.asarray(points, dtype=float), edges, faces)
        self.distance_m = distance_m
        # The default IPC barrier is a function of squared distance: its
        # unscaled value has units m^4. This coefficient gives energy in joules.
        self.potential = self.ipc.BarrierPotential(distance_m, energy_j/distance_m**4)
        initial = self.collisions(points)
        if len(initial):
            raise ValueError("Contact activation distance overlaps the neutral mesh; reduce it, do not add prestress")

    def collisions(self, points):
        collisions = self.ipc.NormalCollisions()
        collisions.build(self.mesh, np.asarray(points, dtype=float), self.distance_m)
        return collisions

    def evaluate(self, points, hessian=False):
        points = np.asarray(points, dtype=float)
        collisions = self.collisions(points)
        n = points.size
        if not len(collisions):
            return 0.0, np.zeros(n), csc_matrix((n, n)) if hessian else None
        energy = self.potential(collisions, self.mesh, points)
        gradient = self.potential.gradient(collisions, self.mesh, points)
        matrix = None
        if hessian:
            matrix = self.potential.hessian(collisions, self.mesh, points,
                                            self.ipc.PSDProjectionMethod.CLAMP)
        return float(energy), np.asarray(gradient), matrix

    def linear_step_is_feasible(self, first, second, clearance=0.0):
        return bool(self.ipc.is_step_collision_free(self.mesh, first, second, clearance))

    def shell_step_is_feasible(self, shell, first, second, depth=0):
        """Certify the actual curved frame path, not just its endpoint chord.

For R(t)=exp([phi0+t*dphi]x), ||R''(t)|| <= ||dphi||^2 (the
matrix-exponential derivative integral contains rotations of norm one).
Each frame point's chord deviation is therefore <= r*||dphi||^2/8.
Twice the largest deviation bounds relative primitive motion. Recursive
halving reduces this conservative margin without omitting a path segment.
Failure to certify rejects the step; it does not assert actual collision.
"""
        bound = 0.0
        for frame, indices in enumerate(shell.frame_indices):
            offset = shell.frame_start+6*frame+3
            change = second[offset:offset+3]-first[offset:offset+3]
            radius = np.linalg.norm(shell.points[indices.numpy()]-shell.frame_centers[frame].numpy(), axis=1).max()
            bound = max(bound, radius*float(change@change)/8)
        p = shell.positions(shell._tensor(first)).numpy()
        q = shell.positions(shell._tensor(second)).numpy()
        if self.linear_step_is_feasible(p, q, 2*bound):
            return True
        if depth >= 8:
            return False
        middle = (first+second)/2
        return (self.shell_step_is_feasible(shell, first, middle, depth+1)
                and self.shell_step_is_feasible(shell, middle, second, depth+1))

    def report(self, points):
        collisions = self.collisions(points)
        distance_squared = collisions.compute_minimum_distance(self.mesh, points)
        return {"method": "IPC 1.6.0 barrier + conservative curved-frame CCD",
                "scope": "Midsurfaces only; NO PLA/PET thickness clearance or friction",
                "active_stencils": len(collisions),
                "minimum_active_distance_m": float(np.sqrt(distance_squared)) if len(collisions) else None,
                "activation_distance_m": self.distance_m,
                "barrier_energy_j": float(self.potential(collisions, self.mesh, points))}

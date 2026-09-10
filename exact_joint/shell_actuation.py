"""Pull-only elastic winch experiment; not a calibrated actuator model."""

from dataclasses import dataclass

import numpy as np
import torch


@dataclass(frozen=True)
class CableWinch:
    rest_lengths_m: tuple
    active: tuple
    stiffness_n_per_m: float = 1000.0
    force_cap_n: float = 10.0

    def validate(self):
        lengths = np.asarray(self.rest_lengths_m, dtype=float)
        active = np.asarray(self.active)
        if lengths.shape != (12,) or not np.isfinite(lengths).all() or np.min(lengths) <= 0:
            raise ValueError("Winch needs twelve positive finite rest lengths in metres")
        if active.shape != (12,) or not np.isin(active, [0, 1]).all():
            raise ValueError("Winch needs twelve active flags")
        if not np.isfinite([self.stiffness_n_per_m, self.force_cap_n]).all():
            raise ValueError("Winch stiffness and force cap must be finite")
        if not 1 <= self.stiffness_n_per_m <= 1e6 or not 0 < self.force_cap_n <= 100:
            raise ValueError("Winch stiffness must be 1-1e6 N/m and force cap 0-100 N")

    def response(self, lengths):
        rest = torch.as_tensor(self.rest_lengths_m, dtype=lengths.dtype)
        mask = torch.as_tensor(self.active, dtype=lengths.dtype)
        extension = torch.clamp(lengths-rest, min=0)
        elastic = torch.clamp(extension, max=self.force_cap_n/self.stiffness_n_per_m)
        tension = mask*self.stiffness_n_per_m*elastic
        # Continuous potential and gradient through both slack and capped regimes.
        energy = (mask*(.5*self.stiffness_n_per_m*elastic**2
                       + self.force_cap_n*(extension-elastic))).sum()
        return energy, tension


def winch_pull(shell, pattern, pull_m, stiffness_n_per_m=1000.0, force_cap_n=10.0):
    from exact_joint.mechanics import tension_pattern

    if not np.isfinite(pull_m) or pull_m < 0:
        raise ValueError("Cable pull must be finite and nonnegative")
    active = tension_pattern(pattern, 1).astype(bool)
    lengths = np.linalg.norm(shell.cable_top.numpy()-shell.cable_bottom.numpy(), axis=1)
    winch = CableWinch(tuple(map(float, lengths-active*pull_m)), tuple(map(bool, active)),
                      stiffness_n_per_m, force_cap_n)
    winch.validate()
    return winch

"""Display-only interpolation of accepted endpoints; never a numerical integrator.

State arrays are copied on entry/exit. Callers retain authoritative solver states,
forces and results separately. Intermediates are NOT equilibria or FEM samples.
"""

import numpy as np


class SolvedTransition:
    def __init__(self, state, offset=0.0):
        self.reset(state, offset)

    def reset(self, state, offset=0.0):
        self.start = self.end = np.array(state, dtype=float, copy=True)
        self.start_offset = self.end_offset = float(offset)
        self.elapsed = self.duration = 0.0

    @property
    def active(self):
        return self.elapsed < self.duration

    def begin(self, start, end, duration, start_offset=0.0, end_offset=0.0):
        start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        if start.shape != end.shape or start.ndim != 1:
            raise ValueError("Display endpoints must have the same one-dimensional shape")
        if not (np.isfinite(start).all() and np.isfinite(end).all()
                and np.isfinite([duration, start_offset, end_offset]).all()) or duration <= 0:
            raise ValueError("Finite accepted endpoints and positive display duration required")
        self.start, self.end = start.copy(), end.copy()
        self.start_offset, self.end_offset = float(start_offset), float(end_offset)
        self.duration, self.elapsed = float(duration), 0.0

    def sample(self):
        alpha = 1.0 if not self.duration else min(self.elapsed/self.duration, 1.0)
        # Zero endpoint velocity and acceleration for stitched, smooth keyframes.
        blend = alpha**3*(10 + alpha*(-15 + 6*alpha))
        return ((1-blend)*self.start + blend*self.end,
                (1-blend)*self.start_offset + blend*self.end_offset)

    def advance(self, dt, paused=False):
        if not np.isfinite(dt) or dt < 0:
            raise ValueError("Display clock increment must be finite and nonnegative")
        if not paused:
            self.elapsed = min(self.elapsed + dt, self.duration)
        return self.sample()

"""Display-only displacement vectors, separated from all shell mechanics."""

import numpy as np


def displacement_plot(reference, solved, frame_center, frame_translation, frame_rotation, gain, offset):
    """Remove upper-frame rigid motion and magnify displacement, never strain.

    The result is a vector plot, NOT a mechanically admissible configuration.
    In particular, magnified rotations need not preserve rigid-frame lengths.
    Inputs and the physical result are never modified.
    """
    reference, solved = np.asarray(reference), np.asarray(solved)
    center, translation = np.asarray(frame_center), np.asarray(frame_translation)
    rotation, offset = np.asarray(frame_rotation), np.asarray(offset)
    if reference.shape != solved.shape or reference.ndim != 2 or reference.shape[1] != 3:
        raise ValueError("Matching N x 3 reference and solved points required")
    if center.shape != (3,) or translation.shape != (3,) or rotation.shape != (3, 3) or offset.shape != (3,):
        raise ValueError("Invalid display frame")
    if not np.isfinite(gain) or not 1 <= gain <= 500:
        raise ValueError("Display magnification must be 1-500; it does not change FEM")
    if not all(np.isfinite(value).all() for value in (reference, solved, center, translation, rotation, offset)):
        raise ValueError("Display inputs must be finite")
    aligned = (solved-center-translation) @ rotation+center
    displacement = aligned-reference
    return reference+gain*displacement+offset, displacement

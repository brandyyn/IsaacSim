"""Display-only displacement vectors, separated from all shell mechanics."""

import numpy as np


def frame_blend_display(reference, centers, translations, rotations, gain=1.0):
    """Skin a CAD display between two FEM frames, with upper-frame motion removed.

    This is a kinematic visualization, not a solid FEM interpolation or a stress
    result. Height weights preserve the two end-frame motions at physical gain.
    """
    reference = np.asarray(reference, dtype=float)
    centers, translations = np.asarray(centers), np.asarray(translations)
    rotations = np.asarray(rotations)
    if reference.ndim != 2 or reference.shape[1] != 3:
        raise ValueError("CAD reference must contain N x 3 points")
    if centers.shape != (2, 3) or translations.shape != (2, 3) or rotations.shape != (2, 3, 3):
        raise ValueError("Two frame centers, translations and rotations required")
    height = centers[0, 2]-centers[1, 2]
    if not np.isfinite(height) or height <= 0:
        raise ValueError("Upper frame must be above lower frame")
    weight = np.clip((reference[:, 2]-centers[1, 2])/height, 0, 1)[:, None]
    transformed = [(reference-centers[i]) @ rotations[i].T+centers[i]+translations[i] for i in (0, 1)]
    solved = weight*transformed[0]+(1-weight)*transformed[1]
    return displacement_plot(reference, solved, centers[0], translations[0], rotations[0], gain, np.zeros(3))


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

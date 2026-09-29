"""Check the display projection cannot alter the physical result."""

import unittest

import numpy as np

from exact_joint.shell_display_math import displacement_plot, frame_blend_display


class DisplayTests(unittest.TestCase):
    def test_cad_blend_compression_and_gain_leave_reference_unchanged(self):
        reference = np.array([[.01, 0, .02], [.01, 0, .01], [.01, 0, 0]])
        saved = reference.copy()
        centers = [[0, 0, .02], [0, 0, 0]]
        translations = [[0, 0, 0], [0, 0, .001]]
        rotations = np.stack([np.eye(3), np.eye(3)])
        plotted, delta = frame_blend_display(reference, centers, translations, rotations, 100)
        np.testing.assert_allclose(delta[:, 2], [0, .0005, .001])
        np.testing.assert_allclose(plotted, reference+100*delta)
        np.testing.assert_array_equal(reference, saved)

    def test_cad_blend_preserves_end_frame_twist_and_removes_common_motion(self):
        reference = np.array([[.01, 0, .02], [.01, 0, 0]])
        centers = np.array([[0, 0, .02], [0, 0, 0]])
        rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        plotted, _ = frame_blend_display(reference, centers, np.zeros((2, 3)),
                                         np.stack([np.eye(3), rotation]))
        np.testing.assert_allclose(plotted, [[.01, 0, .02], [0, .01, 0]], atol=1e-15)
        # Same global rigid motion must not be misrepresented as deformation.
        translation = np.array([.3, -.1, .2])
        translations = centers @ rotation.T-centers+translation
        plotted, delta = frame_blend_display(reference, centers, translations,
                                             np.stack([rotation, rotation]), 100)
        np.testing.assert_allclose(delta, 0, atol=1e-15)
        np.testing.assert_allclose(plotted, reference, atol=1e-13)

    def test_gain_and_offset_without_input_mutation(self):
        reference = np.array([[0., 0., .02], [.01, 0., 0.]])
        solved = reference+[[0., 0., 0.], [0., .00002, .00001]]
        saved = solved.copy()
        plotted, delta = displacement_plot(reference, solved, [0, 0, .02], [0, 0, 0], np.eye(3), 100, [.06, 0, 0])
        np.testing.assert_allclose(plotted, reference+100*(solved-reference)+[.06, 0, 0])
        np.testing.assert_array_equal(solved, saved)
        np.testing.assert_allclose(delta, solved-reference)

    def test_whole_leg_rigid_drop_motion_is_not_magnified(self):
        reference = np.array([[0., 0., .02], [.01, 0., 0.], [0., .01, .01]])
        center, translation = np.array([0, 0, .02]), np.array([.2, -.1, -.07])
        rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        solved = (reference-center) @ rotation.T+center+translation
        plotted, delta = displacement_plot(reference, solved, center, translation, rotation, 100, [0, 0, 0])
        np.testing.assert_allclose(delta, 0, atol=1e-16)
        np.testing.assert_allclose(plotted, reference, atol=1e-14)

    def test_invalid_gains_rejected(self):
        points = np.zeros((2, 3))
        for gain in (0, -1, 501, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                displacement_plot(points, points, [0, 0, 0], [0, 0, 0], np.eye(3), gain, [0, 0, 0])


if __name__ == "__main__":
    unittest.main()

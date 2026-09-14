"""Check the display projection cannot alter the physical result."""

import unittest

import numpy as np

from exact_joint.shell_display_math import displacement_plot


class DisplayTests(unittest.TestCase):
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

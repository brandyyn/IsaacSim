import unittest

import numpy as np

from exact_joint.shell_presentation import SolvedTransition


class PresentationTests(unittest.TestCase):
    def test_smooth_monotone_endpoints_and_no_mutation(self):
        a, b = np.zeros(9), np.ones(9)
        view = SolvedTransition(a)
        view.begin(a, b, 1, .1, .2)
        states = [view.sample()[0]]
        for _ in range(60):
            states.append(view.advance(1/60)[0])
        np.testing.assert_array_equal(states[0], a)
        np.testing.assert_array_equal(states[-1], b)
        self.assertTrue(np.all(np.diff(states, axis=0) >= 0))
        self.assertLess(states[1][0], .0001)
        self.assertFalse(view.active)
        self.assertEqual(view.sample()[1], .2)
        b[:] = 9
        states[-1][:] = 7
        np.testing.assert_array_equal(view.sample()[0], np.ones(9))

    def test_pause_cancel_reset_and_large_clock(self):
        view = SolvedTransition([0, 0])
        view.begin([0, 0], [1, 2], 2)
        first = view.advance(.5)[0]
        np.testing.assert_array_equal(view.advance(100, paused=True)[0], first)
        self.assertTrue(view.active)
        np.testing.assert_array_equal(view.advance(100)[0], [1, 2])
        view.reset([0, 0])
        self.assertFalse(view.active)
        np.testing.assert_array_equal(view.sample()[0], [0, 0])

    def test_invalid_endpoints_rejected(self):
        view = SolvedTransition([0])
        for a, b, duration in (([0], [0, 1], 1), ([0], [np.nan], 1), ([0], [1], 0)):
            with self.assertRaises(ValueError):
                view.begin(a, b, duration)
        with self.assertRaises(ValueError):
            view.advance(-1)

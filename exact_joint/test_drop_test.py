"""Analytic ballistic invariants; these tests do not validate impact mechanics."""

import dataclasses
import math
import unittest

from exact_joint.drop_test import DropCase


class DropTests(unittest.TestCase):
    def test_seventy_mm_contact(self):
        case = DropCase()
        result = case.sample(case.contact_time_s)
        self.assertAlmostEqual(result["clearance_m"], 0, places=14)
        self.assertAlmostEqual(result["incident_downward_speed_m_s"], 1.1719214990774767)
        self.assertAlmostEqual(result["kinetic_energy_j"], 0.034335)
        self.assertAlmostEqual(case.contact_time_s, 0.1194619265114655)

    def test_energy_conservation_and_mass_independent_fall(self):
        case = DropCase()
        heavier = dataclasses.replace(case, upper_mass_kg=.060, lower_mass_kg=.040)
        for fraction in (0, .1, .5, .9, 1):
            state = case.sample(fraction * case.contact_time_s)
            other = heavier.sample(fraction * case.contact_time_s)
            self.assertEqual(state["clearance_m"], other["clearance_m"])
            self.assertAlmostEqual(state["kinetic_energy_j"] * 2, other["kinetic_energy_j"])
            self.assertAlmostEqual(state["kinetic_energy_j"] + case.mass_kg * case.gravity_m_s2 * state["clearance_m"], .034335)

    def test_no_post_contact_or_survival_extrapolation(self):
        case = DropCase()
        self.assertEqual(case.sample(10), case.sample(case.contact_time_s))
        for key in ("survives", "impact_force_n", "impact_stress_pa"):
            self.assertIsNone(case.sample(10)[key])
        self.assertEqual(case.report()["status"], "SURVIVAL_INDETERMINATE")

    def test_invalid_inputs(self):
        for key in dataclasses.asdict(DropCase()):
            for value in (-1, 0, math.inf, math.nan):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    DropCase(**{key: value})
        for elapsed in (-1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                DropCase().sample(elapsed)


if __name__ == "__main__":
    unittest.main()

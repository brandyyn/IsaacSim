"""Fold compliance and pull-direction checks for the open-ended shell."""

import dataclasses
import unittest
from pathlib import Path

import numpy as np

from exact_joint.geometry import JointConfig, load_source
from exact_joint.mechanics import tension_pattern
from exact_joint.nonlinear_shell import NonlinearShell, ShellConfig


class FoldControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = load_source(Path(__file__).with_name("source_joint.json"))
        cls.config = ShellConfig(sparse_solver=True, physical_strip_bending=True,
                                 crease_twist_ratio=0, frame_hinge_width_m=.0002, open_ends=True)
        cls.shell = NonlinearShell(cls.source, JointConfig(), cls.config)

    def test_reference_ratio_preserves_pet_law_and_control_changes_only_bending(self):
        reference = self.shell
        softened = NonlinearShell(self.source, reference.material,
                                  dataclasses.replace(self.config, panel_to_crease_ratio=1000))
        self.assertAlmostEqual(reference.effective_strip_rigidity_nm, reference.pet_rigidity_nm)
        self.assertAlmostEqual(softened.effective_strip_rigidity_nm, reference.pet_rigidity_nm/10)
        np.testing.assert_array_equal(reference.membrane_modulus.numpy(), softened.membrane_modulus.numpy())
        np.testing.assert_array_equal(reference.points, softened.points)
        state = np.random.default_rng(91).normal(size=reference.ndof)*1e-5
        original = reference.diagnostics(state)
        changed = softened.diagnostics(state)
        self.assertEqual(original["energy_j"]["membrane"], changed["energy_j"]["membrane"])
        self.assertEqual(original["energy_j"]["panel_bending"], changed["energy_j"]["panel_bending"])
        self.assertGreater(original["energy_j"]["folding"], changed["energy_j"]["folding"])
        self.assertAlmostEqual(changed["fold_line_model"]["strip_bending_scale_from_pet"], .1)
        self.assertAlmostEqual(changed["fold_line_model"]["actual_panel_to_strip_ratio"],
                               softened.panel_rigidity_nm/softened.effective_strip_rigidity_nm)

    def test_softer_folds_increase_accepted_compression(self):
        softer = NonlinearShell(self.source, self.shell.material,
                               dataclasses.replace(self.config, panel_to_crease_ratio=1000))
        rows = [model.solve(tension_pattern("Compression", 1))[1] for model in (self.shell, softer)]
        for row in rows:
            self.assertTrue(row["accepted"])
            self.assertEqual(row["acceptance_failures"], [])
        self.assertGreater(rows[1]["compression_fraction"], rows[0]["compression_fraction"])

    def test_all_seven_patterns_have_correct_opposite_rotation_and_acceptance(self):
        _, compression = self.shell.solve(tension_pattern("Compression", 1))
        self.assertTrue(compression["accepted"])
        self.assertGreater(compression["compression_fraction"], 0)
        for axis, names in enumerate((("Bend X+", "Bend X-"), ("Bend Y+", "Bend Y-"),
                                       ("Twist CW", "Twist CCW"))):
            rotations = []
            for name in names:
                _, report = self.shell.solve(tension_pattern(name, 1))
                self.assertTrue(report["accepted"], (name, report["acceptance_failures"]))
                self.assertLess(report["gradient_max_j_per_scaled_coordinate"], 1e-5)
                rotations.append(report["relative_rotation_rad"][axis])
            self.assertLess(rotations[0]*rotations[1], -1e-10)

    def test_converged_high_load_is_rejected_for_pet_strain(self):
        # This exercises the current numerical guard; 3% is not a measured
        # material failure strain and the result is not a strength prediction.
        _, report = self.shell.solve(tension_pattern("Compression", 30))
        self.assertTrue(report["converged"])
        self.assertFalse(report["accepted"])
        self.assertIn("pet_strip_strain_guard", report["acceptance_failures"])
        self.assertNotIn("force_residual", report["acceptance_failures"])


if __name__ == "__main__":
    unittest.main()

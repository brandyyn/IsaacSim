"""Numerical invariants for the exploratory reduced impact model."""

from __future__ import annotations

import dataclasses
import unittest
from pathlib import Path

import numpy as np

from exact_joint.geometry import JointConfig, load_source
from exact_joint.impact import FemImpact, ImpactConfig, rigid_jacobian
from exact_joint.mechanics import CableFem


class ImpactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = load_source(Path(__file__).with_name("source_joint.json"))
        cls.fem = CableFem(cls.source, JointConfig())

    def make_model(self, **kwargs) -> FemImpact:
        return FemImpact(self.fem, ImpactConfig(**kwargs))

    def test_mass_stiffness_and_rigid_motion(self) -> None:
        model = self.make_model()
        np.testing.assert_allclose(model.mass, model.mass.T, atol=1e-14)
        self.assertGreater(np.linalg.eigvalsh(model.mass).min(), 0)
        np.testing.assert_allclose(model.stiffness, model.stiffness.T, atol=1e-7)
        translation = np.array([.002, .003, -.004])
        rotation = np.array([.001, -.002, .003])
        lower = translation + np.cross(rotation, [0, 0, -self.source["height_m"]])
        rigid = np.r_[translation, rotation, lower, rotation]
        np.testing.assert_allclose(model.relative @ rigid, 0, atol=1e-16)
        np.testing.assert_allclose(model.stiffness @ rigid, 0, atol=1e-9)
        projected = model.modes.T @ (self.fem.stiffness @ model.modes)
        # Assess the whole matrix: thin-solid cancellation makes relative errors
        # on its near-zero coupling entries an ill-conditioned comparison.
        self.assertLess(np.linalg.norm(projected - model.knee_stiffness)
                        / np.linalg.norm(model.knee_stiffness), 1e-8)

    def test_gravity_freefall_has_no_artificial_bending(self) -> None:
        model = self.make_model()
        acceleration = np.linalg.solve(model.mass, model.gravity)
        expected = np.zeros(12)
        expected[[2, 8]] = -9.81
        np.testing.assert_allclose(acceleration, expected, atol=1e-12)
        np.testing.assert_allclose(model.relative @ model.velocity, 0, atol=1e-14)
        np.testing.assert_allclose(model.recover()["displacements"], 0, atol=1e-14)
        self.assertAlmostEqual(model.initial_energy_j, .034335)

    def test_contact_pull_off_and_moment(self) -> None:
        model = self.make_model()
        lifted = np.zeros(12)
        lifted[8] = .001
        np.testing.assert_array_equal(model.contact_forces(lifted, model.velocity), 0)
        pressed = np.zeros(12)
        pressed[8] = -.001
        forces = model.contact_forces(pressed, np.zeros(12))
        np.testing.assert_allclose(forces, 5, atol=1e-12)
        wrench = model.contact_jacobian.T @ forces
        self.assertAlmostEqual(wrench[8], 20)
        self.assertAlmostEqual(wrench[10], -.012 * 20)
        self.assertAlmostEqual(wrench[9], 0)

    def test_contact_changes_fem_pose_and_stress(self) -> None:
        model = self.make_model()
        for _ in range(40):
            model.advance()
        result = model.recover()
        self.assertGreater(model.trace[-1]["ground_force_n"], 0)
        self.assertLess(result["q"][4], 0)
        self.assertGreater(result["q"][2], 0)
        self.assertGreater(result["stats"]["pet_peak_pa"], 0)
        np.testing.assert_allclose(result["displacements"].ravel(), model.modes @ result["q"], atol=1e-15)

    def test_contact_eccentricity_changes_bend(self) -> None:
        forward, backward = self.make_model(), self.make_model()
        backward.contact_points[:, 0] *= -1
        backward.contact_jacobian[:, 6:] = np.stack([rigid_jacobian(p)[2] for p in backward.contact_points])
        # Change contact eccentricity only; keep the same assembly mass properties.
        backward.reset()
        for _ in range(40):
            forward.advance()
            backward.advance()
        self.assertLess((forward.relative @ forward.position)[4] * (backward.relative @ backward.position)[4], 0)

    def test_energy_and_dynamic_balance_until_guard(self) -> None:
        model = self.make_model()
        while not model.stopped:
            model.advance()
        self.assertIn("SMALL_DEFORMATION_LIMIT", model.stop_reason)
        self.assertLess(max(abs(row["relative_energy_error"]) for row in model.trace), 1e-5)
        self.assertLess(max(row["dynamic_residual_max_n_nm"] for row in model.trace), 1e-8)
        self.assertLessEqual(model.trace[-1]["max_principal_strain"], .01)
        self.assertGreater(model.trace[-1]["max_principal_strain"], .00999)
        previous = model.position.copy()
        model.advance()
        np.testing.assert_array_equal(previous, model.position)
        self.assertIsNone(model.report()["survives"])

    def test_timestep_refinement(self) -> None:
        models = [self.make_model(step_s=step) for step in (1e-5, 5e-6, 2.5e-6)]
        for model in models:
            while not model.stopped:
                model.advance()
        for coarse, fine in zip(models, models[1:]):
            self.assertLess(abs(coarse.time_s / fine.time_s - 1), .002)
            self.assertLess(abs(coarse.trace[-1]["ground_force_n"] / fine.trace[-1]["ground_force_n"] - 1), .002)
            self.assertLess(abs((coarse.relative @ coarse.position)[4] / (fine.relative @ fine.position)[4] - 1), .002)

    def test_contact_stiffness_and_material_change_response(self) -> None:
        normal = self.make_model()
        harder = self.make_model(contact_stiffness_n_m=40000)
        config = dataclasses.replace(self.fem.config, pet_modulus_pa=7e9, pla_modulus_pa=4.4e9)
        stiff_fem = FemImpact(CableFem(self.source, config))
        for _ in range(40):
            for model in (normal, harder, stiff_fem):
                model.advance()
        self.assertGreater(harder.trace[-1]["ground_force_n"], normal.trace[-1]["ground_force_n"])
        self.assertLess((stiff_fem.relative @ stiff_fem.position)[2], (normal.relative @ normal.position)[2])
        self.assertFalse(np.allclose(stiff_fem.position, normal.position, atol=1e-10, rtol=0))

    def test_invalid_parameters(self) -> None:
        for kwargs in ({"height_m": 0}, {"upper_mass_kg": -1}, {"contact_stiffness_n_m": 0},
                       {"contact_damping_ns_m": -1}, {"step_s": .001}, {"height_m": np.nan}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.make_model(**kwargs)


if __name__ == "__main__":
    unittest.main()

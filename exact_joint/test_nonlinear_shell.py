"""Implementation invariants, not physical validation of the nonlinear knee."""

import dataclasses
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from exact_joint.geometry import JointConfig, load_source
from exact_joint.mechanics import tension_pattern
from exact_joint.nonlinear_shell import NonlinearShell, ShellConfig, surface_mesh
from exact_joint.shell_contact import intersection_pairs
from exact_joint.shell_impact import ShellImpact, ShellImpactConfig


class NonlinearShellTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = load_source(Path(__file__).with_name("source_joint.json"))
        cls.shell = NonlinearShell(cls.source, JointConfig())

    def test_source_and_reference_surface_invariant(self):
        areas = []
        for level in (1, 2):
            mesh = surface_mesh(self.source, level, self.shell.material.hinge_gap_m)
            np.testing.assert_array_equal(mesh["points"][:28], self.source["points"])
            self.assertEqual(len(mesh["chains"]), 76)
            self.assertEqual(len(mesh["frame_line_ids"]), 8)
            p = mesh["points"][mesh["triangles"]]
            areas.append(np.linalg.norm(np.cross(p[:, 1]-p[:, 0], p[:, 2]-p[:, 0]), axis=1).sum()/2)
        self.assertAlmostEqual(areas[0], areas[1], places=14)

    def test_roof_interiors_free_and_perimeter_exact(self):
        shell = self.shell
        for frame, name in enumerate(("top", "bottom")):
            z = shell.height if frame == 0 else 0
            free_roof = shell.free_nodes[np.isclose(shell.points[shell.free_nodes, 2], z)]
            self.assertEqual(len(free_roof), 9)
            state = np.zeros(shell.ndof)
            state[shell.frame_start+6*frame+3:shell.frame_start+6*frame+6] = [.2, .3, -.4]
            free_slot = np.flatnonzero(shell.free_nodes == free_roof[0])[0]
            state[3*free_slot+2] = .002/shell.width
            p = shell.positions(shell._tensor(state)).numpy()
            ids = shell.mesh[name]
            np.testing.assert_allclose(np.linalg.norm(p[ids, None]-p[ids][None], axis=2),
                                       np.linalg.norm(shell.points[ids, None]-shell.points[ids][None], axis=2), atol=1e-14)
            self.assertAlmostEqual(p[free_roof[0], 2]-shell.points[free_roof[0], 2], .002)

    def test_rigid_motion_has_no_elastic_energy(self):
        shell = self.shell
        rotation = Rotation.from_rotvec([.5, -.3, .8])
        translation = np.array([.01, -.02, .03])
        state = np.zeros(shell.ndof)
        state[:shell.frame_start] = ((rotation.apply(shell.points[shell.free_nodes])+translation-shell.points[shell.free_nodes])/shell.width).ravel()
        for frame, center in enumerate(shell.frame_centers.numpy()):
            state[shell.frame_start+6*frame:shell.frame_start+6*frame+3] = (rotation.apply(center)+translation-center)/shell.width
            state[shell.frame_start+6*frame+3:shell.frame_start+6*frame+6] = rotation.as_rotvec()
        self.assertLess(abs(shell.value_gradient(state)[0]), 1e-20)
        self.assertLess(shell.diagnostics(state)["max_membrane_strain"], 1e-11)

    def test_membrane_uniform_stretch_patch(self):
        shell = self.shell
        scale = 1.001
        strain = shell.strains(shell.rest*scale).numpy()
        expected = np.broadcast_to(np.eye(2)*.5*(scale**2-1), strain.shape)
        # Slender strip cells amplify roundoff; this is still <1e-10 of
        # the imposed 0.1% strain, not a relaxed physical-accuracy tolerance.
        np.testing.assert_allclose(strain, expected, atol=1e-13)

    def test_energy_gradient_matches_finite_difference(self):
        rng = np.random.default_rng(7)
        state = rng.normal(size=self.shell.ndof)*1e-5
        direction = rng.normal(size=self.shell.ndof)
        direction /= np.linalg.norm(direction)
        _, gradient = self.shell.value_gradient(state)
        epsilon = 1e-7
        difference = (self.shell.value_gradient(state+epsilon*direction)[0]
                      - self.shell.value_gradient(state-epsilon*direction)[0])/(2*epsilon)
        self.assertAlmostEqual(difference, gradient@direction, delta=2e-6*max(1, abs(difference)))

    def test_crease_sections_can_warp_and_twist(self):
        shell = self.shell
        state = np.zeros(shell.ndof)
        line = next(i for i in range(76) if i not in shell.mesh["frame_line_ids"])
        middle = shell.mesh["chains"][line][1]
        slot = np.flatnonzero(shell.free_nodes == middle)[0]
        state[3*slot+1] = .00005/shell.width
        report = shell.diagnostics(state)
        indices = shell.mesh["crease_ids"] == line
        self.assertGreater(np.ptp(report["hinge_angle_changes_rad"][indices]), 1e-5)
        self.assertGreater(report["energy_j"]["crease_twist"], 0)

    def test_ratio_changes_folding_not_membrane(self):
        shell = self.shell
        other = NonlinearShell(self.source, shell.material, dataclasses.replace(shell.config, panel_to_crease_ratio=1000))
        rng = np.random.default_rng(4)
        state = rng.normal(size=shell.ndof)*1e-4
        normal = shell.diagnostics(state)["energy_j"]
        softened = other.diagnostics(state)["energy_j"]
        self.assertAlmostEqual(normal["membrane"], softened["membrane"])
        self.assertAlmostEqual(normal["panel_bending"], softened["panel_bending"])
        self.assertLess(softened["folding"], normal["folding"])
        pet_only = ~np.any(shell.mesh["laminate"][shell.mesh["hinge_faces"]], axis=1)
        np.testing.assert_allclose(shell.hinge_stiffness[pet_only].numpy()/10,
                                   other.hinge_stiffness[pet_only].numpy(), rtol=1e-12)

    def test_pet_only_strip_width_and_material(self):
        shell = self.shell
        other = NonlinearShell(self.source, dataclasses.replace(shell.material, hinge_gap_m=.0004))
        pet = ~shell.mesh["laminate"]
        self.assertGreater(pet.sum(), 0)
        self.assertAlmostEqual(float(shell.membrane_modulus[pet][0]),
                               shell.material.pet_modulus_pa*shell.material.pet_thickness_m)
        self.assertGreater(float(other.area[~other.mesh["laminate"]].sum()), float(shell.area[pet].sum()))
        self.assertAlmostEqual(float(shell.area.sum()), float(other.area.sum()), places=14)
        np.testing.assert_array_equal(shell.points[:28], other.points[:28])

    def test_intersection_guard(self):
        shell = self.shell
        self.assertEqual(len(intersection_pairs(shell.points, shell.mesh["triangles"])), 0)
        p = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [.2, .2, -1], [.2, .2, 1], [.5, .2, .5]])
        self.assertEqual(len(intersection_pairs(p, [[0, 1, 2], [3, 4, 5]])), 1)
        # Topological neighbours and near-coplanar roundoff are not contact.
        for scale in (.0001, 1, 100):
            near = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 1e-13]])*scale
            self.assertEqual(len(intersection_pairs(near, [[0, 1, 2], [1, 3, 2]])), 0)

    def test_cable_force_and_moment_follow_anchor_geometry(self):
        shell = self.shell
        tension = np.arange(1, 13)*.01
        _, gradient = shell.value_gradient(np.zeros(shell.ndof), tension)
        top, bottom = shell.cable_top.numpy(), shell.cable_bottom.numpy()
        force_gradient = tension[:, None]*(top-bottom)/np.linalg.norm(top-bottom, axis=1)[:, None]
        for frame, anchor, sign in ((0, top, 1), (1, bottom, -1)):
            offset = shell.frame_start+6*frame
            expected = sign*force_gradient
            np.testing.assert_allclose(gradient[offset:offset+3]/shell.width, expected.sum(axis=0), atol=1e-10)
            moment = np.cross(anchor-shell.frame_centers[frame].numpy(), expected).sum(axis=0)
            np.testing.assert_allclose(gradient[offset+3:offset+6], moment, atol=1e-12)

    def test_force_inputs_change_solved_motion_and_unload(self):
        shell = self.shell
        state, first = shell.solve(tension_pattern("Compression", .1))
        state, second = shell.solve(tension_pattern("Compression", .2), initial=state)
        self.assertTrue(first["accepted"] and second["accepted"])
        self.assertGreater(second["compression_fraction"], first["compression_fraction"]*1.9)
        for pattern, axis in (("Bend Y+", 1), ("Twist CW", 2)):
            _, report = shell.solve(tension_pattern(pattern, .2))
            self.assertTrue(report["accepted"])
            self.assertLess(report["relative_rotation_rad"][axis], -1e-6)
        loads = np.zeros_like(shell.points)
        loads[shell.mesh["bottom"], 2] = .5/len(shell.mesh["bottom"])
        _, loaded = shell.solve(nodal_loads=loads)
        self.assertTrue(loaded["accepted"])
        self.assertGreater(loaded["compression_fraction"], 0)
        _, unloaded = shell.solve(initial=state)
        self.assertTrue(unloaded["accepted"])
        self.assertLess(abs(unloaded["compression_fraction"]), 1e-7)

    def test_elastic_residuals_match_energy(self):
        state = np.random.default_rng(17).normal(size=self.shell.ndof)*1e-5
        residuals = self.shell.residual_vector(self.shell._tensor(state)).numpy()
        energy = self.shell.value_gradient(state)[0]
        self.assertAlmostEqual(.5*residuals@residuals, energy, delta=1e-12)

    def test_dynamic_mass_force_motion_and_energy(self):
        model = ShellImpact(self.shell, ShellImpactConfig(duration_s=.0005))
        self.assertAlmostEqual(float(model.masses.sum()), .05)
        self.assertGreater(np.linalg.eigvalsh(model.mass_scaling).min(), 0)
        while not model.stopped:
            model.step()
        self.assertIn("TIME_WINDOW_COMPLETE", model.reason)
        self.assertGreater(model.trace[-1]["ground_force_n"], 0)
        self.assertGreater(model.trace[-1]["compression_fraction"], 0)
        self.assertLess(model.trace[-1]["relative_rotation_rad"][1], 0)
        self.assertLessEqual(max(row["energy_fraction_of_initial"] for row in model.trace), 1.00001)

    def test_invalid_configuration_rejected(self):
        for changes in ({"panel_to_crease_ratio": 0}, {"crease_twist_ratio": -1},
                        {"panel_bending_scale": 0}, {"panel_strain_limit": np.nan}):
            with self.assertRaises(ValueError):
                dataclasses.replace(ShellConfig(), **changes).validate()
        for changes in ({"height_m": 0}, {"step_s": .1}, {"contact_damping_ns_m": -1}):
            with self.assertRaises(ValueError):
                dataclasses.replace(ShellImpactConfig(), **changes).validate()


if __name__ == "__main__":
    unittest.main()

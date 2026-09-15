"""Numerical invariants for refinement, real-thickness bending and IPC.

These verify implementation, not physical material/crease calibration.
"""

import dataclasses
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from exact_joint.geometry import JointConfig, load_source
from exact_joint.mechanics import tension_pattern
from exact_joint.nonlinear_shell import NonlinearShell, ShellConfig, surface_mesh
from exact_joint.shell_actuation import winch_pull
from exact_joint.shell_impact import ShellImpact, ShellImpactConfig
from exact_joint.shell_sparse import position_jacobian, stiffness


class SparseShellTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = load_source(Path(__file__).with_name("source_joint.json"))
        cls.shell = NonlinearShell(cls.source, JointConfig(),
                                  ShellConfig(sparse_solver=True, physical_strip_bending=True, crease_twist_ratio=0))

    def test_uniform_refinement_preserves_area_material_and_source(self):
        previous = None
        for level in range(3):
            mesh = surface_mesh(self.source, gap_m=.0002, interior_refinement=level)
            np.testing.assert_array_equal(mesh["points"][:28], self.source["points"])
            p = mesh["points"][mesh["triangles"]]
            areas = np.linalg.norm(np.cross(p[:, 1]-p[:, 0], p[:, 2]-p[:, 0]), axis=1)/2
            per_owner = np.bincount(mesh["owners"], weights=areas)
            laminate_area = areas[mesh["laminate"]].sum()
            self.assertTrue(np.all(areas > 0))
            if previous is not None:
                np.testing.assert_allclose(per_owner, previous[0], atol=1e-17)
                self.assertAlmostEqual(laminate_area, previous[1], places=14)
                self.assertEqual(len(p), 4*previous[2])
                self.assertEqual(len(mesh["chains"][0])-1, 2*previous[3])
            previous = per_owner, laminate_area, len(p), len(mesh["chains"][0])-1

    def test_position_jacobian_at_rotated_frames(self):
        shell = self.shell
        q = np.zeros(shell.ndof); q[-3:] = [.3, -.2, .5]
        direction = np.random.default_rng(24).normal(size=shell.ndof)
        epsilon = 1e-7
        difference = (shell.positions(shell._tensor(q+epsilon*direction)).numpy()
                      - shell.positions(shell._tensor(q-epsilon*direction)).numpy())/(2*epsilon)
        np.testing.assert_allclose(position_jacobian(shell, q)@direction, difference.ravel(), atol=1e-10)

    def test_sparse_tangent_matches_dense_with_winch(self):
        shell = self.shell
        q = np.zeros(shell.ndof); q[-3:] = [.001, -.002, .003]
        winch = winch_pull(shell, "Compression", .0001)
        dense = shell.gauss_newton_stiffness(q)+shell.winch_stiffness(q, winch)
        np.testing.assert_allclose(stiffness(shell, q, winch).toarray(), dense, atol=1e-9, rtol=1e-8)

    def test_physical_pet_bending_ignores_ratio_and_scales_cubically(self):
        shell = self.shell
        other = NonlinearShell(self.source, shell.material,
                               dataclasses.replace(shell.config, panel_to_crease_ratio=9999))
        np.testing.assert_array_equal(shell.hinge_stiffness, other.hinge_stiffness)
        thicker = NonlinearShell(self.source, dataclasses.replace(shell.material, pet_thickness_m=.00016), shell.config)
        self.assertAlmostEqual(thicker.effective_strip_rigidity_nm/shell.effective_strip_rigidity_nm, 8)
        self.assertAlmostEqual(shell.effective_strip_rigidity_nm, shell.pet_rigidity_nm)

    def test_sparse_force_signs_and_unloading(self):
        shell = self.shell
        for plus, minus, axis in (("Bend X+", "Bend X-", 0), ("Bend Y+", "Bend Y-", 1), ("Twist CW", "Twist CCW", 2)):
            q, first = shell.solve(tension_pattern(plus, 1))
            _, second = shell.solve(tension_pattern(minus, 1))
            self.assertTrue(first["accepted"] and second["accepted"])
            self.assertLess(first["relative_rotation_rad"][axis]*second["relative_rotation_rad"][axis], 0)
            _, unloaded = shell.solve(initial=q)
            self.assertTrue(unloaded["accepted"])
            self.assertLess(abs(unloaded["compression_fraction"]), 1e-7)

    def test_sparse_impact_mass_and_force_response(self):
        model = ShellImpact(self.shell, ShellImpactConfig(duration_s=.0001))
        self.assertAlmostEqual(float(model.masses.sum()), .05)
        self.assertGreater(np.linalg.eigvalsh(model.mass_scaling.toarray()).min(), 0)
        # Local nonlinear Jacobian agrees with direct particle differentiation.
        q = model.state.copy(); q[-3:] = [.1, -.2, .15]
        direction = np.random.default_rng(4).normal(size=self.shell.ndof)
        epsilon = 1e-7
        difference = (model.particles(self.shell._tensor(q+epsilon*direction)).numpy()
                      - model.particles(self.shell._tensor(q-epsilon*direction)).numpy())/(2*epsilon)
        np.testing.assert_allclose(model.particle_jacobian(q)@direction, difference.ravel(), atol=2e-10)
        result = model.step()
        self.assertIn("TIME_WINDOW_COMPLETE", model.reason)
        self.assertGreater(result["ground_force_n"], 0)
        self.assertGreater(result["compression_fraction"], 0)
        self.assertLess(result["relative_rotation_rad"][1], 0)
        self.assertLessEqual(result["energy_fraction_of_initial"], 1.00001)

    def test_impact_progress_count_matches_time_window_tolerance(self):
        for duration, microseconds, expected in ((.001, 200, 5), (.015, 25, 600), (.00101, 200, 6)):
            config = ShellImpactConfig(duration_s=duration, step_s=microseconds*1e-6)
            config.validate()
            self.assertEqual(config.step_count, expected)

    def test_invalid_refinement_and_contact_combinations(self):
        for change in ({"interior_refinement": 3}, {"interior_refinement": 1}, {"self_contact": True},
                       {"contact_distance_m": 0}, {"contact_energy_j": -1}, {"subdivision": 3},
                       {"subdivision": 6}, {"subdivision": 1.5},
                       {"subdivision": 5, "interior_refinement": 2, "sparse_solver": True}):
            with self.assertRaises(ValueError):
                dataclasses.replace(ShellConfig(), **change).validate()

    def test_fine_boundary_subdivision_preserves_source_and_material_areas(self):
        previous = None
        for level in (1, 3, 5):
            ShellConfig(subdivision=level, sparse_solver=True).validate()
            mesh = surface_mesh(self.source, subdivision=level, gap_m=.0002)
            np.testing.assert_array_equal(mesh["points"][:28], self.source["points"])
            p = mesh["points"][mesh["triangles"]]
            area = np.linalg.norm(np.cross(p[:, 1]-p[:, 0], p[:, 2]-p[:, 0]), axis=1)/2
            values = np.r_[np.bincount(mesh["owners"], weights=area), area[mesh["laminate"]].sum()]
            if previous is not None:
                np.testing.assert_allclose(values, previous, atol=1e-16, rtol=1e-12)
            previous = values
            self.assertTrue(all(len(chain) == 2**level+1 for chain in mesh["chains"]))
            self.assertEqual(len(mesh["free_boundary_edges"]), 0)


class IPCTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from exact_joint.shell_ipc import MidsurfaceContact
        cls.points = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0], [.2, .2, .3], [.4, .2, .3], [.2, .4, .3]])
        cls.faces = np.array([[0, 1, 2], [3, 4, 5]])
        try:
            cls.contact = MidsurfaceContact(cls.points, cls.faces, .2, .001)
        except (ImportError, RuntimeError) as exc:
            raise unittest.SkipTest("Optional pinned IPC dependency unavailable: "+str(exc))

    def test_collision_free_sign_and_crossing_detection(self):
        other = self.points.copy(); other[3:, 2] = -.3
        self.assertTrue(self.contact.linear_step_is_feasible(self.points, self.points))
        self.assertFalse(self.contact.linear_step_is_feasible(self.points, other))
        # Both endpoints are nonintersecting: the swept path is what matters.
        self.assertTrue(self.contact.linear_step_is_feasible(other, other))

    def test_barrier_gradient_force_balance_and_psd_search_hessian(self):
        p = self.points.copy(); p[3:, 2] = .1
        energy, gradient, matrix = self.contact.evaluate(p, hessian=True)
        self.assertGreater(energy, 0)
        self.assertGreater(-gradient.reshape(-1, 3)[3:, 2].sum(), 0)
        np.testing.assert_allclose(gradient.reshape(-1, 3).sum(axis=0), 0, atol=1e-14)
        direction = np.random.default_rng(11).normal(size=p.shape)
        epsilon = 1e-7
        finite_difference = (self.contact.evaluate(p+epsilon*direction)[0]
                             - self.contact.evaluate(p-epsilon*direction)[0])/(2*epsilon)
        self.assertAlmostEqual(finite_difference, gradient@direction.ravel(), delta=1e-9)
        self.assertGreaterEqual(np.linalg.eigvalsh(matrix.toarray()).min(), -1e-10)
        self.assertAlmostEqual(self.contact.report(p)["minimum_active_distance_m"], .1)

    def test_no_spurious_reference_forces(self):
        energy, gradient, _ = self.contact.evaluate(self.points)
        self.assertEqual(energy, 0)
        np.testing.assert_array_equal(gradient, 0)
        from exact_joint.shell_ipc import MidsurfaceContact
        with self.assertRaises(ValueError):
            MidsurfaceContact(self.points, self.faces, .4, .001)

    def test_shell_contact_force_solution_and_ccd(self):
        source = load_source(Path(__file__).with_name("source_joint.json"))
        shell = NonlinearShell(source, JointConfig(), ShellConfig(sparse_solver=True, self_contact=True,
                                                                  physical_strip_bending=True))
        q, report = shell.solve(tension_pattern("Compression", 1))
        self.assertTrue(report["accepted"])
        self.assertEqual(report["self_contact"]["active_stencils"], 0)
        self.assertTrue(shell.contact.shell_step_is_feasible(shell, np.zeros(shell.ndof), q))
        with self.assertRaises(ValueError):
            shell.solve(lower_constraints={2: .001})

    def test_rotation_chord_bound(self):
        rng = np.random.default_rng(71)
        for _ in range(30):
            phi = rng.normal(size=3)*2
            change = rng.normal(size=3)
            point = rng.normal(size=3)*.02
            first = Rotation.from_rotvec(phi).apply(point)
            last = Rotation.from_rotvec(phi+change).apply(point)
            bound = np.linalg.norm(point)*float(change@change)/8
            for t in np.linspace(0, 1, 25):
                actual = Rotation.from_rotvec(phi+t*change).apply(point)
                self.assertLessEqual(np.linalg.norm(actual-((1-t)*first+t*last)), bound+1e-14)


if __name__ == "__main__":
    unittest.main()

"""Roof flexure implementation checks, not a calibrated PET failure model."""

import dataclasses
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from exact_joint.geometry import JointConfig, load_source
from exact_joint.mechanics import tension_pattern
from exact_joint.nonlinear_shell import NonlinearShell, ShellConfig, surface_mesh
from exact_joint.shell_impact import ShellImpact, ShellImpactConfig
from exact_joint.shell_sparse import stiffness


class FrameHingeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = load_source(Path(__file__).with_name("source_joint.json"))
        cls.config = ShellConfig(sparse_solver=True, physical_strip_bending=True,
                                 crease_twist_ratio=0, frame_hinge_width_m=.0002)
        cls.shell = NonlinearShell(cls.source, JointConfig(), cls.config)

    def test_exact_roofs_and_pet_area_survive_refinement_and_scaling(self):
        for width in (.010, .030, .040):
            source = load_source(Path(__file__).with_name("source_joint.json"), width)
            previous = None
            for level in (0, 1):
                m = surface_mesh(source, gap_m=.0002, interior_refinement=level, frame_hinge_width_m=.0002)
                np.testing.assert_array_equal(m["points"][:28], source["points"])
                p = m["points"][m["triangles"]]
                area = np.linalg.norm(np.cross(p[:, 1]-p[:, 0], p[:, 2]-p[:, 0]), axis=1)/2
                self.assertTrue(np.all(area > 0))
                self.assertEqual(len(m["free_boundary_edges"]), 0)
                self.assertEqual(len(m["chains"]), 76)
                self.assertEqual(len(np.unique(m["owners"])), 50)
                roofs = [i for i, panel in enumerate(source["panels"]) if len(panel["vertices"]) == 4]
                self.assertEqual(len(roofs), 2)
                for roof in roofs:
                    mask = m["owners"] == roof
                    self.assertAlmostEqual(area[mask].sum(), width**2, places=14)
                    self.assertAlmostEqual(area[mask & m["frame_hinge"]].sum(), width**2-(width-.0004)**2, places=14)
                    self.assertFalse(np.any(m["laminate"][mask & m["frame_hinge"]]))
                values = np.r_[np.bincount(m["owners"], weights=area), area[m["laminate"]].sum()]
                if previous is not None:
                    np.testing.assert_allclose(values, previous, atol=1e-16)
                previous = values

    def test_zero_width_is_legacy_and_side_panel_material_is_unchanged(self):
        old = surface_mesh(self.source, gap_m=.0002)
        zero = surface_mesh(self.source, gap_m=.0002, frame_hinge_width_m=0)
        for key in old:
            if key != "chains":
                np.testing.assert_array_equal(old[key], zero[key])
        self.assertFalse(np.any(zero["frame_hinge"]))
        new = self.shell.mesh
        for owner, panel in enumerate(self.source["panels"]):
            if len(panel["vertices"]) == 3:
                for m in (old, new):
                    p = m["points"][m["triangles"][m["owners"] == owner]]
                    if m is old:
                        expected = p
                    else:
                        np.testing.assert_array_equal(p, expected)

    def test_only_perimeters_rigid_plate_and_strip_nodes_are_free(self):
        shell = self.shell
        m = shell.mesh
        for name, z in (("top", shell.height), ("bottom", 0)):
            np.testing.assert_allclose(m["points"][m[name], 2], z, atol=1e-14)
        inner = np.unique(m["frame_hinge_edges"])
        self.assertTrue(set(inner).issubset(set(shell.free_nodes)))
        self.assertFalse(set(inner) & set(np.r_[m["top"], m["bottom"]]))
        q = np.zeros(shell.ndof)
        node = inner[0]
        index = np.flatnonzero(shell.free_nodes == node)[0]
        q[index*3+2] = 1e-5
        deformed = shell.positions(shell._tensor(q)).numpy()
        np.testing.assert_array_equal(deformed[np.r_[m["top"], m["bottom"]]], shell.points[np.r_[m["top"], m["bottom"]]])
        self.assertGreater(deformed[node, 2]-shell.points[node, 2], 0)
        self.assertGreater(float(sum(shell.elastic_terms(shell._tensor(deformed)))), 0)

    def test_neutral_energy_rigid_body_invariance_and_physical_strip_law(self):
        shell = self.shell
        self.assertLess(float(sum(shell.elastic_terms(shell.rest))), 1e-20)
        moved = shell.points @ Rotation.from_rotvec([.4, -.2, .1]).as_matrix().T + [.01, -.03, .02]
        self.assertLess(float(sum(shell.elastic_terms(shell._tensor(moved)))), 1e-18)
        band = shell.mesh["frame_hinge"]
        expected = shell.material.pet_modulus_pa*shell.material.pet_thickness_m
        np.testing.assert_allclose(shell.membrane_modulus.numpy()[band], expected)
        ratio = NonlinearShell(self.source, shell.material, dataclasses.replace(self.config, panel_to_crease_ratio=9999))
        np.testing.assert_array_equal(shell.hinge_stiffness, ratio.hinge_stiffness)
        thicker = NonlinearShell(self.source, dataclasses.replace(shell.material, pet_thickness_m=.00016), self.config)
        adjacent = shell.mesh["hinge_faces"]
        pure_pet = np.all(band[adjacent], axis=1)
        self.assertTrue(np.any(pure_pet))
        np.testing.assert_allclose(thicker.hinge_stiffness.numpy()[pure_pet], 8*shell.hinge_stiffness.numpy()[pure_pet])

    def test_sparse_tangent_matches_dense_for_new_flexures(self):
        q = np.zeros(self.shell.ndof)
        q[-3:] = [.0001, -.0002, .0003]
        np.testing.assert_allclose(stiffness(self.shell, q).toarray(), self.shell.gauss_newton_stiffness(q), rtol=1e-8, atol=1e-9)

    def test_cables_load_hinges_opposite_twist_and_return_to_neutral(self):
        reports = {}
        for pattern in ("Compression", "Bend Y+", "Twist CW", "Twist CCW"):
            q, report = self.shell.solve(tension_pattern(pattern, 1))
            self.assertTrue(report["accepted"], report)
            self.assertGreater(report["frame_hinge"]["max_local_dihedral_change_rad"], 0)
            self.assertGreater(report["frame_hinge"]["bending_energy_j"], 0)
            self.assertEqual(report["frame_hinge"]["width_m"], .0002)
            reports[pattern] = report
        self.assertGreater(reports["Compression"]["compression_fraction"], 0)
        self.assertLess(reports["Twist CW"]["relative_rotation_rad"][2]*reports["Twist CCW"]["relative_rotation_rad"][2], 0)
        _, unloaded = self.shell.solve(initial=q)
        self.assertTrue(unloaded["accepted"])
        self.assertLess(abs(unloaded["compression_fraction"]), 1e-7)

    def test_impact_uses_the_same_hinged_shell(self):
        impact = ShellImpact(self.shell, ShellImpactConfig(duration_s=.0001))
        result = impact.step()
        self.assertIn("TIME_WINDOW_COMPLETE", impact.reason)
        self.assertGreater(result["ground_force_n"], 0)
        report = self.shell.diagnostics(impact.state)
        self.assertGreater(report["frame_hinge"]["bending_energy_j"], 0)
        self.assertLessEqual(result["energy_fraction_of_initial"], 1.00001)

    def test_invalid_widths_rejected(self):
        for width in (-.0001, .0021, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                dataclasses.replace(self.config, frame_hinge_width_m=width).validate()
        for width in (-.0001, .015):
            with self.assertRaises(ValueError):
                surface_mesh(self.source, frame_hinge_width_m=width)


if __name__ == "__main__":
    unittest.main()

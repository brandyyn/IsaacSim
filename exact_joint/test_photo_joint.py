"""Open-frame topology and material checks; not experimental calibration."""

import dataclasses
import copy
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from exact_joint.fold_compatibility import rigid_facet_audit
from exact_joint.geometry import JointConfig, load_source
from exact_joint.mechanics import tension_pattern
from exact_joint.nonlinear_shell import NonlinearShell, ShellConfig, end_plane_panel_ids, surface_mesh
from exact_joint.shell_impact import ShellImpact, ShellImpactConfig


class PhotoJointTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = load_source(Path(__file__).with_name("source_joint.json"))
        cls.config = ShellConfig(sparse_solver=True, physical_strip_bending=True,
                                 crease_twist_ratio=0, frame_hinge_width_m=.0002, open_ends=True)
        cls.shell = NonlinearShell(cls.source, JointConfig(), cls.config)

    def test_open_ends_preserve_source_side_area_and_continuous_pet(self):
        areas = []
        for level in (0, 1):
            m = surface_mesh(self.source, gap_m=.0002, frame_hinge_width_m=.0002,
                             open_ends=True, interior_refinement=level)
            np.testing.assert_array_equal(m["points"][:28], self.source["points"])
            self.assertEqual(set(m["owners"]), set(range(48)))
            np.testing.assert_array_equal(m["excluded_panel_ids"], [48, 49])
            np.testing.assert_array_equal(m["retained_panel_ids"], np.arange(48))
            self.assertEqual(len(m["chains"]), 76)
            self.assertEqual(m["relief_boundary_edges"].shape, (0, 2))
            np.testing.assert_array_equal(m["end_boundary_edges"], m["free_boundary_edges"])
            edges = m["points"][m["free_boundary_edges"], 2]
            self.assertTrue(np.all(np.all(np.isclose(edges, 0), axis=1)
                                   | np.all(np.isclose(edges, self.source["height_m"]), axis=1)))
            p = m["points"][m["triangles"]]
            a = np.linalg.norm(np.cross(p[:, 1]-p[:, 0], p[:, 2]-p[:, 0]), axis=1)/2
            self.assertTrue(np.all(a > 0))
            self.assertFalse(np.any(m["laminate"][m["frame_hinge"]]))
            for owner in range(48):
                face = self.source["points"][self.source["panels"][owner]["vertices"]]
                self.assertAlmostEqual(a[m["owners"] == owner].sum(), np.linalg.norm(np.cross(face[1]-face[0], face[2]-face[0]))/2, places=14)
            areas.append([a.sum(), a[m["laminate"]].sum(), a[m["frame_hinge"]].sum()])
        np.testing.assert_allclose(areas[0], areas[1], atol=1e-16)

    def test_end_removal_is_independent_of_cap_triangulation(self):
        source = copy.deepcopy(self.source)
        source["panels"] = source["panels"][:48]
        for cap in self.source["panels"][48:]:
            a, b, c, d = cap["vertices"]
            source["panels"].extend([{"vertices": [a, b, c]}, {"vertices": [a, c, d]}])
        np.testing.assert_array_equal(end_plane_panel_ids(source), [48, 49, 50, 51])
        mesh = surface_mesh(source, gap_m=.0002, frame_hinge_width_m=.0002, open_ends=True)
        # Cap faces and their diagonal never enter the shell, mass, contact, or
        # rendered triangles; the original side topology is byte-for-byte equal.
        np.testing.assert_array_equal(mesh["triangles"], self.shell.mesh["triangles"])
        np.testing.assert_array_equal(mesh["points"], self.shell.mesh["points"])
        np.testing.assert_array_equal(mesh["hinges"], self.shell.mesh["hinges"])

    def test_open_surface_is_one_connected_annulus_without_roof_triangles(self):
        mesh = self.shell.mesh
        triangles = mesh["triangles"]
        nodes = set(triangles.ravel())
        adjacency = {node: set() for node in nodes}
        edges = set()
        for a, b, c in triangles:
            for u, v in ((a, b), (b, c), (c, a)):
                adjacency[u].add(v)
                adjacency[v].add(u)
                edges.add(tuple(sorted((u, v))))
        visited, pending = set(), [next(iter(nodes))]
        while pending:
            node = pending.pop()
            if node not in visited:
                visited.add(node)
                pending.extend(adjacency[node]-visited)
        self.assertEqual(visited, nodes)
        self.assertEqual(len(nodes)-len(edges)+len(triangles), 0)
        z = mesh["points"][triangles, 2]
        self.assertFalse(np.any(np.all(np.isclose(z, 0, atol=1e-12), axis=1)))
        self.assertFalse(np.any(np.all(np.isclose(z, self.source["height_m"], atol=1e-12), axis=1)))
        # Exactly two closed boundary loops: no unintentional waist/corner slit.
        boundaries = mesh["end_boundary_edges"]
        _, degrees = np.unique(boundaries, return_counts=True)
        self.assertTrue(np.all(degrees == 2))
        self.assertEqual(len(boundaries), len(mesh["top"])+len(mesh["bottom"]))

    def test_no_orphan_dofs_after_caps_are_removed(self):
        shell = self.shell
        active = np.unique(shell.mesh["triangles"])
        self.assertEqual(set(shell.free_nodes) | set(shell.mesh["top"]) | set(shell.mesh["bottom"]), set(active))
        np.testing.assert_array_equal(active, np.arange(len(shell.points)))

    def test_frame_interfaces_are_free_and_have_specified_setback(self):
        shell = self.shell
        nodes = np.unique(shell.mesh["frame_hinge_edges"])
        self.assertTrue(set(nodes).issubset(set(shell.free_nodes)))
        for owner in (9, 10, 21, 22, 32, 33, 44, 45):
            face = self.source["points"][self.source["panels"][owner]["vertices"]]
            # Two original vertices lie on the relevant perimeter edge.
            pairs = [(face[i], face[j]) for i, j in ((0, 1), (1, 2), (2, 0))
                     if np.isclose(face[i, 2], face[j, 2])]
            self.assertEqual(len(pairs), 1)
            a, b = pairs[0]
            triangles = shell.mesh["triangles"][(shell.mesh["owners"] == owner) & shell.mesh["laminate"]]
            p = shell.points[np.unique(triangles)]
            distance = np.linalg.norm(np.cross(b-a, p-a), axis=1)/np.linalg.norm(b-a)
            self.assertAlmostEqual(distance.min(), .0002, places=12)

    def test_neutral_and_rigid_body_energy(self):
        shell = self.shell
        self.assertLess(float(sum(shell.elastic_terms(shell.rest))), 1e-20)
        p = shell.points @ Rotation.from_rotvec([.2, -.1, .4]).as_matrix().T + [.01, .02, .03]
        self.assertLess(float(sum(shell.elastic_terms(shell._tensor(p)))), 1e-18)
        self.assertEqual(shell.material.pla_thickness_m, .0004)
        self.assertEqual(shell.material.pet_thickness_m, .00008)
        self.assertEqual(shell.config.membrane_scale, 1)

    def test_force_response_and_opposite_twist(self):
        rows = []
        for pattern in ("Compression", "Bend Y+", "Twist CW", "Twist CCW"):
            _, r = self.shell.solve(tension_pattern(pattern, 1))
            self.assertTrue(r["accepted"], r)
            self.assertGreater(r["frame_hinge"]["bending_energy_j"], 0)
            rows.append(r)
        self.assertGreater(rows[0]["compression_fraction"], 0)
        self.assertLess(rows[2]["relative_rotation_rad"][2]*rows[3]["relative_rotation_rad"][2], 0)

    def test_impact_same_open_mesh(self):
        model = ShellImpact(self.shell, ShellImpactConfig(duration_s=.0001))
        row = model.step()
        self.assertIn("TIME_WINDOW_COMPLETE", model.reason)
        self.assertGreater(row["ground_force_n"], 0)
        self.assertLessEqual(row["energy_fraction_of_initial"], 1.00001)

    def test_open_design_requires_frame_hinge(self):
        with self.assertRaises(ValueError):
            dataclasses.replace(self.config, frame_hinge_width_m=0).validate()

    def test_ideal_rigid_facet_audit_is_full_rank_at_neutral(self):
        report = rigid_facet_audit(self.source, .03)
        self.assertEqual(report["coordinates"], 66)
        self.assertEqual(report["rank"], 66)
        self.assertEqual(report["nullity"], 0)

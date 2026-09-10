"""Winch/cut implementation checks, not experimental material calibration."""

import dataclasses
import json
import unittest
from pathlib import Path

import numpy as np
import torch

from exact_joint.geometry import JointConfig, load_source
from exact_joint.nonlinear_shell import NonlinearShell, ShellConfig, surface_mesh
from exact_joint.shell_actuation import CableWinch, winch_pull
from exact_joint.shell_contact import intersection_pairs
from exact_joint.shell_impact import ShellImpact


class WinchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = load_source(Path(__file__).with_name("source_joint.json"))
        cls.shell = NonlinearShell(cls.source)

    def test_slack_elastic_capped_force_is_potential_gradient(self):
        winch = CableWinch((.02,)*12, (True,)*11+(False,), 1000, 2)
        winch.validate()
        lengths = torch.tensor([.019, .0205, .023]*4, dtype=torch.float64, requires_grad=True)
        energy, force = winch.response(lengths)
        np.testing.assert_allclose(torch.autograd.grad(energy, lengths)[0], force.detach(), atol=1e-12)
        np.testing.assert_allclose(force.detach()[:3], [0, .5, 2], atol=1e-12)
        self.assertEqual(float(force[-1].detach()), 0)

    def test_winch_gradient_and_tangent(self):
        shell = self.shell
        winch = winch_pull(shell, "Twist CW", .0001)
        state = np.zeros(shell.ndof)
        _, gradient = shell.value_gradient(state, winch=winch)
        direction = np.zeros(shell.ndof)
        direction[-1] = 1
        epsilon = 1e-6
        # Isolate cable energy: rotating only a frame also strains the narrow
        # PET cells, whose large third derivatives pollute this finite step.
        def cable_energy(value):
            return shell.value_gradient(value, winch=winch)[0]-shell.value_gradient(value)[0]
        difference = (cable_energy(state+epsilon*direction)-cable_energy(state-epsilon*direction))/(2*epsilon)
        self.assertAlmostEqual(difference, gradient@direction, delta=1e-9)
        tangent = shell.winch_stiffness(state, winch)
        self.assertGreater(tangent[-1, -1], 0)
        np.testing.assert_allclose(tangent, tangent.T, atol=1e-14)
        json.dumps(dataclasses.asdict(winch))

    def test_pull_is_load_not_imposed_frame_pose_and_unloads(self):
        shell = self.shell
        zero, rest = shell.solve(winch=winch_pull(shell, "Compression", 0))
        self.assertTrue(rest["accepted"])
        np.testing.assert_allclose(zero, 0, atol=1e-10)
        state, report = shell.solve(winch=winch_pull(shell, "Compression", .0001))
        self.assertTrue(report["accepted"])
        self.assertEqual(report["load_type"], "elastic_winch_pull")
        self.assertGreater(report["compression_fraction"], 0)
        self.assertLess(shell.height*report["compression_fraction"], .0001)
        self.assertGreater(max(report["tensions_n"]), 0)
        unloaded, report = shell.solve(winch=winch_pull(shell, "Compression", 0), initial=state)
        self.assertTrue(report["accepted"])
        self.assertLess(np.linalg.norm(unloaded), np.linalg.norm(state)*.01)

    def test_relief_is_real_free_boundary_with_no_orphan_dofs(self):
        shell = NonlinearShell(self.source, JointConfig(hinge_gap_m=.0008),
                               ShellConfig(vertex_relief_fraction=.2))
        self.assertGreater(len(shell.mesh["free_boundary_edges"]), 0)
        self.assertTrue(set(shell.free_nodes).issubset(np.unique(shell.mesh["triangles"])))
        np.testing.assert_array_equal(shell.points[:28], self.source["points"])
        intact = surface_mesh(self.source, 1, .0008)
        p = intact["points"][intact["triangles"]]
        area = np.linalg.norm(np.cross(p[:, 1]-p[:, 0], p[:, 2]-p[:, 0]), axis=1).sum()/2
        self.assertLess(float(shell.area.sum()), area)
        self.assertEqual(len(intersection_pairs(shell.points, shell.mesh["triangles"])), 0)
        self.assertLess(abs(shell.value_gradient(np.zeros(shell.ndof))[0]), 1e-20)
        impact = ShellImpact(shell)
        self.assertAlmostEqual(float(impact.masses.sum()), .05, places=14)
        self.assertEqual(shell.twist_pairs.shape, (0, 2))

    def test_invalid_winch_or_mixed_loading_rejected(self):
        with self.assertRaises(ValueError):
            winch_pull(self.shell, "Compression", .1)
        with self.assertRaises(ValueError):
            self.shell.solve(np.ones(12), winch=winch_pull(self.shell, "Compression", .001))
        with self.assertRaises(ValueError):
            ShellConfig(vertex_relief_fraction=.3).validate()


if __name__ == "__main__":
    unittest.main()

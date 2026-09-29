"""Regression checks for the supplied joint STL provenance and split."""

import unittest
from pathlib import Path

import numpy as np

from exact_joint.stl_assets import ASSET_DIR, load_joint_stl_assets


class SuppliedStlAssetTests(unittest.TestCase):
    def test_files_are_byte_identical_to_the_recorded_manifest(self):
        assets = load_joint_stl_assets(ASSET_DIR)
        self.assertEqual(assets["rigid_layout"].triangle_count, 448)
        self.assertEqual(assets["assembled"].triangle_count, 608)
        self.assertEqual(assets["soft"].triangle_count, 160)
        self.assertEqual(assets["assembled_rigid_triangles"].shape, (448, 3, 3))
        self.assertEqual(assets["rigid_layout"].sha256,
                         "4024ffc36300685089293e2708a8b07dab6e63a89d850ec2f7166a84d2d0d5ef")
        self.assertEqual(assets["assembled"].sha256,
                         "182956b662f61680372b6ac5d3e5478bece34de4f71ba57bb06b280d6b4d7f86")
        self.assertEqual(assets["soft"].sha256,
                         "60a900edd721c97526ca914448153059881a84287d553b4b2f91904e1dc0f6da")

    def test_soft_triangles_are_an_exact_subset_of_assembled_reference(self):
        assets = load_joint_stl_assets(ASSET_DIR)
        combined = assets["assembled"].triangles
        soft = assets["soft"].triangles
        combined_keys = {
            tuple(sorted(tuple(float(v) for v in point) for point in np.round(triangle, 5)))
            for triangle in combined
        }
        soft_keys = {
            tuple(sorted(tuple(float(v) for v in point) for point in np.round(triangle, 5)))
            for triangle in soft
        }
        self.assertEqual(len(soft_keys), soft.shape[0])
        self.assertTrue(soft_keys.issubset(combined_keys))

    def test_layout_is_recorded_as_separate_manufacturing_reference(self):
        assets = load_joint_stl_assets(ASSET_DIR)
        layout_low, layout_high = assets["rigid_layout"].bounds
        assembled_low, assembled_high = assets["assembled"].bounds
        # The flat print-bed sheet is intentionally not used as the assembled
        # FEM overlay: its XY footprint is much larger than the joint.
        self.assertGreater(layout_high[0] - layout_low[0], 200.0)
        self.assertLess(assembled_high[0] - assembled_low[0], 60.0)


if __name__ == "__main__":
    unittest.main()

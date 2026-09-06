"""Source-identity, TET10 and cable-force FEM regression tests (NumPy/SciPy)."""

import dataclasses
from pathlib import Path
import unittest

import numpy as np

from exact_joint.elements import elevate,b_matrices,GAUSS
from exact_joint.geometry import JointConfig,load_source,laminate_mesh
from exact_joint.mechanics import CableFem,tension_pattern


SOURCE=Path(__file__).with_name("source_joint.json")


class ExactJointTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=JointConfig()
        cls.source=load_source(SOURCE)
        cls.model=CableFem(cls.source,cls.config)

    def test_exact_source_identity(self):
        source=self.source
        self.assertEqual(source["sha256"],"d578618a0f90342bd0093b4febe130dc0b07c18f1b223c4d6bf02c9f3d93ce35")
        np.testing.assert_allclose(np.ptp(source["points"],axis=0),[.03,.03,.0264],atol=1e-14)
        restored=(source["points"]+[0,0,-.0132]) @ source["rotation"] / source["source_units_to_m"]
        np.testing.assert_allclose(restored,source["source_points"],atol=1e-11)
        for i,p in enumerate(source["points"]):
            np.testing.assert_allclose(self.model.points[self.model.coarse_map[i]],p,atol=1e-13)

    def test_quadratic_patch(self):
        corners=np.array([[0,0,0],[.003,0,0],[.001,.004,0],[0,.001,.00008]])
        points,tets,_,_,_=elevate(corners,np.array([[0,1,2,3]]),[],[])
        b=b_matrices(points,tets)[0]
        displacement=points**2
        expected=np.zeros((4,6))
        expected[:,:3]=2*(GAUSS @ corners)
        np.testing.assert_allclose(np.einsum("gij,j->gi",b,displacement.ravel()),expected,atol=1e-12)
        rigid=np.cross([.01,.02,.03],points)+[.2,.3,.4]
        np.testing.assert_allclose(np.einsum("gij,j->gi",b,rigid.ravel()),0,atol=1e-10)

    def test_zero_load(self):
        result=self.model.solve(np.zeros(12),initial=np.zeros(6))
        np.testing.assert_array_equal(result["displacements"],0)
        self.assertEqual(result["stats"]["strain_energy_j"],0)

    def test_cable_equilibrium_and_rigid_roofs(self):
        for mode in ("Compression","Bend X+","Bend X-","Bend Y+","Bend Y-","Twist CW","Twist CCW"):
            result=self.model.solve(tension_pattern(mode,.1),initial=np.zeros(6))
            stats=result["stats"]
            self.assertTrue(stats["within_small_deformation_model"])
            self.assertLess(stats["cap_equilibrium_residual_n_equivalent"],1e-6)
            np.testing.assert_allclose(stats["force_balance_n"],0,atol=1e-7)
            for ids in (self.model.top_nodes,self.model.bottom_nodes):
                rest=self.model.points[ids]
                moved=result["points"][ids]
                np.testing.assert_allclose(np.linalg.norm(rest[:,None]-rest[None,:],axis=2),
                                           np.linalg.norm(moved[:,None]-moved[None,:],axis=2),atol=1e-12)
            residual=self.model.stiffness @ result["displacements"].ravel()
            self.assertLess(np.linalg.norm(residual[self.model.free]),1e-7)

    def test_twist_reversal_and_compression(self):
        plus=self.model.solve(tension_pattern("Twist CW",.1),initial=np.zeros(6))
        minus=self.model.solve(tension_pattern("Twist CCW",.1),initial=np.zeros(6))
        self.assertLess(plus["q"][5]*minus["q"][5],0)
        self.assertGreater(plus["q"][2],0)
        self.assertGreater(minus["q"][2],0)

    def test_material_and_force_scaling(self):
        other=CableFem(self.source,dataclasses.replace(self.config,pet_modulus_pa=7e9,pla_modulus_pa=4.4e9))
        first=self.model.solve(tension_pattern("Twist CW",.1),initial=np.zeros(6))
        second=other.solve(tension_pattern("Twist CW",.2),initial=np.zeros(6))
        np.testing.assert_allclose(first["q"],second["q"],atol=1e-9)

    def test_negative_tension_rejected(self):
        with self.assertRaises(ValueError):
            self.model.solve(-np.ones(12))


if __name__=="__main__":
    unittest.main()

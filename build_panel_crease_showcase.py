"""Build the scalable multi-axis showcase for the original panel joint.

The current geometry-coupled stage is the one-actuator physics/calibration
asset. This companion stage is intentionally a presentation asset: it keeps
the same 50 source surfaces, 28 shared vertices, 76 crease segments, and
batched rigid-facet visual runtime, while adding a serial X/Z/Y physical
mechanism so the joint's lateral bend, twist, and knee compression can be
shown without a Python optimizer running every frame.
"""

from __future__ import annotations

import argparse
import os
import runpy
from pathlib import Path

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics


BASE = runpy.run_path(
    str(Path(__file__).with_name("build_stable_panel_crease_leg.py")),
    run_name="stable_panel_crease_leg_builder_for_showcase",
)
COUPLED = runpy.run_path(
    str(Path(__file__).with_name("build_coupled_fold_panel_leg.py")),
    run_name="coupled_panel_crease_leg_builder_for_showcase",
)

_apply_body = BASE["_apply_body"]
_material = BASE["HELPERS"]["_material"]
_metadata = BASE["HELPERS"]["_metadata"]
_set_translate = BASE["HELPERS"]["_set_translate"]
KNEE_Z = BASE["KNEE_Z"]
PAPER_LIMITS_DEG = BASE["PAPER_LIMITS_DEG"]

TOP = "/World/PanelCreaseLeg/OriginalJointShell/TopShell"
BOTTOM = "/World/PanelCreaseLeg/OriginalJointShell/BottomShell"
PHYSICS = "/World/PanelCreaseLeg/Physics"


def _author_carrier(stage: Usd.Stage, path: str, mass: float, role: str):
    carrier = UsdGeom.Xform.Define(stage, path)
    _set_translate(UsdGeom.Xformable(carrier.GetPrim()), (0.0, 0.0, KNEE_Z))
    _apply_body(carrier.GetPrim(), mass, inertia=2.0e-6)
    _metadata(
        carrier.GetPrim(),
        {
            "role": role,
            "collision": "none; presentation carrier has mass/inertia only",
            "visual": "hidden; original panel shell remains the showcase visual",
        },
    )
    return carrier.GetPrim()


def _author_showcase_joint(
    stage: Usd.Stage,
    path: str,
    body0: Usd.Prim,
    body1: Usd.Prim,
    axis: str,
    limits: tuple[float, float],
    stiffness: float,
    damping: float,
    max_force: float,
    role: str,
):
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0.GetPath()])
    joint.CreateBody1Rel().SetTargets([body1.GetPath()])
    joint.CreateAxisAttr(axis)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    identity = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    joint.CreateLocalRot0Attr().Set(identity)
    joint.CreateLocalRot1Attr().Set(identity)
    joint.CreateLowerLimitAttr(float(limits[0]))
    joint.CreateUpperLimitAttr(float(limits[1]))
    physx_joint = PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())
    physx_joint.CreateJointFrictionAttr().Set(0.002)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(stiffness))
    drive.CreateDampingAttr(float(damping))
    drive.CreateMaxForceAttr(float(max_force))
    drive.CreateTargetPositionAttr(0.0)
    _metadata(
        joint.GetPrim(),
        {
            "role": role,
            "limitsDeg": list(limits),
            "showcaseDrive": "smooth force drive; targets are clamped to these USD stops",
        },
    )
    return joint.GetPrim()


def build(source_usd: Path, topology_json: Path, profile_json: Path, output_usd: Path) -> None:
    COUPLED["build"](source_usd, topology_json, profile_json, output_usd)
    stage = Usd.Stage.Open(str(output_usd))
    if stage is None:
        raise RuntimeError(f"could not reopen generated stage {output_usd}")

    old_knee = stage.GetPrimAtPath(f"{PHYSICS}/KneeActuator")
    if old_knee.IsValid():
        stage.RemovePrim(old_knee.GetPath())

    pitch = _author_carrier(
        stage,
        "/World/PanelCreaseLeg/OriginalJointShell/LateralBendCarrier",
        0.028,
        "showcase lateral bend carrier about X",
    )
    twist = _author_carrier(
        stage,
        "/World/PanelCreaseLeg/OriginalJointShell/TwistCarrier",
        0.028,
        "showcase torsional carrier about Z",
    )

    _author_showcase_joint(
        stage,
        f"{PHYSICS}/LateralBendJoint",
        stage.GetPrimAtPath(TOP),
        pitch,
        "X",
        (-22.0, 22.0),
        stiffness=8.0,
        damping=1.2,
        max_force=12.0,
        role="physical lateral bend stop for showcase envelope",
    )
    _author_showcase_joint(
        stage,
        f"{PHYSICS}/TwistJoint",
        pitch,
        twist,
        "Z",
        (-30.0, 30.0),
        stiffness=8.0,
        damping=1.2,
        max_force=12.0,
        role="physical torsional twist stop for showcase envelope",
    )
    _author_showcase_joint(
        stage,
        f"{PHYSICS}/MainKneeBendJoint",
        twist,
        stage.GetPrimAtPath(BOTTOM),
        "Y",
        PAPER_LIMITS_DEG["knee"],
        stiffness=12.0,
        damping=1.8,
        max_force=20.0,
        role="physical sagittal knee stop for showcase envelope",
    )

    controller = stage.GetPrimAtPath("/World/PanelCreaseLeg/Controller")
    controller.GetAttribute("jointNames").Set("Hip,LateralBend,Twist,KneeBend,Ankle")
    controller.GetAttribute("defaultTargetsDeg").Set("0,0,0,0,0")
    controller.GetAttribute("motionMode").Set(
        "showcase serial X lateral bend, Z twist, Y knee compression"
    )
    controller.GetAttribute("motionDriver").Set("supported_full_envelope")
    controller.GetAttribute("motionDemoEnabled").Set(False)
    controller.GetAttribute("motionDemoRateHz").Set(0.025)
    controller.GetAttribute("motionDemoAmplitudesDeg").Set("22,30,60")
    controller.GetAttribute("fullMotionRateHz").Set(0.030)
    controller.GetAttribute("fullMotionPattern").Set("combined")
    controller.GetAttribute("fullMotionCombinedScale").Set(0.75)
    controller.GetAttribute("fullMotionShowcaseScale").Set(0.75)
    controller.GetAttribute("visualControllerMode").Set("kinematic_facet")
    controller.GetAttribute("visualControllerScript").Set(
        "baked_panel_crease_controller.py"
    )
    controller.GetAttribute("kinematicFacetPasses").Set(2)
    if controller.GetAttribute("kinematicMetricPeriod").IsValid():
        controller.GetAttribute("kinematicMetricPeriod").Set(30)
    controller.GetAttribute("foldCommandMaxDeg").Set(60.0)
    controller.GetAttribute("geometryFoldEnvelopeDeg").Set(60.0)

    root = stage.GetPrimAtPath("/World/PanelCreaseLeg")
    _metadata(
        root,
        {
            "assetType": "original_panel_crease_joint_showcase",
            "kneeMechanism": "serial physical X lateral bend, Z twist, Y knee fold",
            "physicalConstraintCount": 3,
            "showcaseLimitsDeg": {
                "lateral": [-22.0, 22.0],
                "twist": [-30.0, 30.0],
                "fold": [-1.5, 127.0],
            },
            "geometryValidFoldLimitDeg": 60.0,
            "geometryValidFoldLimitNote": "60 degrees is the conservative video/geometry-safe showcase envelope; the 127 degree paper stop remains physical metadata until FEA/metrology confirms the shell response",
            "motionDemo": "bounded coupled fold plus lateral bend and twist with smooth quintic transitions",
            "runtimeVisualModel": "batched deterministic rigid-facet projection over the original 50 surfaces, 28 shared vertices, and 76 source crease edges; no per-frame nonlinear optimizer",
            "baselineStage": "panel_crease_leg_constraints_v8_geometry_fold.usd",
        },
    )

    physics_scene = stage.GetPrimAtPath(f"{PHYSICS}/PhysicsScene")
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene)
    physx_scene.CreateTimeStepsPerSecondAttr().Set(120)
    physx_scene.CreateMaxPositionIterationCountAttr().Set(8)
    physx_scene.CreateMaxVelocityIterationCountAttr().Set(2)
    physx_scene.CreateMinPositionIterationCountAttr().Set(1)
    physx_scene.CreateMinVelocityIterationCountAttr().Set(0)
    physx_scene.CreateEnableStabilizationAttr().Set(True)
    physx_scene.CreateSolverTypeAttr().Set("TGS")
    physics_scene.CreateAttribute(
        "newton:timeStepsPerSecond", Sdf.ValueTypeNames.Int
    ).Set(120)

    stage.GetRootLayer().Export(str(output_usd))
    print(f"Wrote {output_usd}")
    print("Showcase mechanism: physical X lateral bend, Z twist, Y knee fold")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=os.environ.get("ORIGINAL_JOINT_USD"))
    parser.add_argument("--topology", type=Path, default=os.environ.get("ORIGINAL_JOINT_TOPOLOGY_JSON"))
    parser.add_argument("--profile", type=Path, default=os.environ.get("PANEL_CREASE_COUPLED_PROFILE"))
    parser.add_argument("--output", type=Path, default=os.environ.get("PANEL_CREASE_SHOWCASE_OUTPUT"))
    args = parser.parse_args()
    if args.source is None or args.topology is None or args.profile is None or args.output is None:
        parser.error("provide --source/--topology/--profile/--output or set project environment variables")
    build(args.source.resolve(), args.topology.resolve(), args.profile.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()

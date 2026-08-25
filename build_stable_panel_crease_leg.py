"""Build a stable reduced-order knee driven by the original panel geometry.

The earlier per-triangle implementation made every one of the 76 source
edges a separate closed-loop rigid-body constraint.  That is not a stable
PhysX representation of this thin origami shell: the solver has to satisfy
many redundant loops while a motor is moving the two end plates, which causes
edge separation and chatter.

This builder keeps the original source vertices, triangular faces, colours,
and crease lines as one continuous visual shell.  The physical mechanism is a
single revolute knee between a top and bottom interface body.  The companion
controller deforms the visual shell from those two physical body poses so the
source panel boundaries remain coincident while the joint moves.
"""

import argparse
import json
import math
import os
import runpy
from pathlib import Path

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt


HELPERS = runpy.run_path(
    str(Path(__file__).with_name("build_leg_with_panel_crease_joint.py")),
    run_name="panel_crease_helpers",
)

_metadata = HELPERS["_metadata"]
_material = HELPERS["_material"]
_set_translate = HELPERS["_set_translate"]
_set_scale = HELPERS["_set_scale"]
_capsule = HELPERS["_capsule"]
_cube = HELPERS["_cube"]
_author_mesh = HELPERS["_author_mesh"]
_apply_body = HELPERS["_apply_body"]
_author_actuator_joint = HELPERS["_author_actuator_joint"]
_load_topology = HELPERS["_load_topology"]
_safe_name = HELPERS["_safe_name"]

HIP_Z = HELPERS["HIP_Z"]
KNEE_Z = HELPERS["KNEE_Z"]
ANKLE_Z = HELPERS["ANKLE_Z"]
SOURCE_SCALE = HELPERS["SOURCE_SCALE"]
ROOF_HALF_SPAN = HELPERS["ROOF_HALF_SPAN"]
ROOF_PLATE_THICKNESS = 0.0024
PAPER_LIMITS_DEG = HELPERS["PAPER_LIMITS_DEG"]


def _author_panel_group_mesh(stage: Usd.Stage, path: str, panels: list[dict], topology: dict, material):
    """Author one continuous mesh for one visual panel/material group."""

    vertex_ids: list[str] = []
    indices: list[int] = []
    counts: list[int] = []
    index_by_vertex: dict[str, int] = {}
    for panel in panels:
        counts.append(len(panel["vertexIds"]))
        for vertex_id in panel["vertexIds"]:
            if vertex_id not in index_by_vertex:
                index_by_vertex[vertex_id] = len(vertex_ids)
                vertex_ids.append(vertex_id)
            indices.append(index_by_vertex[vertex_id])

    points = [topology["verticesJoint"][vertex_id] for vertex_id in vertex_ids]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(indices))
    mesh.CreateDoubleSidedAttr().Set(True)
    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material)
    mesh.GetPrim().SetCustomDataByKey("sourceVertexIds", json.dumps(vertex_ids, separators=(",", ":")))
    mesh.GetPrim().SetCustomDataByKey(
        "sourcePanelNames",
        json.dumps([panel["name"] for panel in panels], separators=(",", ":")),
    )
    return mesh


def _author_baked_crease_mesh(
    stage: Usd.Stage,
    path: str,
    topology: dict,
    material,
    radius: float = 0.00026,
    sides: int = 6,
):
    """Author one render mesh containing every source crease segment.

    BasisCurves are useful as a topology registry but are not consistently
    rendered by every Kit viewport configuration. A single welded-free prism
    mesh keeps the original 76 crease segments visible while requiring only
    one point-array update at runtime instead of 76 cylinder transform-stack
    edits per frame.
    """

    def cross(first, second):
        return (
            first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0],
        )

    def scale(vector, amount):
        return tuple(float(value) * amount for value in vector)

    def add(first, second):
        return tuple(float(first[index]) + float(second[index]) for index in range(3))

    def unit(vector):
        length = math.sqrt(sum(float(value) * float(value) for value in vector))
        if length < 1.0e-12:
            return (1.0, 0.0, 0.0)
        return scale(vector, 1.0 / length)

    points = []
    counts = []
    indices = []
    edge_keys = topology["referenceEdgeKeys"]
    for edge_index, edge in enumerate(edge_keys):
        start = tuple(float(value) for value in topology["verticesJoint"][str(edge[0])])
        end = tuple(float(value) for value in topology["verticesJoint"][str(edge[1])])
        direction = unit(tuple(end[index] - start[index] for index in range(3)))
        reference = (0.0, 0.0, 1.0)
        if abs(sum(direction[index] * reference[index] for index in range(3))) > 0.9:
            reference = (0.0, 1.0, 0.0)
        first_normal = unit(cross(direction, reference))
        second_normal = unit(cross(direction, first_normal))
        base = edge_index * 2 * sides
        for endpoint in (start, end):
            for side in range(sides):
                angle = 2.0 * math.pi * side / sides
                radial = add(
                    scale(first_normal, math.cos(angle) * radius),
                    scale(second_normal, math.sin(angle) * radius),
                )
                points.append(add(endpoint, radial))
        for side in range(sides):
            counts.append(4)
            indices.extend(
                (
                    base + side,
                    base + (side + 1) % sides,
                    base + sides + (side + 1) % sides,
                    base + sides + side,
                )
            )
        counts.extend((sides, sides))
        indices.extend(base + side for side in range(sides - 1, -1, -1))
        indices.extend(base + sides + side for side in range(sides))

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(indices))
    mesh.CreateDoubleSidedAttr().Set(True)
    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material)
    mesh.GetPrim().SetCustomDataByKey("referenceCreaseLineCount", len(edge_keys))
    mesh.GetPrim().SetCustomDataByKey(
        "referenceCreaseEdgeKeys",
        json.dumps(edge_keys, separators=(",", ":")),
    )
    mesh.GetPrim().SetCustomDataByKey("creasePrismSides", sides)
    mesh.GetPrim().SetCustomDataByKey("creasePrismRadiusM", radius)
    return mesh


def _author_shell_body(stage: Usd.Stage, path: str, mass: float, material, roof_panel: dict):
    body = UsdGeom.Xform.Define(stage, path)
    _set_translate(UsdGeom.Xformable(body.GetPrim()), (0.0, 0.0, KNEE_Z))
    _apply_body(body.GetPrim(), mass, inertia=1.5e-5)
    _author_mesh(stage, f"{path}/SourceRoofFace", roof_panel["points"], material)
    roof_z = float(roof_panel["points"][0][2])
    plate = _cube(
        stage,
        f"{path}/InterfacePlate",
        (0.0, 0.0, roof_z),
        (2.0 * (ROOF_HALF_SPAN + 0.0007), 2.0 * (ROOF_HALF_SPAN + 0.0007), ROOF_PLATE_THICKNESS),
        material,
        collision=False,
    )
    _metadata(
        plate.GetPrim(),
        {
            "role": "continuous physical interface plate",
            "sourceRoof": roof_panel["name"],
            "panelBoundaryConnection": "shared source vertices follow this body pose",
        },
    )
    _metadata(
        body.GetPrim(),
        {
            "role": "stable physical half of original panel crease joint",
            "sourceRoof": roof_panel["name"],
            "physicalModel": "one rigid interface body plus continuous driven source-panel visual shell",
            "collision": "disabled on zero-thickness visual shell",
        },
    )
    return body


def build(source_usd: Path, topology_json: Path, output_usd: Path) -> None:
    topology = _load_topology(source_usd, topology_json)
    stage = Usd.Stage.CreateNew(str(output_usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    root = UsdGeom.Xform.Define(stage, "/World/PanelCreaseLeg")
    _metadata(
        root.GetPrim(),
        {
            "assetType": "stable_original_panel_crease_knee",
            "sourceAsset": source_usd.name,
            "sourceTopology": topology_json.name,
            "sourcePanelCount": len(topology["panels"]),
            "sourceFoldLineCount": len(topology["lines"]),
            "referenceCreaseSet": "canonical unique shared edges from input_improved.json",
            "referenceCreaseLineCount": len(topology["referenceEdgeKeys"]),
            "foldMechanism": "continuous source-panel visual shell driven by two physical interface bodies",
            "kneeMechanism": "single physical revolute actuator between the two original flat interfaces",
            "flatInterfaceOrientation": HELPERS["SOURCE_AXIS_ROTATION"],
            "jointOrder": ["Hip", "Knee", "Ankle"],
            "jointCentersMm": {"hip": 136.8, "knee": 88.3, "ankle": 39.8},
            "jointSpacingMm": 48.5,
            "paperKneeLimitsDeg": list(PAPER_LIMITS_DEG["knee"]),
            "policyRateHz": 100,
            "lowLevelRateHz": 120,
        },
    )

    body_mat = _material(stage, "/World/Materials/Body", (0.07, 0.08, 0.10), metallic=0.7, roughness=0.3)
    link_mat = _material(stage, "/World/Materials/Links", (0.12, 0.37, 0.60), metallic=0.45, roughness=0.35)
    c_panel_mat = _material(stage, "/World/Materials/JointPanels", (0.86, 0.22, 0.07), metallic=0.55, roughness=0.32)
    s_panel_mat = _material(stage, "/World/Materials/JointConnectorPanels", (0.95, 0.52, 0.08), metallic=0.6, roughness=0.28)
    roof_mat = _material(stage, "/World/Materials/JointFlatInterfaces", (0.95, 0.78, 0.10), metallic=0.75, roughness=0.25)
    crease_mat = _material(stage, "/World/Materials/CreaseLines", (0.035, 0.025, 0.018), metallic=0.1, roughness=0.5)
    foot_mat = _material(stage, "/World/Materials/Foot", (0.10, 0.12, 0.15), metallic=0.2, roughness=0.6)
    floor_mat = _material(stage, "/World/Materials/Floor", (0.04, 0.05, 0.07), roughness=0.9)

    roofs = {panel["name"]: panel for panel in topology["panels"] if panel["kind"] == "roof"}
    # The source flat interfaces are at y=+/-110 mm, while the square plate
    # half-span is 125 mm. Use the actual roof-plane offset for rigid link
    # anchors; the old half-span anchor left a 1.8 mm plate/shank mismatch.
    interface_offset = abs(float(roofs["Roof 1"]["points"][0][2]))
    if interface_offset <= 1.0e-9:
        raise RuntimeError("source roof interface offset must be non-zero")
    root.GetPrim().SetCustomDataByKey("interfaceOffsetM", interface_offset)

    body = UsdGeom.Xform.Define(stage, "/World/PanelCreaseLeg/Body")
    _set_translate(UsdGeom.Xformable(body.GetPrim()), (0.0, 0.0, HIP_Z))
    _apply_body(body.GetPrim(), 0.25, inertia=2.0e-4)
    _cube(stage, "/World/PanelCreaseLeg/Body/Mount", (0.0, 0.0, 0.012), (0.035, 0.03, 0.012), body_mat)

    thigh = UsdGeom.Xform.Define(stage, "/World/PanelCreaseLeg/Thigh")
    _set_translate(UsdGeom.Xformable(thigh.GetPrim()), (0.0, 0.0, HIP_Z))
    _apply_body(thigh.GetPrim(), 0.1074, inertia=1.5e-4)
    upper_face_z = KNEE_Z + interface_offset
    thigh_length = HIP_Z - upper_face_z
    _capsule(stage, "/World/PanelCreaseLeg/Thigh/Link", (0.0, 0.0, -thigh_length / 2.0), 0.008, thigh_length, link_mat)

    shank = UsdGeom.Xform.Define(stage, "/World/PanelCreaseLeg/Shank")
    lower_face_z = KNEE_Z - interface_offset
    _set_translate(UsdGeom.Xformable(shank.GetPrim()), (0.0, 0.0, lower_face_z))
    _apply_body(shank.GetPrim(), 0.1070, inertia=1.5e-4)
    shank_length = lower_face_z - ANKLE_Z
    _capsule(stage, "/World/PanelCreaseLeg/Shank/Link", (0.0, 0.0, -shank_length / 2.0), 0.0075, shank_length, link_mat)

    foot = UsdGeom.Xform.Define(stage, "/World/PanelCreaseLeg/Foot")
    _set_translate(UsdGeom.Xformable(foot.GetPrim()), (0.0, 0.0, ANKLE_Z))
    _apply_body(foot.GetPrim(), 0.0676, inertia=8.0e-5)
    _cube(stage, "/World/PanelCreaseLeg/Foot/Sole", (0.014, 0.0, -0.006), (0.030, 0.016, 0.006), foot_mat, collision=True)

    shell_root = UsdGeom.Scope.Define(stage, "/World/PanelCreaseLeg/OriginalJointShell")
    top_shell = _author_shell_body(stage, "/World/PanelCreaseLeg/OriginalJointShell/TopShell", 0.030, roof_mat, roofs["Roof 1"])
    bottom_shell = _author_shell_body(stage, "/World/PanelCreaseLeg/OriginalJointShell/BottomShell", 0.030, roof_mat, roofs["Roof 3"])

    visual_root = UsdGeom.Scope.Define(stage, "/World/PanelCreaseLeg/OriginalJointVisual")
    _metadata(
        visual_root.GetPrim(),
        {
            "role": "continuous source-panel shell visual",
            "controller": "baked_panel_crease_controller.py",
            "sourceTopology": topology_json.name,
            "deformation": "top/bottom roof vertices follow physical bodies; central ring is blended",
        },
    )
    visual_root.GetPrim().SetCustomDataByKey(
        "sourceVertexPositionsM",
        json.dumps(
            {
                vertex_id: [float(value) for value in position]
                for vertex_id, position in topology["vertices"].items()
            },
            separators=(",", ":"),
        ),
    )
    visual_root.GetPrim().SetCustomDataByKey(
        "referenceCreaseEdgeKeys",
        json.dumps(topology["referenceEdgeKeys"], separators=(",", ":")),
    )
    visual_root.GetPrim().SetCustomDataByKey(
        "referenceCreaseLineNames",
        json.dumps(topology["referenceLineNames"], separators=(",", ":")),
    )
    upper_panels = [
        panel
        for panel in topology["panels"]
        if panel["kind"] != "roof" and sum(float(topology["vertices"][vertex_id][1]) for vertex_id in panel["vertexIds"]) > 0.0
    ]
    lower_panels = [
        panel
        for panel in topology["panels"]
        if panel["kind"] != "roof" and sum(float(topology["vertices"][vertex_id][1]) for vertex_id in panel["vertexIds"]) < 0.0
    ]
    groups = (
        ("UpperC", [panel for panel in upper_panels if panel["kind"] == "c"], c_panel_mat),
        ("UpperS", [panel for panel in upper_panels if panel["kind"] == "s"], s_panel_mat),
        ("LowerC", [panel for panel in lower_panels if panel["kind"] == "c"], c_panel_mat),
        ("LowerS", [panel for panel in lower_panels if panel["kind"] == "s"], s_panel_mat),
    )
    for group_name, panels, material in groups:
        _author_panel_group_mesh(stage, f"/World/PanelCreaseLeg/OriginalJointVisual/{group_name}", panels, topology, material)

    # The crease prism mesh is the sole render representation of the source
    # fold lines.  Do not author the old 76-cylinder fallback layer: it has no
    # physical role and, if a legacy controller discovers it, its local-origin
    # transforms appear as a detached wireframe below the joint.
    _author_baked_crease_mesh(
        stage,
        "/World/PanelCreaseLeg/OriginalJointVisual/BakedCreaseMesh",
        topology,
        crease_mat,
    )

    physics_root = UsdGeom.Scope.Define(stage, "/World/PanelCreaseLeg/Physics")
    body_mount = UsdPhysics.FixedJoint.Define(stage, "/World/PanelCreaseLeg/Physics/BodyMount")
    body_mount.CreateBody1Rel().SetTargets([body.GetPath()])
    body_mount.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, HIP_Z))
    body_mount.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    _author_actuator_joint(
        stage,
        "/World/PanelCreaseLeg/Physics/HipJoint",
        body.GetPrim(),
        thigh.GetPrim(),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        "Y",
        Gf.Vec3f(0.0, 1.0, 0.0),
        PAPER_LIMITS_DEG["hip"],
        0.0,
    )

    thigh_roof = UsdPhysics.FixedJoint.Define(stage, "/World/PanelCreaseLeg/Physics/ThighToTopShell")
    thigh_roof.CreateBody0Rel().SetTargets([thigh.GetPath()])
    thigh_roof.CreateBody1Rel().SetTargets([top_shell.GetPath()])
    thigh_roof.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -(HIP_Z - upper_face_z)))
    thigh_roof.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, interface_offset))
    _metadata(thigh_roof.GetPrim(), {"role": "upper flat interface to stable original shell half", "sourcePanel": "Roof 1"})

    knee = _author_actuator_joint(
        stage,
        "/World/PanelCreaseLeg/Physics/KneeActuator",
        top_shell,
        bottom_shell,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        "Y",
        Gf.Vec3f(0.0, 1.0, 0.0),
        PAPER_LIMITS_DEG["knee"],
        45.0,
    )
    knee.GetPrim().GetAttribute("drive:angular:physics:damping").Set(0.12)
    _metadata(
        knee.GetPrim(),
        {
            "role": "single physical knee actuator for original source shell",
            "sourceGeometry": source_usd.name,
            "foldNetwork": "visual source topology: 50 panels and 76 lines",
            "physicalConstraintCount": 1,
            "stabilityNote": "no redundant closed panel hinge loop",
        },
    )

    bottom_shank = UsdPhysics.FixedJoint.Define(stage, "/World/PanelCreaseLeg/Physics/BottomShellToShank")
    bottom_shank.CreateBody0Rel().SetTargets([bottom_shell.GetPath()])
    bottom_shank.CreateBody1Rel().SetTargets([shank.GetPath()])
    bottom_shank.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -interface_offset))
    bottom_shank.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    _metadata(bottom_shank.GetPrim(), {"role": "lower flat interface to stable original shell half", "sourcePanel": "Roof 3"})

    _author_actuator_joint(
        stage,
        "/World/PanelCreaseLeg/Physics/AnkleJoint",
        shank.GetPrim(),
        foot.GetPrim(),
        (0.0, 0.0, -shank_length),
        (0.0, 0.0, 0.0),
        "Y",
        Gf.Vec3f(0.0, 1.0, 0.0),
        PAPER_LIMITS_DEG["ankle"],
        0.0,
    )

    controller = stage.DefinePrim("/World/PanelCreaseLeg/Controller", "Scope")
    controller.CreateAttribute("policyRateHz", Sdf.ValueTypeNames.Int).Set(100)
    controller.CreateAttribute("lowLevelRateHz", Sdf.ValueTypeNames.Int).Set(120)
    controller.CreateAttribute("jointNames", Sdf.ValueTypeNames.String).Set("Hip,Knee,Ankle")
    controller.CreateAttribute("defaultTargetsDeg", Sdf.ValueTypeNames.String).Set("0,45,0")
    controller.CreateAttribute("commandType", Sdf.ValueTypeNames.Token).Set("target_joint_position")
    controller.CreateAttribute("originalJointModel", Sdf.ValueTypeNames.String).Set("50 source panels + 76 source fold lines, continuous visual shell")
    controller.CreateAttribute("referenceCreaseLineCount", Sdf.ValueTypeNames.Int).Set(
        len(topology["referenceEdgeKeys"])
    )
    controller.CreateAttribute("visualControllerScript", Sdf.ValueTypeNames.String).Set("stable_panel_crease_controller.py")

    ground = UsdGeom.Cube.Define(stage, "/World/PanelCreaseLeg/ReferenceGround")
    ground.CreateSizeAttr(1.0)
    _set_translate(UsdGeom.Xformable(ground.GetPrim()), (0.0, 0.0, -0.002))
    _set_scale(UsdGeom.Xformable(ground.GetPrim()), (0.20, 0.20, 0.002))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(floor_mat)

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PanelCreaseLeg/Physics/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
    # 1000 solver steps/sec was useful while diagnosing the compliant shell,
    # but it makes every copy of this joint unnecessarily expensive. The
    # showcase uses a bounded 120 Hz physics loop with explicit iteration
    # limits; the offline solver remains available for calibration.
    physx_scene.CreateTimeStepsPerSecondAttr().Set(120)
    physx_scene.CreateMaxPositionIterationCountAttr().Set(8)
    physx_scene.CreateMaxVelocityIterationCountAttr().Set(2)
    physx_scene.CreateMinPositionIterationCountAttr().Set(1)
    physx_scene.CreateMinVelocityIterationCountAttr().Set(0)
    physics_scene.GetPrim().CreateAttribute(
        "newton:timeStepsPerSecond", Sdf.ValueTypeNames.Int
    ).Set(120)
    physx_scene.CreateEnableStabilizationAttr().Set(True)
    physx_scene.CreateSolverTypeAttr().Set("TGS")

    dome = UsdLux.DomeLight.Define(stage, "/World/PanelCreaseLeg/Lights/DomeLight")
    dome.CreateIntensityAttr().Set(500.0)
    distant = UsdLux.DistantLight.Define(stage, "/World/PanelCreaseLeg/Lights/DistantLight")
    distant.CreateIntensityAttr().Set(1200.0)

    stage.SetDefaultPrim(world.GetPrim())
    output_usd.parent.mkdir(parents=True, exist_ok=True)
    stage.GetRootLayer().Export(str(output_usd))
    print(f"Wrote {output_usd}")
    print("Stable physical model: 2 interface bodies, 1 knee revolute, continuous 50-panel visual shell")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=os.environ.get("ORIGINAL_JOINT_USD"))
    parser.add_argument("--topology", type=Path, default=os.environ.get("ORIGINAL_JOINT_TOPOLOGY_JSON"))
    parser.add_argument("--output", type=Path, default=os.environ.get("PANEL_CREASE_LEG_OUTPUT"))
    args = parser.parse_args()
    if args.source is None or args.topology is None or args.output is None:
        parser.error("provide --source/--topology/--output or set ORIGINAL_JOINT_USD/ORIGINAL_JOINT_TOPOLOGY_JSON/PANEL_CREASE_LEG_OUTPUT")
    build(args.source.resolve(), args.topology.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()

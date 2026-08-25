"""Build a paper-guided leg whose original joint folds along its panel creases.

The source asset is not a two-piece hinge.  It is a closed, triangulated shell:
the 50 source panels meet along the 76 named edges in ``input_improved.json``.
This builder keeps every source triangle as a visual rigid panel and connects
each pair of neighbouring panels with a revolute hinge whose axis is the
corresponding shared edge.  The two source roof faces are rigid end plates.
The knee actuator drives those end plates around the paper knee axis; the
panel hinges then carry the motion through the actual fold network.

This is a rigid-panel/origami approximation, not a finite-element material
model.  The source file does not contain thickness, elastic modulus, or
material data, so the physical compression is represented by real PhysX
revolute creases with the source topology and passive hinge friction.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
from pathlib import Path

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt


HIP_Z = 0.1368
KNEE_Z = 0.0883
ANKLE_Z = 0.0398
SOURCE_SCALE = 0.12
SOURCE_AXIS_ROTATION = "+90deg about X: source +Y roof becomes leg +Z roof"
ROOF_HALF_SPAN = 0.125 * SOURCE_SCALE
# The source roof faces are zero-thickness quads.  These dimensions give the
# two interfaces a small manufactured thickness and an overlap at every
# shared edge so the side panels cannot appear to float away from the plates.
ROOF_PLATE_THICKNESS = 0.0030
ROOF_PLATE_OVERHANG = 0.0012
ROOF_PERIMETER_RADIUS = 0.00150
ROOF_SKIRT_DEPTH = 0.0040
ROOF_SKIRT_RADIUS = 0.00150
SIDE_BOUNDARY_RADIUS = 0.00120
ROOF_CORNER_PAD_SIZE = 0.0024

SEA_STIFFNESS = 0.012 * 180.0 / math.pi
SEA_TRAVEL_DEG = 40.0
SEA_TORQUE_LIMIT = 0.6
# The source asset gives the fold topology but no sheet thickness or crease
# spring law.  A small distributed return spring prevents the closed panel
# shell from choosing an arbitrary collapsed equilibrium while still letting
# the knee actuator fold it.  This is deliberately much softer than the
# active knee drive.
CREASE_STIFFNESS = 0.0005
CREASE_DAMPING = 0.0005
PAPER_LIMITS_DEG = {
    "hip": (-93.0, 37.0),
    "knee": (-1.5, 127.0),
    "ankle": (-82.0, 37.0),
}


def _metadata(prim: Usd.Prim, values: dict) -> None:
    for key, value in values.items():
        if isinstance(value, (dict, list, tuple)):
            value = json.dumps(value, separators=(",", ":"))
        prim.SetCustomDataByKey(key, value)


def _op(xformable: UsdGeom.Xformable, op_type):
    return next((op for op in xformable.GetOrderedXformOps() if op.GetOpType() == op_type), None)


def _set_translate(xformable: UsdGeom.Xformable, value) -> None:
    op = _op(xformable, UsdGeom.XformOp.TypeTranslate)
    if op is None:
        op = xformable.AddTranslateOp()
    op.Set(Gf.Vec3d(*value))


def _set_scale(xformable: UsdGeom.Xformable, value) -> None:
    op = _op(xformable, UsdGeom.XformOp.TypeScale)
    if op is None:
        op = xformable.AddScaleOp()
    op.Set(Gf.Vec3f(*value))


def _set_orient(xformable: UsdGeom.Xformable, quat: Gf.Quatf) -> None:
    op = next((op for op in xformable.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeOrient), None)
    if op is None:
        op = xformable.AddOrientOp()
    op.Set(quat)


def _material(stage: Usd.Stage, path: str, color, metallic=0.0, roughness=0.45):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _apply_body(prim: Usd.Prim, mass_kg: float, inertia: float = 1.0e-5) -> None:
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass = UsdPhysics.MassAPI.Apply(prim)
    mass.CreateMassAttr().Set(mass_kg)
    mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(inertia, inertia, inertia))
    mass.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    physx_body.CreateSolverPositionIterationCountAttr().Set(8)
    physx_body.CreateSolverVelocityIterationCountAttr().Set(2)


def _capsule(stage: Usd.Stage, path: str, translation, radius: float, height: float, material):
    capsule = UsdGeom.Capsule.Define(stage, path)
    capsule.CreateRadiusAttr(radius)
    capsule.CreateHeightAttr(height)
    capsule.CreateAxisAttr(UsdGeom.Tokens.z)
    _set_translate(UsdGeom.Xformable(capsule.GetPrim()), translation)
    UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
    UsdShade.MaterialBindingAPI(capsule.GetPrim()).Bind(material)
    return capsule


def _cube(stage: Usd.Stage, path: str, translation, scale, material, collision=False):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    _set_translate(UsdGeom.Xformable(cube.GetPrim()), translation)
    _set_scale(UsdGeom.Xformable(cube.GetPrim()), scale)
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if material is not None:
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)
    return cube


def _safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]", "_", value)


def _source_to_joint(position, scale=SOURCE_SCALE) -> Gf.Vec3f:
    """Convert the source frame to the leg frame.

    Source long axis is Y.  A +90 degree X rotation makes the two source flat
    faces point along leg +Z/-Z, while the knee bend axis remains world Y.
    """

    x, y, z = float(position[0]), float(position[1]), float(position[2])
    return Gf.Vec3f(x * scale, -z * scale, y * scale)


def _unit(vector: Gf.Vec3f) -> Gf.Vec3f:
    length = vector.GetLength()
    if length < 1.0e-9:
        raise ValueError("cannot normalize a zero-length crease edge")
    return vector / length


def _quat_align(from_axis: Gf.Vec3f, to_axis: Gf.Vec3f) -> Gf.Quatf:
    rotation = Gf.Rotation(Gf.Vec3d(*from_axis), Gf.Vec3d(*_unit(to_axis)))
    quat = rotation.GetQuat()
    return Gf.Quatf(quat.GetReal(), *quat.GetImaginary())


def _edge_key(a: str, b: str) -> tuple[str, str]:
    return tuple(sorted((a, b)))


def _load_topology(source_usd: Path, topology_json: Path):
    """Load the source mesh and the stable panel/edge names from the improved JSON."""

    source_stage = Usd.Stage.Open(str(source_usd))
    panel_mesh_prim = source_stage.GetPrimAtPath("/World/InputEnvironment/PanelMesh")
    if not panel_mesh_prim or not panel_mesh_prim.IsValid():
        raise RuntimeError("source USD is missing /World/InputEnvironment/PanelMesh")
    source_mesh = UsdGeom.Mesh(panel_mesh_prim)
    source_points = list(source_mesh.GetPointsAttr().Get())
    source_counts = list(source_mesh.GetFaceVertexCountsAttr().Get())
    source_indices = list(source_mesh.GetFaceVertexIndicesAttr().Get())

    with topology_json.open("r", encoding="utf-8") as handle:
        topology = json.load(handle)

    vertices = {
        item["id"]: Gf.Vec3f(*item["positionM"])
        for item in topology["geometry"]["vertices"]
    }
    panels = []
    for item in topology["panels"]:
        panels.append(
            {
                "name": item["name"],
                "vertexIds": list(item["vertexIds"]),
                "points": [_source_to_joint(vertices[vertex_id]) for vertex_id in item["vertexIds"]],
                "kind": "roof" if item["name"].lower().startswith("roof") else item["name"][0],
            }
        )

    lines = []
    line_edges_seen: set[tuple[str, str]] = set()
    for item in topology["lines"]:
        vertex_ids = list(item["vertexIds"])
        if len(vertex_ids) != 2:
            continue
        edge = _edge_key(vertex_ids[0], vertex_ids[1])
        if edge in line_edges_seen:
            raise RuntimeError(
                f"topology contains a duplicate named fold line for edge {edge}"
            )
        line_edges_seen.add(edge)
        lines.append(
            {
                "name": item["name"],
                "vertexIds": vertex_ids,
                "edgeKey": edge,
                "a": _source_to_joint(vertices[vertex_ids[0]]),
                "b": _source_to_joint(vertices[vertex_ids[1]]),
            }
        )

    # Validate that the USD source and the improved topology really have the
    # same 28-point/50-face source structure before authoring a new mechanism.
    if len(source_points) != len(vertices):
        raise RuntimeError(f"source point count {len(source_points)} != topology vertex count {len(vertices)}")
    if len(panels) != len(source_counts):
        raise RuntimeError(f"source face count {len(source_counts)} != topology panel count {len(panels)}")
    if len(lines) != 76:
        raise RuntimeError(f"expected 76 source crease lines, found {len(lines)}")

    panel_by_name = {panel["name"]: panel for panel in panels}
    edge_owners: dict[tuple[str, str], list[str]] = {}
    for panel in panels:
        vertex_ids = panel["vertexIds"]
        for index, start in enumerate(vertex_ids):
            end = vertex_ids[(index + 1) % len(vertex_ids)]
            edge_owners.setdefault(_edge_key(start, end), []).append(panel["name"])
    if len(edge_owners) != len(lines):
        raise RuntimeError(f"topology has {len(edge_owners)} unique panel edges but {len(lines)} named lines")
    if any(len(owners) != 2 for owners in edge_owners.values()):
        raise RuntimeError("source panel shell is not a closed two-panel-per-edge manifold")

    line_by_edge = {_edge_key(line["vertexIds"][0], line["vertexIds"][1]): line for line in lines}
    if set(line_by_edge) != set(edge_owners):
        raise RuntimeError("source lines do not cover the panel adjacency graph")

    return {
        "vertices": vertices,
        "verticesJoint": {
            vertex_id: _source_to_joint(position)
            for vertex_id, position in vertices.items()
        },
        "panels": panels,
        "lines": lines,
        "referenceEdgeKeys": sorted(line_edges_seen),
        "referenceLineNames": [line["name"] for line in lines],
        "panel_by_name": panel_by_name,
        "edge_owners": edge_owners,
        "line_by_edge": line_by_edge,
    }


def _author_mesh(stage: Usd.Stage, path: str, points, material, face_count=None):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    count = len(points) if face_count is None else face_count
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray([count]))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(list(range(count))))
    mesh.CreateDoubleSidedAttr().Set(True)
    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material)
    return mesh


def _author_mass_proxy(stage: Usd.Stage, body_path: str, centroid: Gf.Vec3f) -> None:
    """Give each panel a tiny disabled shape so PhysX has a concrete body shape.

    The original panels are zero-thickness visual surfaces.  They are not used
    as colliders because convexifying all 50 coplanar triangles would make the
    closed shell self-collide.  Mass and inertia are authored on the body;
    this disabled proxy prevents the panel visual from becoming a collider.
    """

    proxy = _cube(stage, f"{body_path}/InertiaProxy", centroid, (0.0002, 0.0002, 0.0002), None)
    imageable = UsdGeom.Imageable(proxy.GetPrim())
    imageable.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
    collision = UsdPhysics.CollisionAPI.Apply(proxy.GetPrim())
    collision.CreateCollisionEnabledAttr().Set(False)


def _author_panel_body(stage: Usd.Stage, root_path: str, panel: dict, material, mass: float):
    body = UsdGeom.Xform.Define(stage, root_path)
    _set_translate(UsdGeom.Xformable(body.GetPrim()), (0.0, 0.0, KNEE_Z))
    _apply_body(body.GetPrim(), mass, inertia=2.0e-7)
    mesh_name = _safe_name(panel["name"])
    _author_mesh(stage, f"{root_path}/Panel_{mesh_name}", panel["points"], material)
    centroid = sum((point for point in panel["points"]), Gf.Vec3f(0.0)) / len(panel["points"])
    _author_mass_proxy(stage, root_path, centroid)
    _metadata(
        body.GetPrim(),
        {
            "role": "original joint rigid fold panel",
            "sourcePanel": panel["name"],
            "sourceVertexIds": panel["vertexIds"],
            "sourceScale": SOURCE_SCALE,
            "sourceAxisRotation": SOURCE_AXIS_ROTATION,
            "collision": "disabled on zero-thickness panel; hinge constraints remain physical",
        },
    )
    return body


def _author_crease_visual(
    stage: Usd.Stage,
    parent_path: str,
    name: str,
    a: Gf.Vec3f,
    b: Gf.Vec3f,
    material,
    radius: float = 0.00028,
    name_prefix: str = "Crease",
    offset: Gf.Vec3f | None = None,
):
    delta = b - a
    length = delta.GetLength()
    midpoint = (a + b) * 0.5
    if offset is not None:
        midpoint += offset
    cylinder = UsdGeom.Cylinder.Define(stage, f"{parent_path}/{name_prefix}_{_safe_name(name)}")
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(float(length))
    cylinder.CreateAxisAttr(UsdGeom.Tokens.z)
    xformable = UsdGeom.Xformable(cylinder.GetPrim())
    _set_translate(xformable, midpoint)
    _set_orient(xformable, _quat_align(Gf.Vec3f(0.0, 0.0, 1.0), delta))
    UsdShade.MaterialBindingAPI(cylinder.GetPrim()).Bind(material)
    return cylinder


def _author_roof_interface_geometry(
    stage: Usd.Stage,
    roof_path: str,
    roof_panel: dict,
    topology: dict,
    panel_paths: dict[str, str],
    roof_material,
    seam_material,
) -> None:
    """Give each source roof quad a real interface and edge-to-panel overlap.

    The original roof faces are intentionally retained as the source visual
    mesh.  A thick plate, corner pads, and a short skirt around every perimeter
    edge are added as children of the same rigid roof body.  The skirt extends
    into the side-panel edge, so the manufactured interface remains visually
    continuous even when the closed crease solver has a small residual error.
    The matching rail on the neighbouring side panel shares the exact source
    edge; the source revolute hinge remains the physical fold constraint.
    """

    connection_root = f"{roof_path}/InterfaceConnection"
    UsdGeom.Scope.Define(stage, connection_root)
    roof_z = float(roof_panel["points"][0][2])
    plate_size = 2.0 * (ROOF_HALF_SPAN + ROOF_PLATE_OVERHANG)
    plate = _cube(
        stage,
        f"{connection_root}/StructuralPlate",
        (0.0, 0.0, roof_z),
        (plate_size, plate_size, ROOF_PLATE_THICKNESS),
        roof_material,
        collision=False,
    )
    _metadata(
        plate.GetPrim(),
        {
            "role": "thickened flat interface for source roof face",
            "sourcePanel": roof_panel["name"],
            "overlapAtSourceBoundaryM": ROOF_PLATE_OVERHANG,
            "thicknessM": ROOF_PLATE_THICKNESS,
        },
    )

    vertex_ids = roof_panel["vertexIds"]
    for vertex_id in vertex_ids:
        point = topology["verticesJoint"][vertex_id]
        pad = _cube(
            stage,
            f"{connection_root}/CornerPad_{_safe_name(vertex_id)}",
            point,
            (ROOF_CORNER_PAD_SIZE, ROOF_CORNER_PAD_SIZE, ROOF_PLATE_THICKNESS),
            roof_material,
            collision=False,
        )
        _metadata(pad.GetPrim(), {"role": "roof-to-side corner connection", "sourceVertex": vertex_id})

    skirt_offset = Gf.Vec3f(0.0, 0.0, -math.copysign(ROOF_SKIRT_DEPTH * 0.5, roof_z))
    for vertex_id in vertex_ids:
        point = topology["verticesJoint"][vertex_id] + skirt_offset
        skirt_pad = _cube(
            stage,
            f"{connection_root}/CornerSkirt_{_safe_name(vertex_id)}",
            point,
            (ROOF_CORNER_PAD_SIZE, ROOF_CORNER_PAD_SIZE, ROOF_SKIRT_DEPTH),
            roof_material,
            collision=False,
        )
        _metadata(
            skirt_pad.GetPrim(),
            {"role": "roof-to-side vertical corner skirt", "sourceVertex": vertex_id},
        )

    for index, start in enumerate(vertex_ids):
        end = vertex_ids[(index + 1) % len(vertex_ids)]
        line = topology["line_by_edge"][_edge_key(start, end)]
        edge = _author_crease_visual(
            stage,
            connection_root,
            line["name"],
            line["a"],
            line["b"],
            roof_material,
            radius=ROOF_PERIMETER_RADIUS,
            name_prefix="RoofPerimeter",
        )
        _metadata(
            edge.GetPrim(),
            {
                "role": "roof perimeter rail overlapping source side-panel edge",
                "sourceLine": line["name"],
                "sourceEdge": [start, end],
            },
        )
        skirt_edge = _author_crease_visual(
            stage,
            connection_root,
            line["name"],
            line["a"],
            line["b"],
            roof_material,
            radius=ROOF_SKIRT_RADIUS,
            name_prefix="RoofSkirt",
            offset=skirt_offset,
        )
        _metadata(
            skirt_edge.GetPrim(),
            {
                "role": "rigid roof skirt bridging the side-panel edge",
                "sourceLine": line["name"],
                "skirtDepthM": ROOF_SKIRT_DEPTH,
            },
        )

        owners = topology["edge_owners"][_edge_key(start, end)]
        side_owner = next(owner for owner in owners if owner != roof_panel["name"])
        side_root = f"{panel_paths[side_owner]}/InterfaceConnection"
        UsdGeom.Scope.Define(stage, side_root)
        side_edge = _author_crease_visual(
            stage,
            side_root,
            line["name"],
            line["a"],
            line["b"],
            seam_material,
            radius=SIDE_BOUNDARY_RADIUS,
            name_prefix="RoofBoundary",
        )
        _metadata(
            side_edge.GetPrim(),
            {
                "role": "side-panel boundary rail matched to roof interface",
                "sourceLine": line["name"],
                "connectedRoof": roof_panel["name"],
            },
        )


def _author_hinge(stage: Usd.Stage, path: str, body0, body1, anchor: Gf.Vec3f, axis: Gf.Vec3f, source_line: str):
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0.GetPath()])
    joint.CreateBody1Rel().SetTargets([body1.GetPath()])
    joint.CreateAxisAttr("Z")
    joint.CreateLocalPos0Attr().Set(anchor)
    joint.CreateLocalPos1Attr().Set(anchor)
    frame_quat = _quat_align(Gf.Vec3f(0.0, 0.0, 1.0), axis)
    joint.CreateLocalRot0Attr().Set(frame_quat)
    joint.CreateLocalRot1Attr().Set(frame_quat)
    joint.CreateLowerLimitAttr(-175.0)
    joint.CreateUpperLimitAttr(175.0)
    physx_joint = PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())
    physx_joint.CreateJointFrictionAttr().Set(0.006)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(CREASE_STIFFNESS)
    drive.CreateDampingAttr(CREASE_DAMPING)
    drive.CreateMaxForceAttr(0.01)
    drive.CreateTargetPositionAttr(0.0)
    _metadata(
        joint.GetPrim(),
        {
            "role": "source panel fold line",
            "sourceLine": source_line,
            "foldAxis": [float(value) for value in axis],
            "foldAnchor": [float(value) for value in anchor],
            "passiveCrease": True,
            "creaseSpringStiffness": CREASE_STIFFNESS,
            "creaseSpringDamping": CREASE_DAMPING,
        },
    )
    return joint


def _author_actuator_joint(stage: Usd.Stage, path: str, body0, body1, local0, local1, axis_token: str, axis: Gf.Vec3f, limits, target):
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0.GetPath()])
    joint.CreateBody1Rel().SetTargets([body1.GetPath()])
    joint.CreateAxisAttr(axis_token)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local1))
    frame_quat = _quat_align(Gf.Vec3f(0.0, 1.0, 0.0) if axis_token == "Y" else Gf.Vec3f(0.0, 0.0, 1.0), axis)
    joint.CreateLocalRot0Attr().Set(frame_quat)
    joint.CreateLocalRot1Attr().Set(frame_quat)
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(SEA_STIFFNESS)
    drive.CreateDampingAttr(0.05)
    drive.CreateMaxForceAttr(SEA_TORQUE_LIMIT)
    drive.CreateTargetPositionAttr(target)
    _metadata(
        joint.GetPrim(),
        {
            "jointAxis": axis_token,
            "paperJointLimitsDeg": list(limits),
            "seaSpringTravelDeg": [-SEA_TRAVEL_DEG, SEA_TRAVEL_DEG],
            "seaSpringStiffnessNmPerDeg": 0.012,
            "seaSpringStiffnessNmPerRad": SEA_STIFFNESS,
            "seaDesignTorqueLimitNm": SEA_TORQUE_LIMIT,
        },
    )
    return joint


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
            "assetType": "paper_guided_leg_with_original_panel_crease_joint",
            "sourceAsset": str(source_usd),
            "sourceTopology": str(topology_json),
            "sourcePanelCount": len(topology["panels"]),
            "sourceFoldLineCount": len(topology["lines"]),
            "foldMechanism": "one rigid body per source panel, revolute joint on every shared source line",
            "kneeMechanism": "motorized revolute between the two source roof end plates; panel creases carry compression",
            "flatInterfaceOrientation": SOURCE_AXIS_ROTATION,
            "jointOrder": ["Hip", "Knee", "Ankle"],
            "jointCentersMm": {"hip": 136.8, "knee": 88.3, "ankle": 39.8},
            "jointSpacingMm": 48.5,
            "paperKneeLimitsDeg": list(PAPER_LIMITS_DEG["knee"]),
            "policyRateHz": 100,
            "lowLevelRateHz": 1000,
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

    body = UsdGeom.Xform.Define(stage, "/World/PanelCreaseLeg/Body")
    _set_translate(UsdGeom.Xformable(body.GetPrim()), (0.0, 0.0, HIP_Z))
    _apply_body(body.GetPrim(), 0.25, inertia=2.0e-4)
    _cube(stage, "/World/PanelCreaseLeg/Body/Mount", (0.0, 0.0, 0.012), (0.035, 0.03, 0.012), body_mat)

    thigh = UsdGeom.Xform.Define(stage, "/World/PanelCreaseLeg/Thigh")
    _set_translate(UsdGeom.Xformable(thigh.GetPrim()), (0.0, 0.0, HIP_Z))
    _apply_body(thigh.GetPrim(), 0.1074, inertia=1.5e-4)
    thigh_length = HIP_Z - (KNEE_Z + 0.125 * SOURCE_SCALE)
    _capsule(stage, "/World/PanelCreaseLeg/Thigh/Link", (0.0, 0.0, -thigh_length / 2.0), 0.008, thigh_length, link_mat)

    shank = UsdGeom.Xform.Define(stage, "/World/PanelCreaseLeg/Shank")
    lower_interface_z = KNEE_Z - 0.125 * SOURCE_SCALE
    _set_translate(UsdGeom.Xformable(shank.GetPrim()), (0.0, 0.0, lower_interface_z))
    _apply_body(shank.GetPrim(), 0.1070, inertia=1.5e-4)
    shank_length = lower_interface_z - ANKLE_Z
    _capsule(stage, "/World/PanelCreaseLeg/Shank/Link", (0.0, 0.0, -shank_length / 2.0), 0.0075, shank_length, link_mat)

    foot = UsdGeom.Xform.Define(stage, "/World/PanelCreaseLeg/Foot")
    _set_translate(UsdGeom.Xformable(foot.GetPrim()), (0.0, 0.0, ANKLE_Z))
    _apply_body(foot.GetPrim(), 0.0676, inertia=8.0e-5)
    _cube(stage, "/World/PanelCreaseLeg/Foot/Sole", (0.014, 0.0, -0.006), (0.030, 0.016, 0.006), foot_mat, collision=True)

    panel_bodies: dict[str, Usd.Prim] = {}
    panel_paths: dict[str, str] = {}
    panel_root = UsdGeom.Scope.Define(stage, "/World/PanelCreaseLeg/OriginalJointPanels")
    for panel in topology["panels"]:
        name = panel["name"]
        if name == "Roof 1":
            path_name = "RoofTop"
            material = roof_mat
            mass = 0.012
        elif name == "Roof 3":
            path_name = "RoofBottom"
            material = roof_mat
            mass = 0.012
        elif panel["kind"] == "s":
            path_name = f"Panel_{_safe_name(name)}"
            material = s_panel_mat
            mass = 0.0015
        else:
            path_name = f"Panel_{_safe_name(name)}"
            material = c_panel_mat
            mass = 0.0015
        body_path = f"/World/PanelCreaseLeg/OriginalJointPanels/{path_name}"
        body_prim = _author_panel_body(stage, body_path, panel, material, mass)
        panel_bodies[name] = body_prim
        panel_paths[name] = body_path

    # The source roof quads are the two flat interfaces in the original joint.
    # Thicken them and add matching rails on both sides of each roof boundary;
    # the already-authored crease hinge remains the physical connection.
    _author_roof_interface_geometry(
        stage,
        panel_paths["Roof 1"],
        topology["panel_by_name"]["Roof 1"],
        topology,
        panel_paths,
        roof_mat,
        crease_mat,
    )
    _author_roof_interface_geometry(
        stage,
        panel_paths["Roof 3"],
        topology["panel_by_name"]["Roof 3"],
        topology,
        panel_paths,
        roof_mat,
        crease_mat,
    )

    # Attach one visible dark rod to each source line on its first panel.  The
    # rod makes it obvious in the viewport which edges are active creases and
    # follows that panel when the joint compresses.
    for line in topology["lines"]:
        edge = _edge_key(line["vertexIds"][0], line["vertexIds"][1])
        owners = topology["edge_owners"][edge]
        parent = panel_paths[owners[0]]
        _author_crease_visual(stage, parent, line["name"], line["a"], line["b"], crease_mat)

    physics_root = UsdGeom.Scope.Define(stage, "/World/PanelCreaseLeg/Physics")
    for index, line in enumerate(topology["lines"], start=1):
        edge = _edge_key(line["vertexIds"][0], line["vertexIds"][1])
        owners = topology["edge_owners"][edge]
        anchor = (line["a"] + line["b"]) * 0.5
        axis = _unit(line["b"] - line["a"])
        hinge_path = f"/World/PanelCreaseLeg/Physics/Crease_{index:02d}_{_safe_name(line['name'])}"
        _author_hinge(stage, hinge_path, panel_bodies[owners[0]], panel_bodies[owners[1]], anchor, axis, line["name"])

    body_mount = UsdPhysics.FixedJoint.Define(stage, "/World/PanelCreaseLeg/Physics/BodyMount")
    body_mount.CreateBody1Rel().SetTargets([body.GetPath()])
    # With body0 omitted, localPos0 is in world space.  The source leg's
    # pinned body frame is at the paper hip height, not at world origin.
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

    top_roof = panel_bodies["Roof 1"]
    bottom_roof = panel_bodies["Roof 3"]
    upper_face_z = KNEE_Z + 0.125 * SOURCE_SCALE
    lower_face_z = KNEE_Z - 0.125 * SOURCE_SCALE
    thigh_roof = UsdPhysics.FixedJoint.Define(stage, "/World/PanelCreaseLeg/Physics/ThighToOriginalRoofTop")
    thigh_roof.CreateBody0Rel().SetTargets([thigh.GetPath()])
    thigh_roof.CreateBody1Rel().SetTargets([top_roof.GetPath()])
    thigh_roof.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -(HIP_Z - upper_face_z)))
    thigh_roof.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.125 * SOURCE_SCALE))
    _metadata(thigh_roof.GetPrim(), {"role": "upper flat interface to original joint", "sourcePanel": "Roof 1"})

    knee = _author_actuator_joint(
        stage,
        "/World/PanelCreaseLeg/Physics/KneeActuator",
        top_roof,
        bottom_roof,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        "Y",
        Gf.Vec3f(0.0, 1.0, 0.0),
        PAPER_LIMITS_DEG["knee"],
        45.0,
    )
    _metadata(
        knee.GetPrim(),
        {
            "role": "knee actuator across original joint flat interfaces",
            "sourceGeometry": str(source_usd),
            "foldNetwork": "all 76 source shared panel edges",
            "compressionMechanism": "roof plates are actuated; each source panel remains rigid and each shared line is a revolute crease",
        },
    )

    bottom_shank = UsdPhysics.FixedJoint.Define(stage, "/World/PanelCreaseLeg/Physics/OriginalRoofBottomToShank")
    bottom_shank.CreateBody0Rel().SetTargets([bottom_roof.GetPath()])
    bottom_shank.CreateBody1Rel().SetTargets([shank.GetPath()])
    bottom_shank.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -0.125 * SOURCE_SCALE))
    bottom_shank.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    _metadata(bottom_shank.GetPrim(), {"role": "lower flat interface to original joint", "sourcePanel": "Roof 3"})

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
    controller.CreateAttribute("lowLevelRateHz", Sdf.ValueTypeNames.Int).Set(1000)
    controller.CreateAttribute("jointNames", Sdf.ValueTypeNames.String).Set("Hip,Knee,Ankle")
    controller.CreateAttribute("defaultTargetsDeg", Sdf.ValueTypeNames.String).Set("0,45,0")
    controller.CreateAttribute("commandType", Sdf.ValueTypeNames.Token).Set("target_joint_position")
    controller.CreateAttribute("originalJointModel", Sdf.ValueTypeNames.String).Set("50 rigid source panels + 76 revolute source crease hinges")

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
    physx_scene.CreateTimeStepsPerSecondAttr().Set(1000)
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
    print("Original source topology: 50 rigid panels, 76 shared-edge revolute creases")
    print("Knee actuator: Roof 1 -> Roof 3, world Y, paper limits -1.5..127 deg")


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

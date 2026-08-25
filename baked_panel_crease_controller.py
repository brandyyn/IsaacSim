"""Fast, deterministic runtime for the original panel-crease shell.

The physical bodies and the single PhysX knee revolute remain authoritative.
At showcase time we do not run the Python/NumPy least-squares optimizer every
frame: that optimizer is useful for offline/validation measurements, but its
Python constraint loops can starve Kit's update loop when several joints are
present. Instead, this controller uses the same authored 50-panel/28-vertex/
76-edge topology and projects each source triangle to a rigid pose from the
current top and bottom physical interfaces. Shared vertices are averaged once
from those facet poses, then the same welded positions drive the colored
panels and the single baked crease-prism mesh.

This is a reduced-order kinematic display model, not a replacement for FEA.
The solver controller remains available through
``PANEL_CREASE_VISUAL_MODE=solver`` for calibration and validation runs.
"""

from __future__ import annotations

import asyncio
import json
import math

import numpy as np
import omni.kit.app
import omni.usd
from pxr import UsdGeom, Vt

import stable_panel_crease_controller as stable


TOP_BODY = stable.TOP_BODY
BOTTOM_BODY = stable.BOTTOM_BODY
VISUAL_ROOT = stable.VISUAL_ROOT
CONTROLLER = "/World/PanelCreaseLeg/Controller"

_controller_task = None


def _custom_json(prim, key, default):
    value = prim.GetCustomDataByKey(key)
    if value is None:
        return default
    if isinstance(value, str):
        return json.loads(value)
    return value


def _attribute_value(prim, name, default):
    attribute = prim.GetAttribute(name)
    if not attribute or not attribute.IsValid():
        return default
    value = attribute.Get()
    return default if value is None else value


def _distance(first, second):
    return float(np.linalg.norm(np.asarray(first, dtype=float) - np.asarray(second, dtype=float)))


def _normalize(quaternion):
    magnitude = math.sqrt(sum(float(value) * float(value) for value in quaternion))
    if magnitude < 1.0e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(float(value) / magnitude for value in quaternion)


def _slerp(first, second, amount):
    """Interpolate two world quaternions in Isaac's wxyz ordering."""

    first = _normalize(first)
    second = _normalize(second)
    dot = sum(first[index] * second[index] for index in range(4))
    if dot < 0.0:
        second = tuple(-value for value in second)
        dot = -dot
    if dot > 0.9995:
        return _normalize(
            tuple(
                first[index] + amount * (second[index] - first[index])
                for index in range(4)
            )
        )
    angle = math.acos(max(-1.0, min(1.0, dot)))
    sine = math.sin(angle)
    first_weight = math.sin((1.0 - amount) * angle) / sine
    second_weight = math.sin(amount * angle) / sine
    return tuple(
        first_weight * first[index] + second_weight * second[index]
        for index in range(4)
    )


def _interpolate_pose(bottom_pose, top_pose, amount):
    bottom_position, bottom_quaternion = bottom_pose
    top_position, top_quaternion = top_pose
    position = tuple(
        bottom_position[index]
        + amount * (top_position[index] - bottom_position[index])
        for index in range(3)
    )
    return position, _slerp(bottom_quaternion, top_quaternion, amount)


def _rigid_facet_transform(rest_points, current_points):
    """Fit a proper rigid transform from a source triangle to its target."""

    rest_center = np.mean(rest_points, axis=0)
    current_center = np.mean(current_points, axis=0)
    covariance = (rest_points - rest_center).T @ (current_points - current_center)
    left, _, right_transposed = np.linalg.svd(covariance)
    rotation = left @ right_transposed
    if float(np.linalg.det(rotation)) < 0.0:
        left[:, -1] *= -1.0
        rotation = left @ right_transposed
    translation = current_center - rest_center @ rotation
    return rotation, translation


def _batch_svd_rigid_facet_transforms(rest_points, current_points):
    """Fit all source triangles with SVD (validation fallback)."""

    rest_centers = np.mean(rest_points, axis=1)
    current_centers = np.mean(current_points, axis=1)
    covariance = np.einsum(
        "fpi,fpj->fij",
        rest_points - rest_centers[:, None, :],
        current_points - current_centers[:, None, :],
    )
    left, _, right_transposed = np.linalg.svd(covariance)
    rotations = left @ right_transposed
    reflected = np.linalg.det(rotations) < 0.0
    if np.any(reflected):
        left[reflected, :, -1] *= -1.0
        rotations[reflected] = left[reflected] @ right_transposed[reflected]
    translations = current_centers - np.einsum(
        "fi,fij->fj", rest_centers, rotations
    )
    return rotations, translations


def _batch_rigid_facet_transforms(rest_points, current_points):
    """Fit ordered non-degenerate triangles without a per-facet SVD.

    Three non-collinear points define a rigid triangle pose exactly. Building
    a right-handed frame from the first two edges is materially cheaper than
    running 52 independent 3x3 SVDs on every Kit update. The SVD path remains
    a safe fallback for malformed or temporarily collapsed facets.
    """

    rest_edges = rest_points[:, 1:] - rest_points[:, :1]
    current_edges = current_points[:, 1:] - current_points[:, :1]
    rest_first_norm = np.linalg.norm(rest_edges[:, 0], axis=1)
    current_first_norm = np.linalg.norm(current_edges[:, 0], axis=1)

    rest_u = rest_edges[:, 0] / np.maximum(rest_first_norm[:, None], 1.0e-12)
    current_u = current_edges[:, 0] / np.maximum(current_first_norm[:, None], 1.0e-12)
    rest_normal_raw = np.cross(rest_u, rest_edges[:, 1])
    current_normal_raw = np.cross(current_u, current_edges[:, 1])
    rest_normal_norm = np.linalg.norm(rest_normal_raw, axis=1)
    current_normal_norm = np.linalg.norm(current_normal_raw, axis=1)
    rest_n = rest_normal_raw / np.maximum(rest_normal_norm[:, None], 1.0e-12)
    current_n = current_normal_raw / np.maximum(current_normal_norm[:, None], 1.0e-12)
    rest_v = np.cross(rest_n, rest_u)
    current_v = np.cross(current_n, current_u)

    rest_basis = np.stack((rest_u, rest_v, rest_n), axis=1)
    current_basis = np.stack((current_u, current_v, current_n), axis=1)
    rotations = np.einsum("fji,fjk->fik", rest_basis, current_basis)
    rest_centers = np.mean(rest_points, axis=1)
    current_centers = np.mean(current_points, axis=1)
    translations = current_centers - np.einsum(
        "fi,fij->fj", rest_centers, rotations
    )

    degenerate = (
        (rest_first_norm < 1.0e-10)
        | (current_first_norm < 1.0e-10)
        | (rest_normal_norm < 1.0e-10)
        | (current_normal_norm < 1.0e-10)
    )
    if np.any(degenerate):
        fallback_rotations, fallback_translations = _batch_svd_rigid_facet_transforms(
            rest_points[degenerate], current_points[degenerate]
        )
        rotations[degenerate] = fallback_rotations
        translations[degenerate] = fallback_translations
    return rotations, translations


def _pose_changed(previous, current, position_epsilon, quaternion_epsilon):
    if previous is None:
        return True
    previous_position, previous_quaternion = previous
    current_position, current_quaternion = current
    return (
        max(
            abs(float(previous_position[index]) - float(current_position[index]))
            for index in range(3)
        )
        > position_epsilon
        or max(
            abs(float(previous_quaternion[index]) - float(current_quaternion[index]))
            for index in range(4)
        )
        > quaternion_epsilon
    )


def _make_updater(stage):
    visual_prim = stage.GetPrimAtPath(VISUAL_ROOT)
    if not visual_prim.IsValid():
        return None

    embedded_vertices = visual_prim.GetCustomDataByKey("sourceVertexPositionsM")
    if not embedded_vertices:
        return None
    if isinstance(embedded_vertices, str):
        embedded_vertices = json.loads(embedded_vertices)

    vertex_ids = [str(vertex_id) for vertex_id in embedded_vertices]
    vertex_index = {vertex_id: index for index, vertex_id in enumerate(vertex_ids)}
    local_points = np.asarray(
        [
            stable._source_to_joint(embedded_vertices[vertex_id])
            for vertex_id in vertex_ids
        ],
        dtype=float,
    )
    source_y = np.asarray(
        [float(embedded_vertices[vertex_id][1]) for vertex_id in vertex_ids],
        dtype=float,
    )
    bottom_y = float(np.min(source_y))
    top_y = float(np.max(source_y))
    source_height = max(top_y - bottom_y, 1.0e-12)

    top_ids = set(_custom_json(visual_prim, "constraintTopVertexIds", []))
    bottom_ids = set(_custom_json(visual_prim, "constraintBottomVertexIds", []))
    facet_data = _custom_json(visual_prim, "constraintFacetTriples", [])
    if not top_ids or not bottom_ids or not facet_data:
        return _make_height_fallback(stage, visual_prim, embedded_vertices)

    facet_indices = []
    for item in facet_data:
        ids = [str(vertex_id) for vertex_id in item.get("vertexIds", [])]
        if len(ids) == 3 and all(vertex_id in vertex_index for vertex_id in ids):
            facet_indices.append(
                np.asarray([vertex_index[vertex_id] for vertex_id in ids], dtype=int)
            )
    if not facet_indices:
        return None
    facet_index_array = np.asarray(facet_indices, dtype=int)
    rest_facet_points = local_points[facet_index_array]
    facet_edge_first = facet_index_array[:, (0, 1, 2)]
    facet_edge_second = facet_index_array[:, (1, 2, 0)]
    facet_rest_edge_lengths = np.linalg.norm(
        local_points[facet_edge_second] - local_points[facet_edge_first], axis=2
    )

    anchor_indices = np.asarray(
        [
            vertex_index[vertex_id]
            for vertex_id in vertex_ids
            if vertex_id in top_ids or vertex_id in bottom_ids
        ],
        dtype=int,
    )
    top_index_set = {vertex_index[vertex_id] for vertex_id in top_ids}
    bottom_index_set = {vertex_index[vertex_id] for vertex_id in bottom_ids}

    edge_indices = []
    edge_rest_lengths = []
    for item in _custom_json(visual_prim, "constraintEdgePairs", []):
        first_id = str(item.get("a"))
        second_id = str(item.get("b"))
        if first_id not in vertex_index or second_id not in vertex_index:
            continue
        first = vertex_index[first_id]
        second = vertex_index[second_id]
        edge_indices.append((first, second))
        edge_rest_lengths.append(
            float(item.get("restLengthM", _distance(local_points[first], local_points[second])))
        )
    edge_index_array = np.asarray(edge_indices, dtype=int) if edge_indices else None
    edge_rest_array = np.asarray(edge_rest_lengths, dtype=float) if edge_rest_lengths else None

    mesh_prims = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(f"{VISUAL_ROOT}/"):
            continue
        if prim.GetTypeName() == "Mesh" and prim.GetCustomDataByKey("sourceVertexIds"):
            source_ids = _custom_json(prim, "sourceVertexIds", [])
            mesh_prims.append(
                (
                    UsdGeom.Mesh(prim),
                    np.asarray(
                        [vertex_index[str(vertex_id)] for vertex_id in source_ids],
                        dtype=int,
                    ),
                )
            )

    baked_mesh_prim = stage.GetPrimAtPath(f"{VISUAL_ROOT}/BakedCreaseMesh")
    baked_mesh = UsdGeom.Mesh(baked_mesh_prim) if baked_mesh_prim.IsValid() else None
    baked_curve = stage.GetPrimAtPath(f"{VISUAL_ROOT}/BakedCreaseNetwork")
    if baked_mesh is not None:
        UsdGeom.Imageable(baked_mesh_prim).MakeVisible()
    if baked_curve.IsValid():
        UsdGeom.Imageable(baked_curve).MakeInvisible()
    for prim in stable._reference_crease_prims(stage):
        UsdGeom.Imageable(prim).MakeInvisible()

    curve_edge_indices = []
    for edge in _custom_json(visual_prim, "referenceCreaseEdgeKeys", []):
        if len(edge) != 2:
            continue
        first_id, second_id = str(edge[0]), str(edge[1])
        if first_id in vertex_index and second_id in vertex_index:
            curve_edge_indices.append((vertex_index[first_id], vertex_index[second_id]))
    crease_mesh_sides = (
        int(_attribute_value(baked_mesh_prim, "creasePrismSides", 6))
        if baked_mesh is not None
        else 6
    )
    crease_mesh_radius = (
        float(_attribute_value(baked_mesh_prim, "creasePrismRadiusM", 0.00026))
        if baked_mesh is not None
        else 0.00026
    )

    controller = stage.GetPrimAtPath(CONTROLLER)
    neutral_center = np.asarray(stable._pose(TOP_BODY)[0], dtype=float)
    neutral_positions = local_points + neutral_center
    last_top_pose = None
    last_bottom_pose = None
    display_positions = None
    update_count = 0
    facet_passes = max(
        1,
        min(3, int(_attribute_value(controller, "kinematicFacetPasses", 1))),
    )
    metric_update_period = max(
        1,
        min(120, int(_attribute_value(controller, "kinematicMetricPeriod", 15))),
    )
    metric_frame = 0

    def anchor_targets(top_pose, bottom_pose):
        top_position, top_quaternion = top_pose
        bottom_position, bottom_quaternion = bottom_pose
        targets = np.zeros_like(local_points)
        for index in top_index_set:
            targets[index] = stable._transform(
                top_position, top_quaternion, local_points[index]
            )
        for index in bottom_index_set:
            targets[index] = stable._transform(
                bottom_position, bottom_quaternion, local_points[index]
            )
        return targets

    def interface_targets(top_pose, bottom_pose, anchors):
        """Make pose-aware targets before fitting rigid source facets."""

        targets = np.zeros_like(local_points)
        for index in range(len(vertex_ids)):
            if index in top_index_set or index in bottom_index_set:
                targets[index] = anchors[index]
                continue
            amount = max(
                0.0,
                min(1.0, (source_y[index] - bottom_y) / source_height),
            )
            position, quaternion = _interpolate_pose(bottom_pose, top_pose, amount)
            targets[index] = stable._transform(position, quaternion, local_points[index])
        return targets

    def rigid_facet_projection(targets, anchors):
        """Fit source triangles rigidly and weld their shared vertices."""

        projected = targets.copy()
        projected[anchor_indices] = anchors[anchor_indices]
        for _ in range(facet_passes):
            rotations, translations = _batch_rigid_facet_transforms(
                rest_facet_points, projected[facet_index_array]
            )
            predicted = np.einsum(
                "fpi,fij->fpj", rest_facet_points, rotations
            ) + translations[:, None, :]
            accumulated = np.zeros_like(local_points)
            counts = np.zeros(len(local_points), dtype=float)
            flat_indices = facet_index_array.reshape(-1)
            np.add.at(accumulated, flat_indices, predicted.reshape(-1, 3))
            np.add.at(counts, flat_indices, 1.0)
            covered = counts > 0.0
            projected[covered] = accumulated[covered] / counts[covered, None]
            projected[anchor_indices] = anchors[anchor_indices]
        return projected

    def crease_mesh_points(positions):
        points = []
        for first, second in curve_edge_indices:
            start = positions[first]
            end = positions[second]
            direction = end - start
            length = max(float(np.linalg.norm(direction)), 1.0e-12)
            direction = direction / length
            reference = np.asarray((0.0, 0.0, 1.0), dtype=float)
            if abs(float(np.dot(direction, reference))) > 0.9:
                reference = np.asarray((0.0, 1.0, 0.0), dtype=float)
            first_normal = np.cross(direction, reference)
            first_normal /= max(float(np.linalg.norm(first_normal)), 1.0e-12)
            second_normal = np.cross(direction, first_normal)
            second_normal /= max(float(np.linalg.norm(second_normal)), 1.0e-12)
            for endpoint in (start, end):
                for side in range(crease_mesh_sides):
                    angle = 2.0 * math.pi * side / crease_mesh_sides
                    radial = crease_mesh_radius * (
                        math.cos(angle) * first_normal
                        + math.sin(angle) * second_normal
                    )
                    points.append(tuple(float(value) for value in endpoint + radial))
        return points

    def apply_metrics(positions, targets):
        nonlocal metric_frame
        metric_frame += 1
        if edge_index_array is not None:
            edge_deltas = positions[edge_index_array[:, 1]] - positions[edge_index_array[:, 0]]
            edge_lengths = np.linalg.norm(edge_deltas, axis=1)
            edge_errors = np.abs(edge_lengths - edge_rest_array)
            edge_strains = edge_errors / np.maximum(edge_rest_array, 1.0e-9)
            values = {
                "constraintMaxEdgeStrain": float(np.max(edge_strains)),
                "constraintMeanEdgeStrain": float(np.mean(edge_strains)),
                "constraintMaxEdgeErrorM": float(np.max(edge_errors)),
                "constraintMeanEdgeErrorM": float(np.mean(edge_errors)),
            }
        else:
            values = {
                "constraintMaxEdgeStrain": 0.0,
                "constraintMeanEdgeStrain": 0.0,
                "constraintMaxEdgeErrorM": 0.0,
                "constraintMeanEdgeErrorM": 0.0,
            }

        values["constraintMaxAnchorErrorM"] = float(
            max((_distance(positions[index], targets[index]) for index in anchor_indices), default=0.0)
        )
        values["constraintMaxSelfClearanceViolationM"] = 0.0
        values["constraintMeanSelfClearanceViolationM"] = 0.0
        if metric_frame == 1 or metric_frame % metric_update_period == 0:
            rotations, translations = _batch_rigid_facet_transforms(
                rest_facet_points, positions[facet_index_array]
            )
            fitted = np.einsum(
                "fpi,fij->fpj", rest_facet_points, rotations
            ) + translations[:, None, :]
            facet_fit_errors = np.linalg.norm(
                positions[facet_index_array] - fitted, axis=2
            ).max(axis=1)
            facet_edge_lengths = np.linalg.norm(
                positions[facet_edge_second] - positions[facet_edge_first], axis=2
            )
            facet_distortions = np.abs(
                facet_edge_lengths - facet_rest_edge_lengths
            ) / np.maximum(facet_rest_edge_lengths, 1.0e-9)
            values.update(
                {
                    "constraintMaxFacetEdgeDistortion": float(np.max(facet_distortions)),
                    "constraintMeanFacetEdgeDistortion": float(np.mean(facet_distortions)),
                    "constraintMaxFacetFitErrorM": float(np.max(facet_fit_errors)),
                    "constraintMeanFacetFitErrorM": float(np.mean(facet_fit_errors)),
                }
            )
        for name, value in values.items():
            attribute = controller.GetAttribute(name)
            if attribute and attribute.IsValid():
                attribute.Set(float(value))
        for name, value in (
            ("constraintSolverPending", False),
            ("constraintSolverIterationsUsed", 0),
            ("constraintLastSolveMs", 0.0),
        ):
            attribute = controller.GetAttribute(name)
            if attribute and attribute.IsValid():
                attribute.Set(value)

    def apply_visual_positions(positions):
        nonlocal update_count
        for mesh, indices in mesh_prims:
            mesh.GetPointsAttr().Set(
                Vt.Vec3fArray(
                    [tuple(float(value) for value in positions[index]) for index in indices]
                )
            )
        if baked_mesh is not None and curve_edge_indices:
            baked_mesh.GetPointsAttr().Set(Vt.Vec3fArray(crease_mesh_points(positions)))
        update_count += 1

    def update():
        nonlocal last_top_pose, last_bottom_pose, display_positions
        top_pose = stable._pose(TOP_BODY)
        bottom_pose = stable._pose(BOTTOM_BODY)
        position_epsilon = float(_attribute_value(controller, "visualUpdateEpsilonM", 1.0e-7))
        quaternion_epsilon = float(_attribute_value(controller, "visualUpdateEpsilonQuat", 1.0e-6))
        changed = _pose_changed(last_top_pose, top_pose, position_epsilon, quaternion_epsilon) or _pose_changed(last_bottom_pose, bottom_pose, position_epsilon, quaternion_epsilon)
        if not changed and display_positions is not None:
            return

        targets = anchor_targets(top_pose, bottom_pose)
        neutral_tolerance = float(_attribute_value(controller, "neutralSnapToleranceM", 5.0e-5))
        neutral_error = max(
            (_distance(neutral_positions[index], targets[index]) for index in anchor_indices),
            default=0.0,
        )
        if neutral_error <= neutral_tolerance:
            positions = neutral_positions.copy()
        else:
            positions = rigid_facet_projection(
                interface_targets(top_pose, bottom_pose, targets),
                targets,
            )
            positions[anchor_indices] = targets[anchor_indices]

        if (
            display_positions is not None
            and float(np.max(np.abs(positions - display_positions))) <= position_epsilon
        ):
            last_top_pose = top_pose
            last_bottom_pose = bottom_pose
            return
        apply_visual_positions(positions)
        display_positions = positions
        last_top_pose = top_pose
        last_bottom_pose = bottom_pose
        apply_metrics(positions, targets)
        count_attribute = controller.GetAttribute("visualUpdateCount")
        if count_attribute and count_attribute.IsValid():
            count_attribute.Set(int(update_count))
        for name, value in (
            ("deterministicMaxAnchorErrorM", 0.0),
            ("deterministicMaxSelfClearanceViolationM", 0.0),
        ):
            attribute = controller.GetAttribute(name)
            if attribute and attribute.IsValid():
                attribute.Set(float(value))

    return update


def _make_height_fallback(stage, visual_prim, embedded_vertices):
    """Compatibility path for pre-geometry-coupled stages."""

    vertex_ids = [str(vertex_id) for vertex_id in embedded_vertices]
    local_points = {
        vertex_id: stable._source_to_joint(embedded_vertices[vertex_id])
        for vertex_id in vertex_ids
    }
    source_y = {vertex_id: float(embedded_vertices[vertex_id][1]) for vertex_id in vertex_ids}
    bottom_y = min(source_y.values())
    top_y = max(source_y.values())
    height = max(top_y - bottom_y, 1.0e-12)
    mesh_prims = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith(f"{VISUAL_ROOT}/") and prim.GetTypeName() == "Mesh" and prim.GetCustomDataByKey("sourceVertexIds"):
            mesh_prims.append((UsdGeom.Mesh(prim), _custom_json(prim, "sourceVertexIds", [])))
    baked_mesh = stage.GetPrimAtPath(f"{VISUAL_ROOT}/BakedCreaseMesh")
    if baked_mesh.IsValid():
        UsdGeom.Imageable(baked_mesh).MakeInvisible()
    for prim in stable._reference_crease_prims(stage):
        UsdGeom.Imageable(prim).MakeInvisible()

    def update():
        top_pose = stable._pose(TOP_BODY)
        bottom_pose = stable._pose(BOTTOM_BODY)
        points = {}
        for vertex_id in vertex_ids:
            amount = max(0.0, min(1.0, (source_y[vertex_id] - bottom_y) / height))
            pose = _interpolate_pose(bottom_pose, top_pose, amount)
            points[vertex_id] = stable._transform(pose[0], pose[1], local_points[vertex_id])
        for mesh, source_ids in mesh_prims:
            mesh.GetPointsAttr().Set(
                Vt.Vec3fArray([points[str(vertex_id)] for vertex_id in source_ids])
            )

    return update


async def _run():
    updater = None
    last_stage_key = None
    while True:
        await omni.kit.app.get_app().next_update_async()
        try:
            stage = omni.usd.get_context().get_stage()
            if stage is None or not stage.GetPrimAtPath(TOP_BODY).IsValid():
                updater = None
                last_stage_key = None
                continue
            stage_key = str(stage.GetRootLayer().realPath or stage.GetRootLayer().identifier)
            if stage_key != last_stage_key or updater is None:
                updater = _make_updater(stage)
                last_stage_key = stage_key
            if updater is not None:
                updater()
        except Exception as error:
            print(f"baked panel crease controller paused: {error}")
            await omni.kit.app.get_app().next_update_async()


def start():
    global _controller_task
    if _controller_task is None or _controller_task.done():
        _controller_task = asyncio.ensure_future(_run())
    return _controller_task


def stop():
    global _controller_task
    if _controller_task is not None and not _controller_task.done():
        _controller_task.cancel()
    _controller_task = None

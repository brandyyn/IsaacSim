"""Facet-rigid, crease-specific solver for the original panel shell.

The v3 controller minimized source-edge strain globally, which removed the
edge-order bias but still allowed a single triangle to shear when the hard
upper and lower interfaces were driven to an arbitrary pose.  This pass adds
the geometry that the physical joint actually has: each source triangle is a
facet, and adjacent facets share the same vertex positions at their crease
lines.

At each quasi-static update the controller fits a rigid transform to every
source triangle (the local step), then solves all shared free vertices against
those facet targets and the normalized edge-strain objective (the global
step).  The two source interface quads are represented by two solver
triangles each, without adding visible diagonal crease lines.  Upper and
lower interface vertices remain hard anchors.  This is an ARAP-style
reduced-order crease model, not a replacement for FEA; its facet distortion
metrics are deliberately exposed for later calibration.
"""

from __future__ import annotations

import asyncio
from concurrent.futures import ThreadPoolExecutor
import json
import math
import time

import numpy as np
import omni.kit.app
import omni.usd
from pxr import Gf, UsdGeom, Vt

import stable_panel_crease_controller as stable


TOP_BODY = stable.TOP_BODY
BOTTOM_BODY = stable.BOTTOM_BODY
VISUAL_ROOT = stable.VISUAL_ROOT
CONTROLLER = "/World/PanelCreaseLeg/Controller"

_controller_task = None
_SOLVER_EXECUTOR = ThreadPoolExecutor(max_workers=1, thread_name_prefix="panel_facet_solver")


def _custom_json(prim, key, default):
    value = prim.GetCustomDataByKey(key)
    if value is None:
        return default
    if isinstance(value, str):
        return json.loads(value)
    return value


def _distance(a, b):
    return math.sqrt(sum((float(a[index]) - float(b[index])) ** 2 for index in range(3)))


def _attribute_value(prim, name, default):
    attribute = prim.GetAttribute(name)
    if not attribute:
        return default
    value = attribute.Get()
    return default if value is None else value


def _bounded_step(step, maximum):
    vectors = step.reshape((-1, 3))
    norms = np.linalg.norm(vectors, axis=1)
    scales = np.minimum(1.0, maximum / np.maximum(norms, 1.0e-12))
    return (vectors * scales[:, None]).reshape(-1)


def _orthonormalize_rotation(matrix):
    """Return the closest proper rotation to a 3x3 matrix."""

    left, _, right_transposed = np.linalg.svd(matrix)
    rotation = left @ right_transposed
    if float(np.linalg.det(rotation)) < 0.0:
        left[:, -1] *= -1.0
        rotation = left @ right_transposed
    return rotation


def _rigid_facet_transform(rest_points, current_points):
    """Fit a proper rigid transform from ``rest_points`` to ``current_points``."""

    rest_center = np.mean(rest_points, axis=0)
    current_center = np.mean(current_points, axis=0)
    rest_centered = rest_points - rest_center
    current_centered = current_points - current_center
    covariance = rest_centered.T @ current_centered
    left, _, right_transposed = np.linalg.svd(covariance)
    rotation = left @ right_transposed
    if float(np.linalg.det(rotation)) < 0.0:
        left[:, -1] *= -1.0
        rotation = left @ right_transposed
    translation = current_center - rest_center @ rotation
    return rotation, translation


def _rigid_facet_fit(rest_points, current_points):
    """Return the best-fit rigid image of a triangular facet.

    Points are row vectors.  The returned positions have the same ordering as
    ``rest_points``.  The reflection guard matters when a highly compressed
    triangle is momentarily close to a degenerate configuration.
    """

    rotation, translation = _rigid_facet_transform(rest_points, current_points)
    return rest_points @ rotation + translation


def _make_updater(stage):
    visual_prim = stage.GetPrimAtPath(VISUAL_ROOT)
    if not visual_prim.IsValid():
        return None
    baked_curve = stage.GetPrimAtPath(f"{VISUAL_ROOT}/BakedCreaseNetwork")
    curve = UsdGeom.BasisCurves(baked_curve) if baked_curve.IsValid() else None
    baked_mesh = stage.GetPrimAtPath(f"{VISUAL_ROOT}/BakedCreaseMesh")
    crease_mesh = UsdGeom.Mesh(baked_mesh) if baked_mesh.IsValid() else None

    embedded_vertices = visual_prim.GetCustomDataByKey("sourceVertexPositionsM")
    if not embedded_vertices:
        return None
    if isinstance(embedded_vertices, str):
        embedded_vertices = json.loads(embedded_vertices)

    vertex_ids = [str(vertex_id) for vertex_id in embedded_vertices]
    vertex_index = {vertex_id: index for index, vertex_id in enumerate(vertex_ids)}
    local_points = np.asarray(
        [stable._source_to_joint(embedded_vertices[vertex_id]) for vertex_id in vertex_ids],
        dtype=float,
    )
    # The generated source mesh is authored at the physical knee centre.  Keep
    # an explicit neutral copy so returning to zero is an exact geometric
    # reset, rather than asking the numerical solver to rediscover a pose that
    # is already known.  This removes the small default ``playdough`` drift
    # that used to remain after a motion cycle.
    neutral_center = np.asarray(stable._pose(TOP_BODY)[0], dtype=float)
    neutral_positions = local_points + neutral_center

    edge_data = _custom_json(visual_prim, "constraintEdgePairs", [])
    facet_data = _custom_json(visual_prim, "constraintFacetTriples", [])
    top_ids = set(_custom_json(visual_prim, "constraintTopVertexIds", []))
    bottom_ids = set(_custom_json(visual_prim, "constraintBottomVertexIds", []))
    if not edge_data or not facet_data or not top_ids or not bottom_ids:
        return None

    edge_indices = []
    edge_rest_lengths = []
    for item in edge_data:
        first = vertex_index[str(item["a"])]
        second = vertex_index[str(item["b"])]
        edge_indices.append((first, second))
        edge_rest_lengths.append(float(item["restLengthM"]))
    edge_indices = np.asarray(edge_indices, dtype=int)
    edge_rest_lengths = np.asarray(edge_rest_lengths, dtype=float)
    edge_set = {
        tuple(sorted((int(first), int(second))))
        for first, second in edge_indices.tolist()
    }

    facet_indices = []
    for item in facet_data:
        ids = [str(vertex_id) for vertex_id in item["vertexIds"]]
        if len(ids) != 3:
            continue
        facet_indices.append([vertex_index[vertex_id] for vertex_id in ids])
    facet_indices = np.asarray(facet_indices, dtype=int)
    if len(facet_indices) != 52:
        print(f"facet crease controller: expected 52 solver triangles, found {len(facet_indices)}")
        return None

    anchor_ids = top_ids | bottom_ids
    anchor_indices = np.asarray(
        [vertex_index[vertex_id] for vertex_id in vertex_ids if vertex_id in anchor_ids],
        dtype=int,
    )
    free_indices = np.asarray(
        [vertex_index[vertex_id] for vertex_id in vertex_ids if vertex_id not in anchor_ids],
        dtype=int,
    )
    anchor_index_set = {int(index) for index in anchor_indices.tolist()}
    # Keep non-adjacent parts of the shell from collapsing through one
    # another under a hard end-pose. These are solver barriers, not added
    # crease lines; the canonical 76 source edges remain unchanged.
    clearance_pairs = []
    for first in range(len(vertex_ids)):
        for second in range(first + 1, len(vertex_ids)):
            if first in anchor_index_set and second in anchor_index_set:
                continue
            if (first, second) in edge_set:
                continue
            if _distance(local_points[first], local_points[second]) > 1.0e-9:
                clearance_pairs.append((first, second))
    midpoint_clearance_pairs = []
    for first_edge in range(len(edge_indices)):
        first_start, first_end = edge_indices[first_edge]
        first_midpoint = (local_points[first_start] + local_points[first_end]) * 0.5
        first_vertices = {int(first_start), int(first_end)}
        for second_edge in range(first_edge + 1, len(edge_indices)):
            second_start, second_end = edge_indices[second_edge]
            if first_vertices & {int(second_start), int(second_end)}:
                continue
            second_midpoint = (local_points[second_start] + local_points[second_end]) * 0.5
            # Keep a compact candidate set. A fold can bring these edges
            # together, while distant source regions are not a credible
            # first-order crease-stack interaction at this scale.
            if _distance(first_midpoint, second_midpoint) <= 0.03:
                midpoint_clearance_pairs.append((first_edge, second_edge))
    variable_offsets = {int(index): 3 * offset for offset, index in enumerate(free_indices)}
    variable_count = 3 * len(free_indices)

    edge_first_offsets = np.asarray(
        [variable_offsets.get(int(first), -1) for first, _ in edge_indices],
        dtype=int,
    )
    edge_second_offsets = np.asarray(
        [variable_offsets.get(int(second), -1) for _, second in edge_indices],
        dtype=int,
    )

    # The visual meshes use one shared vertex map.  Updating these meshes and
    # the crease cylinders from that same map is what prevents seam gaps.
    mesh_prims = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(VISUAL_ROOT):
            continue
        if prim.GetTypeName() == "Mesh" and prim.GetCustomDataByKey("sourceVertexIds"):
            source_ids = _custom_json(prim, "sourceVertexIds", [])
            mesh_prims.append(
                (
                    UsdGeom.Mesh(prim),
                    np.asarray([vertex_index[str(vertex_id)] for vertex_id in source_ids], dtype=int),
                )
            )
    crease_prims = stable._reference_crease_prims(stage)
    if crease_mesh is not None:
        # The mesh is the fast render representation of the authored source
        # lines: one USD point-array update moves all 76 prism segments.
        UsdGeom.Imageable(baked_mesh).MakeVisible()
        for prim in crease_prims:
            UsdGeom.Imageable(prim).MakeInvisible()
    else:
        # Backward-compatible fallback for older stages. The authored
        # cylinders remain the real crease structure and are driven below.
        for prim in crease_prims:
            UsdGeom.Imageable(prim).MakeVisible()
    if baked_curve.IsValid():
        # Keep the curve as a canonical topology registry, not a second render
        # layer. Some Kit viewport configurations do not render two-point
        # BasisCurves consistently.
        UsdGeom.Imageable(baked_curve).MakeInvisible()
    curve_edge_indices = []
    for edge in _custom_json(visual_prim, "referenceCreaseEdgeKeys", []):
        if len(edge) != 2:
            continue
        curve_edge_indices.append(
            (vertex_index[str(edge[0])], vertex_index[str(edge[1])])
        )
    crease_mesh_sides = int(
        _attribute_value(baked_mesh, "creasePrismSides", 6)
        if baked_mesh.IsValid()
        else 6
    )
    crease_mesh_radius = float(
        _attribute_value(baked_mesh, "creasePrismRadiusM", 0.00026)
        if baked_mesh.IsValid()
        else 0.00026
    )
    # Cache the USD transform operations once. Re-discovering and allocating
    # them for all 76 crease cylinders on every visual update was a measurable
    # part of the old render-thread stall.
    crease_visuals = []
    for prim in crease_prims:
        source_ids = _custom_json(prim, "sourceVertexIds", [])
        if len(source_ids) != 2:
            continue
        xformable = UsdGeom.Xformable(prim)
        translate_op = None
        orient_op = None
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                orient_op = op
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
        if orient_op is None:
            orient_op = xformable.AddOrientOp()
        crease_visuals.append(
            (
                prim.GetAttribute("height"),
                translate_op,
                orient_op,
                vertex_index[str(source_ids[0])],
                vertex_index[str(source_ids[1])],
            )
        )
    # Keep the crease render in lockstep with the colored facets.  The mesh is
    # small (912 points), and updating it every display frame avoids a visible
    # one-frame lag when the physical drive changes direction.
    crease_mesh_update_period = 1
    crease_mesh_update_frame = 0

    def enforce_crease_visibility():
        """Keep the single active crease representation selected.

        The stable visual helper is also used by legacy controllers and can
        discover the authored cylinders again after a stage edit.  Enforcing
        visibility alongside the point update prevents those fallback lines
        from reappearing at their authored origin below the joint.
        """

        if crease_mesh is not None:
            UsdGeom.Imageable(baked_mesh).MakeVisible()
            for prim in crease_prims:
                UsdGeom.Imageable(prim).MakeInvisible()
        if baked_curve.IsValid():
            UsdGeom.Imageable(baked_curve).MakeInvisible()

    state_positions = None

    def anchor_targets(top_pose=None, bottom_pose=None):
        if top_pose is None:
            top_pose = stable._pose(TOP_BODY)
        if bottom_pose is None:
            bottom_pose = stable._pose(BOTTOM_BODY)
        top_position, top_quaternion = top_pose
        bottom_position, bottom_quaternion = bottom_pose
        targets = np.zeros_like(local_points)
        for index, vertex_id in enumerate(vertex_ids):
            if vertex_id in top_ids:
                targets[index] = stable._transform(
                    top_position,
                    top_quaternion,
                    local_points[index],
                )
            elif vertex_id in bottom_ids:
                targets[index] = stable._transform(
                    bottom_position,
                    bottom_quaternion,
                    local_points[index],
                )
        return targets

    def initial_positions(targets, top_pose=None, bottom_pose=None):
        if top_pose is None:
            top_pose = stable._pose(TOP_BODY)
        if bottom_pose is None:
            bottom_pose = stable._pose(BOTTOM_BODY)
        top_position, top_quaternion = top_pose
        bottom_position, bottom_quaternion = bottom_pose
        positions = np.zeros_like(local_points)
        for index, vertex_id in enumerate(vertex_ids):
            if vertex_id in top_ids or vertex_id in bottom_ids:
                positions[index] = targets[index]
                continue
            top_point = stable._transform(top_position, top_quaternion, local_points[index])
            bottom_point = stable._transform(bottom_position, bottom_quaternion, local_points[index])
            positions[index] = (np.asarray(top_point) + np.asarray(bottom_point)) * 0.5
        return positions

    def facet_targets(positions):
        fitted = []
        for indices in facet_indices:
            rest = local_points[indices]
            current = positions[indices]
            fitted.append((indices, _rigid_facet_fit(rest, current)))
        return fitted

    def facet_rigid_transforms(positions):
        return [
            _rigid_facet_transform(local_points[indices], positions[indices])
            for indices in facet_indices
        ]

    def average_rigid_facets(facet_transforms, alpha):
        """Interpolate rigid facet poses, then weld them at shared vertices.

        A vertex belongs to several source triangles. Each triangle is moved
        by an interpolated proper rotation and translation, then the predicted
        positions are averaged at that shared vertex. This preserves the
        origami/facet character during the display transition; directly
        interpolating vertex coordinates would shear every triangular facet.
        """

        accumulated = np.zeros_like(local_points)
        counts = np.zeros(len(local_points), dtype=float)
        for facet_index, indices in enumerate(facet_indices):
            first_rotation, first_translation = facet_transforms[0][facet_index]
            second_rotation, second_translation = facet_transforms[1][facet_index]
            blended_rotation = _orthonormalize_rotation(
                (1.0 - alpha) * first_rotation + alpha * second_rotation
            )
            blended_translation = (
                (1.0 - alpha) * first_translation
                + alpha * second_translation
            )
            predicted = local_points[indices] @ blended_rotation + blended_translation
            accumulated[indices] += predicted
            counts[indices] += 1.0

        positions = local_points.copy()
        covered = counts > 0.0
        positions[covered] = accumulated[covered] / counts[covered, None]
        return positions

    def rigid_display_endpoint(positions, targets):
        """Project a solver/display endpoint back onto rigid source facets."""

        # One fit/average pass is enough to remove ordinary animation shear,
        # but the shared-vertex weld can reintroduce a small error where three
        # or more facets meet. A few relaxed alternating projections keep the
        # facets rigid without replacing the authored crease topology with a
        # smoothed surface.
        projected = positions.copy()
        for _ in range(64):
            transforms = facet_rigid_transforms(projected)
            candidate = average_rigid_facets((transforms, transforms), 0.0)
            candidate[anchor_indices] = targets[anchor_indices]
            projected = 0.25 * projected + 0.75 * candidate
            projected[anchor_indices] = targets[anchor_indices]
        transforms = facet_rigid_transforms(projected)
        projected = average_rigid_facets((transforms, transforms), 0.0)
        projected[anchor_indices] = targets[anchor_indices]
        return projected, transforms

    def evaluate(
        x,
        reference,
        targets,
        fitted_facets,
        objective_power,
        facet_fit_weight,
        position_regularization,
        self_clearance_m,
        self_clearance_weight,
        midpoint_clearance_m,
        midpoint_clearance_weight,
        include_jacobian,
        base_positions=None,
    ):
        positions = (state_positions if base_positions is None else base_positions).copy()
        positions[free_indices] = x.reshape((-1, 3))
        positions[anchor_indices] = targets[anchor_indices]

        residuals = []
        rows = []
        exponent = objective_power * 0.5
        for edge_index, (first, second) in enumerate(edge_indices):
            delta = positions[second] - positions[first]
            length = max(float(np.linalg.norm(delta)), 1.0e-12)
            direction = delta / length
            strain = (length - edge_rest_lengths[edge_index]) / edge_rest_lengths[edge_index]
            magnitude = abs(strain)
            residuals.append(math.copysign(magnitude**exponent, strain))

            if include_jacobian:
                derivative = (
                    exponent
                    * max(magnitude, 1.0e-12) ** max(exponent - 1.0, 0.0)
                    / edge_rest_lengths[edge_index]
                )
                row = np.zeros(variable_count, dtype=float)
                first_offset = edge_first_offsets[edge_index]
                second_offset = edge_second_offsets[edge_index]
                if first_offset >= 0:
                    row[first_offset : first_offset + 3] -= derivative * direction
                if second_offset >= 0:
                    row[second_offset : second_offset + 3] += derivative * direction
                rows.append(row)

        if self_clearance_m > 0.0 and self_clearance_weight > 0.0:
            coefficient = math.sqrt(self_clearance_weight) / self_clearance_m
            for first, second in clearance_pairs:
                delta = positions[second] - positions[first]
                length = float(np.linalg.norm(delta))
                if length >= self_clearance_m:
                    continue
                if length < 1.0e-9:
                    delta = local_points[second] - local_points[first]
                    length = max(float(np.linalg.norm(delta)), 1.0e-9)
                direction = delta / length
                residuals.append(coefficient * (self_clearance_m - length))
                if include_jacobian:
                    row = np.zeros(variable_count, dtype=float)
                    first_offset = variable_offsets.get(int(first), -1)
                    second_offset = variable_offsets.get(int(second), -1)
                    if first_offset >= 0:
                        row[first_offset : first_offset + 3] += coefficient * direction
                    if second_offset >= 0:
                        row[second_offset : second_offset + 3] -= coefficient * direction
                    rows.append(row)

        if midpoint_clearance_m > 0.0 and midpoint_clearance_weight > 0.0:
            coefficient = math.sqrt(midpoint_clearance_weight) / midpoint_clearance_m
            for first_edge, second_edge in midpoint_clearance_pairs:
                first_start, first_end = edge_indices[first_edge]
                second_start, second_end = edge_indices[second_edge]
                first_midpoint = (positions[first_start] + positions[first_end]) * 0.5
                second_midpoint = (positions[second_start] + positions[second_end]) * 0.5
                delta = second_midpoint - first_midpoint
                length = float(np.linalg.norm(delta))
                if length >= midpoint_clearance_m:
                    continue
                if length < 1.0e-9:
                    delta = (
                        (local_points[second_start] + local_points[second_end])
                        - (local_points[first_start] + local_points[first_end])
                    ) * 0.5
                    length = max(float(np.linalg.norm(delta)), 1.0e-9)
                direction = delta / length
                residuals.append(coefficient * (midpoint_clearance_m - length))
                if include_jacobian:
                    row = np.zeros(variable_count, dtype=float)
                    for index, sign in (
                        (first_start, 0.5),
                        (first_end, 0.5),
                        (second_start, -0.5),
                        (second_end, -0.5),
                    ):
                        offset = variable_offsets.get(int(index), -1)
                        if offset >= 0:
                            row[offset : offset + 3] += sign * coefficient * direction
                    rows.append(row)

        if facet_fit_weight > 0.0:
            # Scale the local rigid-facet residual by a typical facet radius so
            # its dimensionless weight can be compared with edge strain.
            for indices, fitted in fitted_facets:
                rest = local_points[indices]
                radius = float(np.mean(np.linalg.norm(rest - np.mean(rest, axis=0), axis=1)))
                coefficient = math.sqrt(facet_fit_weight) / max(radius, 1.0e-6)
                for local_index, vertex_index_value in enumerate(indices):
                    offset = variable_offsets.get(int(vertex_index_value), -1)
                    if offset < 0:
                        continue
                    residual = coefficient * (
                        positions[vertex_index_value] - fitted[local_index]
                    )
                    residuals.extend(residual.tolist())
                    if include_jacobian:
                        identity = np.eye(3, variable_count, offset, dtype=float)
                        rows.extend((coefficient * identity).tolist())

        if position_regularization > 0.0:
            coefficient = math.sqrt(position_regularization) / 0.02
            residuals.extend((coefficient * (x - reference)).tolist())
            if include_jacobian:
                rows.extend((np.eye(variable_count, dtype=float) * coefficient).tolist())

        if include_jacobian:
            return np.asarray(residuals, dtype=float), np.asarray(rows, dtype=float)
        return np.asarray(residuals, dtype=float)

    pending_future = None
    last_submitted_targets = None
    last_submit_time = 0.0
    display_positions = None
    transition_from = None
    transition_to = None
    transition_from_facets = None
    transition_to_facets = None
    transition_start = 0.0
    transition_duration = 1.0 / 10.0
    solver_count = 0

    def solver_settings(controller):
        return {
            "iterations": max(1, min(8, int(_attribute_value(controller, "constraintSolverIterations", 3)))),
            "objective_power": max(2.0, min(6.0, float(_attribute_value(controller, "constraintObjectivePower", 4.0)))),
            "facet_fit_weight": max(0.0, min(10.0, float(_attribute_value(controller, "constraintFacetFitWeight", 0.05)))),
            "line_search_steps": max(1, min(8, int(_attribute_value(controller, "constraintLineSearchSteps", 6)))),
            "maximum_step": max(1.0e-5, float(_attribute_value(controller, "constraintMaxStepM", 0.0025))),
            "position_regularization": max(0.0, float(_attribute_value(controller, "constraintPositionRegularization", 1.0e-6))),
            "self_clearance_m": max(0.0, min(0.01, float(_attribute_value(controller, "constraintSelfClearanceM", 0.0012)))),
            "self_clearance_weight": max(0.0, min(100.0, float(_attribute_value(controller, "constraintSelfClearanceWeight", 4.0)))),
            "midpoint_clearance_m": max(0.0, min(0.01, float(_attribute_value(controller, "constraintMidpointClearanceM", 0.0010)))),
            "midpoint_clearance_weight": max(0.0, min(100.0, float(_attribute_value(controller, "constraintMidpointClearanceWeight", 1.0)))),
        }

    def solve_positions(base_positions, targets, settings):
        """Solve numeric facet positions away from the Kit/render thread."""

        started = time.perf_counter()
        solved = base_positions.copy()
        solved[anchor_indices] = targets[anchor_indices]
        reference = solved[free_indices].reshape(-1).copy()
        solution = reference.copy()
        used_iterations = 0

        for iteration in range(settings["iterations"]):
            trial_positions = solved.copy()
            trial_positions[free_indices] = solution.reshape((-1, 3))
            trial_positions[anchor_indices] = targets[anchor_indices]
            fitted_facets = facet_targets(trial_positions)
            residuals, jacobian = evaluate(
                solution,
                reference,
                targets,
                fitted_facets,
                settings["objective_power"],
                settings["facet_fit_weight"],
                settings["position_regularization"],
                settings["self_clearance_m"],
                settings["self_clearance_weight"],
                settings["midpoint_clearance_m"],
                settings["midpoint_clearance_weight"],
                True,
                base_positions=solved,
            )
            cost = float(residuals @ residuals)
            accepted = False
            damping = 1.0e-3
            for _ in range(7):
                system = jacobian.T @ jacobian + damping * np.eye(variable_count, dtype=float)
                gradient = -(jacobian.T @ residuals)
                try:
                    step = np.linalg.solve(system, gradient)
                except np.linalg.LinAlgError:
                    step = np.linalg.lstsq(system, gradient, rcond=None)[0]
                step = _bounded_step(step, settings["maximum_step"])
                if float(np.linalg.norm(step)) < 1.0e-9:
                    break

                for line_step in range(settings["line_search_steps"]):
                    candidate = solution + (0.5**line_step) * step
                    candidate_residuals = evaluate(
                        candidate,
                        reference,
                        targets,
                        fitted_facets,
                        settings["objective_power"],
                        settings["facet_fit_weight"],
                        settings["position_regularization"],
                        settings["self_clearance_m"],
                        settings["self_clearance_weight"],
                        settings["midpoint_clearance_m"],
                        settings["midpoint_clearance_weight"],
                        False,
                        base_positions=solved,
                    )
                    if float(candidate_residuals @ candidate_residuals) < cost:
                        solution = candidate
                        accepted = True
                        break
                if accepted:
                    break
                damping *= 10.0

            if not accepted:
                break
            used_iterations = iteration + 1

        solved[free_indices] = solution.reshape((-1, 3))
        solved[anchor_indices] = targets[anchor_indices]
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        return solved, used_iterations, elapsed_ms, targets.copy()

    def measure(positions, targets, settings):
        edge_deltas = positions[edge_indices[:, 1]] - positions[edge_indices[:, 0]]
        edge_lengths = np.linalg.norm(edge_deltas, axis=1)
        edge_errors = np.abs(edge_lengths - edge_rest_lengths)
        edge_strains = edge_errors / np.maximum(edge_rest_lengths, 1.0e-9)
        clearance_violations = [
            max(0.0, settings["self_clearance_m"] - _distance(positions[first], positions[second]))
            for first, second in clearance_pairs
        ]
        clearance_violations.extend(
            max(
                0.0,
                settings["midpoint_clearance_m"]
                - _distance(
                    (positions[edge_indices[first_edge][0]] + positions[edge_indices[first_edge][1]]) * 0.5,
                    (positions[edge_indices[second_edge][0]] + positions[edge_indices[second_edge][1]]) * 0.5,
                ),
            )
            for first_edge, second_edge in midpoint_clearance_pairs
        )
        facet_distortions = []
        facet_fit_errors = []
        for indices in facet_indices:
            local_errors = []
            for first_local, second_local in ((0, 1), (1, 2), (2, 0)):
                first = int(indices[first_local])
                second = int(indices[second_local])
                rest_length = _distance(local_points[first], local_points[second])
                local_errors.append(
                    abs(_distance(positions[first], positions[second]) - rest_length)
                    / max(rest_length, 1.0e-9)
                )
            facet_distortions.append(max(local_errors))
            fitted = _rigid_facet_fit(local_points[indices], positions[indices])
            facet_fit_errors.append(
                max(
                    _distance(positions[int(index)], fitted[offset])
                    for offset, index in enumerate(indices)
                )
            )
        anchor_errors = [
            _distance(positions[index], targets[index]) for index in anchor_indices
        ]
        return {
            "max_edge_error": float(np.max(edge_errors)),
            "mean_edge_error": float(np.mean(edge_errors)),
            "max_edge_strain": float(np.max(edge_strains)),
            "mean_edge_strain": float(np.mean(edge_strains)),
            "max_facet_distortion": float(max(facet_distortions)),
            "mean_facet_distortion": float(sum(facet_distortions) / len(facet_distortions)),
            "max_facet_fit_error": float(max(facet_fit_errors)),
            "mean_facet_fit_error": float(sum(facet_fit_errors) / len(facet_fit_errors)),
            "max_anchor_error": float(max(anchor_errors) if anchor_errors else 0.0),
            "max_clearance_violation": float(max(clearance_violations) if clearance_violations else 0.0),
            "mean_clearance_violation": float(sum(clearance_violations) / len(clearance_violations))
            if clearance_violations
            else 0.0,
        }

    def set_controller_attr(controller, name, value):
        attribute = controller.GetAttribute(name)
        if attribute and attribute.IsValid():
            attribute.Set(value)

    def apply_metrics(controller, values, used_iterations, solve_ms):
        set_controller_attr(controller, "constraintMaxEdgeErrorM", values["max_edge_error"])
        set_controller_attr(controller, "constraintMeanEdgeErrorM", values["mean_edge_error"])
        set_controller_attr(controller, "constraintMaxEdgeStrain", values["max_edge_strain"])
        set_controller_attr(controller, "constraintMeanEdgeStrain", values["mean_edge_strain"])
        set_controller_attr(controller, "constraintMaxFacetEdgeDistortion", values["max_facet_distortion"])
        set_controller_attr(controller, "constraintMeanFacetEdgeDistortion", values["mean_facet_distortion"])
        set_controller_attr(controller, "constraintMaxFacetFitErrorM", values["max_facet_fit_error"])
        set_controller_attr(controller, "constraintMeanFacetFitErrorM", values["mean_facet_fit_error"])
        set_controller_attr(controller, "constraintMaxAnchorErrorM", values["max_anchor_error"])
        set_controller_attr(controller, "constraintMaxSelfClearanceViolationM", values["max_clearance_violation"])
        set_controller_attr(controller, "constraintMeanSelfClearanceViolationM", values["mean_clearance_violation"])
        set_controller_attr(controller, "constraintSolverIterationsUsed", int(used_iterations))
        set_controller_attr(controller, "constraintLastSolveMs", float(solve_ms))

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

    def apply_visual_positions(positions):
        nonlocal crease_mesh_update_frame
        enforce_crease_visibility()
        for mesh, indices in mesh_prims:
            mesh.GetPointsAttr().Set(
                Vt.Vec3fArray([tuple(float(value) for value in positions[index]) for index in indices])
            )
        update_crease_mesh = (
            crease_mesh is not None
            and curve_edge_indices
            and crease_mesh_update_frame % crease_mesh_update_period == 0
        )
        if update_crease_mesh:
            crease_mesh.GetPointsAttr().Set(Vt.Vec3fArray(crease_mesh_points(positions)))
        else:
            for height_attribute, translate_op, orient_op, first, second in (
                crease_visuals if crease_mesh is None else []
            ):
                start = positions[first]
                end = positions[second]
                delta = end - start
                length = float(np.linalg.norm(delta))
                if length < 1.0e-9:
                    continue
                midpoint = (start + end) * 0.5
                direction = Gf.Vec3d(*(float(value) / length for value in delta))
                rotation = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), direction).GetQuat()
                orient_op.Set(Gf.Quatf(rotation.GetReal(), *rotation.GetImaginary()))
                translate_op.Set(Gf.Vec3d(*(float(value) for value in midpoint)))
                height_attribute.Set(length)
        crease_mesh_update_frame += 1

    def update():
        nonlocal state_positions, pending_future, last_submitted_targets, last_submit_time
        nonlocal display_positions, transition_from, transition_to
        nonlocal transition_from_facets, transition_to_facets, transition_start
        nonlocal transition_duration, solver_count

        controller = stage.GetPrimAtPath(CONTROLLER)
        top_pose = stable._pose(TOP_BODY)
        bottom_pose = stable._pose(BOTTOM_BODY)
        targets = anchor_targets(top_pose, bottom_pose)
        neutral_anchor_error = max(
            [
                _distance(neutral_positions[index], targets[index])
                for index in anchor_indices
            ]
            or [0.0]
        )
        neutral_snap_tolerance = float(
            _attribute_value(controller, "neutralSnapToleranceM", 5.0e-5)
        )
        at_exact_neutral = neutral_anchor_error <= neutral_snap_tolerance
        if state_positions is None:
            if at_exact_neutral:
                state_positions = neutral_positions.copy()
                display_positions = neutral_positions.copy()
                initial_facets = facet_rigid_transforms(neutral_positions)
            else:
                state_positions = initial_positions(targets, top_pose, bottom_pose)
                state_positions[anchor_indices] = targets[anchor_indices]
                display_positions, initial_facets = rigid_display_endpoint(
                    state_positions,
                    targets,
                )
            transition_from = display_positions.copy()
            transition_to = display_positions.copy()
            transition_from_facets = initial_facets
            transition_to_facets = initial_facets
            apply_visual_positions(display_positions)

        if at_exact_neutral:
            # A neutral command is a hard kinematic reference, not another
            # optimization target. Cancel stale work from the previous pose,
            # restore the exact authored source coordinates, and keep the
            # welded crease mesh on that same coordinate map.
            if pending_future is not None and not pending_future.done():
                pending_future.cancel()
            pending_future = None
            last_submitted_targets = targets.copy()
            last_submit_time = time.perf_counter()
            state_positions = neutral_positions.copy()
            display_positions = neutral_positions.copy()
            neutral_facets = facet_rigid_transforms(neutral_positions)
            transition_from = display_positions.copy()
            transition_to = display_positions.copy()
            transition_from_facets = neutral_facets
            transition_to_facets = neutral_facets
            settings = solver_settings(controller)
            apply_metrics(controller, measure(neutral_positions, targets, settings), 0, 0.0)
            apply_visual_positions(display_positions)
            set_controller_attr(controller, "constraintSolverPending", False)
            return

        if pending_future is not None and pending_future.done():
            try:
                state_positions, used_iterations, solve_ms, solved_targets = pending_future.result()
                settings = solver_settings(controller)
                apply_metrics(
                    controller,
                    measure(state_positions, solved_targets, settings),
                    used_iterations,
                    solve_ms,
                )
                transition_from, transition_from_facets = rigid_display_endpoint(
                    display_positions
                    if display_positions is not None
                    else state_positions,
                    targets,
                )
                transition_to, transition_to_facets = rigid_display_endpoint(
                    state_positions,
                    solved_targets,
                )
                transition_start = time.perf_counter()
                solver_count += 1
                set_controller_attr(controller, "constraintSolverCount", int(solver_count))
            except Exception as error:
                print(f"facet crease async solve failed: {error}")
                last_submitted_targets = None
            pending_future = None

        settings = solver_settings(controller)
        solve_rate_hz = max(
            1.0,
            min(60.0, float(_attribute_value(controller, "constraintSolveRateHz", 10.0))),
        )
        transition_duration = 1.0 / solve_rate_hz
        target_epsilon = max(
            1.0e-9,
            float(_attribute_value(controller, "constraintTargetEpsilonM", 1.0e-7)),
        )
        target_changed = (
            last_submitted_targets is None
            or float(np.max(np.abs(targets - last_submitted_targets))) > target_epsilon
        )
        now = time.perf_counter()
        if (
            pending_future is None
            and target_changed
            and now - last_submit_time >= transition_duration
        ):
            pending_future = _SOLVER_EXECUTOR.submit(
                solve_positions,
                state_positions.copy(),
                targets.copy(),
                settings,
            )
            last_submitted_targets = targets.copy()
            last_submit_time = now

        if (
            transition_to is not None
            and transition_from_facets is not None
            and transition_to_facets is not None
        ):
            alpha = min(
                1.0,
                max(0.0, (now - transition_start) / max(transition_duration, 1.0e-6)),
            )
            display_positions = average_rigid_facets(
                (transition_from_facets, transition_to_facets),
                alpha,
            )
            # The shell interface remains coincident with the actual physical
            # plates even while the interior facet result is being blended.
            display_positions[anchor_indices] = targets[anchor_indices]
            apply_visual_positions(display_positions)

        set_controller_attr(
            controller,
            "constraintSolverPending",
            bool(pending_future is not None and not pending_future.done()),
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
            # Usd.Stage Python wrappers are not guaranteed to compare by
            # identity across calls. Comparing the root-layer key prevents a
            # new solver/updater from being spawned every render frame.
            stage_key = str(stage.GetRootLayer().realPath or stage.GetRootLayer().identifier)
            if stage_key != last_stage_key or updater is None:
                updater = _make_updater(stage)
                last_stage_key = stage_key
            if updater is not None:
                updater()
        except Exception as error:
            print(f"facet crease controller paused: {error}")
            await omni.kit.app.get_app().next_update_async()


def start():
    global _controller_task
    if _controller_task is None or _controller_task.done():
        _controller_task = asyncio.ensure_future(_run())
    return _controller_task

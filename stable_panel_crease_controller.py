"""Drive the continuous source-panel visual shell from the physical knee bodies."""

import asyncio
import json
import math

import omni.kit.app
import omni.usd
from isaacsim.core.experimental.utils import xform
from pxr import Gf, UsdGeom, Vt


TOP_BODY = "/World/PanelCreaseLeg/OriginalJointShell/TopShell"
BOTTOM_BODY = "/World/PanelCreaseLeg/OriginalJointShell/BottomShell"
VISUAL_ROOT = "/World/PanelCreaseLeg/OriginalJointVisual"

_controller_task = None


def _array(value):
    if hasattr(value, "numpy"):
        value = value.numpy()
    return [float(item) for item in value.reshape(-1)]


def _pose(path):
    position, quaternion = xform.get_world_pose(path)
    return _array(position)[:3], _array(quaternion)[:4]


def _rotate(quaternion, point):
    """Rotate a 3-vector by a wxyz quaternion without a matrix allocation."""

    w, x, y, z = quaternion
    px, py, pz = point
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    return (
        px + w * tx + (y * tz - z * ty),
        py + w * ty + (z * tx - x * tz),
        pz + w * tz + (x * ty - y * tx),
    )


def _transform(position, quaternion, point):
    rotated = _rotate(quaternion, point)
    return tuple(position[index] + rotated[index] for index in range(3))


def _source_to_joint(position_m):
    x, y, z = [float(value) for value in position_m]
    return (x * 0.12, -z * 0.12, y * 0.12)


def _custom_json(prim, key, default):
    value = prim.GetCustomDataByKey(key)
    if value is None:
        return default
    if isinstance(value, str):
        return json.loads(value)
    return value


def _set_visual_transform(prim, midpoint, delta):
    length = math.sqrt(sum(float(value) * float(value) for value in delta))
    if length < 1.0e-9:
        return
    direction = Gf.Vec3f(*(float(value) / length for value in delta))
    rotation = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), Gf.Vec3d(*direction)).GetQuat()
    quat = Gf.Quatf(rotation.GetReal(), *rotation.GetImaginary())
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
    translate_op.Set(Gf.Vec3d(*midpoint))
    orient_op.Set(quat)


def _make_updater(stage):
    visual_prim = stage.GetPrimAtPath(VISUAL_ROOT)
    embedded_vertices = visual_prim.GetCustomDataByKey("sourceVertexPositionsM")
    if embedded_vertices:
        vertices = json.loads(embedded_vertices) if isinstance(embedded_vertices, str) else embedded_vertices
    else:
        topology_prim = stage.GetPrimAtPath("/World/PanelCreaseLeg")
        topology_path = topology_prim.GetCustomDataByKey("sourceTopology")
        if not topology_path:
            return None
        with open(str(topology_path), "r", encoding="utf-8") as handle:
            topology = json.load(handle)
        vertices = {item["id"]: item["positionM"] for item in topology["geometry"]["vertices"]}
    local_points = {vertex_id: _source_to_joint(position) for vertex_id, position in vertices.items()}
    source_y = {vertex_id: float(position[1]) for vertex_id, position in vertices.items()}

    mesh_prims = []
    crease_prims = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(VISUAL_ROOT):
            continue
        if prim.GetTypeName() == "Mesh" and prim.GetCustomDataByKey("sourceVertexIds"):
            mesh_prims.append(prim)
        if prim.GetTypeName() == "Cylinder" and prim.GetName().startswith("Crease_"):
            crease_prims.append(prim)

    def world_points():
        top_position, top_quaternion = _pose(TOP_BODY)
        bottom_position, bottom_quaternion = _pose(BOTTOM_BODY)
        result = {}
        for vertex_id, point in local_points.items():
            if source_y[vertex_id] > 1.0e-8:
                result[vertex_id] = _transform(top_position, top_quaternion, point)
            elif source_y[vertex_id] < -1.0e-8:
                result[vertex_id] = _transform(bottom_position, bottom_quaternion, point)
            else:
                top_point = _transform(top_position, top_quaternion, point)
                bottom_point = _transform(bottom_position, bottom_quaternion, point)
                result[vertex_id] = tuple((top_point[index] + bottom_point[index]) * 0.5 for index in range(3))
        return result

    def update():
        points_by_id = world_points()
        for prim in mesh_prims:
            vertex_ids = _custom_json(prim, "sourceVertexIds", [])
            points = [points_by_id[vertex_id] for vertex_id in vertex_ids]
            UsdGeom.Mesh(prim).GetPointsAttr().Set(Vt.Vec3fArray(points))
        for prim in crease_prims:
            vertex_ids = _custom_json(prim, "sourceVertexIds", [])
            if len(vertex_ids) != 2:
                continue
            start = points_by_id[vertex_ids[0]]
            end = points_by_id[vertex_ids[1]]
            delta = tuple(end[index] - start[index] for index in range(3))
            midpoint = tuple((start[index] + end[index]) * 0.5 for index in range(3))
            prim.GetAttribute("height").Set(math.sqrt(sum(value * value for value in delta)))
            _set_visual_transform(prim, midpoint, delta)

    return update


async def _run():
    while True:
        await omni.kit.app.get_app().next_update_async()
        try:
            stage = omni.usd.get_context().get_stage()
            if stage is None or not stage.GetPrimAtPath(TOP_BODY).IsValid():
                continue
            updater = _make_updater(stage)
            if updater is not None:
                updater()
                while True:
                    await omni.kit.app.get_app().next_update_async()
                    current_stage = omni.usd.get_context().get_stage()
                    if current_stage is None or not current_stage.GetPrimAtPath(TOP_BODY).IsValid():
                        break
                    updater()
        except Exception as error:
            print(f"stable panel visual controller paused: {error}")
            await omni.kit.app.get_app().next_update_async()


def start():
    global _controller_task
    if _controller_task is None or _controller_task.done():
        _controller_task = asyncio.ensure_future(_run())
    return _controller_task

"""Smoke-test the stable original-panel knee in the live Isaac Sim server."""

import math

import numpy as np
import omni.usd
import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.experimental.utils import xform


def _as_array(value):
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=float).reshape(-1)


def _fmt_pose(path):
    position, quaternion = xform.get_world_pose(path)
    p = _as_array(position)[:3]
    q = _as_array(quaternion)[:4]
    angle_y = math.degrees(2.0 * math.atan2(float(q[2]), float(q[0])))
    return f"{path}: p=({p[0]:.6f},{p[1]:.6f},{p[2]:.6f}) q=({q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}) y={angle_y:.3f}deg"


stage = omni.usd.get_context().get_stage()
knee_path = "/World/PanelCreaseLeg/Physics/KneeActuator"
knee = stage.GetPrimAtPath(knee_path)
if not knee.IsValid():
    raise RuntimeError(f"Missing knee actuator: {knee_path}")

target_attr = knee.GetAttribute("drive:angular:physics:targetPosition")
target_before = float(target_attr.Get())
target_attr.Set(0.0)
app_utils.pause()
app_utils.play()
await app_utils.update_app_async(steps=20)
print("neutral")
for path in [
    "/World/PanelCreaseLeg/OriginalJointShell/TopShell",
    "/World/PanelCreaseLeg/OriginalJointShell/BottomShell",
    "/World/PanelCreaseLeg/Shank",
]:
    print(_fmt_pose(path))

target_attr.Set(target_before)
await app_utils.update_app_async(steps=250)
app_utils.pause()
print(f"target_deg={target_before:.3f}")
print("folded")
for path in [
    "/World/PanelCreaseLeg/OriginalJointShell/TopShell",
    "/World/PanelCreaseLeg/OriginalJointShell/BottomShell",
    "/World/PanelCreaseLeg/Shank",
]:
    print(_fmt_pose(path))

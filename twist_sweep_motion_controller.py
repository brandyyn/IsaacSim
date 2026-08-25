"""Drive the physical twist joint through its complete authored limit range.

This is deliberately separate from the coupled fold controller.  The
coupled-fold demo is useful for the measured knee-fold response, while this
driver keeps the model straight and exercises the independent torsional axis.
The sweep reads the limits from the USD joint itself, so it cannot command
past the authored PhysX stops when the stage is recalibrated.
"""

from __future__ import annotations

import asyncio
import math
import time

import omni.kit.app
import omni.usd


CONTROLLER = "/World/PanelCreaseLeg/Controller"
PHYSICS = "/World/PanelCreaseLeg/Physics"
DRIVER_MODE = "twist_sweep"

_motion_task = None
_started_at = time.monotonic()


def _attribute_value(prim, name, default):
    attribute = prim.GetAttribute(name)
    if not attribute:
        return default
    value = attribute.Get()
    return default if value is None else value


def _joint_limit(stage, joint_name, attribute_name, default):
    joint = stage.GetPrimAtPath(f"{PHYSICS}/{joint_name}")
    if not joint.IsValid():
        raise RuntimeError(f"missing twist-sweep joint {joint_name}")
    value = joint.GetAttribute(attribute_name).Get()
    return float(default if value is None else value)


def _clamp(value, lower, upper):
    return max(float(lower), min(float(upper), float(value)))


def _set_target(stage, joint_name, value):
    joint = stage.GetPrimAtPath(f"{PHYSICS}/{joint_name}")
    if not joint.IsValid():
        raise RuntimeError(f"missing twist-sweep joint {joint_name}")
    joint.GetAttribute("drive:angular:physics:targetPosition").Set(float(value))


def apply_twist_command(stage, twist_deg, fold_deg=0.0, lateral_deg=0.0):
    """Set a bounded three-axis pose with twist as the independent command.

    USD angular drive targets and limits use degrees in this project.  The
    values are clamped against the live joint attributes rather than against
    duplicated controller constants.
    """

    controller = stage.GetPrimAtPath(CONTROLLER)
    twist_lower = _joint_limit(stage, "TwistJoint", "physics:lowerLimit", -30.0)
    twist_upper = _joint_limit(stage, "TwistJoint", "physics:upperLimit", 30.0)
    lateral_lower = _joint_limit(stage, "LateralBendJoint", "physics:lowerLimit", -22.0)
    lateral_upper = _joint_limit(stage, "LateralBendJoint", "physics:upperLimit", 22.0)
    fold_lower = _joint_limit(stage, "MainKneeBendJoint", "physics:lowerLimit", -1.5)
    fold_upper = _joint_limit(stage, "MainKneeBendJoint", "physics:upperLimit", 127.0)

    targets = {
        "lateral": _clamp(lateral_deg, lateral_lower, lateral_upper),
        "twist": _clamp(twist_deg, twist_lower, twist_upper),
        "fold": _clamp(fold_deg, fold_lower, fold_upper),
    }
    _set_target(stage, "LateralBendJoint", targets["lateral"])
    _set_target(stage, "TwistJoint", targets["twist"])
    _set_target(stage, "MainKneeBendJoint", targets["fold"])

    if controller.IsValid():
        controller.GetAttribute("twistCommandDeg").Set(float(targets["twist"]))
        controller.GetAttribute("twistSweepLowerDeg").Set(float(twist_lower))
        controller.GetAttribute("twistSweepUpperDeg").Set(float(twist_upper))
        controller.GetAttribute("coupledLateralTargetDeg").Set(float(targets["lateral"]))
        controller.GetAttribute("coupledTwistTargetDeg").Set(float(targets["twist"]))
        controller.GetAttribute("coupledKneeTargetDeg").Set(float(targets["fold"]))
    return targets


async def _run():
    global _started_at
    while True:
        await omni.kit.app.get_app().next_update_async()
        try:
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                continue
            controller = stage.GetPrimAtPath(CONTROLLER)
            if not controller.IsValid():
                continue
            driver = _attribute_value(controller, "motionDriver", "")
            if driver != DRIVER_MODE:
                continue
            enabled = controller.GetAttribute("motionDemoEnabled")
            if enabled and enabled.Get() is False:
                continue

            lower = _joint_limit(stage, "TwistJoint", "physics:lowerLimit", -30.0)
            upper = _joint_limit(stage, "TwistJoint", "physics:upperLimit", 30.0)
            if upper < lower:
                raise RuntimeError("TwistJoint upper limit is below its lower limit")
            center = 0.5 * (lower + upper)
            half_range = 0.5 * (upper - lower)
            requested_amplitude = abs(
                float(_attribute_value(controller, "twistSweepAmplitudeDeg", half_range))
            )
            amplitude = min(requested_amplitude, half_range)
            configured_center = float(
                _attribute_value(controller, "twistSweepCenterDeg", center)
            )
            center = _clamp(configured_center, lower + amplitude, upper - amplitude)
            rate_hz = max(
                0.0,
                float(_attribute_value(controller, "twistSweepRateHz", 0.05)),
            )
            phase = 2.0 * math.pi * rate_hz * (time.monotonic() - _started_at)
            twist = center + amplitude * math.sin(phase)
            fold = float(_attribute_value(controller, "twistSweepFoldDeg", 0.0))
            lateral = float(_attribute_value(controller, "twistSweepLateralDeg", 0.0))
            apply_twist_command(stage, twist, fold_deg=fold, lateral_deg=lateral)
        except Exception as error:
            print(f"twist sweep motion controller paused: {error}")
            await omni.kit.app.get_app().next_update_async()


def start():
    global _motion_task, _started_at
    _started_at = time.monotonic()
    if _motion_task is None or _motion_task.done():
        _motion_task = asyncio.ensure_future(_run())
    return _motion_task

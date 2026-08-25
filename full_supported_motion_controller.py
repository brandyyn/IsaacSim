"""Run a neutral-first, full supported-range motion envelope.

The first half visits the authored lateral, twist, and geometry-safe fold
stops from neutral. The second half adds a bounded combined pass: lateral and
twist excursions are blended with the embedded measured/provisional fold
response and scaled back near the fold peak. This exposes more of the joint's
coupled workspace without commanding an uncalibrated simultaneous hard-stop
corner. It is a repeatable mechanical envelope test, not a claim that the
unmeasured extreme poses are already FEA-calibrated.
"""

from __future__ import annotations

import asyncio
import math
import time

import omni.kit.app
import omni.usd

import coupled_fold_motion_controller as coupled


CONTROLLER = "/World/PanelCreaseLeg/Controller"
PHYSICS = "/World/PanelCreaseLeg/Physics"
DRIVER_MODE = "supported_full_envelope"

_motion_task = None
_started_at = time.monotonic()


def _attribute_value(prim, name, default):
    attribute = prim.GetAttribute(name)
    if not attribute:
        return default
    value = attribute.Get()
    return default if value is None else value


def _joint_limits(stage, joint_name, default_lower, default_upper):
    joint = stage.GetPrimAtPath(f"{PHYSICS}/{joint_name}")
    if not joint.IsValid():
        raise RuntimeError(f"missing full-motion joint {joint_name}")
    lower = joint.GetAttribute("physics:lowerLimit").Get()
    upper = joint.GetAttribute("physics:upperLimit").Get()
    return float(default_lower if lower is None else lower), float(
        default_upper if upper is None else upper
    )


def _clamp(value, lower, upper):
    return max(float(lower), min(float(upper), float(value)))


def _smoothstep(value):
    # Quintic smootherstep keeps both velocity and acceleration continuous at
    # each authored stop.  The old cubic easing reached zero velocity but
    # still changed acceleration abruptly, which showed up as a visible kick
    # when the showcase reversed direction.
    t = _clamp(value, 0.0, 1.0)
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def _set_target(stage, joint_name, value):
    joint = stage.GetPrimAtPath(f"{PHYSICS}/{joint_name}")
    if not joint.IsValid():
        raise RuntimeError(f"missing full-motion joint {joint_name}")
    joint.GetAttribute("drive:angular:physics:targetPosition").Set(float(value))


def _fold_limit(stage):
    _, physical_upper = _joint_limits(stage, "MainKneeBendJoint", -1.5, 127.0)
    root = stage.GetPrimAtPath("/World/PanelCreaseLeg")
    geometry_limit = root.GetCustomDataByKey("geometryValidFoldLimitDeg")
    safe_upper = float(geometry_limit if geometry_limit is not None else 60.0)
    return max(0.0, min(physical_upper, safe_upper))


def _neutral():
    return {"fold": 0.0, "lateral": 0.0, "twist": 0.0}


def _limits(stage):
    lateral = _joint_limits(stage, "LateralBendJoint", -22.0, 22.0)
    twist = _joint_limits(stage, "TwistJoint", -30.0, 30.0)
    return lateral, twist, _fold_limit(stage)


def _sequential_waypoints(stage):
    (lateral_lower, lateral_upper), (twist_lower, twist_upper), fold_upper = _limits(stage)
    # Every extreme is reached from (and returned to) neutral. This keeps the
    # full-stop validation useful without forcing the shell through a
    # simultaneous lateral/twist/fold corner that is not FEA-calibrated.
    return [
        (0.00, _neutral()),
        (0.08, {"fold": 0.0, "lateral": lateral_upper, "twist": 0.0}),
        (0.16, _neutral()),
        (0.24, {"fold": 0.0, "lateral": 0.0, "twist": twist_upper}),
        (0.32, _neutral()),
        (0.40, {"fold": fold_upper, "lateral": 0.0, "twist": 0.0}),
        (0.48, _neutral()),
        (0.56, {"fold": 0.0, "lateral": lateral_lower, "twist": 0.0}),
        (0.64, _neutral()),
        (0.72, {"fold": 0.0, "lateral": 0.0, "twist": twist_lower}),
        (0.80, _neutral()),
        (0.88, {"fold": fold_upper, "lateral": 0.0, "twist": 0.0}),
        (1.00, _neutral()),
    ]


def _profiled_target(stage, fold, extra_lateral=0.0, extra_twist=0.0):
    (lateral_lower, lateral_upper), (twist_lower, twist_upper), fold_upper = _limits(stage)
    fold = _clamp(fold, 0.0, fold_upper)
    measured = coupled.profile_targets(stage, fold)
    return {
        "fold": fold,
        "lateral": _clamp(measured.get("lateral", 0.0) + extra_lateral, lateral_lower, lateral_upper),
        "twist": _clamp(measured.get("twist", 0.0) + extra_twist, twist_lower, twist_upper),
    }


def _combined_waypoints(stage):
    (lateral_lower, lateral_upper), (twist_lower, twist_upper), fold_upper = _limits(stage)
    controller = stage.GetPrimAtPath(CONTROLLER)
    scale = _clamp(
        float(_attribute_value(controller, "fullMotionCombinedScale", 0.75)),
        0.0,
        1.0,
    )
    lateral_span = scale * max(abs(lateral_lower), abs(lateral_upper))
    twist_span = scale * max(abs(twist_lower), abs(twist_upper))
    half_fold = min(30.0, 0.5 * fold_upper)
    positive = _profiled_target(stage, half_fold, lateral_span, twist_span)
    fold_peak = _profiled_target(stage, fold_upper)
    negative = _profiled_target(stage, half_fold, -lateral_span, -twist_span)
    return [
        (0.00, _neutral()),
        (0.25, positive),
        (0.50, fold_peak),
        (0.75, negative),
        (1.00, _neutral()),
    ]


def _waypoints(stage):
    controller = stage.GetPrimAtPath(CONTROLLER)
    pattern = str(_attribute_value(controller, "fullMotionPattern", "hybrid"))
    if pattern == "sequential":
        return _sequential_waypoints(stage)
    if pattern == "combined":
        return _combined_waypoints(stage)
    # Hybrid keeps the individual stop checks and adds a coupled combined
    # pass during the second half of the same 360-degree phase cycle.
    sequential = _sequential_waypoints(stage)
    combined = _combined_waypoints(stage)
    hybrid = [(phase * 0.5, values) for phase, values in sequential[:-1]]
    hybrid.extend((0.5 + phase * 0.5, values) for phase, values in combined[1:])
    return hybrid


def _showcase_target(stage, phase):
    """Return a smooth, geometry-safe showcase command.

    The showcase is deliberately not a synthetic Lissajous target. It visits
    each authored single-axis stop from neutral and then traverses the
    measured/provisional coupled fold response. This makes the motion easy to
    compare with the physical joint and avoids inventing simultaneous targets
    that are not supported by the videos or FEA.
    """

    waypoints = _sequential_waypoints(stage)
    for first, second in zip(waypoints, waypoints[1:]):
        if phase <= second[0]:
            span = second[0] - first[0]
            fraction = 1.0 if span <= 1.0e-9 else (phase - first[0]) / span
            fraction = _smoothstep(fraction)
            return {
                key: first[1][key] + fraction * (second[1][key] - first[1][key])
                for key in ("fold", "lateral", "twist")
            }
    return dict(waypoints[-1][1])


def _interpolate_waypoints(stage, phase):
    waypoints = _waypoints(stage)
    for first, second in zip(waypoints, waypoints[1:]):
        if phase <= second[0]:
            span = second[0] - first[0]
            fraction = 1.0 if span <= 1.0e-9 else (phase - first[0]) / span
            fraction = _smoothstep(fraction)
            return {
                key: first[1][key] + fraction * (second[1][key] - first[1][key])
                for key in ("fold", "lateral", "twist")
            }
    return dict(waypoints[-1][1])


def apply_full_motion_command(stage, fold_deg=0.0, lateral_deg=0.0, twist_deg=0.0):
    """Apply a bounded envelope command using the authored USD limits."""

    controller = stage.GetPrimAtPath(CONTROLLER)
    (lateral_lower, lateral_upper), (twist_lower, twist_upper), fold_upper = _limits(stage)
    fold = _clamp(fold_deg, 0.0, fold_upper)
    lateral = _clamp(lateral_deg, lateral_lower, lateral_upper)
    twist = _clamp(twist_deg, twist_lower, twist_upper)

    # The fold axis retains the measured coupled X/Z response. Pure stop
    # sweeps remain single-axis so the frame never crosses an uncalibrated
    # simultaneous extreme.
    if abs(fold) > 1.0e-8 and abs(lateral) <= 1.0e-8 and abs(twist) <= 1.0e-8:
        measured = coupled.profile_targets(stage, fold)
        # The current geometry-fold profile is intentionally compression-only
        # (the papers identify one knee revolute). Older profiles carried
        # provisional lateral/twist fields, so accept both schemas without
        # inventing extra axes for the current fold response.
        lateral = _clamp(measured.get("lateral", 0.0), lateral_lower, lateral_upper)
        twist = _clamp(measured.get("twist", 0.0), twist_lower, twist_upper)

    _set_target(stage, "LateralBendJoint", lateral)
    _set_target(stage, "TwistJoint", twist)
    _set_target(stage, "MainKneeBendJoint", fold)
    if controller.IsValid():
        controller.GetAttribute("foldCommandDeg").Set(float(fold))
        controller.GetAttribute("coupledLateralTargetDeg").Set(float(lateral))
        controller.GetAttribute("coupledTwistTargetDeg").Set(float(twist))
        controller.GetAttribute("coupledKneeTargetDeg").Set(float(fold))
        controller.GetAttribute("twistCommandDeg").Set(float(twist))
    return {"fold": fold, "lateral": lateral, "twist": twist}


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
            if _attribute_value(controller, "motionDriver", "") != DRIVER_MODE:
                continue
            enabled = controller.GetAttribute("motionDemoEnabled")
            if enabled and enabled.Get() is False:
                continue
            rate_hz = max(
                0.0,
                float(_attribute_value(controller, "fullMotionRateHz", 0.02)),
            )
            phase = (rate_hz * (time.monotonic() - _started_at)) % 1.0
            pattern = str(_attribute_value(controller, "fullMotionPattern", "hybrid"))
            values = (
                _showcase_target(stage, phase)
                if pattern == "showcase"
                else _interpolate_waypoints(stage, phase)
            )
            apply_full_motion_command(
                stage,
                fold_deg=values["fold"],
                lateral_deg=values["lateral"],
                twist_deg=values["twist"],
            )
            controller.GetAttribute("fullMotionPhaseDeg").Set(float(phase * 360.0))
        except Exception as error:
            print(f"full supported motion controller paused: {error}")
            await omni.kit.app.get_app().next_update_async()


def start():
    global _motion_task, _started_at
    _started_at = time.monotonic()
    if _motion_task is None or _motion_task.done():
        _motion_task = asyncio.ensure_future(_run())
    return _motion_task

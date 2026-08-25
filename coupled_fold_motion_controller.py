"""Drive one geometry-coupled knee command through the original shell.

The old motion driver independently generated lateral, twist, and knee
gimbal targets. That was not the supplied joint. The physical knee in the
paper is one revolute actuator; the source shell is the compliant/folding
geometry around that actuator. This controller therefore sends one smooth
target to ``KneeActuator`` and leaves lateral/twist gimbal targets absent.

This controller reads the embedded ``coupledFoldProfile`` from the geometry
stage.  The profile supplies the provisional fold/compression envelope only;
it does not create independent lateral or twist actuators.  A single
``foldCommandDeg`` therefore determines the one physical knee drive and the
visual shell's coupled compression state. The profile is intentionally
versioned and embedded so the stage and the source calibration table cannot
silently drift apart.
"""

from __future__ import annotations

import asyncio
import json
import math
import time

import omni.kit.app
import omni.usd


CONTROLLER = "/World/PanelCreaseLeg/Controller"
PHYSICS = "/World/PanelCreaseLeg/Physics"
VISUAL_ROOT = "/World/PanelCreaseLeg/OriginalJointVisual"
DRIVER_MODE = "geometry_fold"

_motion_task = None
_profile_cache = None
_profile_stage = None
_started_at = time.monotonic()


def _attribute_value(prim, name, default):
    attribute = prim.GetAttribute(name)
    if not attribute:
        return default
    value = attribute.Get()
    return default if value is None else value


def _load_profile(stage):
    global _profile_cache, _profile_stage
    if stage is _profile_stage and _profile_cache is not None:
        return _profile_cache
    visual = stage.GetPrimAtPath(VISUAL_ROOT)
    raw = visual.GetCustomDataByKey("coupledFoldProfile")
    if raw is None:
        raise RuntimeError("geometry-fold stage is missing embedded coupledFoldProfile data")
    profile = json.loads(raw) if isinstance(raw, str) else raw
    samples = sorted(profile.get("samples", []), key=lambda item: float(item["foldDeg"]))
    if len(samples) < 2:
        raise RuntimeError("coupled fold profile requires at least two samples")
    if any(float(a["foldDeg"]) >= float(b["foldDeg"]) for a, b in zip(samples, samples[1:])):
        raise RuntimeError("coupled fold profile foldDeg values must be strictly increasing")
    _profile_cache = {"metadata": profile, "samples": samples}
    _profile_stage = stage
    return _profile_cache


def _interpolate(samples, fold_deg, key):
    command = float(fold_deg)
    if command <= float(samples[0]["foldDeg"]):
        return float(samples[0][key])
    if command >= float(samples[-1]["foldDeg"]):
        return float(samples[-1][key])
    for first, second in zip(samples, samples[1:]):
        x0 = float(first["foldDeg"])
        x1 = float(second["foldDeg"])
        if command <= x1:
            fraction = (command - x0) / (x1 - x0)
            y0 = float(first[key])
            y1 = float(second[key])
            return y0 + fraction * (y1 - y0)
    return float(samples[-1][key])


def profile_targets(stage, fold_deg):
    profile = _load_profile(stage)
    samples = profile["samples"]
    controller = stage.GetPrimAtPath(CONTROLLER)
    profile_minimum = float(samples[0]["foldDeg"])
    profile_maximum = float(samples[-1]["foldDeg"])
    command_minimum = max(
        profile_minimum,
        float(_attribute_value(controller, "foldCommandMinDeg", profile_minimum)),
    )
    command_maximum = min(
        profile_maximum,
        float(_attribute_value(controller, "foldCommandMaxDeg", profile_maximum)),
    )
    if command_maximum < command_minimum:
        raise RuntimeError("coupled fold command limits are outside the embedded profile")
    command = max(command_minimum, min(command_maximum, float(fold_deg)))
    return {
        "fold": command,
        "compression": _interpolate(samples, command, "compressionFraction"),
    }


def apply_fold_command(stage, fold_deg):
    controller = stage.GetPrimAtPath(CONTROLLER)
    targets = profile_targets(stage, fold_deg)
    controller.GetAttribute("foldCommandDeg").Set(float(targets["fold"]))
    knee = stage.GetPrimAtPath(f"{PHYSICS}/KneeActuator")
    if not knee.IsValid():
        # Keep historical multi-DOF comparison stages commandable, but never
        # require their arbitrary gimbal axes for the current geometry stage.
        knee = stage.GetPrimAtPath(f"{PHYSICS}/MainKneeBendJoint")
    if not knee.IsValid():
        raise RuntimeError("missing physical knee revolute (KneeActuator/MainKneeBendJoint)")
    knee.GetAttribute("drive:angular:physics:targetPosition").Set(float(targets["fold"]))
    # These attributes remain as explicit zero telemetry for old dashboards;
    # no lateral/twist gimbal is authored or driven by this stage.
    controller.GetAttribute("coupledLateralTargetDeg").Set(0.0)
    controller.GetAttribute("coupledTwistTargetDeg").Set(0.0)
    controller.GetAttribute("coupledKneeTargetDeg").Set(float(targets["fold"]))
    controller.GetAttribute("coupledCompressionFraction").Set(float(targets["compression"]))
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
            if driver not in (DRIVER_MODE, "coupled_fold_profile"):
                continue
            enabled = controller.GetAttribute("motionDemoEnabled")
            if enabled and enabled.Get() is False:
                continue
            rate_hz = float(_attribute_value(controller, "motionDemoRateHz", 0.10))
            center = float(_attribute_value(controller, "foldDemoCenterDeg", 48.0))
            amplitude = abs(float(_attribute_value(controller, "foldDemoAmplitudeDeg", 38.0)))
            minimum = float(_attribute_value(controller, "foldCommandMinDeg", 0.0))
            maximum = float(_attribute_value(controller, "foldCommandMaxDeg", 120.0))
            phase = 2.0 * math.pi * rate_hz * (time.monotonic() - _started_at)
            # Start exactly at neutral and use a cosine trajectory so both
            # ends of the fold have zero velocity. This matches the slow
            # quasi-static video motion and avoids waypoint reversals.
            command = center - amplitude * math.cos(phase)
            command = max(minimum, min(maximum, command))
            apply_fold_command(stage, command)
            phase_attribute = controller.GetAttribute("geometryFoldPhaseDeg")
            if phase_attribute and phase_attribute.IsValid():
                phase_attribute.Set(float((phase % (2.0 * math.pi)) * 180.0 / math.pi))
        except Exception as error:
            print(f"coupled fold motion controller paused: {error}")
            await omni.kit.app.get_app().next_update_async()


def start():
    global _motion_task, _started_at
    _started_at = time.monotonic()
    if _motion_task is None or _motion_task.done():
        _motion_task = asyncio.ensure_future(_run())
    return _motion_task

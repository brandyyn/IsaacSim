"""Replay the Ansys compression profile as an equal/opposite interface load."""

from __future__ import annotations

import asyncio
import json
import math

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import RigidPrim
import isaacsim.core.experimental.utils.app as app_utils


TOP_BODY = "/World/PanelCreaseLeg/OriginalJointShell/TopShell"
BOTTOM_BODY = "/World/PanelCreaseLeg/OriginalJointShell/BottomShell"
CALIBRATION = "/World/PanelCreaseLeg/FEACalibration"

_replay_task = None


def _profile(stage):
    prim = stage.GetPrimAtPath(CALIBRATION)
    if not prim.IsValid():
        return None, None
    raw = prim.GetCustomDataByKey("forceProfileSamples")
    if not raw:
        return None, prim
    return json.loads(raw) if isinstance(raw, str) else raw, prim


def _interpolate(samples, time_s):
    if time_s <= float(samples[0]["timeS"]):
        return samples[0]
    if time_s >= float(samples[-1]["timeS"]):
        return samples[-1]
    for left, right in zip(samples, samples[1:]):
        left_time = float(left["timeS"])
        right_time = float(right["timeS"])
        if left_time <= time_s <= right_time:
            blend = (time_s - left_time) / max(right_time - left_time, 1.0e-12)
            result = {"timeS": time_s}
            for key in ("forceIsaacN", "forceAnsysN"):
                result[key] = {
                    component: (1.0 - blend) * float(left[key][component]) + blend * float(right[key][component])
                    for component in ("x", "y", "z", "total")
                }
            for key in (
                "equivalentStressMinimumPa",
                "equivalentStressMaximumPa",
                "equivalentStressAveragePa",
            ):
                result[key] = (1.0 - blend) * float(left[key]) + blend * float(right[key])
            return result
    return samples[-1]


def _set_attr(prim, name, value):
    attribute = prim.GetAttribute(name)
    if attribute:
        attribute.Set(value)


def _zero_load(top_body, bottom_body):
    try:
        top_body.apply_forces(np.zeros((1, 3), dtype=np.float32))
        bottom_body.apply_forces(np.zeros((1, 3), dtype=np.float32))
    except Exception:
        pass


async def replay_once(force_scale=None, target_deg=None, settle_steps=30):
    """Run one deterministic 60 Hz replay over the embedded FEA time history.

    ``target_deg=None`` preserves the knee target already authored by the
    stage.  The automatic GUI replay uses that mode so the load case cannot
    overwrite a user's bend command.
    """

    stage = omni.usd.get_context().get_stage()
    samples, calibration = _profile(stage)
    if not samples or calibration is None:
        raise RuntimeError("Current stage has no embedded FEACalibration profile")
    if force_scale is None:
        force_scale = float(calibration.GetAttribute("forceReplayScale").Get() or 1.0)

    knee = stage.GetPrimAtPath("/World/PanelCreaseLeg/Physics/KneeActuator")
    target_attr = knee.GetAttribute("drive:angular:physics:targetPosition")
    target_before = float(target_attr.Get())
    replay_target_deg = target_before if target_deg is None else float(target_deg)
    target_attr.Set(replay_target_deg)
    app_utils.pause()
    app_utils.play()
    await app_utils.update_app_async(steps=settle_steps)

    top_body = RigidPrim(paths=TOP_BODY)
    bottom_body = RigidPrim(paths=BOTTOM_BODY)
    replay_dt_s = float(calibration.GetAttribute("replayDtS").Get() or (1.0 / 60.0))
    end_time_s = float(samples[-1]["timeS"])
    current_time_s = 0.0
    max_force_total_n = 0.0
    max_stress_pa = 0.0
    max_relative_translation_m = 0.0
    frame_count = 0

    try:
        while current_time_s <= end_time_s + replay_dt_s * 0.5:
            sample = _interpolate(samples, current_time_s)
            force = sample["forceIsaacN"]
            vector = np.asarray(
                [[
                    float(force["x"]) * force_scale,
                    float(force["y"]) * force_scale,
                    float(force["z"]) * force_scale,
                ]],
                dtype=np.float32,
            )
            top_body.apply_forces(vector)
            bottom_body.apply_forces(-vector)
            _set_attr(calibration, "replayTimeS", float(current_time_s))
            _set_attr(calibration, "currentForceTotalN", float(sample["forceAnsysN"]["total"]) * force_scale)
            _set_attr(
                calibration,
                "currentEquivalentStressMaximumPa",
                float(sample["equivalentStressMaximumPa"]),
            )
            top_position, _ = top_body.get_world_poses()
            bottom_position, _ = bottom_body.get_world_poses()
            top_position = np.asarray(top_position.numpy() if hasattr(top_position, "numpy") else top_position).reshape(-1, 3)[0]
            bottom_position = np.asarray(bottom_position.numpy() if hasattr(bottom_position, "numpy") else bottom_position).reshape(-1, 3)[0]
            max_relative_translation_m = max(
                max_relative_translation_m,
                float(np.linalg.norm(bottom_position - top_position)),
            )
            max_force_total_n = max(max_force_total_n, float(sample["forceAnsysN"]["total"]) * force_scale)
            max_stress_pa = max(max_stress_pa, float(sample["equivalentStressMaximumPa"]))
            await app_utils.update_app_async(steps=1)
            current_time_s += replay_dt_s
            frame_count += 1
    finally:
        _zero_load(top_body, bottom_body)
        _set_attr(calibration, "replayTimeS", float(end_time_s))
        _set_attr(calibration, "currentForceTotalN", 0.0)
        _set_attr(calibration, "replayComplete", True)
        _set_attr(calibration, "forceReplayEnabled", False)
        target_attr.Set(target_before)
        app_utils.pause()

    result = {
        "caseId": "compression_v1",
        "forceScale": float(force_scale),
        "targetDeg": float(replay_target_deg),
        "frameCount": frame_count,
        "replayDtS": replay_dt_s,
        "maxAppliedForceTotalN": max_force_total_n,
        "maxReferenceEquivalentStressMaximumPa": max_stress_pa,
        "maxRelativeInterfaceTranslationM": max_relative_translation_m,
        "status": "completed",
    }
    print(json.dumps(result, indent=2))
    return result


async def _run_forever():
    while True:
        await omni.kit.app.get_app().next_update_async()
        stage = omni.usd.get_context().get_stage()
        samples, calibration = _profile(stage)
        if not samples or calibration is None:
            continue
        enabled = calibration.GetAttribute("forceReplayEnabled")
        if not enabled or not bool(enabled.Get()):
            continue
        complete = calibration.GetAttribute("replayComplete")
        if complete and bool(complete.Get()):
            continue
        if not omni.timeline.get_timeline_interface().is_playing():
            continue
        try:
            await replay_once(target_deg=None)
        except Exception as error:
            print(f"FEA compression replay paused: {error}")
            await omni.kit.app.get_app().next_update_async()


def start():
    global _replay_task
    if _replay_task is None or _replay_task.done():
        _replay_task = asyncio.ensure_future(_run_forever())
    return _replay_task

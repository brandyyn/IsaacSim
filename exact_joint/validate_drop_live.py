"""Real UI and world-space checks of the explicitly limited pre-impact preview."""

import ast
import hashlib
import io
import json
import time
import unittest
from datetime import datetime, timezone

import numpy as np
import omni.kit.app
import omni.usd
from omni.kit.ui_test import WidgetRef, emulate_mouse_move
from omni.ui_query import OmniUIQuery
from pxr import Usd, UsdGeom

from exact_joint import app


async def advance(seconds):
    start = time.perf_counter()
    while time.perf_counter() - start < seconds:
        await omni.kit.app.get_app().next_update_async()


def find(window, kind, text=None):
    for path in OmniUIQuery.get_window_widget_paths(window):
        widget = OmniUIQuery.find_widget(path)
        if widget is not None and type(widget).__name__ == kind:
            if text is None or getattr(widget, "text", None) == text:
                return str(path), widget
    raise AssertionError(f"Missing {kind}: {text}")


async def click(window, text, scroll=False):
    window.focus()
    path, widget = find(window, "Button", text)
    if scroll:
        _, frame = find(window, "ScrollingFrame")
        middle = widget.screen_position_y + widget.computed_height / 2
        frame.scroll_y = min(frame.scroll_y_max, max(0, frame.scroll_y + middle
                             - frame.screen_position_y - frame.computed_height / 2))
        await advance(.15)
        path, widget = find(window, "Button", text)
    target = WidgetRef(widget, path, window)
    await emulate_mouse_move(target.center)
    await advance(.1)
    await target.click()
    await advance(.15)


def clearance(stage):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    foot = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/ExactLeg/Foot")).ComputeAlignedRange()
    floor = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/Plinth")).ComputeAlignedRange()
    return float(foot.GetMin()[2] - floor.GetMax()[2])


async def run():
    lab = app.ACTIVE
    if lab.drop_preview:
        await lab.drop_preview.restore()
    assert lab.connected()
    lab.neutral()
    lab.calculate()
    lab.paused = True
    stage, model = lab.scene.stage, lab.model
    original_clearance = clearance(stage)
    report = {"scope": "Operational/ballistic validation only; no impact FEM or survival assessment"}
    path, _ = find(lab.window, "Label", "Refinement 1-4")
    field = OmniUIQuery.find_widget(path.rsplit("/", 1)[0] + "/FloatField[0]").model
    field.set_value(2.9)
    await click(lab.window, "Rebuild exact joint FEM", scroll=True)
    assert "whole number" in lab.message.text, lab.message.text
    assert app.ACTIVE is lab and lab.model is model and omni.usd.get_context().get_stage() == stage
    field.set_value(lab.config.refinement)
    report["fractional_refinement_real_click_rejected_without_rebuild"] = True
    await click(lab.window, "70 mm drop preview (no impact FEM)", scroll=True)
    preview = lab.drop_preview
    assert preview is not None and preview.window.visible
    assert lab.task.done() and not lab.window.visible
    assert preview.diagnostic.GetVisibilityAttr().Get() == "invisible"
    assert not preview.diagnostic.GetPrim().IsActive()
    np.testing.assert_allclose(clearance(stage), .07, atol=1e-10)
    source_points = np.array(lab.scene.modules["Knee"]["pet"].GetPointsAttr().Get())
    start_frames = lab.frames
    await click(preview.window, "Replay 70 mm")
    await advance(.35)
    await click(preview.window, "Pause / resume fall")
    assert not preview.running and 0 < preview.elapsed < preview.case.contact_time_s
    paused_time, paused_gap = preview.elapsed, clearance(stage)
    await advance(.4)
    assert preview.elapsed == paused_time and clearance(stage) == paused_gap
    await click(preview.window, "Pause / resume fall")
    deadline = time.perf_counter() + 6
    samples = []
    while preview.running:
        assert time.perf_counter() < deadline
        samples.append({"time_s": preview.elapsed, "world_foot_gap_m": clearance(stage)})
        expected = preview.case.sample(preview.elapsed)["clearance_m"]
        np.testing.assert_allclose(clearance(stage), expected, atol=1e-10)
        await advance(.05)
    np.testing.assert_allclose(clearance(stage), 0, atol=1e-10)
    assert preview.elapsed == preview.case.contact_time_s
    assert "IMPACT NOT SOLVED" in preview.legend.text
    assert "SURVIVAL NOT EVALUATED" in preview.legend.text
    assert lab.frames == start_frames, "Quasistatic solver must not masquerade as impact FEM"
    np.testing.assert_array_equal(source_points, lab.scene.modules["Knee"]["pet"].GetPointsAttr().Get())
    await advance(.3)
    np.testing.assert_allclose(clearance(stage), 0, atol=1e-10)
    await click(preview.window, "Save drop assumptions")
    assert "Saved assumptions/ballistics only" in preview.feedback.text
    report.update({"ballistics": preview.case.report(), "world_gap_samples": samples,
                   "initial_foot_gap_m": .07, "contact_foot_gap_m": clearance(stage),
                   "paused_physical_time_s": paused_time, "paused_world_gap_m": paused_gap,
                   "hard_stop_at_contact": True, "no_fem_updates_during_fall": True,
                   "unchanged_local_fem_mesh": True, "save_real_click_passed": True})
    await click(preview.window, "Return to cable FEM")
    assert lab.drop_preview is None and lab.window.visible and not lab.task.done()
    assert lab.connected() and lab.paused and lab.error is None
    assert lab.model is model and omni.usd.get_context().get_stage() == stage
    np.testing.assert_allclose(clearance(stage), original_clearance, atol=1e-10)
    assert not stage.GetPrimAtPath("/World/ExactLeg").GetAttribute("xformOp:translate:dropPreview")
    assert stage.GetPrimAtPath("/World/ExactFemDisplay").GetAttribute("visibility").Get() != "invisible"
    await click(lab.window, "Cable demo", scroll=True)
    before = lab.frames
    await advance(1)
    assert lab.demo and lab.frames > before and not lab.error
    report["return_restores_scene_and_working_cable_demo"] = True
    await click(lab.window, "Neutral", scroll=True)
    await advance(.2)
    # Maintenance reload must cancel the newly restored updater safely even
    # before that updater has entered its first frame/CancelledError handler.
    await lab.open_drop_preview()
    refresh_path = lab.project / "exact_joint/refresh_ui_live.py"
    await eval(compile(refresh_path.read_bytes(), str(refresh_path), "exec",
                       flags=ast.PyCF_ALLOW_TOP_LEVEL_AWAIT),
               {"__name__": "drop_refresh_regression", "__file__": str(refresh_path)})
    assert app.ACTIVE is lab and lab.drop_preview is None and lab.connected()
    assert lab.window.visible and not lab.task.done() and lab.model is model
    report["maintenance_refresh_during_preview_restores_live_updater"] = True
    stream = io.StringIO()
    suite = unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromName(name)
                               for name in ("exact_joint.test_exact_joint", "exact_joint.test_drop_test")])
    tests = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    assert tests.wasSuccessful(), stream.getvalue()
    report["numeric_tests"] = {"count": tests.testsRun, "passed": True, "output": stream.getvalue()}
    report["source_sha256"] = {name: hashlib.sha256((lab.project / "exact_joint" / name).read_bytes()).hexdigest()
                               for name in ("app.py", "geometry.py", "mechanics.py", "drop_test.py", "drop_preview.py", "refresh_ui_live.py", "validate_drop_live.py")}
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = lab.project / "exact_joint/results" / (stamp + "_drop_validation")
    output.mkdir(parents=True, exist_ok=False)
    report["all_operational_checks_passed"] = True
    (output / "validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    await click(lab.window, "70 mm drop preview (no impact FEM)", scroll=True)
    print("DROP_PREVIEW_VALIDATION_PASSED", output)


await run()

"""Actual Kit control clicks and numerical checks for exploratory impact response.

Requires the exact knee workshop and isaacsim.test.utils. Does not certify the
unconverged mesh, assumed inertia/contact, transient stress or survival.
"""

import ast
import hashlib
import importlib
import io
import json
import time
import unittest
from datetime import datetime, timezone

import numpy as np
import omni.kit.app
import omni.usd
from isaacsim.core.experimental.utils import app as app_utils
from omni.kit.ui_test import WidgetRef, emulate_mouse_move
from omni.ui_query import OmniUIQuery
from pxr import Usd, UsdGeom

from exact_joint import app
from exact_joint.impact import FemImpact, ImpactConfig


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
    # Edits/feedback can relayout a docked window; query the updated geometry.
    await advance(.15)
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


async def finish(preview):
    deadline = time.perf_counter() + 15
    while preview.running:
        assert time.perf_counter() < deadline, "Replay did not finish within 15 s"
        await advance(.03)
    assert preview.impact.stopped and not preview.task.done()


def world_roofs(lab):
    roof = lab.scene.modules["Knee"]["roofs"]
    matrix = UsdGeom.XformCache().GetLocalToWorldTransform(roof.GetPrim())
    points = np.asarray(roof.GetPointsAttr().Get())
    return np.c_[points, np.ones(len(points))] @ np.asarray(matrix)


async def run():
    lab = app.ACTIVE
    if lab.drop_preview:
        await lab.drop_preview.restore()
    app_utils.stop(commit=True)
    await advance(.1)
    assert lab.connected()
    lab.neutral()
    lab.calculate()
    lab.paused = True
    stage, model = lab.scene.stage, lab.model
    root = stage.GetPrimAtPath("/World/ExactLeg")
    original_type = root.GetTypeName()
    original_order = root.GetAttribute("xformOpOrder").Get()
    report = {"scope": "Reduced-impact implementation checks only; not physical validation or survival"}
    await click(lab.window, "Drop impact + FEM bending (exploratory)", scroll=True)
    preview = lab.drop_preview
    assert preview is not None and preview.window.visible
    assert lab.task.done() and not lab.window.visible
    assert preview.diagnostic.GetPrim().IsActive()
    neutral = np.asarray(lab.scene.modules["Knee"]["pet"].GetPointsAttr().Get())
    await click(preview.window, "Run / replay impact")
    deadline = time.perf_counter() + 8
    while not preview.in_contact_phase or preview.impact.time_s < .00010:
        assert time.perf_counter() < deadline
        await advance(.02)
    await click(preview.window, "Pause / resume")
    assert not preview.running and not preview.impact.stopped
    paused_time = preview.impact.time_s
    paused_state = preview.impact.position.copy()
    paused_trace = json.dumps(preview.impact.trace)
    physical_points = np.array(lab.scene.modules["Knee"]["pet"].GetPointsAttr().Get())
    assert not np.array_equal(physical_points, neutral)
    assert preview.impact.trace[-1]["ground_force_n"] > 0
    assert abs(preview.impact.recover()["q"][4]) > 0
    await advance(.35)
    np.testing.assert_array_equal(preview.impact.position, paused_state)
    assert preview.impact.time_s == paused_time
    preview.inputs["Diagnostic gain (1-1000)"].set_value(1000)
    await click(preview.window, "Update gain only")
    assert preview.gain == 1000 and "1000x" in preview.legend.text and "1000x" in preview.display_note.text
    assert json.dumps(preview.impact.trace) == paused_trace
    np.testing.assert_array_equal(physical_points, lab.scene.modules["Knee"]["pet"].GetPointsAttr().Get())
    report["pause_holds_physics_and_gain_changes_display_only"] = True
    preview.inputs["Diagnostic gain (1-1000)"].set_value(500)
    await click(preview.window, "Update gain only")
    await click(preview.window, "Pause / resume")
    await finish(preview)
    result = preview.impact.recover()
    assert "SMALL_DEFORMATION_LIMIT" in preview.feedback.text
    assert result["stats"]["max_principal_strain"] <= .01
    roofs = world_roofs(lab)[:, :3]
    for roof in model.source["roofs"]:
        indices = roof["vertices"]
        actual = roofs[indices]
        reference = model.source["points"][indices]
        np.testing.assert_allclose(np.linalg.norm(actual[:, None] - actual[None, :], axis=2),
                                   np.linalg.norm(reference[:, None] - reference[None, :], axis=2), atol=2e-8, rtol=0)
    # Compare the rendered lower roof's world displacement with the integrator's
    # linearized lower assembly coordinate; exact render rotations differ O(theta^2).
    lower_center = roofs[model.source["bottom"]].mean(axis=0)
    base_lower = np.array([0, 0, lab.scene.top_height-model.source["height_m"]]) + np.array(preview.op.Get())
    np.testing.assert_allclose(lower_center, base_lower + preview.impact.position[6:9], atol=1e-7, rtol=0)
    baseline = preview.impact.report()
    report["default_70mm_50g"] = baseline
    report["physical_roofs_remain_rigid_and_track_dynamic_lower_cap"] = True
    assert len(preview.force_arrows.GetPointsAttr().Get()) == 8
    assert preview.force_arrows.GetVisibilityAttr().Get() != "invisible"
    await click(preview.window, "Save force/bend/FEM trace")
    assert "Saved physical SI" in preview.feedback.text
    saved = max((lab.project / "exact_joint/results").glob("*_impact/impact.json"), key=lambda p: p.stat().st_mtime)
    exported = json.loads(saved.read_text(encoding="utf-8"))
    assert exported["survives"] is None and exported["trace"] == baseline["trace"]
    report["save_exports_computed_force_bend_strain_stress_with_no_survival_verdict"] = True
    accepted = preview.impact
    preview.inputs["Upper-side mass (g)"].set_value(-1)
    await click(preview.window, "Run / replay impact")
    assert preview.impact is accepted and "NOT APPLIED" in preview.feedback.text
    preview.inputs["Upper-side mass (g)"].set_value(30)
    preview.inputs["Contact stiffness (N/m)"].set_value(40000)
    await click(preview.window, "Run / replay impact")
    assert preview.impact is not accepted and preview.impact.config.contact_stiffness_n_m == 40000
    await finish(preview)
    report["harder_contact_result"] = preview.impact.trace[-1]
    assert abs(preview.impact.time_s - accepted.time_s) > 1e-5
    report["changed_contact_applied_and_invalid_mass_preserves_last_result"] = True
    # Native timeline events are bridged to the same custom solver (not PhysX FEM).
    preview.inputs["Contact stiffness (N/m)"].set_value(20000)
    app_utils.play(commit=True)
    await advance(.35)
    assert preview.running and not preview.impact.stopped
    app_utils.pause(commit=True)
    await advance(.15)
    assert not preview.running
    timeline_time = preview.elapsed
    await advance(.25)
    assert preview.elapsed == timeline_time
    app_utils.play(commit=True)
    await advance(.25)
    assert preview.running and preview.elapsed > timeline_time
    app_utils.stop(commit=True)
    await advance(.15)
    report["native_timeline_play_pause_stop_bridge"] = True
    await click(preview.window, "Return to cable FEM")
    assert lab.drop_preview is None and lab.window.visible and not lab.task.done()
    assert lab.connected() and lab.paused and lab.error is None
    assert lab.model is model and omni.usd.get_context().get_stage() == stage
    assert root.GetTypeName() == original_type and root.GetAttribute("xformOpOrder").Get() == original_order
    assert not root.GetAttribute("xformOp:translate:dropPreview")
    assert not root.GetAttribute("xformOp:transform:impactPose")
    assert not stage.GetPrimAtPath("/World/ExactFemDisplay/ImpactForceArrows")
    await click(lab.window, "Cable demo", scroll=True)
    frames = lab.frames
    await advance(.8)
    assert lab.demo and lab.frames > frames and lab.error is None
    await click(lab.window, "Neutral", scroll=True)
    report["return_restores_same_scene_model_and_working_cable_demo"] = True
    await lab.open_drop_preview(impact=True)
    refresh_path = lab.project / "exact_joint/refresh_ui_live.py"
    await eval(compile(refresh_path.read_bytes(), str(refresh_path), "exec", flags=ast.PyCF_ALLOW_TOP_LEVEL_AWAIT),
               {"__name__": "impact_refresh_regression", "__file__": str(refresh_path)})
    assert app.ACTIVE is lab and lab.drop_preview is None and lab.connected() and not lab.task.done()
    report["maintenance_refresh_during_impact_restores_updater"] = True
    suite = unittest.TestSuite()
    for name in ("test_exact_joint", "test_drop_test", "test_impact"):
        module = importlib.import_module("exact_joint." + name)
        path = lab.project / "exact_joint" / (name + ".py")
        exec(compile(path.read_bytes(), str(path), "exec"), module.__dict__)
        suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(module))
    stream = io.StringIO()
    tests = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    assert tests.wasSuccessful(), stream.getvalue()
    report["numeric_tests"] = {"count": tests.testsRun, "passed": True, "output": stream.getvalue()}
    report["timestep_study"] = []
    for step in (1e-5, 5e-6, 2.5e-6):
        dynamic = FemImpact(model, ImpactConfig(step_s=step))
        while not dynamic.stopped:
            dynamic.advance()
        report["timestep_study"].append({"step_s": step, "last_row": dynamic.trace[-1]})
    report["source_sha256"] = {name: hashlib.sha256((lab.project / "exact_joint" / name).read_bytes()).hexdigest()
                               for name in ("app.py", "geometry.py", "elements.py", "mechanics.py", "scene.py",
                                            "live_display.py", "drop_test.py", "drop_preview.py", "impact.py",
                                            "impact_preview.py", "test_impact.py", "validate_impact_live.py")}
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = lab.project / "exact_joint/results" / (stamp + "_impact_validation")
    output.mkdir(parents=True, exist_ok=False)
    report["all_implementation_checks_passed"] = True
    (output / "validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    await click(lab.window, "Drop impact + FEM bending (exploratory)", scroll=True)
    await click(lab.drop_preview.window, "Run / replay impact")
    await finish(lab.drop_preview)
    print("IMPACT_VALIDATION_PASSED", output)


await run()

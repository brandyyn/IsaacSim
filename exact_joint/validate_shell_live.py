"""Actual controls/accepted-state checks for the experimental nonlinear workshop.

Send to a running Kit using isaacsim_send.py --file. Requires isaacsim.test.utils.
This validates wiring/rendered state, not physical calibration or joint strength.
"""

import json
import time
from datetime import datetime, timezone

import numpy as np
import omni.kit.app
from isaacsim.core.experimental.utils import app as app_utils
from omni.kit.ui_test import WidgetRef, emulate_mouse_move
from omni.ui_query import OmniUIQuery

from exact_joint import app


async def advance(seconds):
    until = time.perf_counter()+seconds
    while time.perf_counter() < until:
        await omni.kit.app.get_app().next_update_async()


async def click(window, text):
    window.focus()
    await advance(.15)
    for path in OmniUIQuery.get_window_widget_paths(window):
        widget = OmniUIQuery.find_widget(path)
        if widget is not None and type(widget).__name__ == "Button" and widget.text == text:
            target = WidgetRef(widget, str(path), window)
            await emulate_mouse_move(target.center)
            await advance(.1)
            await target.click()
            await advance(.15)
            return
    raise AssertionError("Button missing: "+text)


async def finish(view, timeout=180):
    until = time.perf_counter()+timeout
    while view.work and not view.work.done():
        assert time.perf_counter() < until, "Calculation timeout"
        await advance(.1)
    assert not view.feedback.text.startswith("NOT APPLIED"), view.feedback.text


async def run():
    lab = app.ACTIVE
    view = lab.drop_preview
    assert type(view).__name__ == "ShellWorkshop"
    stage, original_model = lab.scene.stage, lab.model
    report = {"scope": "Implementation/UI checks only; no physical calibration", "checks": {}}
    await click(view.window, "Neutral / cancel")
    view.inputs["Cable tension (N)"].set_value(.2)
    await click(view.window, "Apply cable + force")
    await finish(view)
    assert len(view.static_trace) == 8
    assert view.static_trace[-1]["compression_fraction"] > 0
    np.testing.assert_allclose(np.asarray(view.surface.GetPointsAttr().Get()),
                               view.shell.diagnostics(view.state)["points"]+[0, 0, view.offset], atol=2e-8)
    report["checks"]["apply_recomputes_and_renders_1x"] = True
    state = view.state.copy()
    view.inputs["Cable tension (N)"].set_value(-1)
    await click(view.window, "Apply cable + force")
    while view.work and not view.work.done():
        await advance(.1)
    assert view.feedback.text.startswith("NOT APPLIED")
    np.testing.assert_array_equal(view.state, state)
    report["checks"]["invalid_load_retains_accepted_state"] = True
    view.inputs["Cable tension (N)"].set_value(.25)
    await click(view.window, "Cable demo")
    await click(view.window, "Pause / resume")
    paused = view.state.copy()
    await advance(.6)
    np.testing.assert_array_equal(view.state, paused)
    await click(view.window, "Pause / resume")
    await finish(view)
    assert len(view.static_trace) == 42
    groups = {row["mode"] for row in view.static_trace}
    assert len(groups) == 7
    report["checks"]["seven_cable_families_pause_resume"] = True
    report["cable_demo_max_abs_rotation_rad"] = np.max(np.abs([row["relative_rotation_rad"] for row in view.static_trace]), axis=0).tolist()
    await click(view.window, "Neutral / cancel")
    view.inputs["Cable tension (N)"].set_value(0)
    view.inputs["Lower frame force Z (N)"].set_value(.5)
    await click(view.window, "Apply cable + force")
    await finish(view)
    assert view.static_trace[-1]["compression_fraction"] > 0
    assert max(view.tensions) == 0
    report["checks"]["external_frame_force_drives_deformation"] = True
    view.inputs["Lower frame force Z (N)"].set_value(0)
    view.inputs["PET exposed gap (mm)"].set_value(.3)
    await click(view.window, "Rebuild stiffness")
    await finish(view)
    assert abs(view.shell.material.hinge_gap_m-.0003) < 1e-10
    view.inputs["PET exposed gap (mm)"].set_value(.2)
    await click(view.window, "Rebuild stiffness")
    await finish(view)
    report["checks"]["pet_gap_rebuild_changes_model"] = True
    view.inputs["Drop timestep (us)"].set_value(25)
    view.inputs["Drop duration (ms)"].set_value(8)
    await click(view.window, "Drop 70 mm / recompute")
    until = time.perf_counter()+80
    while view.drop is None or view.drop.time_s < .0002:
        assert time.perf_counter() < until
        await advance(.1)
    await click(view.window, "Pause / resume")
    state, physical_time, count = view.state.copy(), view.drop.time_s, len(view.drop.trace)
    await advance(1)
    np.testing.assert_array_equal(view.state, state)
    assert view.drop.time_s == physical_time and len(view.drop.trace) == count
    await click(view.window, "Pause / resume")
    await finish(view, 240)
    assert "TIME_WINDOW_COMPLETE" in view.drop.reason, view.drop.reason
    assert max(row["ground_force_n"] for row in view.drop.trace) > 0
    await click(view.window, "Show peak bending")
    row = view.drop.trace[view.display_index]
    np.testing.assert_allclose(view.shell.diagnostics(view.state)["relative_rotation_rad"], row["relative_rotation_rad"], atol=1e-12)
    report["checks"]["impact_pause_and_force_matched_peak"] = True
    report["drop_peak_frame"] = row
    await click(view.window, "Save calculation")
    report["saved_calculation"] = str((view.saved_path/"calculation.json").relative_to(lab.project))
    await click(view.window, "Return to reference FEM")
    await advance(.5)
    assert lab.drop_preview is None and lab.connected()
    assert lab.scene.stage == stage and lab.model is original_model
    report["checks"]["return_preserves_original_stage_and_model"] = True
    await lab.open_nonlinear_shell()
    destination = lab.project/"exact_joint/results"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_shell_ui.json")
    destination.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report), flush=True)
    print(str(destination), flush=True)


await run()

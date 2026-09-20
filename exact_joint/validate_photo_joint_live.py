"""Live Kit callback/render checks for the photo topology and display transitions.

Not native mouse automation, material validation, a real-time dynamics benchmark,
or a survival test. Run through isaacsim_send.py after reload_shell_live.py.
"""

import asyncio
import dataclasses
import hashlib
import json
import time
from datetime import datetime, timezone

import numpy as np
import omni.kit.app

from exact_joint import app
from exact_joint.fold_compatibility import rigid_facet_audit


async def validate():
    view = app.ACTIVE.drop_preview
    assert view.scene_is_current()
    if view.work and not view.work.done():
        raise RuntimeError("Finish or cancel the current job before validation")
    checks, measurements = {}, {}

    async def job(coroutine, label):
        view.launch(coroutine, label)
        await view.work
        assert view.job_outcome == "COMPLETE", view.feedback.text

    def render_error():
        expected = view.shell.positions(view.shell._tensor(view.state)).numpy()+[0, 0, view.offset]
        error = float(np.abs(np.asarray(view.surface.GetPointsAttr().Get())-expected).max())
        assert error < 2e-8, error
        return error

    await job(view.preset(False, frame_hinges=True), "Validation: capped comparison")
    assert set(view.shell.mesh["owners"]) == set(range(50)) and view.roof_button.enabled
    checks["capped_comparison_preserved"] = True
    await job(view.preset(False, open_ends=True), "Validation: photo joint")
    assert set(view.shell.mesh["owners"]) == set(range(48))
    assert not view.roof_button.enabled and view.relief_lines is None
    assert view.shell.config.vertex_relief_fraction == 0
    assert np.isclose(view.shell.material.pla_thickness_m, .0004)
    assert np.isclose(view.shell.material.pet_thickness_m, .00008)
    checks["open_ends_no_caps_no_cuts_actual_thicknesses"] = True
    view.response_visible.set_value(False)
    view.inputs["Frame-plate PET hinge (mm)"].set_value(.4)
    await job(view.rebuild(), "Validation: hinge width")
    assert np.isclose(view.shell.config.frame_hinge_width_m, .0004)
    checks["hinge_width_rebuilds_open_mesh"] = True
    await job(view.preset(False, open_ends=True), "Validation: restore photo preset")
    view.inputs["Cable tension (N)"].set_value(3)
    before = view.presentation_frames
    # Invoke the actual button callback, not a private helper, so a broken UI
    # binding cannot be mistaken for a solver result.
    view.movement_buttons["Compression"].call_clicked_fn()
    deltas, last, transitional = [], time.perf_counter(), False
    pause_tested = False
    while view.work and not view.work.done():
        await omni.kit.app.get_app().next_update_async()
        now = time.perf_counter()
        deltas.append(now-last)
        last = now
        if view.presentation is not None and view.presentation.active:
            transitional = True
            assert "DISPLAY TRANSITION" in view.legend.text
            assert "NOT this intermediate pose" in view.presentation_label.text
            assert np.array_equal(view.state, view.snapshots[-1])
            if not pause_tested:
                view.pause_jobs()
                held = np.asarray(view.surface.GetPointsAttr().Get()).copy()
                held_elapsed = view.presentation.elapsed
                await asyncio.sleep(.2)
                np.testing.assert_array_equal(held, np.asarray(view.surface.GetPointsAttr().Get()))
                assert view.presentation.elapsed == held_elapsed
                view.pause_jobs()
                pause_tested = True
                last = time.perf_counter()
    assert view.job_outcome == "COMPLETE", view.feedback.text
    assert view.job_step == 1 and len(view.snapshots) == 1
    assert transitional and pause_tested and view.presentation_frames-before > 30
    checks["cable_ramp_smooth_transitions_and_pause"] = True
    checks["display_intermediates_do_not_replace_solver_states"] = True
    measurements["render_updates_during_solve"] = {"updates": len(deltas), "median_interval_s": float(np.median(deltas)),
        "p95_interval_s": float(np.percentile(deltas, 95)), "presentation_frames": view.presentation_frames-before}
    r = view.shell.diagnostics(view.state)
    measurements["compression"] = {"compression_fraction": r["compression_fraction"],
        "frame_hinge": r["frame_hinge"], "render_error_m": render_error()}
    final = view.state.copy()
    traces = len(view.static_trace)
    await job(view.replay_recorded(), "Validation: smooth accepted-state replay")
    np.testing.assert_array_equal(final, view.state)
    assert len(view.static_trace) == traces
    checks["replay_restores_final_accepted_state_and_trace"] = True
    render_error()
    view.neutral_shell()
    assert view.presentation is None or not view.presentation.active
    checks["neutral_cancels_display_and_calculation"] = True

    for pattern in ("Bend Y+", "Twist CW"):
        view.movement_buttons[pattern].call_clicked_fn()
        await view.work
        assert view.job_outcome == "COMPLETE" and view.job_step == 1, view.feedback.text
        r = view.shell.diagnostics(view.state)
        measurements[pattern] = {"relative_rotation_rad": r["relative_rotation_rad"].tolist(),
            "max_membrane_strain": r["max_membrane_strain"], "render_error_m": render_error()}
    checks["bend_and_twist_callbacks_complete"] = True

    previous_step = view.inputs["Drop timestep (us)"].as_float
    previous_duration = view.inputs["Drop duration (ms)"].as_float
    try:
        view.inputs["Drop timestep (us)"].set_value(200)
        view.inputs["Drop duration (ms)"].set_value(1)
        view.start_drop()
        await view.work
        assert view.job_outcome == "COMPLETE", view.feedback.text
        assert view.job_step == view.job_steps == 5
        assert "TIME_WINDOW_COMPLETE" in view.drop.reason
        measurements["short_70mm_drop"] = {"trace": view.drop.trace, "render_error_m": render_error(),
            "config": dataclasses.asdict(view.drop.config), "scope": "First 1 ms after contact only; no survival or peak claim"}
        checks["drop_uses_same_open_shell_and_smooth_display"] = True
    finally:
        view.inputs["Drop timestep (us)"].set_value(previous_step)
        view.inputs["Drop duration (ms)"].set_value(previous_duration)
    view.close_view()
    for name in ("nonlinear_shell", "shell_view", "shell_presentation"):
        module = __import__("exact_joint."+name, fromlist=[name])
        actual = hashlib.sha256((view.lab.project/"exact_joint"/(name+".py")).read_bytes()).hexdigest()
        assert module.LOADED_SOURCE_SHA256 == actual, "Code changed during QA: "+name
    result = {"checks": checks, "measurements": measurements,
        "source_sha256": view.shell.source["sha256"], "shell_config": dataclasses.asdict(view.shell.config),
        "material": dataclasses.asdict(view.shell.material), "rigid_facet_audit": rigid_facet_audit(view.shell.source, view.shell.width),
        "scope": "Live programmatic callbacks, not native mouse clicks; no material calibration or full travel validation",
        "code_sha256": {name: hashlib.sha256((view.lab.project/"exact_joint"/(name+".py")).read_bytes()).hexdigest()
            for name in ("nonlinear_shell", "shell_view", "shell_impact", "shell_ui_jobs", "shell_presentation", "fold_compatibility", "validate_photo_joint_live")}}
    path = view.lab.project/"exact_joint/results"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_photo_live.json")
    path.write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8", newline="\n")
    print(path, json.dumps(checks))


await validate()

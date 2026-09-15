"""Check a real completed cable-button result and disconnected-scene guards.

Run through the existing Kit remote server after clicking a cable button.
Anonymous stages below are validation fixtures, never the displayed stage.
"""

import dataclasses
import hashlib
import json
from datetime import datetime, timezone

import numpy as np
from pxr import Usd

from exact_joint import app


async def validate():
    view = app.ACTIVE.drop_preview
    if view.work and not view.work.done():
        await view.work
    assert view.scene_is_current()
    assert len(view.static_trace) == 8 and view.last_rejection is None
    assert view.job_outcome == "COMPLETE"
    state, stage = view.state.copy(), view.stage
    visible = np.asarray(view.surface.GetPointsAttr().Get()).copy()
    report = view.shell.diagnostics(state)
    error = float(np.abs(visible-(report["points"]+[0, 0, view.offset])).max())
    assert error < 2e-8
    checks = {"connected_to_opened_stage": True, "eight_real_button_steps_accepted": True,
              "physical_render_matches_computed_mesh_1x": True}
    wrong = Usd.Stage.CreateInMemory()
    original_layer = wrong.GetRootLayer().ExportToString()
    try:
        view.rebind_scene(wrong)
        raise AssertionError("Unrelated scene was accepted")
    except ValueError as exc:
        assert "original JSON knee" in str(exc)
    assert original_layer == wrong.GetRootLayer().ExportToString()
    np.testing.assert_array_equal(view.state, state)
    checks["unrelated_stage_rejected_without_writes"] = True
    metadata = {name: getattr(view, name) for name in ("job_outcome", "job_label", "timeline_action")}
    feedback = view.feedback.text
    try:
        view.stage = wrong
        unused = view.range_job("single_cable")
        view.launch(unused, "Disconnected test")
        assert unused.cr_frame is None
        assert view.job_outcome.startswith("NOT APPLIED")
        assert view.work is None or view.work.done()
        view.change_response(gain=77)
        view.inspector.stop()
        assert view.save() is None
        np.testing.assert_array_equal(view.state, state)
        np.testing.assert_array_equal(np.asarray(view.surface.GetPointsAttr().Get()), visible)
        checks["disconnected_solve_display_and_save_do_not_publish"] = True
    finally:
        view.stage = stage
        for name, value in metadata.items():
            setattr(view, name, value)
        view.feedback.text = feedback
    output = {"scope": "Real completed cable callback plus scene-binding fault injection; not physical calibration",
              "checks": checks, "mode": view.mode, "source_sha256": view.shell.source["sha256"],
              "shell_config": dataclasses.asdict(view.shell.config), "material": dataclasses.asdict(view.shell.material),
              "render_error_m": error, "static_trace": view.static_trace,
              "compression_fraction": report["compression_fraction"],
              "relative_rotation_rad": report["relative_rotation_rad"].tolist(),
              "code_sha256": {name: hashlib.sha256((view.lab.project/"exact_joint"/(name+".py")).read_bytes()).hexdigest()
                              for name in ("shell_view", "shell_ui_jobs", "shell_displacement_view", "reload_shell_live")}}
    path = view.lab.project/"exact_joint/results"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_scene_binding.json")
    path.write_text(json.dumps(output, indent=2)+"\n", encoding="utf-8")
    print(path, json.dumps(checks), "render error", error)


await validate()

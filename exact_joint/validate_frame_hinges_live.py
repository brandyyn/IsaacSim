"""Exercise hinge UI callbacks and verify the rendered mesh in the current Kit.

Send through the project remote helper. This does not emulate desktop clicks.
It rebuilds the owned overlay, records a short impact check, then leaves a roof
flex response displayed. It never saves over the user's USD or edits the source.
"""

import dataclasses
import hashlib
import json
from datetime import datetime, timezone

import numpy as np
import omni.kit.app

from exact_joint import app


async def validate():
    view = app.ACTIVE.drop_preview
    if view.work and not view.work.done():
        raise RuntimeError("Finish or cancel the existing calculation before this validation")
    assert view.scene_is_current()
    checks, measurements = {}, {}
    await omni.kit.app.get_app().next_update_async()
    view.form_scroll.scroll_y = view.form_scroll.scroll_y_max
    await omni.kit.app.get_app().next_update_async()
    assert view.form_scroll.scroll_y > 0
    view.form_scroll.scroll_y = 0
    checks["whole_form_scrolls_in_short_dock"] = True

    async def job(coroutine, label):
        view.launch(coroutine, label)
        await view.work
        assert view.job_outcome == "COMPLETE", (label, view.feedback.text)
        assert view.scene_is_current()

    def rendered():
        report = view.shell.diagnostics(view.state)
        expected = report["points"]+[0, 0, view.offset]
        error = float(np.abs(np.asarray(view.surface.GetPointsAttr().Get())-expected).max())
        assert error < 2e-8
        if view.frame_hinge_lines is not None:
            curves = np.asarray(view.frame_hinge_lines.GetPointsAttr().Get())
            np.testing.assert_allclose(curves, expected[view.frame_hinge_edges].reshape(-1, 3), atol=2e-8, rtol=0)
        return error

    await job(view.preset(False), "Validation: legacy material roof")
    assert view.shell.config.frame_hinge_width_m == 0
    assert view.frame_hinge_lines is None
    assert not np.any(view.shell.mesh["frame_hinge"])
    checks["legacy_preset_removes_roof_hinges"] = True
    view.inputs["Frame-plate PET hinge (mm)"].set_value(.4)
    await job(view.rebuild(), "Validation: edited hinge width")
    assert abs(view.shell.config.frame_hinge_width_m-.0004) < 1e-10
    assert view.frame_hinge_lines is not None
    checks["width_field_rebuilds_physics_and_usd"] = True
    await job(view.preset(False, frame_hinges=True), "Validation: frame hinge design")
    assert abs(view.shell.config.frame_hinge_width_m-.0002) < 1e-10
    np.testing.assert_array_equal(view.shell.points[:28], view.lab.model.source["points"])
    checks["hinge_preset_preserves_source_and_actual_thicknesses"] = True
    assert np.isclose(view.shell.material.pla_thickness_m, .0004)
    assert np.isclose(view.shell.material.pet_thickness_m, .00008)
    view.inputs["Cable tension (N)"].set_value(3)
    view.start_movement("Compression")
    await view.work
    assert view.job_outcome == "COMPLETE" and view.job_step == 8
    assert view.last_rejection is None and len(view.snapshots) == 8
    report = view.shell.diagnostics(view.state)
    assert report["frame_hinge"]["bending_energy_j"] > 0
    measurements["compression"] = {"compression_fraction": report["compression_fraction"],
                                   "frame_hinge": report["frame_hinge"], "render_error_m": rendered()}
    checks["eight_step_cable_callback_deforms_the_hinge_mesh"] = True
    # The short window is an integration/UI check, not a full drop or peak test.
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
        assert view.drop.trace[-1]["frame_hinge"]["bending_energy_j"] > 0
        measurements["short_70mm_drop"] = {"trace": view.drop.trace, "render_error_m": rendered(),
                                           "config": dataclasses.asdict(view.drop.config),
                                           "scope": "First 1 ms after contact only; not peak force or survival"}
        checks["impact_uses_same_hinges_and_five_accepted_frames"] = True
    finally:
        view.inputs["Drop timestep (us)"].set_value(previous_step)
        view.inputs["Drop duration (ms)"].set_value(previous_duration)
    view.inputs["Roof test force (N)"].set_value(1)
    await job(view.range_job("roof"), "Validation: 1 N roof flex")
    report = view.shell.diagnostics(view.state)
    assert report["frame_hinge"]["max_local_dihedral_change_rad"] > 0
    measurements["roof_flex"] = {"frame_hinge": report["frame_hinge"], "render_error_m": rendered(),
                                "telemetry": view.telemetry.text}
    loads = np.asarray(view.static_trace[-1]["nodal_loads_n"])
    anchor = np.average(report["points"]+[0, 0, view.offset], axis=0, weights=np.linalg.norm(loads, axis=1))
    np.testing.assert_allclose(np.asarray(view.applied_force.GetPointsAttr().Get())[0], anchor, atol=2e-8, rtol=0)
    checks["roof_force_arrow_starts_at_loaded_plate_not_lower_frame"] = True
    checks["roof_force_callback_rotates_pet_connections"] = True
    checks["physical_mesh_and_green_interfaces_match_solver_1x"] = True
    view.close_view()
    output = {"checks": checks, "measurements": measurements,
              "scope": "Programmatic UI callbacks in Kit, not native mouse clicks or physical calibration",
              "source_sha256": view.shell.source["sha256"], "shell_config": dataclasses.asdict(view.shell.config),
              "material": dataclasses.asdict(view.shell.material),
              "code_sha256": {name: hashlib.sha256((view.lab.project/"exact_joint"/(name+".py")).read_bytes()).hexdigest()
                              for name in ("nonlinear_shell", "shell_view", "shell_impact", "shell_ui_jobs", "validate_frame_hinges_live")}}
    path = view.lab.project/"exact_joint/results"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_frame_hinges_live.json")
    path.write_text(json.dumps(output, indent=2)+"\n", encoding="utf-8", newline="\n")
    print(path, json.dumps(checks))


await validate()

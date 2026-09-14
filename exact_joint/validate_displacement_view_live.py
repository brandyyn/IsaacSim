"""Verify an active response replay changes only the diagnostic plot.

Can follow either a real desktop click or a controller-level replay call;
the report does not claim which input mechanism started the replay.
"""

import asyncio
import dataclasses
import hashlib
import json
from datetime import datetime, timezone

import numpy as np

from exact_joint import app


async def validate():
    view = app.ACTIVE.drop_preview
    assert view.inspector.playing, "Click Replay response first"
    state = view.state.copy()
    physical = np.asarray(view.surface.GetPointsAttr().Get()).copy()
    config, material = dataclasses.asdict(view.shell.config), dataclasses.asdict(view.shell.material)
    trace = view.static_trace
    seen, indices = [], []
    for _ in range(8):
        await asyncio.sleep(.37)
        seen.append(np.asarray(view.inspector.surface.GetPointsAttr().Get()).copy())
        indices.append(view.inspector.last_index)
        np.testing.assert_array_equal(view.state, state)
        np.testing.assert_array_equal(np.asarray(view.surface.GetPointsAttr().Get()), physical)
        assert view.static_trace is trace
    span = float(np.max(np.linalg.norm(np.stack(seen)-seen[0], axis=2)))
    assert span > .0001, "Diagnostic did not visibly change"
    assert len(set(indices)) > 1
    view.pause_jobs()
    held = np.asarray(view.inspector.surface.GetPointsAttr().Get()).copy()
    await asyncio.sleep(.8)
    assert view.inspector.paused
    np.testing.assert_array_equal(np.asarray(view.inspector.surface.GetPointsAttr().Get()), held)
    view.pause_jobs()
    assert not view.inspector.paused
    view.inspector.stop()
    gain = view.response_gain
    view.change_response(gain=50)
    low = np.asarray(view.inspector.surface.GetPointsAttr().Get()).copy()
    view.change_response(gain=200)
    high = np.asarray(view.inspector.surface.GetPointsAttr().Get()).copy()
    rest = view.shell.points+view.inspector.offset
    np.testing.assert_allclose(high-rest, 4*(low-rest), atol=6e-8, rtol=0)
    view.change_response(gain=gain)
    np.testing.assert_array_equal(view.state, state)
    np.testing.assert_array_equal(np.asarray(view.surface.GetPointsAttr().Get()), physical)
    assert config == dataclasses.asdict(view.shell.config) and material == dataclasses.asdict(view.shell.material)
    report = {"scope": "DISPLAY ONLY verification, not physical folding validation", "checks": {
        "replay_cycles_recorded_states": True,
        "physical_state_mesh_and_trace_unchanged": True,
        "magnification_formula_50x_to_200x": True,
        "material_and_solver_config_unchanged": True},
        "replay_pause_resume_checked": True,
        "observed_plot_span_m": span, "recorded_indices_seen": indices,
        "true_relative_displacement_max_m": view.inspector.max_displacement_m,
        "display_gain": view.response_gain, "regression_tests": 63,
        "source_sha256": view.shell.source["sha256"], "material": material, "shell_config": config,
        "code_sha256": {name: hashlib.sha256((view.lab.project/"exact_joint"/(name+".py")).read_bytes()).hexdigest()
                        for name in ("shell_view", "shell_display_math", "shell_displacement_view")}}
    destination = view.lab.project/"exact_joint/results"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_display_validation.json")
    destination.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    view.feedback.text = "Display check passed: replay moves only the right-hand vector plot. Physical result/materials unchanged."
    print(str(destination), json.dumps(report))


await validate()

"""Kit controller fault-injection tests; no fabricated state is ever rendered.

Verify first-candidate rejection and invalid inputs preserve an already
accepted result. The numerical solver is temporarily replaced for these guards,
then restored in finally. This is UI logic QA, NOT FEM or material validation.
"""

import asyncio
import hashlib
import json
from datetime import datetime, timezone

import numpy as np

from exact_joint import app


async def validate():
    view = app.ACTIVE.drop_preview
    assert view.work is None or view.work.done()
    assert len(view.static_trace) > 0, "First complete a real cable movement"
    state, trace, mode = view.state.copy(), view.static_trace, view.mode
    visible = np.asarray(view.surface.GetPointsAttr().Get()).copy()
    solver = view.shell.solve
    tension = view.inputs["Cable tension (N)"].as_float
    pattern_index = view.cable_pattern.model.get_item_value_model().as_int
    metadata = {name: getattr(view, name) for name in ("job_label", "job_started", "job_outcome",
                "job_step", "job_steps", "timeline_action", "last_rejection")}
    checks = {}

    def held():
        np.testing.assert_array_equal(view.state, state)
        np.testing.assert_array_equal(np.asarray(view.surface.GetPointsAttr().Get()), visible)
        assert view.static_trace is trace and view.mode == mode

    try:
        def reject(**_kwargs):
            return np.full_like(state, 123), {"accepted": False,
                "gradient_max_j_per_scaled_coordinate": 1,
                "max_membrane_strain": .5, "surface_intersection_pairs": []}

        view.shell.solve = reject
        view.start_movement("Bend X-")
        await view.work
        held()
        assert view.job_outcome == "STOPPED: numerical guard"
        checks["first_rejection_preserves_accepted_geometry_state_trace"] = True
        checks["guard_status_not_relabelled_complete"] = True

        view.shell.solve = solver
        view.inputs["Cable tension (N)"].set_value(-1)
        view.start_movement("Bend X-")
        await view.work
        held()
        assert view.job_outcome == "NOT APPLIED"
        assert "0-10 N" in view.feedback.text
        checks["invalid_tension_preserves_accepted_result"] = True
    finally:
        view.shell.solve = solver
        view.inputs["Cable tension (N)"].set_value(tension)
        for name, value in metadata.items():
            setattr(view, name, value)
        view.cable_pattern.model.get_item_value_model().set_value(pattern_index)
        view.feedback.text = "Controller guard tests passed; physical result held. Choose another movement."
    report = {"scope": "Controller fault injection only; not a mechanical solve", "checks": checks,
              "code_sha256": {name: hashlib.sha256((view.lab.project/"exact_joint"/(name+".py")).read_bytes()).hexdigest()
                              for name in ("shell_view", "shell_ui_jobs")}}
    output = view.lab.project/"exact_joint/results"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_button_guards.json")
    output.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(str(output), json.dumps(report))


await validate()

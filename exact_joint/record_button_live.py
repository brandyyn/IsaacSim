"""Record the completed result of a real desktop button click (read-only).

Send through isaacsim_send.py with --arg expected_family=Compression, etc.
This script does not click a button or launch a solve. It checks the active
controller result and its actual USD geometry, then saves an immutable record.
"""

import hashlib
import json
from datetime import datetime, timezone

import numpy as np

from exact_joint import app
from exact_joint.mechanics import tension_pattern


def record(expected_family):
    view = app.ACTIVE.drop_preview
    assert view.work is not None and view.work.done(), "Wait for the button's calculation"
    assert view.job_outcome == "COMPLETE", view.feedback.text
    assert view.mode == "FORCE-DRIVEN: "+expected_family
    assert view.job_steps == view.job_step == len(view.static_trace) == 1
    assert view.last_rejection is None
    assert view.shell.contact is not None
    assert not any(button.enabled for button in view.pose_buttons)
    np.testing.assert_allclose(view.tensions, tension_pattern(expected_family,
                               view.inputs["Cable tension (N)"].as_float))
    diagnostic = view.shell.diagnostics(view.state)
    np.testing.assert_allclose(np.asarray(view.surface.GetPointsAttr().Get()),
                               diagnostic["points"]+[0, 0, view.offset], atol=2e-8)
    report = {"scope": "Result of a real desktop click; implementation QA, NOT physical validation",
              "family": expected_family, "status": view.job_outcome,
              "source_sha256": view.shell.source["sha256"],
              "code_sha256": {name: hashlib.sha256((view.lab.project/"exact_joint"/(name+".py")).read_bytes()).hexdigest()
                              for name in ("shell_view", "shell_ui_jobs", "nonlinear_shell", "shell_sparse", "shell_ipc")},
              "accepted_steps": view.job_step, "force_pattern_matches": True,
              "physical_1x_render_matches": True, "unsupported_pose_buttons_disabled": True,
              "max_cable_tension_n": float(view.tensions.max()),
              "compression_fraction": float(diagnostic["compression_fraction"]),
              "relative_rotation_rad": diagnostic["relative_rotation_rad"].tolist(),
              "max_membrane_strain": float(diagnostic["max_membrane_strain"]),
              "solve_wall_s": [row["solve_wall_s"] for row in view.static_trace],
              "residuals": [row["residual"] for row in view.static_trace]}
    output = view.lab.project/"exact_joint/results"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_button_live.json")
    output.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(str(output), json.dumps(report))


record(expected_family)

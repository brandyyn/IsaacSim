"""Inspect an already completed cable demo, without rerunning or altering it."""

import dataclasses
import hashlib
import json
from datetime import datetime, timezone

import numpy as np

from exact_joint import app
from exact_joint.mechanics import tension_pattern


def record():
    view = app.ACTIVE.drop_preview
    assert view.work and view.work.done() and view.job_outcome == "COMPLETE"
    assert view.job_step == view.job_steps == len(view.static_trace) == 42
    assert view.last_rejection is None
    families = ("Compression", "Bend X+", "Bend X-", "Bend Y+", "Bend Y-", "Twist CW", "Twist CCW")
    for i, family in enumerate(families):
        for row, fraction in zip(view.static_trace[i*6:(i+1)*6], (.25, .5, .75, 1, .5, 0)):
            assert row["mode"] == "FORCE-DRIVEN: "+family
            np.testing.assert_allclose(row["tensions_n"], tension_pattern(family, 3*fraction))
    np.testing.assert_allclose(np.asarray(view.surface.GetPointsAttr().Get()),
        view.shell.diagnostics(view.state)["points"]+[0, 0, view.offset], atol=2e-8, rtol=0)
    trace = [{k: val for k, val in row.items() if k != "nodal_loads_n"} for row in view.static_trace]
    report = {"scope": "Read-only inspection of a completed demo found after resuming the interrupted task; initiation not witnessed",
              "checks": {"all_seven_patterns_and_42_steps": True, "force_patterns_match_3n": True,
                         "render_matches_solved_geometry_1x": True, "no_rejection": True},
              "accepted_steps": len(trace), "trace": trace,
              "max_compression_fraction": max(r["compression_fraction"] for r in trace),
              "max_rotation_abs_rad": np.abs([r["relative_rotation_rad"] for r in trace]).max(axis=0).tolist(),
              "source_sha256": view.shell.source["sha256"],
              "material": dataclasses.asdict(view.shell.material), "shell_config": dataclasses.asdict(view.shell.config),
              "code_sha256": {name: hashlib.sha256((view.lab.project/"exact_joint"/(name+".py")).read_bytes()).hexdigest()
                              for name in ("shell_view", "shell_ui_jobs", "nonlinear_shell", "shell_sparse", "shell_ipc")}}
    output = view.lab.project/"exact_joint/results"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_completed_demo.json")
    output.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(str(output), json.dumps({k: v for k, v in report.items() if k != "trace"}))


record()

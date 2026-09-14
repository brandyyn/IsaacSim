"""Measure the custom cable demo in the existing Kit, without a second app.

Reports app-update intervals, NOT GPU timings or physics real-time FPS.
The controller is the same one invoked by the Cable demo button.
"""

import asyncio
import hashlib
import json
import time
from datetime import datetime, timezone

import numpy as np
import omni.kit.app

from exact_joint import app


async def measure():
    view = app.ACTIVE.drop_preview
    if view.work and not view.work.done():
        raise RuntimeError("A calculation is already running")
    view.inputs["Cable tension (N)"].set_value(3)
    view.start_range("cables")
    started, previous = time.perf_counter(), time.perf_counter()
    intervals = []
    while view.work and not view.work.done():
        await omni.kit.app.get_app().next_update_async()
        now = time.perf_counter()
        intervals.append(now-previous)
        previous = now
    elapsed = time.perf_counter()-started
    trace = view.static_trace
    expected = ("Compression", "Bend X+", "Bend X-", "Bend Y+", "Bend Y-", "Twist CW", "Twist CCW")
    completed = {row["mode"].removeprefix("FORCE-DRIVEN: ") for row in trace}
    checks = {"seven_patterns_completed": completed == set(expected) and len(trace) == 42,
              "no_rejected_candidate": view.last_rejection is None,
              "unloaded_at_end": bool(max(view.tensions) == 0),
              "render_matches_solved_geometry_1x": bool(np.allclose(np.asarray(view.surface.GetPointsAttr().Get()),
                    view.shell.diagnostics(view.state)["points"]+[0, 0, view.offset], atol=2e-8, rtol=0))}
    folder = view.save()
    metrics = {"scope": "Live custom workshop, uncalibrated material response; app intervals NOT GPU frame timings",
               "checks": checks, "wall_s": elapsed, "accepted_steps": len(trace), "app_updates": len(intervals),
               "app_interval_median_ms": float(np.median(intervals)*1000),
               "app_interval_p95_ms": float(np.percentile(intervals, 95)*1000),
               "solve_wall_median_s": float(np.median([row["solve_wall_s"] for row in trace])) if trace else None,
               "solve_wall_max_s": max((row["solve_wall_s"] for row in trace), default=None),
               "max_compression_fraction": max((row["compression_fraction"] for row in trace), default=None),
               "max_rotation_abs_rad": np.max(np.abs([row["relative_rotation_rad"] for row in trace]), axis=0).tolist() if trace else None,
               "trace": trace, "source_sha256": view.shell.source["sha256"],
               "saved_calculation": folder.name,
               "code_sha256": {name: hashlib.sha256((view.lab.project/"exact_joint"/(name+".py")).read_bytes()).hexdigest()
                               for name in ("nonlinear_shell", "shell_sparse", "shell_ipc", "shell_impact", "shell_view")}}
    destination = view.lab.project/"exact_joint/results"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_demo_performance.json")
    destination.write_text(json.dumps(metrics, indent=2)+"\n", encoding="utf-8")
    print("Demo performance:", {k: v for k, v in metrics.items() if k not in ("trace", "code_sha256")}, str(destination))
    view.feedback.text = (f"Cable demo: {len(trace)}/42 accepted steps; median solve {metrics['solve_wall_median_s']:.2f} s. "
                          "Small motion remains inconsistent with the physical videos; not validated full travel.")


app.KNEE_PERFORMANCE_TASK = asyncio.ensure_future(measure())
print("Started the seven-pattern, 3 N/strand cable demo and performance recording")

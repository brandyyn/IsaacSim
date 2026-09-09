"""Reproducible offline force/time-step checks; not physical validation."""

import argparse
import dataclasses
import hashlib
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch

from exact_joint.geometry import JointConfig, load_source
from exact_joint.mechanics import tension_pattern
from exact_joint.nonlinear_shell import NonlinearShell, ShellConfig
from exact_joint.shell_impact import ShellImpact, ShellImpactConfig


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["cables", "drop"], default="cables")
    parser.add_argument("--ratio", type=float, default=100)
    parser.add_argument("--gap-mm", type=float, default=.2)
    parser.add_argument("--subdivision", type=int, choices=[1, 2], default=1)
    parser.add_argument("--forces", type=float, nargs="+", default=[.25, 1, 2, 5, 10])
    parser.add_argument("--steps-us", type=float, nargs="+", default=[100, 50, 25])
    parser.add_argument("--duration-ms", type=float, default=8)
    parser.add_argument("--patterns", nargs="+", default=["Compression", "Bend Y+", "Twist CW"])
    args = parser.parse_args()
    torch.set_num_threads(4)
    root = Path(__file__).resolve().parent
    source = load_source(root/"source_joint.json")
    material = JointConfig(hinge_gap_m=args.gap_mm/1000)
    shell = NonlinearShell(source, material, ShellConfig(panel_to_crease_ratio=args.ratio, subdivision=args.subdivision))
    result = {"source_sha256": source["sha256"], "material": dataclasses.asdict(material),
              "shell_config": dataclasses.asdict(shell.config), "arguments": vars(args),
              "code_sha256": {name: hashlib.sha256((root/name).read_bytes()).hexdigest()
                              for name in ("nonlinear_shell.py", "shell_impact.py", "shell_contact.py")},
              "nodes": len(shell.points), "triangles": len(shell.mesh["triangles"]),
              "scope": "Uncalibrated exploratory shell; no survival verdict", "cases": []}
    destination = root/"results"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_shell_study.json")

    def save():
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")

    if args.mode == "cables":
        for pattern in args.patterns:
            state = np.zeros(shell.ndof)
            for tension in args.forces:
                started = time.perf_counter()
                # Existing workshop names are case insensitive in this helper.
                candidate, report = shell.solve(tension_pattern(pattern, tension), initial=state)
                row = {"pattern": pattern, "tension_n_per_strand": tension,
                       "wall_s": time.perf_counter()-started, "accepted": report["accepted"],
                       "converged": report["converged"], "iterations": report["iterations"],
                       "residual": report["gradient_max_j_per_scaled_coordinate"],
                       "relative_rotation_rad": report["relative_rotation_rad"].tolist(),
                       "compression_fraction": report["compression_fraction"],
                       "max_membrane_strain": report["max_membrane_strain"],
                       "max_laminate_membrane_strain": report["max_laminate_membrane_strain"],
                       "max_pet_strip_membrane_strain": report["max_pet_strip_membrane_strain"],
                       "intersections": len(report["surface_intersection_pairs"]),
                       "state": candidate.tolist() if report["accepted"] else None}
                result["cases"].append(row)
                print(json.dumps({key: value for key, value in row.items() if key != "state"}), flush=True)
                save()
                if not report["accepted"]:
                    break
                state = candidate
    else:
        for microseconds in args.steps_us:
            started = time.perf_counter()
            model = ShellImpact(shell, ShellImpactConfig(step_s=microseconds*1e-6, duration_s=args.duration_ms/1000))
            while not model.stopped:
                model.step()
            row = {"step_s": model.config.step_s, "duration_s": model.time_s, "stop_reason": model.reason,
                   "peak_force_n": max(x["ground_force_n"] for x in model.trace),
                   "peak_bend_y_rad": max(abs(x["relative_rotation_rad"][1]) for x in model.trace),
                   "peak_compression_fraction": max(x["compression_fraction"] for x in model.trace),
                   "max_membrane_strain": max(x["max_membrane_strain"] for x in model.trace),
                   "final_energy_fraction": model.trace[-1]["energy_fraction_of_initial"],
                   "wall_s": time.perf_counter()-started, "trace": model.trace}
            result["cases"].append(row)
            print(json.dumps({key: value for key, value in row.items() if key != "trace"}), flush=True)
            save()
    print(str(destination), flush=True)


if __name__ == "__main__":
    main()

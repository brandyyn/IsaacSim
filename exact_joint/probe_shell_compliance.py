"""Isolate numerical/material sensitivities; never fit or promote a soft model.

Run with the numerical Python environment. Results are immutable timestamped
JSON; every change from the material reference is recorded explicitly.
"""

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


def main():
    torch.set_num_threads(4)
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=root/"results")
    args = parser.parse_args()
    source = load_source(root / "source_joint.json")
    reference = ShellConfig(sparse_solver=True, physical_strip_bending=True,
                            self_contact=True, crease_twist_ratio=0,
                            interior_refinement=1, max_iterations=300)
    variants = {
        "material_reference": {},
        "soft_strip_DIAGNOSTIC_NOT_MATERIAL": {"physical_strip_bending": False,
                                                "panel_to_crease_ratio": 10000},
        "soft_membrane_DIAGNOSTIC_NOT_MATERIAL": {"membrane_scale": .01},
        "soft_panel_bending_DIAGNOSTIC_NOT_MATERIAL": {"panel_bending_scale": .01},
        "eight_boundary_segments": {"subdivision": 3, "interior_refinement": 0},
        "eight_boundary_segments_refined": {"subdivision": 3},
    }
    report = {"source_sha256": source["sha256"], "scope": "Sensitivity only; no fitted material, ML or survival verdict",
              "material": dataclasses.asdict(JointConfig()), "torch_threads": 4,
              "code_sha256": {name: hashlib.sha256((root / name).read_bytes()).hexdigest()
                              for name in ("nonlinear_shell.py", "shell_sparse.py", "probe_shell_compliance.py")},
              "cases": []}
    destination = args.output_dir / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ") + "_compliance_probe.json")
    destination.parent.mkdir(parents=True, exist_ok=True)
    for name, changes in variants.items():
        shell = NonlinearShell(source, JointConfig(), dataclasses.replace(reference, **changes))
        for pattern in ("Compression", "Bend Y+", "Twist CW"):
            started = time.perf_counter()
            q, result = shell.solve(tension_pattern(pattern, 3), initial=np.zeros(shell.ndof))
            row = {"variant": name, "pattern": pattern, "tension_n_per_active_strand": 3,
                   "configuration": dataclasses.asdict(shell.config), "nodes": len(shell.points),
                   "triangles": len(shell.mesh["triangles"]), "wall_s": time.perf_counter() - started,
                   "compression_fraction": result["compression_fraction"],
                   "rotation_rad": result["relative_rotation_rad"].tolist(),
                   "energy_j": result["energy_j"], "accepted": result["accepted"],
                   "max_membrane_strain": result["max_membrane_strain"],
                   "residual": result["gradient_max_j_per_scaled_coordinate"],
                   "iterations": result["iterations"], "optimizer_message": result["optimizer_message"],
                   "intersection_pairs": result["surface_intersection_pairs"],
                   "self_contact": result.get("self_contact"), "state": q.tolist()}
            report["cases"].append(row)
            destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            print(json.dumps({k: v for k, v in row.items() if k not in ("state", "configuration", "intersection_pairs")}), flush=True)
    print(destination, flush=True)


if __name__ == "__main__":
    main()

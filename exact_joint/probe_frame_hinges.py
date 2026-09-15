"""Compare legacy roofs and PET frame flexures without softening any material.

Run from the project root with the existing numerical Python dependencies:
    python -m exact_joint.probe_frame_hinges
Results are exploratory response, not strength/survival or convergence proof.
"""

import argparse
import dataclasses
import hashlib
import json
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch

from exact_joint.geometry import JointConfig, load_source
from exact_joint.mechanics import tension_pattern
from exact_joint.nonlinear_shell import NonlinearShell, ShellConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--widths-mm", nargs="+", type=float, default=[0, .2, .4])
    parser.add_argument("--tension-n", type=float, default=3)
    parser.add_argument("--roof-force-n", type=float, default=1)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).with_name("results"))
    args = parser.parse_args()
    if not 0 <= args.tension_n <= 10 or not 0 <= args.roof_force_n <= 10:
        parser.error("Use loads between 0 and 10 N")
    torch.set_num_threads(4)
    root = Path(__file__).resolve().parent
    source = load_source(root/"source_joint.json")
    material = JointConfig()
    config = ShellConfig(sparse_solver=True, interior_refinement=1, physical_strip_bending=True,
                         self_contact=True, crease_twist_ratio=0)
    names = ("nonlinear_shell.py", "shell_sparse.py", "shell_ipc.py", "shell_contact.py",
             "shell_impact.py", "shell_actuation.py", "geometry.py", "mechanics.py", "probe_frame_hinges.py")
    result = {
        "source_sha256": source["sha256"], "material": dataclasses.asdict(material),
        "base_config": dataclasses.asdict(config),
        "starting_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
        "code_sha256": {name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in names},
        "scope": "New roof PLA setback/PET flexure layout; original 28/50/76 coarse geometry unchanged. No calibrated failure law.",
        "cases": [],
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    destination = args.output_dir/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_frame_hinges.json")
    for width in args.widths_mm:
        shell = NonlinearShell(source, material, dataclasses.replace(config, frame_hinge_width_m=width/1000))
        for pattern in ("Compression", "Bend Y+", "Twist CW", "Roof center"):
            started = time.perf_counter()
            if pattern == "Roof center":
                nodes = shell.free_nodes[np.isclose(shell.points[shell.free_nodes, 2], shell.height, atol=1e-12)]
                center = nodes[np.argmin(np.linalg.norm(shell.points[nodes, :2], axis=1))]
                loads = np.zeros_like(shell.points)
                loads[center, 2] = -args.roof_force_n
                q, report = shell.solve(nodal_loads=loads)
            else:
                q, report = shell.solve(tension_pattern(pattern, args.tension_n))
            row = {
                "frame_hinge_width_m": width/1000, "pattern": pattern,
                "shell_config": dataclasses.asdict(shell.config), "nodes": len(shell.points),
                "triangles": len(shell.faces), "tension_n_per_active_strand": 0 if pattern == "Roof center" else args.tension_n,
                "roof_force_n": args.roof_force_n if pattern == "Roof center" else 0,
                "accepted": report["accepted"], "converged": report["converged"],
                "residual": report["gradient_max_j_per_scaled_coordinate"],
                "compression_fraction": report["compression_fraction"],
                "relative_rotation_rad": report["relative_rotation_rad"].tolist(),
                "max_membrane_strain": report["max_membrane_strain"], "frame_hinge": report["frame_hinge"],
                "energy_j": report["energy_j"], "self_contact": report["self_contact"],
                "intersections": len(report["surface_intersection_pairs"]),
                "roof_center_deflection_m": float(shell.points[center, 2]-report["points"][center, 2]) if pattern == "Roof center" else None,
                "state": q.tolist(), "wall_s": time.perf_counter()-started,
            }
            result["cases"].append(row)
            destination.write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8", newline="\n")
            print(json.dumps({k: v for k, v in row.items() if k not in ("state", "shell_config")}), flush=True)
    print(destination, flush=True)


if __name__ == "__main__":
    main()

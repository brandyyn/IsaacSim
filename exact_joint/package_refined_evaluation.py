"""Immutable local evaluation package; compress raw calculations without loss."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--implementation-commit", required=True)
    parser.add_argument("--live-result", required=True)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    source_root = project/"exact_joint"
    results = source_root/"results"
    case_id, run_id = "exact_joint_refined_contact_v1", "20260914-exact-knee-refined-contact-v1"
    case_folder, run = project/"fea"/case_id, project/"ml/runs"/run_id
    if run.exists() or case_folder.exists():
        raise RuntimeError("Immutable case/run already exists; choose a new ID for another evaluation")
    run.mkdir(parents=True)
    case_folder.mkdir(parents=True)

    def save(path, data):
        path.write_text(json.dumps(data, indent=2)+"\n", encoding="utf-8")

    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    sources = {
        "refinement_1": results/"20260914T101406_237522Z_shell_study.json",
        "refinement_2": results/"20260914T101514_953111Z_shell_study.json",
        "force_contact": results/"20260914T102429_753929Z_shell_study.json",
        "wide_strip": results/"20260914T103135_941378Z_shell_study.json",
        "impact_before_stopping_fix": results/"20260914T102942_398056Z_shell_study.json",
        "impact_after_stopping_fix": results/"20260914T161600_116951Z_shell_study.json",
        "controller_checks": results/"20260914T103652_446157Z_refined_live.json",
        "live_demo": Path(args.live_result),
    }
    live = json.loads(sources["live_demo"].read_text())
    sources["live_calculation"] = results/live["saved_calculation"]/"calculation.json"
    provenance, studies = {}, {}
    for name, path in sources.items():
        raw = path.read_bytes()
        destination = run/(name+".json.gz")
        destination.write_bytes(gzip.compress(raw, mtime=0))
        provenance[name] = {"original_sha256": hashlib.sha256(raw).hexdigest(),
                            "archive": destination.name, "archive_sha256": digest(destination)}
        studies[name] = json.loads(raw)

    # Source intrinsic angle defects: a fabrication-connectivity diagnostic,
    # not an assertion that the physical module cannot fold.
    source = json.loads((source_root/"source_joint.json").read_text())
    points, lookup, sums = [], {}, []
    for panel in source["panels"]:
        poly = np.array([[p[k] for k in ("x", "y", "z")] for p in panel["points"]], dtype=float)
        for i, p in enumerate(poly):
            key = tuple(p)
            if key not in lookup:
                lookup[key] = len(points); points.append(p.tolist()); sums.append(0.)
            u, v = poly[i-1]-p, poly[(i+1) % len(poly)]-p
            sums[lookup[key]] += float(np.arctan2(np.linalg.norm(np.cross(u, v)), u@v))
    defects = 2*np.pi-np.array(sums)
    save(run/"reference_metric.json", {"source_sha256": digest(source_root/"source_joint.json"),
        "scope": "3D neutral geometry intrinsic-angle diagnostic; fabrication seam continuity unknown",
        "source_vertex_coordinates": points, "angle_defects_rad": defects.tolist(),
        "sum_angle_defects_rad": float(defects.sum())})

    first, second = studies["refinement_1"], studies["refinement_2"]
    differences = {}
    for pattern, axis in (("Compression", None), ("Bend Y+", 1), ("Twist CW", 2)):
        rows = [next(c for c in data["cases"] if c["pattern"] == pattern and c["tension_n_per_strand"] == 3)
                for data in (first, second)]
        values = [r["compression_fraction"] if axis is None else r["relative_rotation_rad"][axis] for r in rows]
        differences[pattern] = {"refinement_1": values[0], "refinement_2": values[1],
                                "relative_change_vs_refinement_1": abs((values[1]-values[0])/values[0])}
    before, after = studies["impact_before_stopping_fix"], studies["impact_after_stopping_fix"]
    impact = []
    for a, b in zip(before["cases"], after["cases"]):
        impact.append({"step_s": b["step_s"], "end_time_s": b["duration_s"], "window_max_ground_force_n": b["peak_force_n"],
            "window_max_bend_y_rad": b["peak_bend_y_rad"], "window_max_compression_fraction": b["peak_compression_fraction"],
            "wall_s_before": a["wall_s"], "wall_s_after": b["wall_s"],
            "maximum_reported_iterations_before": max(r["nonlinear_iterations"] for r in a["trace"]),
            "maximum_reported_iterations_after": max(r["nonlinear_iterations"] for r in b["trace"]),
            "relative_force_change_after_stopping_fix": abs(b["peak_force_n"]/a["peak_force_n"]-1),
            "stop_reason": b["stop_reason"]})
    metrics = {"numerical_tests_passed": 53, "controller_checks": studies["controller_checks"]["checks"],
        "desktop_apply_button": "Actual click verified 8 accepted steps at 3 N; rendered 1x deformation and loaded gold cables inspected",
        "live_demo": {k: v for k, v in live.items() if k not in ("trace", "code_sha256")},
        "live_torch_threads_observed": 20,
        "mesh_comparison_3n": differences, "mesh_5percent_gate_passed": False,
        "impact_stopping_comparison": impact, "full_motion_validated": False, "survives_drop": None,
        "notes": ["Offline benchmarks used 4 Torch threads; live Kit used 20. Not a like-for-like speed comparison.",
                  "Wall clock can include host scheduling/suspension; do not interpret the old long run as a controlled speedup measurement.",
                  "1 ms impact window is not the complete impact peak or a timestep-convergence result.",
                  "Reference/wide-gap calculations preceded final instrumentation; archived original code hashes remain explicit."]}
    save(run/"metrics.json", metrics)
    save(run/"provenance.json", provenance)
    save(run/"config.json", {"material": studies["live_calculation"]["material"],
                             "shell_config": studies["live_calculation"]["shell_config"],
                             "active_strand_force_n": 3, "patterns": 7, "steps_per_pattern": 6})

    parent = json.loads((project/"fea/exact_joint_winch_relief_v1/manifest.json").read_text())
    parent.update({"case_id": case_id, "parent_case": "exact_joint_winch_relief_v1",
                   "status": "implementation_evaluated_mesh_and_physical_calibration_failed_or_missing",
                   "starting_commit": "90576c6a392d49ea00bcb2ab134dda1081af2149",
                   "implementation_commit": args.implementation_commit})
    parent["geometry"]["geometry_revision"] = "Unchanged source 28/50/76 neutral geometry; intact PET, rigid square perimeters and free roof interiors"
    parent["geometry"].pop("relief_definition", None)
    parent["material"]["assumptions"] = "User thicknesses; unmeasured elastic moduli, perfectly bonded isotropic PLA/PET. Material reference uses physical PET bending and zero extra crease coupling. PET pre-creasing/plasticity uncalibrated."
    parent["mesh"].update({"element_count": 3648, "mesh_revision": "Uniform interior refinement 1 for live reference; 0/1/2 supported",
                           "convergence_summary": "Refinement 1 to 2 fails the 5% response gate; metrics in linked run"})
    parent["solver"].update({"software": "Project NumPy/SciPy/Torch sparse nonlinear shell + IPC Toolkit 1.6.0 rendered in Isaac",
                             "notes": "Actual elastic/cable/contact energy gradients; Gauss-Newton search tangent. Midsurface barrier and conservative curved-frame CCD, NOT finite-thickness contact. Impact numerical stopping target 1e-6; physical residual rejection remains 3e-5."})
    parent["outputs"] = {"raw_files": [f"ml/runs/{run_id}/{p.name}" for p in run.glob("*.gz")],
                          "processed_files": [f"ml/runs/{run_id}/metrics.json"], "response_model": None}
    parent["limitations"] = ["Full folding motion is not reproduced", "Spatial convergence not established",
        "Actual PET seam connectivity and cable guides unconfirmed", "Creased-film plasticity, hysteresis, bonds and print anisotropy uncalibrated",
        "Midsurface contact has no laminate thickness clearance, friction or cable contact",
        "Impact benchmark only covers first 1 ms after contact; no survival or full-peak result", "No training or baseline promotion"]
    parent["review"] = {"prepared_by": "Codex", "reviewed_by": [], "date_utc": "2026-09-14", "notes": "Local only; no external publication authorized"}
    save(case_folder/"manifest.json", parent)
    manifest = json.loads((project/"ml/run_manifest.template.json").read_text())
    manifest.update({"run_id": run_id, "status": "evaluated_not_calibrated_full_motion_unestablished",
        "baseline_commit": "5131a9740b3ce82e42331e923e1a45ffa396f71c", "starting_commit": parent["starting_commit"],
        "implementation_commit": args.implementation_commit, "simulator_build_commit": "5131a9740b3ce82e42331e923e1a45ffa396f71c",
        "isaac_sim_version": "6.0.1-rc.7 / Kit 110.1.2", "stage": "exact_joint/open_nonlinear_gui.py runtime overlay; v8 unchanged",
        "environment_version": case_id+"; not ML baseline", "fea_case_id": case_id,
        "fea_artifact_sha256": digest(case_folder/"manifest.json"), "algorithm": "deterministic implementation evaluation; no ML",
        "reward_version": "none", "training": {"command": "none", "steps": 0, "wall_time_s": 0},
        "evaluation": {"command": "See archived arguments, measure_demo_live.py and CONTACT_AND_REFINEMENT.md",
                       "conditions": "Intact 30 mm joint, PLA 0.4 mm / PET 80 um; assumed constitutive properties", "metrics": "metrics.json"},
        "artifacts": {"config": "config.json", "metrics": "metrics.json", "checkpoint": None, "checkpoint_sha256": None,
                      "sha256": {p.name: digest(p) for p in run.iterdir() if p.is_file()}},
        "implementation_sha256": live["code_sha256"],
        "notes": "Exact original raw bytes archived with gzip; no external sharing, training or baseline replacement."})
    save(run/"run_manifest.json", manifest)
    print(run)


if __name__ == "__main__":
    main()

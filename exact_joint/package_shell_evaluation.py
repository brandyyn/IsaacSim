"""Package existing local numerical/UI evidence without overwriting a run."""

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--code-commit", required=True)
    parser.add_argument("--coarse", type=Path, required=True)
    parser.add_argument("--fine", type=Path, required=True)
    parser.add_argument("--sensitivity", type=Path, required=True)
    parser.add_argument("--timesteps", type=Path, required=True)
    parser.add_argument("--ui", type=Path, required=True)
    parser.add_argument("--replay-ui", type=Path, required=True)
    parser.add_argument("--screenshot", type=Path, required=True)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    run_id = "20260909-exact-knee-frame-shell-v1"
    output = project/"ml/runs"/run_id
    case = project/"fea/exact_joint_frame_shell_v1/manifest.json"
    source = project/"exact_joint/source_joint.json"
    data = {name: json.loads(getattr(args, name).read_text(encoding="utf-8"))
            for name in ("coarse", "fine", "sensitivity", "timesteps", "ui", "replay_ui")}
    assert json.loads(case.read_text())["geometry"]["sha256"] == digest(source)
    assert all(data["ui"]["checks"].values())
    assert all(data["replay_ui"][key] for key in ("native_timeline_apply_pause_resume", "replay_recorded_force_matched", "peak_inspection"))
    calculation_path = project/data["replay_ui"]["calculation"]
    calculation = json.loads(calculation_path.read_text(encoding="utf-8"))
    assert calculation["source_sha256"] == digest(source)
    assert len(calculation["state"]) == 1338
    image_array = np.asarray(Image.open(args.screenshot).convert("RGB"))
    assert args.screenshot.stat().st_size >= 150000 and image_array.mean() > 30
    assert len(data["coarse"]["cases"]) == len(data["fine"]["cases"]) == 3
    assert len(data["timesteps"]["cases"]) == 4

    spatial = []
    for coarse, fine in zip(data["coarse"]["cases"], data["fine"]["cases"]):
        assert coarse["pattern"] == fine["pattern"] and coarse["accepted"] and fine["accepted"]
        if coarse["pattern"] == "Compression":
            before, after = coarse["compression_fraction"], fine["compression_fraction"]
        else:
            axis = 1 if coarse["pattern"] == "Bend Y+" else 2
            before, after = coarse["relative_rotation_rad"][axis], fine["relative_rotation_rad"][axis]
        spatial.append({"pattern": coarse["pattern"], "relative_change": abs(after-before)/abs(before),
                        "passes_5_percent": abs(after-before)/abs(before) <= .05})
    temporal = {}
    earlier, final = data["timesteps"]["cases"][-2:]
    for key in ("peak_force_n", "peak_bend_y_rad", "peak_compression_fraction", "max_membrane_strain"):
        temporal[key] = abs(final[key]-earlier[key])/abs(earlier[key])
    trace = calculation["impact_trace"]
    metrics = {
        "numeric_regression_tests": {"passed": 36, "failed": 0, "scope": "Implementation, not physical validation"},
        "actual_ui_checks": data["ui"]["checks"], "timeline_replay_ui": data["replay_ui"],
        "spatial_comparison": spatial, "spatial_gate_passed": all(row["passes_5_percent"] for row in spatial),
        "timestep_25_to_12_5_us_relative_changes": temporal,
        "timestep_response_gate_passed_for_this_case": max(temporal.values()) <= .05,
        "live_drop": {"config": calculation["impact_config"], "shell_mass_kg": calculation["shell_mass_kg"],
                      "steps_recorded": len(trace)-1, "time_after_contact_s": trace[-1]["time_after_contact_s"],
                      "peak_force_n": max(row["ground_force_n"] for row in trace),
                      "peak_bend_y_rad": max(abs(row["relative_rotation_rad"][1]) for row in trace),
                      "peak_compression_fraction": max(row["compression_fraction"] for row in trace),
                      "final_energy_fraction": trace[-1]["energy_fraction_of_initial"]},
        "full_motion_demonstrated": False, "physically_calibrated": False, "survives": None,
        "screenshot": {"width": image_array.shape[1], "height": image_array.shape[0],
                       "mean_rgb": float(image_array.mean()), "std_rgb": float(image_array.std())},
        "limitations": ["spatial response gate failed", "unmeasured pre-crease/material/contact law",
                        "no full stress/failure/self-contact model", "frictionless normal drop; no ground Z torque"]}
    evidence = {name: {"local_path": getattr(args, name).resolve().relative_to(project).as_posix(),
                      "sha256": digest(getattr(args, name))}
                for name in ("coarse", "fine", "sensitivity", "timesteps", "ui", "replay_ui")}
    evidence["full_live_calculation_local_only"] = {"local_path": calculation_path.relative_to(project).as_posix(),
                                                  "sha256": digest(calculation_path)}
    summaries = {name: {**data[name], "cases": [{key: value for key, value in row.items() if key not in ("state", "trace")}
                                               for row in data[name]["cases"]]}
                 for name in ("coarse", "fine", "sensitivity", "timesteps")}
    output.mkdir(parents=True, exist_ok=False)
    (output/".gitattributes").write_text("*.json -text\n*.png -text\n", encoding="utf-8", newline="\n")
    def write(name, payload):
        (output/name).write_text(json.dumps(payload, indent=2)+"\n", encoding="utf-8", newline="\n")
    write("config.json", {"material": calculation["material"], "shell": calculation["shell_config"], "impact": calculation["impact_config"]})
    write("metrics.json", metrics)
    write("studies.json", summaries)
    write("impact_trace.json", trace)
    write("provenance.json", evidence)
    shutil.copyfile(args.screenshot, output/"workshop_peak.png")
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    for row in data["timesteps"]["cases"]:
        times = np.array([sample["time_after_contact_s"] for sample in row["trace"]])*1000
        for ax, key, scale, ylabel in ((axes[0, 0], "ground_force_n", 1, "Ground force (N)"),
                                      (axes[1, 0], "compression_fraction", 100, "Compression (%)"),
                                      (axes[1, 1], "energy_fraction_of_initial", 100, "Mechanical energy / incident (%)")):
            ax.plot(times, [sample[key]*scale for sample in row["trace"]], label=f"{row['step_s']*1e6:g} us")
            ax.set_ylabel(ylabel)
        axes[0, 1].plot(times, np.rad2deg([sample["relative_rotation_rad"][1] for sample in row["trace"]]))
    axes[0, 1].set_ylabel("Y bending (degrees)")
    for ax in axes.flat:
        ax.grid(alpha=.22)
        ax.set_xlabel("Time after first contact (ms)")
    axes[0, 0].legend(frameon=False)
    fig.suptitle("70 mm whole-leg drop | one nonlinear knee | assumed 50 g\nUncalibrated; spatial gate FAILED; no survival verdict", fontsize=14)
    fig.savefig(output/"impact_timestep_comparison.png", dpi=150)
    plt.close(fig)
    hashes = {path.name: digest(path) for path in output.iterdir() if path.is_file()}
    manifest = json.loads((project/"ml/run_manifest.template.json").read_text())
    manifest.update({"run_id": run_id, "status": "evaluation_complete_physical_and_spatial_gates_failed",
                     "baseline_commit": "ad6f15d4e2b785db1b1ccaa5093dc998997d63ef",
                     "parent_local_commit": "cafe147a64b40114529a19c1e5c8b67131f67843",
                     "implementation_commit": args.code_commit, "isaac_sim_version": "6.0.1-rc.7 / Kit 110.1.2",
                     "simulator_build_commit": "5131a9740b3ce82e42331e923e1a45ffa396f71c",
                     "stage": "exact_joint/open_nonlinear_gui.py generated opt-in stage",
                     "environment_version": "exact_joint_frame_shell_v1; not ML baseline",
                     "fea_case_id": "exact_joint_frame_shell_v1", "fea_artifact_sha256": digest(case),
                     "algorithm": "deterministic evaluation; no ML", "observations": [], "actions": [],
                     "reward_version": "none", "training": {"command": "none", "steps": 0, "wall_time_s": 0},
                     "evaluation": {"command": "See exact_joint/NONLINEAR_GUIDE.md reproducibility commands",
                                    "conditions": "30 mm knee, 0.4 mm PLA / 80 um PET; assumed 70 mm/50 g drop",
                                    "metrics": "metrics.json"},
                     "artifacts": {"config": "config.json", "metrics": "metrics.json", "checkpoint": None,
                                   "checkpoint_sha256": None, "sha256": hashes},
                     "notes": "Local evaluation. Publication pending approval. Study summaries retain their own source-code hashes; timestep/sensitivity studies precede the final narrow-cell altitude precision fix. Final-code live drop agrees. Raw local state logs are hashed but not required for reproducibility. No full range or survival demonstrated."})
    write("run_manifest.json", manifest)
    print(json.dumps({"run": str(output), "spatial": spatial, "timestep_changes": temporal, "live_drop": metrics["live_drop"]}), flush=True)


if __name__ == "__main__":
    main()

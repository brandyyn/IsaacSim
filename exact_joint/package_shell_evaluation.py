"""Package existing local numerical/UI evidence without overwriting a run."""

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def plot_timesteps(cases, destination):
    """Dependency-light scientific line figure; all coordinates come from traces."""
    canvas = Image.new("RGB", (1800, 1200), "#ffffff")
    draw = ImageDraw.Draw(canvas)
    def font(size):
        for name in ("arial.ttf", "DejaVuSans.ttf"):
            try:
                return ImageFont.truetype(name, size)
            except OSError:
                pass
        return ImageFont.load_default(size=size)
    draw.text((65, 35), "70 mm whole-leg drop | one nonlinear knee | assumed 50 g", font=font(34), fill="#172b3a")
    draw.text((65, 83), "Uncalibrated shell. Spatial response gate FAILED. No survival verdict.", font=font(27), fill="#a23f2f")
    colors = ("#79838d", "#e69932", "#257bc1", "#ad3977")
    for i, case in enumerate(cases):
        x = 100+420*i
        draw.line((x, 144, x+45, 144), fill=colors[i], width=4)
        draw.text((x+60, 127), f"{case['step_s']*1e6:g} us timestep", font=font(24), fill="#263849")
    panels = (("Ground force (N)", "ground_force_n", 1, 0, 40),
              ("Y bending (degrees)", "relative_rotation_rad", 180/np.pi, -.22, .12),
              ("Compression (%)", "compression_fraction", 100, -.15, .7),
              ("Mechanical energy / incident (%)", "energy_fraction_of_initial", 100, 75, 100))
    for index, (title, key, scale, ymin, ymax) in enumerate(panels):
        left, top = 125+870*(index % 2), 235+465*(index//2)
        width, height = 675, 325
        draw.text((left, top-45), title, font=font(27), fill="#172b3a")
        for tick in range(5):
            x = left+width*tick/4
            y = top+height-height*tick/4
            draw.line((x, top, x, top+height), fill="#e0e5e8", width=1)
            draw.line((left, y, left+width, y), fill="#e0e5e8", width=1)
            draw.text((x, top+height+12), f"{2*tick:g}", font=font(22), fill="#425565", anchor="mt")
            draw.text((left-12, y), f"{ymin+(ymax-ymin)*tick/4:.3g}", font=font(22), fill="#425565", anchor="rm")
        for case, color in zip(cases, colors):
            points = []
            for row in case["trace"]:
                value = row[key][1] if key == "relative_rotation_rad" else row[key]
                x = left+width*row["time_after_contact_s"]/.008
                y = top+height*(1-(value*scale-ymin)/(ymax-ymin))
                if not left-1 <= x <= left+width+1 or not top-1 <= y <= top+height+1:
                    raise ValueError("Plot range does not contain the supplied trace")
                points.append((x, y))
            draw.line(points, fill=color, width=3)
        draw.rectangle((left, top, left+width, top+height), outline="#637382", width=2)
        draw.text((left+width/2, top+height+53), "Time after first contact (ms)", font=font(22), fill="#425565", anchor="mt")
    draw.text((65, 1160), "Backward Euler introduces numerical damping; timestep agreement does not establish physical accuracy.",
              font=font(22), fill="#425565")
    canvas.save(destination)


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
    plot_timesteps(data["timesteps"]["cases"], output/"impact_timestep_comparison.png")
    hashes = {path.name: digest(path) for path in output.iterdir() if path.is_file()}
    manifest = json.loads((project/"ml/run_manifest.template.json").read_text())
    manifest.update({"run_id": run_id, "status": "evaluation_complete_physical_and_spatial_gates_failed",
                     "baseline_commit": "ad6f15d4e2b785db1b1ccaa5093dc998997d63ef",
                     "parent_local_commit": "cafe147a64b40114529a19c1e5c8b67131f67843",
                     "implementation_commit": args.code_commit, "isaac_sim_version": "6.0.1-rc.7 / Kit 110.1.2",
                     "artifact_generator_sha256": digest(Path(__file__)),
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

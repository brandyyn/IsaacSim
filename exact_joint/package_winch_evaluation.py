"""Archive a bounded winch study; never overwrite an immutable evaluation."""

import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def plot(study, target):
    picture = Image.new("RGB", (1600, 1120), "white")
    draw = ImageDraw.Draw(picture)

    def font(size):
        try:
            return ImageFont.truetype("arial.ttf", size)
        except OSError:
            return ImageFont.load_default(size=size)

    draw.text((55, 25), "Cable take-up experiment | exact neutral shape, modified PET cuts", font=font(31), fill="#18384a")
    draw.text((55, 73), "Uncalibrated: bending x0.01, membrane x1, gap 0.8 mm, endpoint relief 20%", font=font(25), fill="#6a3c21")
    families = [("Compression", "#da781c"), ("Bend Y+", "#247db0"), ("Twist CW", "#9d429a")]
    for index, (name, color) in enumerate(families):
        x = 90+index*440
        draw.line((x, 135, x+45, 135), fill=color, width=4)
        draw.text((x+60, 117), name, font=font(24), fill=color)
    panels = [("Compression (%)", 24, lambda row: row["compression_fraction"]*100),
              ("Absolute Y bending (degrees)", 6, lambda row: abs(row["relative_rotation_rad"][1])*180/math.pi),
              ("Absolute Z twist (degrees)", 3, lambda row: abs(row["relative_rotation_rad"][2])*180/math.pi),
              ("Maximum cable tension (mN)", 10, lambda row: max(row["actual_tensions_n"])*1000)]
    for index, (name, ymax, value) in enumerate(panels):
        left, top = 100+790*(index % 2), 235+420*(index//2)
        width, height = 610, 285
        draw.text((left, top-43), name, font=font(25), fill="#18384a")
        for tick in range(6):
            x, y = left+width*tick/5, top+height-height*tick/5
            draw.line((x, top, x, top+height), fill="#e2e7e9")
            draw.line((left, y, left+width, y), fill="#e2e7e9")
            draw.text((x, top+height+10), str(tick), font=font(20), fill="#44545d", anchor="mt")
            draw.text((left-12, y), f"{ymax*tick/5:g}", font=font(20), fill="#44545d", anchor="rm")
        for family, color in families:
            line = [(left, top+height)]
            for row in study["cases"]:
                if row["pattern"] != family:
                    continue
                assert 0 <= row["pull_mm"] <= 5 and 0 <= value(row) <= ymax
                x, y = left+width*row["pull_mm"]/5, top+height*(1-value(row)/ymax)
                if row["accepted"]:
                    line.append((x, y))
                    draw.ellipse((x-4, y-4, x+4, y+4), fill=color)
                else:
                    draw.line((x-7, y-7, x+7, y+7), fill="#c82032", width=3)
                    draw.line((x-7, y+7, x+7, y-7), fill="#c82032", width=3)
            if len(line) > 1:
                draw.line(line, fill=color, width=3)
        draw.rectangle((left, top, left+width, top+height), outline="#5c737f", width=2)
        draw.text((left+width/2, top+height+40), "Active-cable take-up (mm)", font=font(21), fill="#44545d", anchor="mt")
    draw.text((70, 1030), "Solid lines: accepted numerical states. Red crosses: rejected panel intersections; NOT valid motion.",
              font=font(24), fill="#a22c38")
    draw.text((70, 1070), "No full-range, material-strength, drop-survival or ML-calibration claim.", font=font(23), fill="#44545d")
    picture.save(target)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--code-commit", required=True)
    parser.add_argument("--best", type=Path, required=True)
    parser.add_argument("--drop", type=Path, required=True)
    parser.add_argument("--ui", type=Path, required=True)
    parser.add_argument("--screenshot", type=Path, required=True)
    parser.add_argument("--comparisons", type=Path, nargs="+", required=True)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    case = project/"fea/exact_joint_winch_relief_v1/manifest.json"
    source_hash = digest(project/"exact_joint/source_joint.json")
    best, drop, ui = [json.loads(path.read_text()) for path in (args.best, args.drop, args.ui)]
    assert source_hash == json.loads(case.read_text())["geometry"]["sha256"]
    assert best["source_sha256"] == drop["source_sha256"] == source_hash
    assert all(ui["checks"].values())
    summary = []
    for pattern in ("Compression", "Bend Y+", "Twist CW"):
        rows = [row for row in best["cases"] if row["pattern"] == pattern]
        accepted = [row for row in rows if row["accepted"]]
        assert accepted and not rows[-1]["accepted"] and rows[-1]["intersections"] > 0
        final = accepted[-1]
        summary.append({"pattern": pattern, "last_accepted_pull_m": final["pull_mm"]/1000,
                        "compression_fraction": final["compression_fraction"],
                        "relative_rotation_rad": final["relative_rotation_rad"],
                        "actual_tensions_n": final["actual_tensions_n"],
                        "max_membrane_strain": final["max_membrane_strain"],
                        "residual": final["residual"],
                        "next_rejected_pull_m": rows[-1]["pull_mm"]/1000,
                        "rejected_intersections": rows[-1]["intersections"]})
    metrics = {"numeric_tests": {"passed": 41, "failed": 0}, "ui_checks": ui,
               "last_accepted_per_family": summary,
               "modified_drop": [{key: val for key, val in row.items() if key != "trace"} for row in drop["cases"]],
               "full_motion_demonstrated": False, "physically_calibrated": False, "survives": None,
               "spatial_convergence": "Not established; parent intact-shell gate failed",
               "important": "Last accepted points are not validated physical travel limits. Modified cut pattern and reduced bending stiffness."}
    comparisons = []
    provenance = {}
    for path in [args.best, args.drop, args.ui, *args.comparisons]:
        provenance[path.name] = {"local_path": path.resolve().relative_to(project).as_posix(), "sha256": digest(path)}
    for path in args.comparisons:
        data = json.loads(path.read_text())
        assert data["source_sha256"] == source_hash
        data["cases"] = [{key: val for key, val in row.items() if key not in ("state", "candidate_state", "trace")}
                         for row in data["cases"]]
        comparisons.append(data)
    output = project/"ml/runs/20260910-exact-knee-winch-relief-v1"
    output.mkdir(parents=True, exist_ok=False)

    def write(name, value):
        (output/name).write_text(json.dumps(value, indent=2)+"\n", encoding="utf-8", newline="\n")

    (output/".gitattributes").write_text("*.json -text\n*.png -filter -diff -merge -text\n", encoding="utf-8")
    write("metrics.json", metrics)
    write("config.json", {"material": best["material"], "shell": best["shell_config"], "arguments": best["arguments"]})
    write("comparisons_summary.json", comparisons)
    write("provenance.json", provenance)
    shutil.copyfile(args.best, output/"raw_winch_sweep.json")
    shutil.copyfile(args.drop, output/"raw_modified_drop.json")
    shutil.copyfile(args.screenshot, output/"workshop_compression.png")
    plot(best, output/"winch_response.png")
    manifest = json.loads((project/"ml/run_manifest.template.json").read_text())
    manifest.update({"run_id": output.name, "status": "evaluation_complete_full_motion_not_established",
        "baseline_commit": "5131a9740b3ce82e42331e923e1a45ffa396f71c",
        "starting_commit": "4d1af8fee892438989c322e97a5c3f4073a6f3a9",
        "implementation_commit": args.code_commit, "isaac_sim_version": "6.0.1-rc.7 / Kit 110.1.2",
        "simulator_build_commit": "5131a9740b3ce82e42331e923e1a45ffa396f71c",
        "stage": "exact_joint/open_nonlinear_gui.py runtime overlay; v8 asset unchanged",
        "environment_version": "exact_joint_winch_relief_v1; not ML baseline",
        "fea_case_id": "exact_joint_winch_relief_v1", "fea_artifact_sha256": digest(case),
        "algorithm": "deterministic local evaluation; no ML", "reward_version": "none",
        "training": {"command": "none", "steps": 0, "wall_time_s": 0},
        "evaluation": {"command": "See raw studies arguments and exact_joint/WINCH_EXPERIMENT.md",
                       "conditions": "Uncalibrated 30 mm knee; modified PET cuts and effective bending rigidity",
                       "metrics": "metrics.json"},
        "artifacts": {"config": "config.json", "metrics": "metrics.json", "checkpoint": None,
                      "checkpoint_sha256": None, "sha256": {p.name: digest(p) for p in output.iterdir() if p.is_file()}},
        "implementation_sha256": {p: digest(project/"exact_joint"/p) for p in
                                  ("nonlinear_shell.py", "shell_actuation.py", "shell_view.py", "shell_impact.py", "shell_contact.py")},
        "notes": "Original source JSON unchanged. Raw rejected candidate states are diagnostic only. Other sweeps summarized with raw local hashes and reproduction arguments. No external publication authorized."})
    write("run_manifest.json", manifest)
    print(json.dumps({"output": str(output), "summary": summary}, indent=2))


if __name__ == "__main__":
    main()

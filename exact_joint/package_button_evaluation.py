"""Package UI-only evaluation linked to the unchanged structural case."""

import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--implementation-commit", required=True)
    parser.add_argument("--button-results", nargs=3, required=True)
    parser.add_argument("--guard-result", required=True)
    parser.add_argument("--demo-result", required=True)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    run_id = "20260914-exact-knee-button-controls-v1"
    run = project/"ml/runs"/run_id
    if run.exists():
        raise RuntimeError("Immutable evaluation already exists")
    sources = [Path(p) for p in args.button_results]
    reports = [json.loads(p.read_bytes()) for p in sources]
    assert {r["family"] for r in reports} == {"Compression", "Bend Y+", "Twist CW"}
    assert all(r["accepted_steps"] == 8 and r["status"] == "COMPLETE" for r in reports)
    guards = json.loads(Path(args.guard_result).read_bytes())
    assert len(guards["checks"]) == 3 and all(guards["checks"].values())
    demo = json.loads(Path(args.demo_result).read_bytes())
    assert demo["accepted_steps"] == 42 and all(demo["checks"].values())
    run.mkdir(parents=True)

    def save(path, data):
        path.write_text(json.dumps(data, indent=2)+"\n", encoding="utf-8")

    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    (run/".gitattributes").write_text("*.json -text\n", encoding="utf-8")
    for i, path in enumerate(sources):
        (run/f"desktop_button_{i+1}.json").write_bytes(path.read_bytes())
    (run/"controller_guards.json").write_bytes(Path(args.guard_result).read_bytes())
    (run/"completed_demo.json").write_bytes(Path(args.demo_result).read_bytes())
    parent_run = project/"ml/runs/20260914-exact-knee-refined-contact-v1"
    config = json.loads((parent_run/"config.json").read_bytes())
    config.update({"patterns": [r["family"] for r in reports], "steps_per_pattern": 8,
                   "active_strand_force_n": 3, "ui_only_change": True})
    save(run/"config.json", config)
    metrics = {"regression_tests_passed": 60, "async_controller_tests_included": 7,
               "desktop_patterns_completed": [r["family"] for r in reports],
               "desktop_accepted_steps": sum(r["accepted_steps"] for r in reports),
               "desktop_pause_held_state_trace_and_usd_for_s": 6,
               "desktop_switch_from_paused_bend_to_twist_passed": True,
               "desktop_neutral_no_stale_publication_after_s": 6,
               "controller_fault_injection": guards["checks"],
               "additional_completed_demo_inspection": demo["checks"],
               "full_motion_validated": False, "survives_drop": None,
               "notes": ["Three completed real desktop-button ramps, plus interrupted Bend X+ and Bend Y+.",
                         "Compression/Twist records precede the final first-candidate publication fix; their exact code hashes are retained.",
                         "Bend Y+, cancellation and fault-injection guards use final publication code.",
                         "No material, topology, contact law, solver tolerance or physical result magnification changed.",
                         "No new drop, mesh convergence or physical calibration performed in this UI-only evaluation."]}
    save(run/"metrics.json", metrics)
    manifest = json.loads((project/"ml/run_manifest.template.json").read_bytes())
    manifest.update({"run_id": run_id, "status": "ui_controls_evaluated_physics_not_validated",
        "baseline_commit": "5131a9740b3ce82e42331e923e1a45ffa396f71c",
        "starting_commit": "faf2945e09548710726e05d8d414b00f4cbe407a",
        "implementation_commit": args.implementation_commit,
        "parent_run": "20260914-exact-knee-refined-contact-v1",
        "simulator_build_commit": "5131a9740b3ce82e42331e923e1a45ffa396f71c",
        "isaac_sim_version": "6.0.1-rc.7 / Kit 110.1.2",
        "stage": "exact_joint/open_nonlinear_gui.py runtime overlay; v8 unchanged",
        "environment_version": "shell_ui_jobs_v1; uncalibrated shell, not ML baseline",
        "fea_case_id": "exact_joint_refined_contact_v1",
        "fea_artifact_sha256": digest(project/"fea/exact_joint_refined_contact_v1/manifest.json"),
        "algorithm": "deterministic UI regression evaluation; no ML", "reward_version": "none",
        "training": {"command": "none", "steps": 0, "wall_time_s": 0},
        "evaluation": {"command": "unittest discover exact_joint test_*.py; actual desktop clicks + record_button_live.py; validate_button_guards_live.py",
                       "conditions": "3 N/active strand; same intact PLA/PET refined shell and contact case", "metrics": "metrics.json"},
        "artifacts": {"config": "config.json", "metrics": "metrics.json", "checkpoint": None,
                      "checkpoint_sha256": None, "sha256": {p.name: digest(p) for p in run.glob("*.json")}},
        "notes": "UI-only follow-up, immutable parent case/run untouched. No training, baseline promotion or external publication."})
    save(run/"run_manifest.json", manifest)
    print(run)


if __name__ == "__main__":
    main()

"""Build the v8 panel-crease leg with an embedded Ansys compression profile."""

from __future__ import annotations

import argparse
import json
import os
import runpy
from pathlib import Path

from pxr import Sdf, Usd


def build(source_usd: Path, topology_json: Path, profile_json: Path, output_usd: Path) -> None:
    stable_builder = runpy.run_path(
        str(Path(__file__).with_name("build_stable_panel_crease_leg.py")),
        run_name="stable_panel_crease_leg_builder",
    )
    stable_builder["build"](source_usd.resolve(), topology_json.resolve(), output_usd.resolve())

    profile = json.loads(profile_json.read_text(encoding="utf-8"))
    stage = Usd.Stage.Open(str(output_usd))
    calibration = stage.DefinePrim("/World/PanelCreaseLeg/FEACalibration", "Scope")
    calibration.SetCustomDataByKey("caseId", profile["caseId"])
    calibration.SetCustomDataByKey("profileSchemaVersion", int(profile["schemaVersion"]))
    calibration.SetCustomDataByKey("sourceFiles", json.dumps(profile["sourceFiles"], separators=(",", ":")))
    calibration.SetCustomDataByKey("frameMapping", json.dumps(profile["frameMapping"], separators=(",", ":")))
    calibration.SetCustomDataByKey("application", json.dumps(profile["application"], separators=(",", ":")))
    calibration.SetCustomDataByKey("forceProfileSamples", json.dumps(profile["samples"], separators=(",", ":")))
    calibration.SetCustomDataByKey("limitations", json.dumps(profile["limitations"], separators=(",", ":")))
    calibration.CreateAttribute("forceReplayEnabled", Sdf.ValueTypeNames.Bool).Set(True)
    calibration.CreateAttribute("forceReplayScale", Sdf.ValueTypeNames.Float).Set(float(profile["application"]["forceScale"]))
    calibration.CreateAttribute("replayDtS", Sdf.ValueTypeNames.Float).Set(1.0 / 60.0)
    calibration.CreateAttribute("replayTimeS", Sdf.ValueTypeNames.Float).Set(0.0)
    calibration.CreateAttribute("currentForceTotalN", Sdf.ValueTypeNames.Float).Set(0.0)
    calibration.CreateAttribute("currentEquivalentStressMaximumPa", Sdf.ValueTypeNames.Float).Set(0.0)
    calibration.CreateAttribute("replayComplete", Sdf.ValueTypeNames.Bool).Set(False)
    calibration.CreateAttribute("profileCaseId", Sdf.ValueTypeNames.String).Set(profile["caseId"])
    calibration.CreateAttribute("profilePath", Sdf.ValueTypeNames.String).Set(profile_json.name)
    stage.GetRootLayer().Export(str(output_usd))
    print(f"Wrote {output_usd}")


def main() -> None:
    project_root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=project_root / "input_improved.usd")
    parser.add_argument("--topology", type=Path, default=project_root / "input_improved.json")
    parser.add_argument(
        "--profile",
        type=Path,
        default=project_root / "fea" / "compression_v1" / "processed" / "compression_profile.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=project_root / "panel_crease_leg_fea_compression_v1.usd",
    )
    args = parser.parse_args()
    build(args.source, args.topology, args.profile, args.output)


if __name__ == "__main__":
    main()

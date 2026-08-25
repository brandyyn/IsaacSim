"""Project-root wrapper for the coupled video-profile panel knee stage."""

import os
import runpy
from pathlib import Path


project_root = Path(__file__).resolve().parent
if not (project_root / "build_coupled_fold_panel_leg.py").exists():
    project_root = Path(os.environ.get("PANEL_CREASE_PROJECT_ROOT", Path.cwd())).expanduser().resolve()
source_usd = project_root / "input_improved.usd"
topology_json = project_root / "input_improved.json"
profile_json = project_root / "coupled_fold_profile_video_v1.json"
output_usd = project_root / os.environ.get(
    "PANEL_CREASE_COUPLED_OUTPUT_NAME",
    "panel_crease_leg_constraints_v8_geometry_fold.usd",
)

module = runpy.run_path(
    str(project_root / "build_coupled_fold_panel_leg.py"),
    run_name="coupled_fold_panel_leg_builder",
)
module["build"](source_usd, topology_json, profile_json, output_usd)

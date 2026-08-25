"""Build the multi-axis original-joint showcase from project-root assets."""

import os
import runpy
from pathlib import Path


project_root = Path(__file__).resolve().parent
if not (project_root / "build_panel_crease_showcase.py").exists():
    project_root = Path(os.environ.get("PANEL_CREASE_PROJECT_ROOT", Path.cwd())).expanduser().resolve()

module = runpy.run_path(
    str(project_root / "build_panel_crease_showcase.py"),
    run_name="panel_crease_showcase_builder",
)
module["build"](
    project_root / "input_improved.usd",
    project_root / "input_improved.json",
    project_root / "coupled_fold_profile_video_v1.json",
    project_root / os.environ.get(
        "PANEL_CREASE_SHOWCASE_OUTPUT_NAME",
        "panel_crease_leg_constraints_v9_showcase.usd",
    ),
)

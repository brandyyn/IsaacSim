"""Build the FEA compression v1 stage from the project root inside Isaac Sim."""

import os
import runpy
from pathlib import Path


def _project_root() -> Path:
    configured = os.environ.get("PANEL_CREASE_PROJECT_ROOT")
    candidates = [Path(configured).expanduser() if configured else Path.cwd()]
    try:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            candidates.append(Path(stage.GetRootLayer().identifier).resolve().parent)
    except Exception:
        pass
    for candidate in candidates:
        if (candidate / "build_fea_calibrated_panel_crease_leg.py").exists():
            return candidate.resolve()
    raise RuntimeError("Set PANEL_CREASE_PROJECT_ROOT to the cloned project root before building.")


project_root = _project_root()
module = runpy.run_path(
    str(project_root / "build_fea_calibrated_panel_crease_leg.py"),
    run_name="fea_calibrated_panel_crease_leg_builder",
)
module["build"](
    project_root / "input_improved.usd",
    project_root / "input_improved.json",
    project_root / "fea" / "compression_v1" / "processed" / "compression_profile.json",
    project_root / "panel_crease_leg_fea_compression_v1.usd",
)

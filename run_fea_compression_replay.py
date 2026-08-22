"""Run one embedded Ansys compression profile replay in the current Isaac stage."""

import json
import os
from pathlib import Path
import runpy
import sys

import omni.usd


project_root = Path(os.environ.get("PANEL_CREASE_PROJECT_ROOT", Path.cwd())).expanduser().resolve()
if not (project_root / "fea_compression_controller.py").exists():
    stage = omni.usd.get_context().get_stage()
    if stage is not None:
        candidate = Path(stage.GetRootLayer().identifier).resolve().parent
        if (candidate / "fea_compression_controller.py").exists():
            project_root = candidate
sys.path.insert(0, str(project_root))

fea_compression_controller = runpy.run_path(
    str(project_root / "fea_compression_controller.py"),
    run_name="fea_compression_controller_runtime",
)

if "force_scale" not in dir():
    force_scale = None
if "target_deg" not in dir():
    target_deg = 0.0
if "output_path" not in dir():
    output_path = None

result = await fea_compression_controller["replay_once"](force_scale=force_scale, target_deg=target_deg)
if output_path:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {path}")

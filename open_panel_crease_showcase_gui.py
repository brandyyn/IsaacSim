"""Open the optimized multi-axis original-joint showcase in Isaac Sim."""

import os
import runpy


os.environ.setdefault("PANEL_CREASE_PROJECT_ROOT", "S:/dev/isaacsim")
os.environ["PANEL_CREASE_COUPLED_STAGE_NAME"] = os.environ.get(
    "PANEL_CREASE_SHOWCASE_STAGE_NAME",
    "panel_crease_leg_constraints_v9_showcase.usd",
)
os.environ["PANEL_CREASE_MOTION_MODE"] = "showcase"
os.environ["PANEL_CREASE_VISUAL_MODE"] = "baked"
runpy.run_path("S:/dev/isaacsim/open_coupled_fold_panel_gui.py", run_name="__main__")

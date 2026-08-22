"""Open the source-panel crease-folding leg stage in a visible Isaac Sim app."""

import asyncio
import os
from pathlib import Path
import sys

import omni.kit.app
import omni.usd

project_root = Path(__file__).resolve().parent
if not (project_root / "panel_crease_leg_v8.usd").exists():
    project_root = Path(os.environ.get("PANEL_CREASE_PROJECT_ROOT", Path.cwd())).expanduser().resolve()
sys.path.insert(0, str(project_root))
import stable_panel_crease_controller


usd_path = str(project_root / "panel_crease_leg_v8.usd")


async def _open_after_startup():
    # --exec scripts can run before the USD context has finished creating its
    # initial empty stage.  Wait for a few app ticks so the requested file is
    # the stage that actually appears in the visible window.
    for _ in range(10):
        await omni.kit.app.get_app().next_update_async()
    result = omni.usd.get_context().open_stage(usd_path)
    print(f"Opened visible knee stage: {usd_path}; result={result}")
    stable_panel_crease_controller.start()


asyncio.ensure_future(_open_after_startup())

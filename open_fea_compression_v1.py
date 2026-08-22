"""Open the FEA compression v1 knee and start its visual/replay controllers."""

import asyncio
import os
from pathlib import Path
import runpy

import omni.kit.app
import omni.usd
import isaacsim.core.experimental.utils.app as app_utils


project_root = Path(__file__).resolve().parent
if not (project_root / "panel_crease_leg_fea_compression_v1.usd").exists():
    project_root = Path(os.environ.get("PANEL_CREASE_PROJECT_ROOT", Path.cwd())).expanduser().resolve()
stable_panel_crease_controller = runpy.run_path(
    str(project_root / "stable_panel_crease_controller.py"),
    run_name="stable_panel_crease_controller_runtime",
)
fea_compression_controller = runpy.run_path(
    str(project_root / "fea_compression_controller.py"),
    run_name="fea_compression_controller_runtime",
)


usd_path = str(project_root / "panel_crease_leg_fea_compression_v1.usd")


async def _open_after_startup():
    for _ in range(10):
        await omni.kit.app.get_app().next_update_async()
    result = omni.usd.get_context().open_stage(usd_path)
    print(f"Opened FEA compression knee stage: {usd_path}; result={result}")
    stable_panel_crease_controller["start"]()
    fea_compression_controller["start"]()
    app_utils.play()


asyncio.ensure_future(_open_after_startup())

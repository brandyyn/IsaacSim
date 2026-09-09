"""Opt-in launcher for the force-driven perimeter-frame shell experiment.

Keeps open_gui.py and the ML baseline unchanged. Use the installed Kit --exec
mechanism with PANEL_CREASE_PROJECT_ROOT pointing to this checkout.
"""

import asyncio
import os
import sys
from pathlib import Path

import carb.settings
import omni.kit.app
import omni.usd

project = Path(os.environ.get("PANEL_CREASE_PROJECT_ROOT", Path(__file__).resolve().parents[1])).resolve()
if str(project) not in sys.path:
    sys.path.insert(0, str(project))


async def start():
    for _ in range(10):
        await omni.kit.app.get_app().next_update_async()
    from exact_joint import app
    await app.open_workshop(project)
    if omni.usd.get_context().get_stage() is None:
        raise RuntimeError("Native physics must not start without a valid stage")
    # A launch-time false setting avoids the RC's NULL-stage startup crash.
    # Restore normal native simulation once the workshop stage exists.
    carb.settings.get_settings().set("/app/player/playSimulations", True)
    await app.ACTIVE.open_nonlinear_shell()


opening_task = asyncio.ensure_future(start())

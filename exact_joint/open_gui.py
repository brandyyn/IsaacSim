"""Portable Kit --exec launcher for the exact source-joint FEM workshop."""

import asyncio
import os
import sys
from pathlib import Path
import omni.kit.app

project=Path(os.environ.get("PANEL_CREASE_PROJECT_ROOT",Path(__file__).resolve().parents[1])).resolve()
if str(project) not in sys.path:
    sys.path.insert(0,str(project))


async def start():
    for _ in range(10):
        await omni.kit.app.get_app().next_update_async()
    from exact_joint.app import open_workshop
    await open_workshop(project)


opening_task=asyncio.ensure_future(start())

"""Refresh trusted workshop UI code in Kit without replacing the user's scene.

This maintenance script preserves the assembled FEM, current USD, camera, material
settings and accepted result. A reopened matching knee can be reattached; unrelated
stages and incompatible saved settings are rejected.
"""

import asyncio
import hashlib
import os
import sys
import types
from pathlib import Path

import omni.kit.app
import omni.ui as ui
import omni.usd

from exact_joint import app
from exact_joint.mechanics import tension_pattern


async def refresh() -> None:
    lab = app.ACTIVE
    if lab is None:
        raise RuntimeError("No active workshop; use open_live.py to create one.")
    if getattr(lab, "drop_preview", None) is not None:
        await lab.drop_preview.restore()
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No opened scene to reconnect.")
    project = Path(os.environ["PANEL_CREASE_PROJECT_ROOT"])
    path = project / "exact_joint/app.py"
    source = path.read_bytes()
    compiled = compile(source, str(path), "exec")
    # Recover accepted cable families from the actual result, not rejected inputs.
    accepted = {}
    for name, result in lab.results.items():
        tensions = result["stats"]["tensions_n"]
        amplitude = max(tensions)
        mode = next(
            mode for mode in app.MODES
            if all(abs(a-b) < 1e-10 for a, b in zip(tension_pattern(mode, amplitude), tensions))
        )
        accepted[name] = {"mode": mode, "tension_n": amplitude}
    prior_error = lab.error
    if lab.task:
        lab.task.cancel()
        try:
            await lab.task
        except RuntimeError:
            pass  # Recover a prior controller that died on a reopened USD.
    if getattr(lab,"live_display",None):
        lab.live_display.close()
    lab.window.visible = False
    lab.window.destroy()
    # Previous builds hid windows without destroying them. Hide only this
    # workshop's obsolete duplicates so UI clicks cannot reach dead callbacks.
    for window in ui.Workspace.get_windows():
        if window.title == "Exact knee - cable FEM":
            window.visible = False
    # Only fixed, trusted repository modules are executed; never attachments.
    import exact_joint
    for name in ("scene","live_display"):
        qualified="exact_joint."+name
        module=sys.modules.get(qualified) or types.ModuleType(qualified)
        module_path=project/"exact_joint"/(name+".py")
        module_source=module_path.read_bytes()
        module.__file__=str(module_path)
        module.__package__="exact_joint"
        exec(compile(module_source,str(module_path),"exec"),module.__dict__)
        module.LOADED_SOURCE_SHA256=hashlib.sha256(module_source).hexdigest()
        sys.modules[qualified]=module
        setattr(exact_joint,name,module)
    exec(compiled, app.__dict__)
    app.LOADED_SOURCE_SHA256 = hashlib.sha256(source).hexdigest()
    lab.__class__ = app.Workshop
    defaults=app.Workshop(lab.project,lab.config)
    for key,value in vars(defaults).items():
        if not hasattr(lab,key):
            setattr(lab,key,value)
    lab.ramp=None
    if not lab.connected():
        lab.scene=app.ExactLegScene.attach(stage,lab.model)
        prior_error=None
    lab.live_display=app.LiveFemDisplay(lab.scene)
    lab.accepted_commands = accepted
    app.ACTIVE = lab
    lab.build_ui()
    await omni.kit.app.get_app().next_update_async()
    target = next((window for window in ui.Workspace.get_windows() if window.title == "Stage"), None)
    if target is not None:
        lab.window.dock_in(target, ui.DockPosition.SAME)
    lab.window.focus()
    # Re-render the accepted result without applying the failed requested load.
    requested = lab.commands
    lab.commands = {name: dict(value) for name, value in accepted.items()}
    try:
        lab.calculate()
    finally:
        lab.commands = requested
    if prior_error:
        lab.reject_command(prior_error)
    lab.task = asyncio.ensure_future(lab.loop())
    if omni.usd.get_context().get_stage() != stage:
        raise RuntimeError("The active scene changed during UI refresh.")
    print("Refreshed FEM controls; current scene, camera and material settings preserved.")


await refresh()

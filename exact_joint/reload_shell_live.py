"""Reload only the owned nonlinear workshop, preserving the original stage."""

import hashlib
import importlib
import asyncio
import dataclasses
import sys
import types

from exact_joint import app
import exact_joint

lab = app.ACTIVE
if lab is None:
    raise RuntimeError("Open the exact-joint workshop first")
previous = lab.drop_preview
in_place = previous is not None and type(previous).__name__ == "ShellWorkshop"
if in_place:
    values = {key: value.as_float for key, value in previous.inputs.items()}
    flags = {key: value.as_bool for key, value in previous.flags.items()}
    refinement = previous.refinement_input.as_int
    boundary = previous.boundary_input.as_int if hasattr(previous, "boundary_input") else previous.shell.config.subdivision
    pattern = previous.cable_pattern.model.get_item_value_model().as_int
    was_idle = previous.work is None or previous.work.done()
    job_metadata = {name: getattr(previous, name) for name in
                    ("job_label", "job_started", "job_step", "job_steps", "job_outcome")}
    work = previous.work
    previous.cancel_jobs()
    if work:
        await asyncio.gather(work, return_exceptions=True)
    if previous.task:
        previous.task.cancel()
        await asyncio.gather(previous.task, return_exceptions=True)
elif previous is not None:
    await lab.drop_preview.restore()
importlib.invalidate_caches()
loaded = {}
for name in ("shell_ipc", "shell_sparse", "nonlinear_shell", "shell_impact", "shell_ui_jobs", "shell_display_math", "shell_displacement_view", "shell_presentation", "shell_view"):
    qualified = "exact_joint."+name
    module = sys.modules.get(qualified) or types.ModuleType(qualified)
    sys.modules[qualified] = module
    setattr(exact_joint, name, module)
    path = lab.project/"exact_joint"/(name+".py")
    source = path.read_bytes()
    module.__file__ = str(path)
    module.__package__ = "exact_joint"
    exec(compile(source, str(path), "exec"), module.__dict__)
    module.LOADED_SOURCE_SHA256 = hashlib.sha256(source).hexdigest()
    loaded[name] = module.LOADED_SOURCE_SHA256
if in_place:
    from exact_joint.shell_view import ShellWorkshop
    from exact_joint.nonlinear_shell import NonlinearShell, ShellConfig
    previous.__class__ = ShellWorkshop
    if previous.inspector is not None:
        from exact_joint.shell_displacement_view import ShellDisplacementView
        previous.inspector.__class__ = ShellDisplacementView
    previous.shell.__class__ = NonlinearShell
    previous.shell.config = ShellConfig(**dataclasses.asdict(previous.shell.config))
    if previous.drop is not None:
        from exact_joint.shell_impact import ShellImpact, ShellImpactConfig
        previous.drop.__class__ = ShellImpact
        previous.drop.config = ShellImpactConfig(**dataclasses.asdict(previous.drop.config))
        job_metadata["job_steps"] = previous.drop.config.step_count
        job_metadata["job_step"] = int(sum(b["time_after_contact_s"] > a["time_after_contact_s"]
                                          for a, b in zip(previous.drop.trace, previous.drop.trace[1:])))
    previous.scene_disconnected = False
    previous.smooth_motion = getattr(previous, "smooth_motion", True)
    previous.strain_colours = getattr(previous, "strain_colours", False)
    previous.presentation = None
    previous.presentation_clock = __import__("time").perf_counter()
    previous.presentation_frames = 0
    previous.accepted_report = None
    previous.window.destroy()
    previous.build_ui()
    for name, value in values.items():
        # Preserve a live session across the clearer fold-compliance label.
        target = name if name in previous.inputs else {
            "Panel / crease bending ratio": "Fold compliance (100 = PET reference)"
        }.get(name, name)
        if target in previous.inputs:
            previous.inputs[target].set_value(value)
    for name, value in flags.items():
        previous.flags[name].set_value(value)
    previous.refinement_input.set_value(refinement)
    previous.boundary_input.set_value(boundary)
    previous.cable_pattern.model.get_item_value_model().set_value(pattern)
    if not previous.scene_is_current():
        previous.reconnect_opened()
    else:
        if was_idle:
            for name, value in job_metadata.items():
                setattr(previous, name, value)
        previous.render()
        previous.feedback.text = "Code reloaded; active settings and accepted physical results preserved."
    previous.task = asyncio.ensure_future(previous.loop())
    await __import__('omni.kit.app', fromlist=['get_app']).get_app().next_update_async()
    import omni.ui as ui
    target = next((w for w in ui.Workspace.get_windows() if w.title == "Stage"), None)
    if target:
        previous.window.dock_in(target, ui.DockPosition.SAME)
    previous.window.focus()
else:
    await lab.open_nonlinear_shell()
print("Loaded updated shell:", loaded)
print("Active:", lab.drop_preview.shell.config)

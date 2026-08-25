"""Open the coupled, video-profile-driven panel-crease knee experiment."""

import asyncio
import os
import sys
import types
from pathlib import Path

import isaacsim.core.experimental.utils.app as app_utils
import omni.kit.app
import omni.usd


project_root = Path(__file__).resolve().parent
if not (project_root / "panel_crease_leg_constraints_v8_geometry_fold.usd").exists():
    project_root = Path(os.environ.get("PANEL_CREASE_PROJECT_ROOT", Path.cwd())).expanduser().resolve()
sys.path.insert(0, str(project_root))


def _cancel_project_tasks():
    current = asyncio.current_task()
    controller_files = (
        "stable_panel_crease_controller.py",
        "facet_crease_panel_controller.py",
        "coupled_fold_motion_controller.py",
        "twist_sweep_motion_controller.py",
        "full_supported_motion_controller.py",
        "baked_panel_crease_controller.py",
        "multidof_knee_motion_controller.py",
    )
    for task in asyncio.all_tasks():
        if task is current:
            continue
        coroutine = task.get_coro()
        code = getattr(coroutine, "cr_code", None)
        filename = code.co_filename if code is not None else ""
        if any(name in filename for name in controller_files):
            task.cancel()


_cancel_project_tasks()


def _load_project_module(name):
    module_path = project_root / f"{name}.py"
    if not module_path.exists():
        raise RuntimeError(f"could not load project module {name}")
    # Execute current source directly. Kit can otherwise reuse a stale pyc
    # after a live edit when the USD stage is being reloaded in the same app.
    module = types.ModuleType(name)
    module.__file__ = str(module_path)
    sys.modules[name] = module
    source = module_path.read_text(encoding="utf-8")
    exec(compile(source, str(module_path), "exec"), module.__dict__)
    return module


stable_panel_crease_controller = _load_project_module("stable_panel_crease_controller")
coupled_fold_motion_controller = _load_project_module("coupled_fold_motion_controller")
baked_panel_crease_controller = _load_project_module("baked_panel_crease_controller")
twist_sweep_motion_controller = _load_project_module("twist_sweep_motion_controller")
full_supported_motion_controller = _load_project_module("full_supported_motion_controller")
visual_mode = os.environ.get("PANEL_CREASE_VISUAL_MODE", "baked")
if visual_mode not in ("baked", "solver"):
    raise RuntimeError("PANEL_CREASE_VISUAL_MODE must be baked or solver")
facet_crease_panel_controller = None
if visual_mode == "solver":
    facet_crease_panel_controller = _load_project_module("facet_crease_panel_controller")
legacy_motion = sys.modules.get("multidof_knee_motion_controller")
if legacy_motion is not None:
    legacy_task = getattr(legacy_motion, "_motion_task", None)
    if legacy_task is not None and not legacy_task.done():
        legacy_task.cancel()
for module_name in (
    "stable_panel_crease_controller",
    "facet_crease_panel_controller",
    "coupled_fold_motion_controller",
    "twist_sweep_motion_controller",
    "full_supported_motion_controller",
    "baked_panel_crease_controller",
):
    module = sys.modules.get(module_name)
    if module is None:
        continue
    for task_name in ("_motion_task", "_controller_task"):
        task = getattr(module, task_name, None)
        if task is not None and not task.done():
            task.cancel()


usd_path = str(
    project_root
    / os.environ.get(
        "PANEL_CREASE_COUPLED_STAGE_NAME",
        "panel_crease_leg_constraints_v8_geometry_fold.usd",
    )
)


async def _open_after_startup():
    for _ in range(10):
        await omni.kit.app.get_app().next_update_async()
    result = omni.usd.get_context().open_stage(usd_path)
    print(f"Opened coupled fold knee stage: {usd_path}; result={result}")
    await app_utils.update_app_async(steps=30)
    stage = omni.usd.get_context().get_stage()
    controller = stage.GetPrimAtPath("/World/PanelCreaseLeg/Controller")
    controller.GetAttribute("visualControllerMode").Set(visual_mode)
    controller.GetAttribute("visualControllerScript").Set(
        "facet_crease_panel_controller.py"
        if visual_mode == "solver"
        else "baked_panel_crease_controller.py"
    )
    mode = os.environ.get("PANEL_CREASE_MOTION_MODE", "neutral")
    if mode not in ("neutral", "twist_sweep", "supported_full_envelope", "showcase", "coupled_fold_profile", "geometry_fold"):
        raise RuntimeError(
            "PANEL_CREASE_MOTION_MODE must be neutral, twist_sweep, supported_full_envelope, showcase, coupled_fold_profile, or geometry_fold"
        )
    geometry_knee_stage = stage.GetPrimAtPath("/World/PanelCreaseLeg/Physics/KneeActuator").IsValid()
    geometry_mode = mode in ("geometry_fold", "showcase", "coupled_fold_profile") and geometry_knee_stage
    driver_mode = "geometry_fold" if geometry_mode else ("supported_full_envelope" if mode == "showcase" else mode)
    controller.GetAttribute("motionDriver").Set(driver_mode)
    controller.GetAttribute("motionDemoEnabled").Set(False)
    if geometry_mode or (geometry_knee_stage and mode == "neutral"):
        coupled_fold_motion_controller.apply_fold_command(stage, 0.0)
    elif mode == "twist_sweep":
        twist_sweep_motion_controller.apply_twist_command(stage, 0.0, fold_deg=0.0, lateral_deg=0.0)
    elif mode in ("supported_full_envelope", "showcase"):
        if mode == "showcase":
            controller.GetAttribute("fullMotionPattern").Set(
                os.environ.get("PANEL_CREASE_SHOWCASE_PATTERN", "combined")
            )
        full_supported_motion_controller.apply_full_motion_command(stage)
    elif mode == "coupled_fold_profile":
        coupled_fold_motion_controller.apply_fold_command(stage, 0.0)
    elif not geometry_mode:
        full_supported_motion_controller.apply_full_motion_command(stage)
    if visual_mode == "solver":
        facet_crease_panel_controller.start()
    else:
        baked_panel_crease_controller.start()
    if geometry_mode:
        coupled_fold_motion_controller.start()
    elif mode == "twist_sweep":
        twist_sweep_motion_controller.start()
    elif mode == "supported_full_envelope":
        full_supported_motion_controller.start()
    if mode != "neutral":
        app_utils.play()
        if mode == "showcase":
            # Start the presentation from a settled, exactly straight pose.
            # The motors remain enabled but the envelope driver is held off
            # while PhysX resolves the serial carrier chain.
            for _ in range(120):
                await omni.kit.app.get_app().next_update_async()
            controller.GetAttribute("motionDemoEnabled").Set(True)
            full_supported_motion_controller.start()
        else:
            controller.GetAttribute("motionDemoEnabled").Set(True)


asyncio.ensure_future(_open_after_startup())

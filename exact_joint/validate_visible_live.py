"""Verify visible load ramps and real UI controls in Kit; no structural calibration.

Run on a freshly launched workshop. This exports/reopens its generated scene as
a new timestamped test file, never replacing a user-authored saved USD on disk.
"""

import dataclasses
import hashlib
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.ui as ui
import omni.usd
from isaacsim.core.experimental.utils import app as app_utils
from omni.kit.ui_test import WidgetRef, emulate_mouse_move
from omni.ui_query import OmniUIQuery
from pxr import Usd, UsdGeom

from exact_joint import app


async def advance(seconds):
    start = time.perf_counter()
    while time.perf_counter() - start < seconds:
        await omni.kit.app.get_app().next_update_async()


async def click(text, window=None, by_name=False):
    window = window or app.ACTIVE.window
    window.focus()
    await advance(0.15)
    for path in OmniUIQuery.get_window_widget_paths(window):
        widget = OmniUIQuery.find_widget(path)
        key = "name" if by_name else "text"
        if widget is not None and getattr(widget, key, None) == text:
            target = WidgetRef(widget, str(path), window)
            # Let hover/hit-testing see MOVE before DOWN in this Kit build.
            await emulate_mouse_move(target.center)
            await advance(0.15)
            await target.click()
            await advance(0.15)
            return
    raise AssertionError("Missing live UI control: " + text)


def check_plot(lab):
    result = lab.results["Knee"]
    display = lab.live_display
    actual = np.array(lab.scene.modules["Knee"]["pet"].GetPointsAttr().Get())
    plot = np.array(display.surface.GetPointsAttr().Get())
    np.testing.assert_allclose(actual, result["points"] + display.physical_offset, atol=1e-8, rtol=0)
    np.testing.assert_allclose(plot, lab.model.points + lab.display_gain * result["displacements"]
                               + display.offset, atol=1e-8, rtol=0)
    for view in display.loads:
        for i, curve in enumerate(view):
            visible = curve.GetVisibilityAttr().Get() != UsdGeom.Tokens.invisible
            assert visible == bool(result["tensions"][i] > 1e-6), (i, visible)
    return plot


async def run():
    lab = app.ACTIVE
    model = lab.model
    original_stage = omni.usd.get_context().get_stage()
    layer=original_stage.GetRootLayer()
    previous_test=Path(layer.identifier)
    own_test=(previous_test.name=="reopen_test.usda" and previous_test.parent.name.endswith("_visible")
              and previous_test.parent.parent==lab.project/"exact_joint/results")
    assert layer.anonymous or own_test, "Use a fresh generated scene for reopen regression"
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = lab.project / "exact_joint/results" / (stamp + "_visible")
    output.mkdir(parents=True, exist_ok=False)
    report = {"config": dataclasses.asdict(lab.config), "source_sha256": model.source["sha256"],
              "source_hashes": {name: hashlib.sha256((lab.project / "exact_joint" / name).read_bytes()).hexdigest()
                                for name in ("app.py", "scene.py", "live_display.py", "validate_visible_live.py")}}
    await click("Neutral")
    await advance(0.3)
    assert not lab.error and not lab.dirty
    check_plot(lab)
    assert not lab.live_display.arrow_spans
    mode, tension = lab.cable_inputs["Knee"]
    mode.set_value(app.MODES.index("Compression"))
    tension.set_value(0.2)
    await click("Apply")
    samples = []
    start = time.perf_counter()
    while lab.ramp is not None or lab.dirty:
        samples.append(lab.accepted_commands["Knee"]["tension_n"])
        check_plot(lab)
        assert time.perf_counter() - start < 10
        await advance(0.08)
    samples.append(lab.accepted_commands["Knee"]["tension_n"])
    assert len(set(samples)) >= 8 and abs(samples[-1] - 0.2) < 1e-7
    assert np.all(np.diff(samples) >= -1e-10)
    report["apply_ramp"] = {"samples_n": samples, "elapsed_s": time.perf_counter() - start,
                            "physical_and_magnified_coordinates_verified": True}
    original_result = lab.results["Knee"]
    first_plot = check_plot(lab)
    lab.change_display(1000, 5)
    assert lab.results["Knee"] is original_result
    second_plot = check_plot(lab)
    assert np.max(np.abs(second_plot-first_plot)) > 0.001
    lab.change_display(500, 5)
    report["display_only_gain_preserves_solution"] = True
    await click("Play / replay load")
    assert lab.ramp is not None
    await click("Pause")
    assert lab.paused
    frozen = lab.frames, lab.glyph_phase, lab.ramp["elapsed"]
    await advance(0.5)
    assert frozen == (lab.frames, lab.glyph_phase, lab.ramp["elapsed"])
    await click("Play / replay load")
    assert not lab.paused
    await advance(3)
    assert lab.ramp is None and not lab.error
    report["panel_pause_resume_replay"] = True
    toolbar = next(w for w in ui.Workspace.get_windows() if w.title == "Main ToolBar")
    if app_utils.is_playing():
        app_utils.stop(commit=False)
        await advance(0.2)
    await click("play", toolbar, by_name=True)
    assert app_utils.is_playing() and lab.ramp is not None and not lab.paused
    # Kit keeps the widget name "play" while its icon/tooltip changes to Pause.
    await click("play", toolbar, by_name=True)
    assert not app_utils.is_playing() and lab.paused
    report["native_timeline_play_pause_real_clicks"] = True
    app_utils.stop(commit=False)
    await advance(0.2)
    await click("Cable demo")
    assert lab.demo and not lab.paused
    first_frame = lab.frames
    first_plot = check_plot(lab)
    first_arrows = np.array(lab.live_display.arrows.GetPointsAttr().Get())
    await advance(3)
    last_plot = check_plot(lab)
    last_arrows = np.array(lab.live_display.arrows.GetPointsAttr().Get())
    assert lab.frames-first_frame > 10 and not lab.error
    assert np.max(np.abs(last_plot-first_plot)) > 0.001
    assert np.max(np.abs(last_arrows-first_arrows)) > 0.001
    report["cable_demo"] = {"updates_in_3_seconds": lab.frames-first_frame,
                            "plot_change_m": float(np.max(np.abs(last_plot-first_plot))),
                            "glyph_change_m": float(np.max(np.abs(last_arrows-first_arrows)))}
    await click("Pause")
    report["patterns"] = []
    for pattern in app.MODES:
        lab.command("Knee", pattern, 0.2)
        lab.calculate()
        lab.paused = True
        check_plot(lab)
        report["patterns"].append({"mode": pattern, "stats": lab.results["Knee"]["stats"]})
    saved = output / "reopen_test.usda"
    original_stage.GetRootLayer().Export(str(saved))
    await omni.usd.get_context().open_stage_async(str(saved))
    await advance(0.5)
    assert lab.paused and lab.error and not lab.task.done()
    assert "Reconnect" in lab.feedback.text
    await click("Reconnect opened knee")
    assert lab.connected() and not lab.error and lab.model is model
    check_plot(lab)
    assert not omni.usd.get_context().get_stage().GetPrimAtPath("/World/ExactLeg/Hip")
    assert not omni.usd.get_context().get_stage().GetPrimAtPath("/World/ExactLeg/Ankle")
    report["saved_usd_reconnect"] = {"controller_survived": True, "model_preserved": True,
                                     "active_physical_origami_joints": ["Knee"]}
    incompatible = Usd.Stage.CreateInMemory()
    try:
        app.ExactLegScene.attach(incompatible, model)
    except ValueError:
        report["incompatible_scene_rejected"] = True
    else:
        raise AssertionError("Incompatible scene was accepted")
    await click("Cable demo")
    await advance(0.5)
    assert lab.demo and not lab.error and not lab.task.done()
    report["all_passed"] = True
    (output / "validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("VISIBLE_VALIDATION_PASSED", output)
    print("Apply ramp samples:", len(set(samples)), "Demo updates:", report["cable_demo"]["updates_in_3_seconds"])


await run()

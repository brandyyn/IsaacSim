"""Local Kit button checks for experimental cable winch and PET reliefs."""

import json
import time
from datetime import datetime, timezone

import numpy as np
import omni.kit.app
from omni.kit.ui_test import WidgetRef, emulate_mouse_move
from omni.ui_query import OmniUIQuery
from exact_joint import app


async def tick(seconds=.1):
    until = time.perf_counter()+seconds
    while time.perf_counter() < until:
        await omni.kit.app.get_app().next_update_async()


async def click(view, text):
    view.window.focus()
    await tick()
    for path in OmniUIQuery.get_window_widget_paths(view.window):
        widget = OmniUIQuery.find_widget(path)
        if widget is not None and type(widget).__name__ == "Button" and widget.text == text:
            target = WidgetRef(widget, str(path), view.window)
            await emulate_mouse_move(target.center)
            await target.click()
            await tick()
            return
    raise AssertionError("Missing button: "+text)


async def finish(view):
    until = time.perf_counter()+600
    while view.work and not view.work.done():
        assert time.perf_counter() < until, "Timed out"
        await tick()
    assert not view.feedback.text.startswith("NOT APPLIED"), view.feedback.text


async def validate():
    view = app.ACTIVE.drop_preview
    report = {"scope": "UI/implementation checks only", "checks": {}}
    await click(view, "Material reference")
    await finish(view)
    assert view.shell.config.vertex_relief_fraction == 0
    view.inputs["Winch pull (mm)"].set_value(.1)
    await click(view, "Apply winch pull")
    await finish(view)
    assert view.last_rejection is None and len(view.static_trace) == 2
    assert view.static_trace[-1]["winch"] is not None
    assert max(view.tensions) > 0
    np.testing.assert_allclose(np.asarray(view.surface.GetPointsAttr().Get()),
        view.shell.diagnostics(view.state)["points"]+[0, 0, view.offset], atol=2e-8)
    report["checks"]["winch_loads_and_rendered_1x_state"] = True
    accepted = view.state.copy()
    view.inputs["Winch pull (mm)"].set_value(-1)
    await click(view, "Apply winch pull")
    while view.work and not view.work.done():
        await tick()
    assert view.feedback.text.startswith("NOT APPLIED")
    np.testing.assert_array_equal(view.state, accepted)
    report["checks"]["invalid_pull_preserves_accepted_state"] = True
    await click(view, "Relief experiment")
    await finish(view)
    assert view.shell.config.vertex_relief_fraction == .2
    assert len(view.relief_edges) > 0 and view.relief_lines is not None
    assert len(view.surface.GetFaceVertexCountsAttr().Get()) == len(view.shell.mesh["triangles"])
    assert "MODIFIED CUT DESIGN" in view.settings_label.text
    report["checks"]["relief_topology_rebuild_and_active_settings"] = True
    view.inputs["Winch pull (mm)"].set_value(.025)
    await click(view, "Winch demo")
    await click(view, "Pause / resume")
    paused = view.state.copy()
    await tick(.6)
    np.testing.assert_array_equal(view.state, paused)
    await click(view, "Pause / resume")
    await finish(view)
    assert view.last_rejection is None, view.feedback.text
    assert len({row["mode"].split(",")[0] for row in view.static_trace}) == 7
    assert np.max(np.abs(view.state)) < 1e-3
    report["checks"]["seven_winch_families_pause_resume_and_unload"] = True
    await click(view, "Save calculation")
    data = json.loads((view.saved_path/"calculation.json").read_text())
    assert data["static_trace"][-1]["winch"] is not None
    report["checks"]["winch_provenance_serializes"] = True
    await click(view, "Material reference")
    await finish(view)
    assert view.relief_lines is None and view.shell.config.vertex_relief_fraction == 0
    report["checks"]["reference_restores_intact_pattern"] = True
    destination = app.ACTIVE.project/"exact_joint/results"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_winch_ui.json")
    destination.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report), str(destination))


await validate()

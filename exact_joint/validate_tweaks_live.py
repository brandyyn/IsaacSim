"""Exercise the workshop's display/material controls through real UI clicks in Kit.

Run on a freshly generated knee or the visible-regression test scene. Each rebuild
replaces this generated stage, so the test exports a safety copy first. It restores
the user's baseline constants and leaves Cable demo running when all checks pass.
"""

import dataclasses
import hashlib
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.usd
from omni.kit.ui_test import WidgetRef, emulate_mouse_move
from omni.ui_query import OmniUIQuery

from exact_joint import app
from exact_joint.geometry import JointConfig


async def advance(seconds):
    started = time.perf_counter()
    while time.perf_counter() - started < seconds:
        await omni.kit.app.get_app().next_update_async()


def find(kind, text=None):
    for path in OmniUIQuery.get_window_widget_paths(app.ACTIVE.window):
        widget = OmniUIQuery.find_widget(path)
        if widget is not None and type(widget).__name__ == kind:
            if text is None or getattr(widget, "text", None) == text:
                return str(path), widget
    raise AssertionError(f"Missing {kind}: {text}")


async def click(text):
    app.ACTIVE.window.focus()
    path, button = find("Button", text)
    _, scroll = find("ScrollingFrame")
    middle = button.screen_position_y + button.computed_height / 2
    scroll.scroll_y = min(scroll.scroll_y_max, max(0, scroll.scroll_y + middle
                                                 - scroll.screen_position_y - scroll.computed_height / 2))
    await advance(0.2)
    path, button = find("Button", text)
    target = WidgetRef(button, path, app.ACTIVE.window)
    await emulate_mouse_move(target.center)
    await advance(0.1)
    await target.click()
    await advance(0.2)


def field(label):
    path, _ = find("Label", label)
    row = path.rsplit("/", 1)[0]
    return OmniUIQuery.find_widget(row + "/FloatField[0]").model


async def solve():
    lab = app.ACTIVE
    mode, tension = lab.cable_inputs["Knee"]
    mode.set_value(app.MODES.index("Compression"))
    tension.set_value(0.1)
    await click("Apply")
    started = time.perf_counter()
    while lab.ramp is not None or lab.dirty:
        assert not lab.error, lab.error
        assert time.perf_counter() - started < 10, "Apply ramp timeout"
        await advance(0.1)
    return lab.results["Knee"]["stats"]


async def rebuild(values):
    lab = app.ACTIVE
    for key, value in values.items():
        field(key).set_value(value)
    stage = omni.usd.get_context().get_stage()
    # These are test-owned stages only. Preserve the full scene before replacement.
    backup = OUTPUT / f"before_rebuild_{len(REPORT['material_cases'])}.usda"
    stage.Export(str(backup))
    await click("Rebuild exact joint FEM")
    started = time.perf_counter()
    while app.ACTIVE is lab or app.ACTIVE.frames == 0 or app.ACTIVE.task is None:
        assert time.perf_counter() - started < 40, lab.message.text
        await advance(0.2)
    assert lab.task.done()
    assert app.ACTIVE.error is None
    return app.ACTIVE


async def run():
    global OUTPUT, REPORT
    lab = app.ACTIVE
    layer = omni.usd.get_context().get_stage().GetRootLayer()
    path = Path(layer.identifier)
    own_test = path.name == "reopen_test.usda" and path.parent.parent == lab.project / "exact_joint/results"
    assert layer.anonymous or own_test, "Refusing to replace a user-authored scene"
    assert lab.config == JointConfig(), "Start with the documented baseline constants"
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    OUTPUT = lab.project / "exact_joint/results" / (stamp + "_tweaks")
    OUTPUT.mkdir(parents=True, exist_ok=False)
    REPORT = {"material_cases": [], "implementation_source_sha256":
              {name: hashlib.sha256((lab.project / "exact_joint" / name).read_bytes()).hexdigest()
               for name in ("app.py", "geometry.py", "mechanics.py", "validate_tweaks_live.py")}}
    await click("Neutral")
    base_stats = await solve()
    REPORT["baseline"] = base_stats
    result = lab.results["Knee"]
    actual = np.array(lab.scene.modules["Knee"]["pet"].GetPointsAttr().Get())
    display_path, _ = find("Button", "Update display")
    row = display_path.rsplit("/", 1)[0]
    gain = OmniUIQuery.find_widget(row + "/FloatField[0]").model
    stress = OmniUIQuery.find_widget(row + "/FloatField[1]").model
    gain.set_value(1000)
    stress.set_value(5)
    await click("Update display")
    assert lab.display_gain == 1000 and lab.stress_scale_pa == 5e6
    assert lab.results["Knee"] is result
    np.testing.assert_array_equal(actual, lab.scene.modules["Knee"]["pet"].GetPointsAttr().Get())
    gain.set_value(0)
    await click("Update display")
    assert lab.display_gain == 1000 and "Display only" in lab.feedback.text
    gain.set_value(500)
    await click("Update display")
    REPORT["display_controls_real_clicks"] = {"gain_and_scale_work": True, "invalid_gain_rejected": True,
                                             "physical_solution_unchanged": True}
    # An invalid material must retain the existing stage and assembled FEM.
    original_model, original_stage = lab.model, omni.usd.get_context().get_stage()
    field("PET E (GPa)").set_value(0)
    await click("Rebuild exact joint FEM")
    assert app.ACTIVE is lab and lab.model is original_model
    assert omni.usd.get_context().get_stage() == original_stage
    assert "positive" in lab.message.text
    REPORT["invalid_material_preserves_scene"] = True
    baseline = {"PET exposed gap (mm)": 0.2, "PET E (GPa)": 3.5, "PLA E (GPa)": 2.2, "Refinement 1-4": 1}
    cases = [("PET modulus half", {"PET E (GPa)": 1.75}),
             ("PLA modulus half", {"PLA E (GPa)": 1.1}),
             ("Both moduli doubled", {"PET E (GPa)": 7.0, "PLA E (GPa)": 4.4}),
             ("PET gap 0.1 mm", {"PET exposed gap (mm)": 0.1}),
             ("PET gap 0.4 mm", {"PET exposed gap (mm)": 0.4}),
             ("Mesh refinement 2", {"Refinement 1-4": 2}),
             ("Baseline restored", {})]
    for name, edits in cases:
        lab = await rebuild({**baseline, **edits})
        stats = await solve()
        REPORT["material_cases"].append({"name": name, "config": dataclasses.asdict(lab.config),
                                          "nodes": len(lab.model.points), "elements": len(lab.model.tets),
                                          "stats": stats})
        print(name, "Compression um:", stats["compression_m"] * 1e6)
    cases_by_name = {case["name"]: case for case in REPORT["material_cases"]}
    assert cases_by_name["PET modulus half"]["stats"]["compression_m"] > base_stats["compression_m"]
    assert cases_by_name["PLA modulus half"]["stats"]["compression_m"] > base_stats["compression_m"]
    np.testing.assert_allclose(cases_by_name["Both moduli doubled"]["stats"]["compression_m"]
                               / base_stats["compression_m"], 0.5, rtol=0.005)
    np.testing.assert_allclose(cases_by_name["Baseline restored"]["stats"]["compression_m"],
                               base_stats["compression_m"], rtol=1e-7)
    assert lab.config == JointConfig()
    await click("Save FEM results and inputs")
    assert "Saved " in lab.message.text
    REPORT["save_button_real_click"] = True
    # Set the two display fields via their actual controls for the final demo.
    display_path, _ = find("Button", "Update display")
    row = display_path.rsplit("/", 1)[0]
    OmniUIQuery.find_widget(row + "/FloatField[0]").model.set_value(500)
    OmniUIQuery.find_widget(row + "/FloatField[1]").model.set_value(5)
    await click("Update display")
    await click("Cable demo")
    _, scroll = find("ScrollingFrame")
    scroll.scroll_y = 0
    await advance(3)
    assert lab.demo and not lab.paused and not lab.error and not lab.task.done()
    REPORT["all_passed"] = True
    (OUTPUT / "validation.json").write_text(json.dumps(REPORT, indent=2), encoding="utf-8")
    print("TWEAK_VALIDATION_PASSED", OUTPUT)


await run()

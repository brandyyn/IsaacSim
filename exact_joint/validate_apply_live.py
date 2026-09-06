"""Exercise real Apply clicks and rejected-load recovery without replacing the scene."""

import hashlib
import io
import json
import unittest
from datetime import datetime, timezone

import numpy as np
import omni.kit.app
import omni.usd
from omni.kit.ui_test import Vec2, emulate_mouse_move_and_click
from omni.ui_query import OmniUIQuery

from exact_joint import app


def find_widget(kind: str, text: str) -> tuple[str, object]:
    for path in OmniUIQuery.get_window_widget_paths(app.ACTIVE.window):
        widget = OmniUIQuery.find_widget(path)
        if widget is not None and type(widget).__name__ == kind and getattr(widget, "text", None) == text:
            return str(path), widget
    raise RuntimeError(f"Missing visible control: {text}")


async def click(text: str) -> None:
    _, button = find_widget("Button", text)
    await emulate_mouse_move_and_click(
        Vec2(button.screen_position_x + button.computed_width / 2,
             button.screen_position_y + button.computed_height / 2)
    )
    for _ in range(40):
        await omni.kit.app.get_app().next_update_async()


async def run() -> None:
    lab = app.ACTIVE
    stage = omni.usd.get_context().get_stage()
    layer = stage.GetRootLayer()
    model = lab.model
    report = {"cases": [], "source_sha256": model.source["sha256"],
              "ui_source_sha256": hashlib.sha256((lab.project / "exact_joint/app.py").read_bytes()).hexdigest()}
    if lab.show_stress:
        await click("Stress / material")
    await click("Stress / material")
    assert lab.show_stress
    path, _ = find_widget("Button", "Apply")
    row = path.rsplit("/", 1)[0]
    tension = OmniUIQuery.find_widget(row + "/FloatField[0]").model
    mode = OmniUIQuery.find_widget(row + "/ComboBox[0]").model.get_item_value_model()
    mode.set_value(app.MODES.index("Compression"))
    accepted = []
    for value in (0.1, 0.2, 40.0, -1.0, 0.1):
        before = np.array(lab.scene.modules["Knee"]["stress"].GetPointsAttr().Get())
        previous_result = lab.results["Knee"]
        tension.set_value(value)
        await click("Apply")
        applied = lab.error is None and not lab.dirty
        expected = value in (0.1, 0.2)
        assert applied == expected, (value, lab.feedback.text)
        after = np.array(lab.scene.modules["Knee"]["stress"].GetPointsAttr().Get())
        if not expected:
            assert lab.results["Knee"] is previous_result
            np.testing.assert_array_equal(before, after)
            assert "NOT APPLIED" in lab.feedback.text
            assert lab.paused
        else:
            assert "APPLIED - update" in lab.feedback.text
            assert lab.accepted_commands["Knee"]["tension_n"] == value
            accepted.append(lab.results["Knee"]["stats"]["compression_m"])
        item = {"input_n": value, "applied": applied, "feedback": lab.feedback.text,
                "accepted_commands": {key: dict(command) for key, command in lab.accepted_commands.items()},
                "stats": lab.results["Knee"]["stats"], "scene_point_change_m": float(np.max(np.abs(after-before)))}
        if value == 40:
            folder = lab.save()
            payload = json.loads((folder / "result.json").read_text(encoding="utf-8"))
            assert payload["commands"]["Knee"]["tension_n"] == 0.2
            assert payload["requested_commands"]["Knee"]["tension_n"] == 40
            item["rejected_export_preserves_accepted_load"] = True
        report["cases"].append(item)
    np.testing.assert_allclose(accepted[1] / accepted[0], 2, rtol=0.005)
    assert lab.model is model
    assert omni.usd.get_context().get_stage() == stage
    assert omni.usd.get_context().get_stage().GetRootLayer() == layer
    report["current_scene_and_fem_preserved"] = True
    report["compression_ratio_0_2_over_0_1_n"] = accepted[1] / accepted[0]
    report["stress_view_enabled"] = lab.show_stress
    stream = io.StringIO()
    suite = unittest.defaultTestLoader.loadTestsFromName("exact_joint.test_exact_joint")
    tests = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    report["numeric_tests"] = {"count": tests.testsRun, "passed": tests.wasSuccessful(), "output": stream.getvalue()}
    assert tests.wasSuccessful()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = lab.project / "exact_joint/results" / (stamp + "_apply")
    output.mkdir(parents=True, exist_ok=False)
    (output / "validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("Apply UI validation passed", output)
    print("Compression at 0.1 / 0.2 N (um):", accepted[0] * 1e6, accepted[1] * 1e6)


await run()

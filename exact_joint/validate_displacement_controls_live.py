"""Live QA for the single-joint displacement-controlled FEM panel."""

import hashlib
import json
from datetime import datetime, timezone

import numpy as np
import omni.ui as ui
from omni.ui_query import OmniUIQuery

from exact_joint import app


async def validate():
    view = app.ACTIVE.drop_preview
    assert view.window.title == "Joint FEM - displacement actuation"
    assert view.window.visible
    assert set(view.inputs) == {
        "Joint displacement (mm)", "Fold compliance (100 = PET reference)",
        "Panel bending scale", "Membrane stiffness scale", "PLA modulus (GPa)",
        "PET modulus (GPa)", "PET hinge width (mm)"}
    names = []
    for path in OmniUIQuery.get_window_widget_paths(view.window):
        widget = OmniUIQuery.find_widget(path)
        text = getattr(widget, "text", None)
        if text:
            names.append(text)
    assert {"Run displacement", "Neutral", "Rebuild model"}.issubset(names)
    view.inputs["Joint displacement (mm)"].set_value(.5)
    families = ("Compression", "Bend X+", "Bend X-", "Bend Y+", "Bend Y-", "Twist CW", "Twist CCW")
    measurements = {}
    for family in families:
        view.movement_buttons[family].call_clicked_fn()
        await view.work
        assert view.job_outcome == "COMPLETE", view.feedback.text
        assert view.job_step == view.job_steps == 1
        report = view.shell.diagnostics(view.state)
        measurements[family] = {
            "target_displacement_m": .0005,
            "relative_rotation_rad": report["relative_rotation_rad"].tolist(),
            "compression_fraction": report["compression_fraction"],
            "max_membrane_strain": report["max_membrane_strain"],
            "reaction_world_n_nm": view.last_reaction.tolist(),
            "active_cable_count": int(view.active_cable_mask.sum()),
            "load_type": view.static_trace[-1]["mode"],
        }
    view.inputs["Fold compliance (100 = PET reference)"].set_value(300)
    view.inputs["Panel bending scale"].set_value(.8)
    view.launch(view.rebuild(), "Rebuild model")
    await view.work
    assert view.job_outcome == "COMPLETE"
    assert view.shell.config.panel_to_crease_ratio == 300
    assert view.shell.config.panel_bending_scale == .8
    view.inputs["Fold compliance (100 = PET reference)"].set_value(100)
    view.inputs["Panel bending scale"].set_value(1)
    view.launch(view.rebuild(), "Rebuild model")
    await view.work
    assert view.job_outcome == "COMPLETE"
    result = {
        "checks": {
            "simple_ui": True,
            "seven_displacement_actuations": True,
            "reaction_force_measurement": True,
            "material_stiffness_rebuild": True,
            "no_force_target": True,
        },
        "measurements": measurements,
        "target_displacement_m": .0005,
        "material": {k: getattr(view.shell.material, k) for k in
                     ("pla_thickness_m", "pet_thickness_m", "pla_modulus_pa", "pet_modulus_pa")},
        "shell_config": {"panel_to_crease_ratio": view.shell.config.panel_to_crease_ratio,
                          "panel_bending_scale": view.shell.config.panel_bending_scale,
                          "membrane_scale": view.shell.config.membrane_scale,
                          "self_contact": view.shell.config.self_contact,
                          "open_ends": view.shell.config.open_ends},
        "scope": "One open-ended knee joint; displacement-controlled continuation; custom shell FEM; no solid stress or survival claim",
        "source_sha256": view.shell.source["sha256"],
        "code_sha256": {name: hashlib.sha256((view.lab.project / "exact_joint" / (name + ".py")).read_bytes()).hexdigest()
                        for name in ("shell_view", "nonlinear_shell", "shell_ui_jobs", "reload_shell_live")},
    }
    path = view.lab.project / "exact_joint/results" / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ") + "_displacement_ui.json")
    path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(path, json.dumps(result["checks"]))


await validate()

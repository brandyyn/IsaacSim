"""Controller-level tests inside Kit; does not inject desktop UI input.

The visible Apply button is additionally clicked/inspected in the desktop test.
This script verifies model rebuild, result publication and material controls.
"""

import asyncio
import hashlib
import json
from datetime import datetime, timezone

import numpy as np

from exact_joint import app
from exact_joint.mechanics import tension_pattern


async def validate():
    view = app.ACTIVE.drop_preview
    if view.work and not view.work.done():
        raise RuntimeError("Wait for the active calculation before validation")
    view.running = True
    await view.preset(False)
    checks = {}
    assert view.shell.config.self_contact and view.shell.config.sparse_solver
    assert view.shell.config.interior_refinement == 1
    assert view.shell.config.crease_twist_ratio == 0
    assert view.shell.material.pla_thickness_m == .0004
    assert abs(view.shell.material.pet_thickness_m-.00008) < 1e-16
    assert len(view.shell.mesh["triangles"]) == 3648
    np.testing.assert_array_equal(view.shell.points[:28], view.lab.model.source["points"])
    checks["source_frames_and_refined_material_preset"] = True
    shell = view.shell
    _, before = await asyncio.to_thread(shell.solve, tension_pattern("Compression", 1))
    view.inputs["PLA thickness (mm)"].set_value(.2)
    await view.rebuild()
    assert view.shell.material.pla_thickness_m == .0002
    _, after = await asyncio.to_thread(view.shell.solve, tension_pattern("Compression", 1))
    assert before["accepted"] and after["accepted"]
    assert after["compression_fraction"] > before["compression_fraction"]
    assert view.shell.panel_rigidity_nm < shell.panel_rigidity_nm
    checks["thickness_edit_changes_assembled_stiffness_and_force_response"] = True
    accepted = view.shell
    view.inputs["PET thickness (um)"].set_value(-1)
    try:
        await view.rebuild()
        raise AssertionError("Invalid thickness accepted")
    except ValueError:
        pass
    assert view.shell is accepted
    checks["invalid_material_preserves_model"] = True
    await view.preset(False)
    view.inputs["Winch pull (mm)"].set_value(.05)
    await view.range_job("winch")
    assert view.last_rejection is None and len(view.static_trace) == 2
    assert max(view.tensions) > 0
    assert view.static_trace[-1]["self_contact"]["method"].startswith("IPC")
    np.testing.assert_allclose(np.asarray(view.surface.GetPointsAttr().Get()),
        view.shell.diagnostics(view.state)["points"]+[0, 0, view.offset], atol=2e-8)
    checks["winch_force_contact_diagnostics_and_1x_render"] = True
    saved = view.state.copy()
    try:
        await view.range_job("bend")
        raise AssertionError("Unsupported prescribed/contact study accepted")
    except ValueError:
        pass
    np.testing.assert_array_equal(view.state, saved)
    checks["unsupported_contact_pose_preserves_accepted_state"] = True
    destination = view.save()
    data = json.loads((destination/"calculation.json").read_text())
    assert data["self_contact"]["scope"].startswith("Midsurfaces")
    assert data["shell_config"]["physical_strip_bending"]
    checks["saved_material_mesh_and_contact_provenance"] = True
    await view.preset(False)
    view.running = False
    report = {"scope": "Kit controller-level checks; NOT physical calibration", "checks": checks,
              "source_sha256": view.shell.source["sha256"],
              "thickness_comparison": {"pla_0_4mm_compression_fraction": before["compression_fraction"],
                                       "pla_0_2mm_compression_fraction": after["compression_fraction"]},
              "code_sha256": {name: hashlib.sha256((view.lab.project/"exact_joint"/(name+".py")).read_bytes()).hexdigest()
                              for name in ("nonlinear_shell", "shell_sparse", "shell_ipc", "shell_impact", "shell_view")}}
    output = view.lab.project/"exact_joint/results"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_refined_live.json")
    output.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report), str(output))


await validate()

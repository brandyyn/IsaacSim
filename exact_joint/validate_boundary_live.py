"""Controller-level mesh rebuild test; preserves the prior accepted calculation."""
import asyncio
import dataclasses
import json
from datetime import datetime, timezone

import numpy as np

from exact_joint import app
from exact_joint.mechanics import tension_pattern


async def validate():
    v = app.ACTIVE.drop_preview
    assert v.work is None or v.work.done()
    assert v.scene_is_current()
    old_shell = v.shell
    boundary, refinement = v.boundary_input.as_int, v.refinement_input.as_int
    saved = {name: getattr(v, name) for name in ("state", "snapshots", "static_trace", "tensions", "drop",
             "display_index", "last_rejection", "mode", "offset", "job_outcome", "job_label", "job_step", "job_steps")}
    checks = {}
    try:
        v.boundary_input.set_value(3)
        v.refinement_input.set_value(0)
        v.launch(v.rebuild(), "Boundary mesh validation")
        await v.work
        assert v.job_outcome == "COMPLETE"
        assert v.shell.config.subdivision == 3 and v.shell.config.interior_refinement == 0
        assert dataclasses.asdict(v.shell.material) == dataclasses.asdict(old_shell.material)
        np.testing.assert_array_equal(v.shell.points[:28], old_shell.source["points"])
        assert all(len(chain) == 9 for chain in v.shell.mesh["chains"])
        checks["boundary_control_rebuilds_without_changing_material_or_source"] = True
        q, report = await asyncio.to_thread(v.shell.solve, tension_pattern("Bend Y+", 3))
        assert report["accepted"]
        v.state = q
        v.render()
        np.testing.assert_allclose(np.asarray(v.surface.GetPointsAttr().Get()), report["points"]+[0, 0, v.offset], atol=2e-8)
        checks["new_mesh_force_solution_and_1x_render"] = True
        value = {"compression_fraction": report["compression_fraction"],
                 "rotation_rad": report["relative_rotation_rad"].tolist(),
                 "mesh": dataclasses.asdict(v.shell.config)}
    finally:
        v.boundary_input.set_value(boundary)
        v.refinement_input.set_value(refinement)
        v.shell = old_shell
        for name, data in saved.items():
            setattr(v, name, data)
        # Recreate only our owned render topology before restoring its points.
        from pxr import UsdGeom
        v.stage.RemovePrim(v.owned_path)
        v.owned = UsdGeom.Xform.Define(v.stage, v.owned_path)
        v.owned.GetPrim().SetCustomDataByKey("source_sha256", old_shell.source["sha256"])
        v.make_scene()
        v.render()
        v.feedback.text = "Boundary mesh check passed; original material, mesh and accepted result restored."
    checks["prior_mesh_and_accepted_state_restored"] = True
    path = v.lab.project/"exact_joint/results"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_boundary_live.json")
    path.write_text(json.dumps({"checks": checks, "result": value, "scope": "Controller/numerical checks, not physical calibration"}, indent=2)+"\n")
    print(path, checks)


await validate()

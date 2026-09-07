"""Read-only four-mesh audit in Kit; no displayed stage/model replacement."""

import dataclasses
import hashlib
import json
from datetime import datetime, timezone

import numpy as np
import omni.kit.app

from exact_joint import app
from exact_joint.mechanics import CableFem, tension_pattern
from exact_joint.mesh_utils import compute_tet_volumes


async def audit():
    lab = app.ACTIVE
    report = {"baseline_config": dataclasses.asdict(lab.config), "meshes": [],
              "load": {"mode": "Compression", "tension_per_active_cable_n": .1},
              "source_sha256": lab.model.source["sha256"]}
    for refinement in (1, 2, 3, 4):
        config = dataclasses.replace(lab.config, refinement=refinement)
        model = CableFem(lab.model.source, config)
        result = model.solve(tension_pattern("Compression", .1), initial=np.zeros(6))
        volumes = compute_tet_volumes(model.points, model.tets[:, :4])
        report["meshes"].append({
            "refinement": refinement, "nodes": len(model.points), "elements": len(model.tets),
            "pet_volume_m3": float(volumes[model.ids == 0].sum()),
            "pla_volume_m3": float(volumes[model.ids == 1].sum()),
            "compression_m": result["stats"]["compression_m"],
            "peak_strain": result["stats"]["max_principal_strain"],
        })
        print("Mesh", refinement, report["meshes"][-1])
        del model
        await omni.kit.app.get_app().next_update_async()
    first, previous, last = report["meshes"][0], report["meshes"][-2], report["meshes"][-1]
    report["pet_volume_relative_change_ref1_to_ref4"] = last["pet_volume_m3"] / first["pet_volume_m3"] - 1
    report["compression_relative_change_ref3_to_ref4"] = last["compression_m"] / previous["compression_m"] - 1
    report["five_percent_response_convergence_passed"] = abs(report["compression_relative_change_ref3_to_ref4"]) < .05
    report["fixed_reference_geometry_passed"] = abs(report["pet_volume_relative_change_ref1_to_ref4"]) < 1e-8
    report["interpretation"] = "Known defect: refinement changes PET extrusion normals/reference volume. Neither convergence nor survival is established."
    report["source_files_sha256"] = {
        name: hashlib.sha256((lab.project / "exact_joint" / name).read_bytes()).hexdigest()
        for name in ("geometry.py", "mechanics.py", "audit_drop_mesh_live.py")
    }
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = lab.project / "exact_joint/results" / (stamp + "_mesh_audit")
    output.mkdir(parents=True, exist_ok=False)
    (output / "audit.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("MESH_AUDIT_RECORDED", output)


await audit()

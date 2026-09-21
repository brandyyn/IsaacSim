"""Live QA for the photo/video-aligned cable-family workshop.

Run inside Isaac Sim's python server after the nonlinear workshop is open.
Every family is applied through the same callback used by the visible Run cable
button, so this checks the routing registry, the accepted FEM path, and the
simple UI together rather than only unit-testing tension arrays.
"""

import asyncio
import json
from datetime import datetime, timezone

import numpy as np
import omni.kit.app

from exact_joint import app
from exact_joint.mechanics import CABLE_FAMILIES, CABLE_GROUPS


view = app.ACTIVE.drop_preview
assert type(view).__name__ == "ShellWorkshop"
assert view.window.title == "Joint FEM - cable folding pattern"
assert "Cable tension (N)" in view.inputs
assert tuple(CABLE_GROUPS) == CABLE_FAMILIES
view.inputs["Cable tension (N)"].set_value(3.0)


async def wait_job():
    kit = omni.kit.app.get_app()
    for _ in range(1800):
        await kit.next_update_async()
        if view.work is None or view.work.done():
            return
    raise AssertionError("cable FEM job did not finish")


rows = []
for family in CABLE_FAMILIES:
    view.neutral_shell()
    view.start_movement(family)
    await wait_job()
    assert view.job_outcome == "COMPLETE", (family, view.feedback.text)
    assert view.job_step == 1 and view.job_steps == 1
    active = np.flatnonzero(view.tensions > 1e-10).tolist()
    assert active == list(CABLE_GROUPS[family]), (family, active)
    report = view.shell.diagnostics(view.state)
    angles = np.rad2deg(report["relative_rotation_rad"])
    rows.append({"family": family, "active_cables": active,
                 "bend_x_deg": float(angles[0]), "bend_y_deg": float(angles[1]),
                 "twist_deg": float(angles[2]),
                 "compression_fraction": float(report["compression_fraction"]),
                 "max_membrane_strain": float(report["max_membrane_strain"]),
                 "reaction_world_n_nm": view.last_reaction.tolist(),
                 "feedback": view.feedback.text})

view.neutral_shell()
payload = {
    "timestamp_utc": datetime.now(timezone.utc).isoformat(),
    "scope": "one open-ended exact JSON knee; photo/video cable routing; custom nonlinear shell FEM",
    "tension_n_per_active_strand": 3.0,
    "families": rows,
    "config": {"shell": str(view.shell.config), "material": str(view.shell.material)},
    "source_sha256": view.shell.source["sha256"],
    "checks": {"simple_ui": True, "seven_force_families": True,
                "active_group_matches_registry": True, "neutral_restored": True},
}
path = view.lab.project / "exact_joint/results" / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ") + "_photo_cable_pattern.json")
path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
print(json.dumps({"path": str(path), "checks": payload["checks"], "families": rows}, indent=2))

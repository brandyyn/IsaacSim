"""Set the paper-guided leg targets in the active Isaac Sim stage.

Optional injected globals: hip_deg, knee_deg, ankle_deg.
"""

import omni.usd


stage = omni.usd.get_context().get_stage()
targets = {
    "HipJoint": float(hip_deg) if "hip_deg" in dir() else 0.0,
    "KneeActuator": float(knee_deg) if "knee_deg" in dir() else 45.0,
    "AnkleJoint": float(ankle_deg) if "ankle_deg" in dir() else 0.0,
}
for name, value in targets.items():
    path = f"/World/PanelCreaseLeg/Physics/{name}"
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Missing joint: {path}")
    prim.GetAttribute("drive:angular:physics:targetPosition").Set(value)
    print(f"{name}_target_deg={value}")

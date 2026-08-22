# Ansys compression v1

This case is the first FEA handoff applied to the original panel-crease knee.

## What was imported

- 52 synchronized samples from `0.01 s` through `1.00 s`.
- Force reaction history with X, Y, Z, and total components in newtons.
- Minimum, maximum, and average equivalent stress in pascals.
- Raw workbooks are preserved under `raw/`.
- Machine-readable data is in `processed/compression_profile.json`.

Observed values:

- Peak/final total reaction force: `639.69 N`.
- Peak/final maximum equivalent stress: `1.9679 GPa`.
- Peak/final average equivalent stress: `111.79 MPa`.

## How it is applied

The profile is treated as an equal-and-opposite compression load between the two physical v8 interface bodies:

- `/World/PanelCreaseLeg/OriginalJointShell/TopShell`
- `/World/PanelCreaseLeg/OriginalJointShell/BottomShell`

The current replay assumes the Ansys axes use the original source-joint frame and maps the force vector as `[Fx, Fy, Fz] -> [Fx, -Fz, Fy]` in Isaac Sim. This must be confirmed by the FEA owner before the result is called physically validated.

The equivalent-stress history is retained as structural reference telemetry. It is not converted into a PhysX material law because the workbooks do not supply material properties, thickness, yield/failure criteria, or a displacement/angle channel.

To run the replay in a running Isaac Sim Python server:

```powershell
_build\windows-x86_64\release\python.bat skills\isaac-sim-remote\scripts\isaacsim_send.py `
  --file run_fea_compression_replay.py `
  --arg force_scale=1.0 `
  --arg target_deg=45.0 `
  --arg output_path=$env:PANEL_CREASE_PROJECT_ROOT/fea/compression_v1/results/replay_summary.json
```

The FEA stage launcher is `open_fea_compression_v1.py`. It starts the visual-shell controller and runs the compression replay once at the stage's existing knee target. After the one-second replay, the load controller disables itself so later knee target commands can bend the joint normally. To replay again, set `forceReplayEnabled=true` and `replayComplete=false` on `/World/PanelCreaseLeg/FEACalibration` before playing.

## Important limitation

These files are a time-history load case, not a torque-angle characterization. They cannot by themselves determine the knee’s angle-dependent stiffness, damping, or actuator torque. To calibrate those, add the prescribed displacement or joint-angle history and the corresponding reaction torque/moment (plus the FEA frame and boundary-condition details).

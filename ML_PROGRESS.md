# Machine-Learning Progress

This is the shared progress log. Update it whenever the FEA model, simulator environment, training configuration, or evaluation result changes.

## Current state

- Phase: `0 - deterministic simulation baseline`
- Baseline commit: `5131a9740b3ce82e42331e923e1a45ffa396f71c`
- Simulation: `panel_crease_leg_v8.usd`, launched with `open_knee_gui.py`
- FEA case: `compression_v1` imported and replayed; frame mapping still requires FEA-owner confirmation
- ML policy: `none trained yet`
- Last validated behavior: full-scale 639.69 N compression replay completed at neutral and 45 degrees with interface translation below `1e-12 m`
- Current owner/action: provide FEA displacement/angle and reaction-torque channels, plus frame/material/boundary-condition metadata

## Roadmap

1. Package and review the FEA geometry/results handoff.
2. Map FEA coordinates and units to the Isaac Sim joint frame.
3. Fit or tabulate an FEA-derived mechanical response model.
4. Validate static and dynamic response in Isaac Sim without ML.
5. Implement the stabilization environment, observations, actions, rewards, and disturbances.
6. Run reproducible training and evaluation experiments.

## Progress log

| Date | Owner | Phase | Git commit | FEA case | Result | Next action |
|---|---|---|---|---|---|---|
| 2026-08-22 | Codex | Simulation baseline | `5131a974` | None | v8 portable physics/visual-controller checkpoint committed and pushed | Import the first reviewed Ansys FEA case |
| 2026-08-22 | Codex | FEA load replay | `6b6732d8` | `compression_v1` | 52 samples, 639.69 N peak force, neutral/45° replay stable | Add angle/displacement/torque data and confirm frame mapping |

## Run index

Add one row for every completed training or evaluation run. The detailed record belongs in `ml/runs/<run-id>/`.

| Run ID | Baseline commit | FEA case | Seed | Algorithm | Evaluation result | Checkpoint |
|---|---|---|---:|---|---|---|
| None yet | — | — | — | — | — | — |

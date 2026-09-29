# Cable-driven knee joint — progress checkpoint

This directory records the working single-joint experiment as of commit
`044e6594`.

The current scene is one open-ended photo/video-pattern knee joint using the
existing JSON topology, 0.4 mm PLA, 80 um PET, and the seven cable families:
compression, Bend X+/−, Bend Y+/−, and Twist CW/CCW.

## What is visible now

The default workshop view is **Blue + STL side-by-side**:

- the blue model is the displacement diagnostic driven from accepted JSON-shell
  FEM states;
- the assembled supplied STL is placed beside it and follows the accepted
  upper/lower frame pose as a labelled kinematic display;
- the white/red/yellow FEM overlay is hidden in this view;
- the **Blue + STL side-by-side** button restores or hides this comparison;
- the full FEM overlay can be restored without changing the solver.

The blue diagnostic and STL are visualization aids. The JSON shell remains the
FEM authority; the STL is not used as a solid stress or strength model.

## How to run an action

1. Choose a cable family.
2. Enter **Cable tension (N / active strand)**.
3. Press **Apply cable force**.
4. Watch the accepted 25%, 50%, 75%, and 100% FEM load steps in both models.
5. Press **Neutral** to return to the undeformed state.

Use 0.5–3 N/active strand first. The interactive input guard is 10 N/active
strand. This is a numerical experiment limit, not a measured actuator limit.

## What caps the force

The input is rejected above 10 N/active strand. Even below that, a candidate
is not applied when any of these checks fail:

- membrane strain guard: 3% by default;
- minimum frame separation: 8% of the neutral height;
- self-contact/surface-intersection check;
- nonlinear residual/convergence and iteration limit (900 iterations);
- finite, nonnegative twelve-cable tension validation.

These are model-use and numerical safety guards. They are not PLA/PET yield
limits, cable ratings, or a survival calculation. A measured material model and
actuator/cable capacity are still required before using the joint in a robotic
arm.

## Live validation at this checkpoint

The packaged Isaac Sim 6.0.1 runtime completed all seven cable callbacks. Each
family finished four accepted FEM steps and activated the expected cable group.
At 3 N/active strand the exploratory responses were approximately:

| Family | Response |
|---|---:|
| Compression | 0.372% compression |
| Bend X+ / X− | −0.083° / +0.082° bend X |
| Bend Y+ / Y− | −0.083° / +0.082° bend Y |
| Twist CW / CCW | −0.073° / +0.073° twist |

These values are small and uncalibrated; they do not establish the full motion
seen in the physical videos.

## Reproduce

Launch the nonlinear workshop with the project checkout selected through
`PANEL_CREASE_PROJECT_ROOT`, then use the live validation script:

```text
exact_joint/launch.ps1 -RuntimeRoot <Isaac-Sim-release> -Nonlinear
exact_joint/validate_photo_cable_pattern_live.py
```

Related sources:

- [`exact_joint/shell_view.py`](../../exact_joint/shell_view.py)
- [`exact_joint/shell_display_math.py`](../../exact_joint/shell_display_math.py)
- [`exact_joint/assets/stl_joint_v1/manifest.json`](../../exact_joint/assets/stl_joint_v1/manifest.json)
- [`ml/runs/20260929-exact-knee-button-stl-visible-v1/run_manifest.json`](../../ml/runs/20260929-exact-knee-button-stl-visible-v1/run_manifest.json)
- [`PROJECT_CONTEXT.md`](../../PROJECT_CONTEXT.md)
- [`ML_PROGRESS.md`](../../ML_PROGRESS.md)

This remains an exploratory custom shell FEM and display experiment, not a
validated large-motion, drop-survival, or ML-ready robotic-arm component.

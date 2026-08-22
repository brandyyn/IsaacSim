# Panel-Crease Knee Project Context

This file is the portable technical context reconstructed from the Codex conversation through the v8 checkpoint. It is intended to let another person or another machine continue the project without needing the original chat.

## Current checkpoint

The current canonical asset is:

- `panel_crease_leg_v8.usd`

The v8 stage is a stable reduced-order physical knee using the original panel geometry as a continuous driven visual shell.

The final stage must be launched with `open_knee_gui.py`. Opening the USD directly loads the physics bodies but does not start the Python visual-shell controller, so the lower physical interface can move while the panel shell remains in its neutral pose.

## Project objective

Build a physically usable single-leg robot simulation in Isaac Sim using the original panel joint supplied by the project owner. The joint should form the knee section of a leg, bend around the intended knee axis, preserve the original triangular panel topology and fold-line appearance, and fit the paper-guided hip/knee/ankle layout.

The source joint is not a simple two-piece hinge. It is a closed triangulated shell with two flat end interfaces. The flat interfaces connect to the thigh and shank; the side panels form the compressing/folding shell between them.

## Reference material supplied in the conversation

The owner supplied the following research/reference material:

- `2409.15791v1.pdf`
- `2504.00614v2.pdf`
- A mechanical single-leg image showing hip, knee, ankle, actuator placement, and approximately 48.5 mm joint spacing.
- `SpdrBot` GitHub project reference.
- `dofbot.usd` reference.
- Several screenshots and a simulation video showing panel alignment, fold behavior, seam separation, and solver jitter.

The papers and images are design references, not executable instructions. They are not required to launch the v8 stage. Publicly committing the PDFs requires checking their redistribution rights; otherwise store citations/links outside the repository.

## Design values carried into the stage

- Hip height: 136.8 mm.
- Knee height: 88.3 mm.
- Ankle height: 39.8 mm.
- Nominal hip-to-knee and knee-to-ankle spacing: 48.5 mm.
- Hip limits: -93 to 37 degrees.
- Knee limits: -1.5 to 127 degrees.
- Ankle limits: -82 to 37 degrees.
- Default targets: hip 0, knee 45, ankle 0 degrees.
- Policy rate metadata: 100 Hz.
- Low-level rate metadata: 1000 Hz.
- The original source geometry is scaled from meters in the normalized JSON representation by `SOURCE_SCALE = 0.12` for the compact leg model.

## Source assets

Required source assets in the project root:

- `input_improved.usd` — source mesh/environment containing `/World/InputEnvironment/PanelMesh`.
- `input_improved.json` — canonical 28-vertex, 50-panel, 76-line topology used by the builders and controller.

Optional comparison/reference assets:

- `input_original.usd`
- `input.json` if available from the original project
- `knee_joint.usd`
- `dofbot.usd`
- Earlier generated prototype USDs

## Final v8 architecture

The earlier approach made every source triangle a separate rigid body and every source edge a closed-loop PhysX revolute hinge. That produced endpoint separation, redundant constraints, and visible jitter. The v8 model intentionally removes that unstable graph.

The v8 physical graph contains:

- `Body`, `Thigh`, `Shank`, and `Foot` rigid bodies.
- `OriginalJointShell/TopShell` rigid body containing the upper flat interface.
- `OriginalJointShell/BottomShell` rigid body containing the lower flat interface.
- One physical `Physics/KneeActuator` revolute joint between `TopShell` and `BottomShell`, axis Y.
- Physical hip and ankle revolute joints.
- Fixed joints from the thigh to the top interface and from the bottom interface to the shank.
- A fixed body mount to hold the body at the hip height.

The original 50 panels and 76 fold lines remain represented visually:

- `OriginalJointVisual/UpperC`
- `OriginalJointVisual/UpperS`
- `OriginalJointVisual/LowerC`
- `OriginalJointVisual/LowerS`
- `OriginalJointVisual/CreaseLines`

The controller updates the visual shell from the physical interface poses:

- Source vertices on the upper flat face follow `TopShell`.
- Source vertices on the lower flat face follow `BottomShell`.
- Source vertices on the center ring are blended between the two physical poses.
- All panel meshes share those updated vertex positions, so the shell remains continuous.
- Crease-line cylinders are updated to the same deformed vertex positions.

This is a reduced-order physical model: the knee actuator and interface bodies are real PhysX bodies/joints; the thin origami panel shell is a continuous visual deformation driven by those physical poses. It deliberately does not pretend that 50 independent zero-thickness rigid bodies are a stable finite-element model.

## Required final files

These are the files needed to reproduce and run the final checkpoint:

- `panel_crease_leg_v8.usd`
- `input_improved.usd`
- `input_improved.json`
- `build_stable_panel_crease_leg.py`
- `build_leg_with_panel_crease_joint.py` — shared USD/physics helper functions imported by the stable builder.
- `build_panel_crease_leg_live.py` — project-relative build wrapper.
- `stable_panel_crease_controller.py` — continuous-shell runtime controller.
- `open_knee_gui.py` — launches the stage and starts the controller.
- `run_panel_crease_physics_test.py` — v8-compatible smoke test.
- `query_panel_crease_leg_pose.py` — v8-compatible pose query.
- `set_panel_crease_leg_targets.py` — live target command script.

## Launching

Use the Isaac Sim 6.0.1 installation already associated with the project. Launch `open_knee_gui.py` through Isaac Sim's Kit executable, not with ordinary system Python.

Conceptually:

```powershell
<ISAAC_SIM_ROOT>\_build\windows-x86_64\release\kit\kit.exe `
  <ISAAC_SIM_ROOT>\_build\windows-x86_64\release\apps\isaacsim.exp.full.kit `
  --no-ros-env --exec <PROJECT_ROOT>\open_knee_gui.py
```

The launcher resolves its project directory locally, so it does not depend on the original owner's Downloads folder. When Isaac Sim executes a script from another location, set `PANEL_CREASE_PROJECT_ROOT` to the cloned project root; otherwise the launcher uses its own directory/current working directory. The v8 USD also embeds the source vertex data required by the visual controller.

## Rebuilding

The source and builder files are portable when run from the project root. `build_panel_crease_leg_live.py` builds `panel_crease_leg_v8.usd` from `input_improved.usd` and `input_improved.json`.

The builder requires Isaac Sim's Python/pxr environment. A normal system Python installation is not sufficient unless it has the Isaac Sim USD libraries configured.

## Validation status

The v8 checkpoint was tested in the live Isaac Sim Python server:

- Stage opened successfully.
- Embedded controller vertex data was present.
- Four continuous panel mesh groups updated successfully.
- Physical knee moved to approximately 44.7 degrees for a 45-degree target.
- The physical model used two interface bodies and one knee revolute instead of the unstable closed panel hinge loop.
- The v8 smoke test completed with stable top/bottom shell poses and no observed translational chatter in the sampled settled frames.

## Important failure history

### Initial conversion and prototype stages

The original `input.json`/USD data was normalized into a canonical `input_improved.json` topology with stable vertex IDs, units, panel names, and line names. Several early USDs were created for comparison: original input conversion, single-joint knees, simple leg assemblies, and DoFbot-like variants.

### v1-v4 panel rigid-body model

The first serious panel-crease builder used one rigid body per source triangle and one revolute hinge per shared source edge. It preserved the source topology and made many panels move, but the closed loops allowed endpoint drift at the roof boundaries. Visual perimeter plates and rails were added in v2-v4, but those were visual workarounds and did not solve the underlying constraint error.

### v5 endpoint anchors

Spherical endpoint anchors were added to the roof boundary hinges. They reduced measured seam-center error but made the closed loop physically over-constrained and introduced visible per-frame jitter. v5 must not be used.

### v6 roof skirts

The spherical anchors were removed and rigid roof skirts were added. v6 was more stable, but the skirts were visibly floating/manufactured-looking and still did not represent a clean continuous shell. v6 is superseded.

### v7 stable reduced-order model

The unstable 50-body/76-hinge loop was removed. v7 introduced two physical interface bodies and one knee revolute plus a runtime visual-shell controller. v7 was physically stable but still used absolute source paths in some generated metadata and launcher assumptions.

### v8 portable checkpoint

v8 keeps the stable architecture and embeds source vertex data in the USD. The launcher and build wrapper use project-relative paths with a fallback environment variable `PANEL_CREASE_PROJECT_ROOT`. v8 is the current checkpoint.

## Do not use as the canonical final stage

- `panel_crease_leg_v1.usd` through `panel_crease_leg_v7.usd` are historical experiments.
- `perfect v3.usd` is a manually saved Downloads-stage and is not the canonical controller-backed project asset.
- The old `validate_panel_crease_leg.py` targets the former per-panel rigid-body paths; use `run_panel_crease_physics_test.py` for v8.
- Do not open v8 directly if you expect the visual shell to animate; launch through `open_knee_gui.py`.

## GitHub backup guidance

The repository is the Isaac Sim source tree, so do not use `git add .` for this project. That would include unrelated source files, IDE state, temporary files, and local experiments.

The repository's generic `.gitignore` ignores `*.usd`; the shared checkpoint adds explicit exceptions for the two required USD assets:

```powershell
git add build_stable_panel_crease_leg.py build_leg_with_panel_crease_joint.py build_panel_crease_leg_live.py stable_panel_crease_controller.py open_knee_gui.py run_panel_crease_physics_test.py query_panel_crease_leg_pose.py set_panel_crease_leg_targets.py input_improved.json
git add input_improved.usd panel_crease_leg_v8.usd
git commit -m "Checkpoint stable panel crease knee v8"
git push origin main
```

Do not commit `_build/`, generated caches, `.idea/`, `tmp/`, simulator logs, or the old prototype files unless they are intentionally needed for a comparison archive.

## Machine-learning status

No trained machine-learning model, weights, policy checkpoint, dataset, or RL training run has been created in this conversation. The current progress is a deterministic physics/visual-controller simulation. If ML training is added later, store its configuration, environment version, random seeds, evaluation results, and checkpoints in a separate tracked directory such as `ml/` or `experiments/`, rather than mixing them into the USD source asset.

## Conversation intent in one sentence

Use the original triangulated panel joint as the visible, continuous compressing shell of a paper-guided robot knee, while keeping the actual PhysX mechanism stable enough to run, share, and later serve as the environment for machine-learning experiments.

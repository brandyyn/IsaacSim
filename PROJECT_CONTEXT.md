# Panel-Crease Knee Project Context

This file is the portable technical context reconstructed from the Codex conversation through the v8 checkpoint. It is intended to let another person or another machine continue the project without needing the original chat.

## Current checkpoint

The current canonical asset is:

- `panel_crease_leg_v8.usd`

The current FEA load-replay asset is:

- `panel_crease_leg_fea_compression_v1.usd`

The v8 stage is a stable reduced-order physical knee using the original panel geometry as a continuous driven visual shell.

The latest uncommitted geometry-coupled experiment is `panel_crease_leg_constraints_v8_geometry_fold.usd`. Rebuild it with `build_coupled_fold_panel_leg_live.py` and launch it with `open_coupled_fold_panel_gui.py` using `PANEL_CREASE_MOTION_MODE=geometry_fold`; it is not yet the shared ML baseline. It uses one physical `KneeActuator`, starts exactly straight, and limits the provisional visual shell demo to 15 degrees because a 30-degree hard end pose produced excessive source-edge strain without FEA calibration. The default showcase path is the deterministic rigid-facet runtime; the numerical solver is validation-only because it drops the live stage to roughly 3–4 FPS.

The current presentation asset is `panel_crease_leg_constraints_v9_showcase.usd`. Rebuild it with `build_panel_crease_showcase_live.py` and launch it with `open_panel_crease_showcase_gui.py`. It keeps the original 50 surfaces, 28 shared vertices, and 76 crease edges, then uses three physical serial revolutes for the showcase: X lateral bend (−22…+22 degrees), Z twist (−30…+30 degrees), and Y knee fold (−1.5…127 degree physical stop). The default conservative coupled presentation envelope reaches a 60-degree fold peak while simultaneously traversing ±16.5 degrees lateral and ±22.5 degrees twist; the individual physical stops remain ±22/±30 degrees for separate validation. The 60-degree fold cap is the previously validated video/geometry-safe envelope, not a claim of FEA-calibrated material capacity. The visual path is batched rigid-facet projection with throttled diagnostics; the live full-cycle measurement was 54.93 FPS. This is a presentation asset, not the shared ML/FEA baseline.

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

The first Ansys handoff is stored under `fea/compression_v1/`. It contains the two raw workbooks, a machine-readable force/stress profile, the case manifest, and replay validation results.

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
- `OriginalJointVisual/BakedCreaseMesh` — the sole generated crease render layer; the old 76-cylinder fallback and `BakedCreaseNetwork` are not authored in the latest geometry-coupled stage.

The controller updates the visual shell from the physical interface poses:

- Source vertices on the upper flat face follow `TopShell`.
- Source vertices on the lower flat face follow `BottomShell`.
- Source vertices on the center ring are solved against the original 76 shared crease edges and 52 rigid-facet triangles.
- All panel meshes and the single crease prism mesh share the same updated vertex positions, so the shell remains continuous.
- Returning to zero uses the authored source coordinates directly; it does not numerically drift into a slightly twisted neutral pose.

This is a reduced-order physical model: the knee actuator and interface bodies are real PhysX bodies/joints; the thin origami panel shell is a continuous visual deformation driven by those physical poses. It deliberately does not pretend that 50 independent zero-thickness rigid bodies are a stable finite-element model.

## Latest geometry-coupled validation — 2026-08-25

- Generated stage: `panel_crease_leg_constraints_v8_geometry_fold.usd`.
- Physical graph: one `/World/PanelCreaseLeg/Physics/KneeActuator` revolute about Y; no `LateralBendJoint`, `TwistJoint`, or `MainKneeBendJoint` gimbal targets.
- Topology: original 28 shared vertices, 50 source surfaces, and 76 unique source crease edges; the generated stage contains 0 `Cylinder` prims and no `BakedCreaseNetwork`.
- Neutral: command `0°`, exact straight interface pose, maximum anchor error `0.0 m`, maximum facet-edge distortion `0.0`, and no self-clearance violation.
- Demonstration: slow cosine command `0° → 15° → 0°` at `0.0125 Hz`, with the shell solver running asynchronously at `2 Hz`. The 15° settled endpoint had maximum anchor error `0.0 m`, maximum self-clearance violation `0.0 m`, and approximately `0.052` maximum source-edge strain.
- The paper's knee stop of `127°` remains the physical revolute metadata. Extending the visible shell to that stop requires FEA displacement/angle/torque calibration; the current Ansys workbooks provide force/stress telemetry but not that response channel.

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
- `build_coupled_fold_panel_leg.py` and `build_coupled_fold_panel_leg_live.py` — rebuild the geometry-coupled experiment.
- `coupled_fold_profile_video_v1.json` — provisional video-derived compression envelope; no independent lateral/twist targets.
- `coupled_fold_motion_controller.py`, `facet_crease_panel_controller.py`, and `open_coupled_fold_panel_gui.py` — drive, solve, and launch the geometry-coupled stage.

FEA compression replay files:

- `panel_crease_leg_fea_compression_v1.usd` — v8 stage with the embedded `compression_v1` force/stress profile.
- `build_fea_calibrated_panel_crease_leg.py` and `build_fea_calibrated_panel_crease_leg_live.py` — rebuild the FEA stage.
- `fea_compression_controller.py` — equal-and-opposite force replay controller.
- `run_fea_compression_replay.py` and `open_fea_compression_v1.py` — run or launch the replay.
- `fea/compression_v1/` — raw workbooks, processed profile, manifest, and replay summaries.

Experimental multi-DOF motion variant:

- `panel_crease_leg_multidof.usd` — separate, uncommitted experiment; the v8 stage remains the canonical baseline.
- `build_multidof_panel_crease_leg.py` and `build_multidof_panel_crease_leg_live.py` — add a serial three-revolute gimbal around the original joint shell.
- `open_multidof_knee_gui.py` — opens the variant and starts the controlled motion demo.
- `multidof_knee_motion_controller.py` — drives independent lateral bend (X), twist (Z), and main knee bend (Y) targets.
- `set_multidof_knee_targets.py` — disables the demo and sets explicit targets for the three joints.
- `run_multidof_physics_test.py` — neutral/combined-positive/combined-negative physics smoke test.

Constraint-focused compliant-shell experiment:

- `panel_crease_leg_constraints_v2.usd` — generated experimental stage; v8 remains the canonical baseline.
- `build_constrained_panel_crease_leg.py` and `build_constrained_panel_crease_leg_live.py` — embed the original topology's 76 edge rest lengths, upper/lower anchor sets, and constraint metadata.
- `build_compliant_panel_crease_leg.py` and `build_compliant_panel_crease_leg_live.py` — build the v2 compliant constraint stage from the project-relative source files.
- `compliant_panel_crease_controller.py` — updates one shared 28-vertex map using hard interface anchors and bounded under-relaxed edge projections.
- `open_compliant_panel_crease_gui.py` — opens v2 and starts the compliant shell plus the X/Z/Y physical driver demo.
- `set_compliant_panel_crease_targets.py` — disables the demo and sets explicit lateral, twist, and knee targets.
- `run_compliant_panel_crease_physics_test.py` — validates neutral, sagittal, combined-positive, and combined-negative poses.

The constraint interpretation is intentional. An offline rank check of the closed 50-triangle topology found zero free positional modes when all 76 edge lengths and the upper roof are hard constraints. Therefore v2 does not claim impossible rigid folding: it keeps the shared vertices and hard end interfaces, applies finite bounded compliance to the source edges, and records edge strain so the remaining material deformation can later be calibrated from FEA.

Global least-strain v3 experiment:

- `panel_crease_leg_constraints_v3.usd` — generated experimental stage; v8 remains the canonical baseline and v2 remains available for comparison.
- `build_optimized_constraint_panel_crease_leg.py` and `build_optimized_constraint_panel_crease_leg_live.py` — build v3 from the same original USD/topology inputs.
- `optimized_constraint_panel_crease_controller.py` — solves all free shared vertices together with a normalized fourth-power edge-strain objective, damped Gauss-Newton steps, line search, and bounded per-vertex motion.
- `open_optimized_constraint_panel_crease_gui.py`, `set_optimized_constraint_panel_crease_targets.py`, and `run_optimized_constraint_panel_crease_physics_test.py` — launch, command, and validate v3.

V3 specifically removes the edge-order bias in v2. It is still a quasi-static compliant visual-shell model, not a calibrated FEA solver; hard anchors and all source edges remain measurable, while the objective distributes unavoidable deformation across the shell.

Facet-rigid crease v4 experiment:

- `panel_crease_leg_constraints_v4.usd` — generated experimental stage; v8 remains the canonical baseline and v2/v3 remain available for comparison.
- `build_facet_crease_panel_leg.py` and `build_facet_crease_panel_leg_live.py` — build v4 from the original USD/topology inputs.
- `facet_crease_panel_controller.py` — local/global ARAP-style solver. It fits a rigid transform to each source triangle, solves all shared free vertices against those targets plus global normalized edge strain, and hard-anchors the upper/lower interfaces.
- `open_facet_crease_panel_gui.py`, `set_facet_crease_panel_targets.py`, and `run_facet_crease_panel_physics_test.py` — launch, command, and validate v4.

The source contains 48 triangular shell panels and two flat interface quads. V4 keeps the two quads as rigid interface surfaces by splitting them internally into four solver triangles; no diagonal crease lines are added to the visual model. All visible panels and all 76 crease cylinders still read from one shared vertex map, so a shared crease cannot render as two independently drifting endpoints.

V4 is still a bounded quasi-static visual-shell model, not a deformable PhysX body or a calibrated FEA law. The light facet-fit weight is intentionally paired with the global edge objective because the closed shell has no exact rigid solution for every arbitrary independently driven X/Z/Y end pose. The remaining facet distortion is telemetry for the next FEA/video calibration pass.

Coupled video-profile v5 experiment:

- `panel_crease_leg_constraints_v5.usd` — generated experimental stage; v8 remains the canonical baseline and v4 remains available for independent-axis comparison.
- `coupled_fold_profile_video_v1.json` — versioned coupling table. It records the three reference videos, axis convention, measurement method, confidence limitation, and fold-to-X/Z response samples.
- `build_coupled_fold_panel_leg.py` and `build_coupled_fold_panel_leg_live.py` — build v5 and embed the profile into the USD.
- `coupled_fold_motion_controller.py` — exposes one `foldCommandDeg`, reads the embedded compression profile, and drives only the physical `KneeActuator` in the current geometry-coupled stage.
- `open_coupled_fold_panel_gui.py`, `set_coupled_fold_target.py`, and `run_coupled_fold_panel_physics_test.py` — launch, command, and validate v5.

The historical V5 experiment removed the arbitrary independent target generator from its active demo. The current geometry-coupled pass goes further: it does not author independent X/Z gimbals at all. A single fold command drives the physical Y-axis knee and the shared source-panel solver. The profile is explicitly `provisional_visual_measurement`: it is derived from video geometry, not precise metrology, and must be replaced or fitted when the FEA owner supplies angle/displacement/torque channels.

Canonical reference-crease v6 pass:

- `panel_crease_leg_constraints_v6.usd` — generated experimental stage; v8 remains the canonical ML baseline and v5 remains available for comparison.
- `build_leg_with_panel_crease_joint.py` now rejects duplicate named edges while loading the source topology and exposes the canonical edge registry to the builders.
- Historical v6 builds embedded the expected edge keys and authored one crease cylinder per source edge; the current stable builder uses the single baked crease mesh instead.
- `stable_panel_crease_controller.py` and `facet_crease_panel_controller.py` hide non-canonical/duplicate visual crease prims before updating the shared vertex map. This protects against the old manually saved dynamic visual layer that doubled crease lines.
- `validate_reference_crease_topology.py` compares the original legacy JSON, `input_improved.json`, and a generated stage. The v6 validation result is 76 original lines, 76 improved unique edges, 76 stage crease prims, zero duplicates, and matching source edge sets.

V6 changes the crease authoring/visibility contract only; it does not change the physical v8 baseline, the coupled profile, or the FEA interpretation. The attached images and videos remain visual reference evidence, not executable topology input.

Corrected coupled-fold v7 safe-range pass:

- `panel_crease_leg_constraints_v7_safe.usd` is a historical live comparison stage; v8 remains the canonical ML baseline and the geometry-coupled v8 experiment is the current live pass.
- `facet_crease_panel_controller.py` keeps the 76 canonical source edges and adds a light midpoint-separation barrier only between nearby non-adjacent crease edges. This prevents the visible crease bundle from stacking while preserving shared vertex continuity.
- The coupled controller applies the stage's fold limits inside `profile_targets`, so manual requests cannot bypass the geometry limit. The current geometry-coupled stage uses a provisional 0–15° shell range; the paper's 127° revolute stop is retained only on the physical knee metadata until FEA displacement/torque response is available.
- The embedded 0–120° profile is retained as measured/provisional source data, but the current demo clamps it to 15° because poses above that range require calibrated shell/contact response rather than forcing the reduced-order visual shell.

Neutral-start twist-sweep pass:

- `build_coupled_fold_panel_leg.py` now explicitly resets the inherited multi-DOF knee target to `0°`; the previous `30°` inherited target was why the model opened already bent.
- `twist_sweep_motion_controller.py` is a separate driver from the coupled fold profile. It holds knee fold and lateral bend at `0°` and sweeps the live `TwistJoint` from its authored lower limit to upper limit.
- The current USD limits are `TwistJoint = −30°…+30°`, so this is a complete supported-range cycle. It is not a literal 360° physical twist; changing the stops to permit 360° would be a new mechanical design and must be supported by joint/FEA evidence.
- The normal launcher default is now `neutral` and paused; it does not start a motor sweep or apply a hidden twist. Use `PANEL_CREASE_MOTION_MODE=twist_sweep` only for the isolated torsion test.

Neutral-start full motion envelope pass:

- The source roof planes are at ±13.2 mm from the knee center, while the old fixed link anchors were at ±15.0 mm. The builder now derives the fixed-joint anchors from the actual roof planes, eliminating the 1.8 mm plate/shank mismatch.
- `full_supported_motion_controller.py` runs a smooth 360° phase cycle through lateral `−22°…+22°`, twist `−30°…+30°`, and the current geometry-valid fold limit `0°…60°`. The first half returns to neutral between individual stop tests; the second half adds a bounded combined pass at 75% of the lateral/twist spans around the measured fold response, with the fold peak kept on its coupled profile.
- The generated v7 stage's paused neutral state has identity rotations for the top shell, gimbal carriers, and bottom shell; the bottom plate center and shank anchor coincide at `z=0.0751 m`.
- Endpoint checks reached both lateral stops, both twist stops, and the 60° fold target with zero anchor error, zero self-clearance violation, and a plate/shank gap of approximately 1–4 μm. This is a bounded reduced-order envelope, not yet a claim of real-world FEA-calibrated dynamics.
- The combined live targets are approximately `(fold=30°, lateral=+18.5°, twist=+24.1°)`, `(fold=60°, lateral=+4°, twist=+3.2°)`, and `(fold=30°, lateral=−14.5°, twist=−20.9°)` before returning to neutral. The shared-anchor and self-clearance monitors remained zero in the endpoint pass; facet-edge distortion remains a reduced-order deformation metric and must be calibrated against FEA before training.

Facet-rigid canonical showcase runtime:

- `facet_crease_panel_controller.py` is the normal visual controller. It preserves the shared 28-vertex map, solves the 50 source surfaces as 52 internal rigid triangles, and uses rigid-facet endpoint projection plus rigid-facet display interpolation. Direct linear vertex blending is not used because it shears the triangular panels.
- `BakedCreaseMesh` is one combined six-sided render mesh containing the exact 76 canonical source crease segments. It is driven from the same welded vertex positions as the colored panels. The latest geometry-coupled builder does not author the old `CreaseLines` cylinders or the `BakedCreaseNetwork` curve layer.
- The v7 physics scene is authored at 120 Hz with explicit 8 position / 2 velocity iteration caps and matching Newton/PhysX step-rate fields. This replaces the old 1000 Hz experimental setting so copies of the joint can scale to a larger robot.
- Live measurements on the rigid-mesh stage were approximately 64 FPS paused, 17 FPS with neutral physics, and 34 FPS during the slower showcase motion. These are runtime measurements, not a claim of FEA-calibrated dynamics.

## Launching

Use the Isaac Sim 6.0.1 installation already associated with the project. Launch `open_knee_gui.py` through Isaac Sim's Kit executable, not with ordinary system Python.

Conceptually:

```powershell
<ISAAC_SIM_ROOT>\_build\windows-x86_64\release\kit\kit.exe `
  <ISAAC_SIM_ROOT>\_build\windows-x86_64\release\apps\isaacsim.exp.full.kit `
  --no-ros-env --exec <PROJECT_ROOT>\open_knee_gui.py
```

The launcher resolves its project directory locally, so it does not depend on the original owner's Downloads folder. When Isaac Sim executes a script from another location, set `PANEL_CREASE_PROJECT_ROOT` to the cloned project root; otherwise the launcher uses its own directory/current working directory. The v8 USD also embeds the source vertex data required by the visual controller.

For the experimental multi-axis demo, use the same Kit command with `open_multidof_knee_gui.py`. It opens `panel_crease_leg_multidof.usd`, starts the motion demo, and enables the Python server for live target commands. To take manual control, run `set_multidof_knee_targets.py`; its `lateral_deg`, `twist_deg`, and `knee_deg` arguments are degrees and it disables the automatic demo first.

For the constraint-focused v2 experiment, use `open_compliant_panel_crease_gui.py`. It opens `panel_crease_leg_constraints_v2.usd`, starts the bounded compliant solver, and enables the same X/Z/Y motion demo. Rebuild it with `build_compliant_panel_crease_leg_live.py` before launching after a source-topology change.

For the v3 global constraint experiment, use `open_optimized_constraint_panel_crease_gui.py`. It opens `panel_crease_leg_constraints_v3.usd`, starts the global least-strain solver, and enables the same X/Z/Y motion demo.

For the v4 facet-rigid crease experiment, use `open_facet_crease_panel_gui.py`. It opens `panel_crease_leg_constraints_v4.usd`, starts the local/global facet solver, and enables the same X/Z/Y motion demo. Rebuild it with `build_facet_crease_panel_leg_live.py` after a source-topology change.

For the current geometry-coupled experiment, use `open_coupled_fold_panel_gui.py` with `PANEL_CREASE_MOTION_MODE=geometry_fold` and `PANEL_CREASE_VISUAL_MODE=baked` (the default). It opens `panel_crease_leg_constraints_v8_geometry_fold.usd`, starts from exact neutral, and drives one physical `KneeActuator` through a smooth 0–15° shell-safe fold response at `0.025 Hz`. The baked runtime uses the original 50 source surfaces, 52 rigid triangles, 28 shared vertices, and 76 crease segments without a blocking per-frame optimizer, and measured about 41–43 FPS in the live app. Use `PANEL_CREASE_VISUAL_MODE=solver` only for numerical validation; it is not the scalable showcase path. The legacy `showcase`, `supported_full_envelope`, and `twist_sweep` modes remain for historical stages. Use `PANEL_CREASE_MOTION_MODE=neutral` for a paused exact-straight inspection.

For the optimized presentation, use `open_panel_crease_showcase_gui.py`; it selects the v9 stage, baked visual mode, and the coupled `combined` envelope automatically. Set `PANEL_CREASE_SHOWCASE_PATTERN=sequential` to show each single-axis stop from neutral, or set it to `hybrid` to include both the full single-axis stops and the bounded coupled pass. The launcher settles in neutral before enabling motion.

## Rebuilding

The source and builder files are portable when run from the project root. `build_panel_crease_leg_live.py` builds `panel_crease_leg_v8.usd` from `input_improved.usd` and `input_improved.json`.

The v2 experiment is rebuilt with `build_compliant_panel_crease_leg_live.py`; it writes `panel_crease_leg_constraints_v2.usd`. The generated USD is ignored by the repository's broad USD rule, so share the source builders and controller files and regenerate the stage on each machine.

The v3 experiment is rebuilt with `build_optimized_constraint_panel_crease_leg_live.py`; it writes `panel_crease_leg_constraints_v3.usd`. The generated USD is likewise ignored, so regenerate it from the source builders on each machine.

The v4 experiment is rebuilt with `build_facet_crease_panel_leg_live.py`; it writes `panel_crease_leg_constraints_v4.usd`. The generated USD is likewise ignored, so regenerate it from the source builders and controller files on each machine.

The current geometry-coupled experiment is rebuilt with `build_coupled_fold_panel_leg_live.py`; it writes `panel_crease_leg_constraints_v8_geometry_fold.usd`, embeds `coupled_fold_profile_video_v1.json` (schema 2.0), authors only the welded `BakedCreaseMesh`, and sets the scalable 120 Hz physics scene. The generated USD is likewise ignored, so regenerate it from the source builder, profile, controller, and topology files on each machine. Set `PANEL_CREASE_COUPLED_OUTPUT_NAME` to a different filename when preserving a comparison stage.

The presentation asset is rebuilt with `build_panel_crease_showcase_live.py`; it writes `panel_crease_leg_constraints_v9_showcase.usd` from the same source USD, topology, and profile. It layers the physical X/Z/Y showcase mechanism onto the coupled stage and retains the batched `baked_panel_crease_controller.py` runtime. The generated USD is ignored, so regenerate it on each machine rather than treating it as the ML baseline.

The builder requires Isaac Sim's Python/pxr environment. A normal system Python installation is not sufficient unless it has the Isaac Sim USD libraries configured.

## Validation status

The v8 checkpoint was tested in the live Isaac Sim Python server:

- Stage opened successfully.
- Embedded controller vertex data was present.
- Four continuous panel mesh groups updated successfully.
- Physical knee moved to approximately 44.7 degrees for a 45-degree target.
- The physical model used two interface bodies and one knee revolute instead of the unstable closed panel hinge loop.
- The v8 smoke test completed with stable top/bottom shell poses and no observed translational chatter in the sampled settled frames.

The multi-DOF experiment was then tested separately in Isaac Sim 6.0.1:

- The original 50-panel visual shell was preserved and driven from the physical end-body poses.
- The three physical axes are `/World/PanelCreaseLeg/Physics/LateralBendJoint` (X, ±22°), `/World/PanelCreaseLeg/Physics/TwistJoint` (Z, ±30°), and `/World/PanelCreaseLeg/Physics/MainKneeBendJoint` (Y, −1.5° to 127°).
- The smoke test produced distinct neutral, combined-positive, and combined-negative shank poses; the automatic motion demo also changed the pose between samples.
- This is a controlled reduced-order motion model, not yet a validated deformable-contact or FEA-calibrated panel solver. Do not use it as the ML baseline until the multi-axis response and FEA frame mapping are reviewed.

The v9 presentation stage was clean-reloaded and profiled in Isaac Sim 6.0.1:

- The stage contains the original shell topology plus physical `/World/PanelCreaseLeg/Physics/LateralBendJoint`, `TwistJoint`, and `MainKneeBendJoint` revolutes. The authored limits are lateral ±22°, twist ±30°, and physical fold −1.5°…127°.
- The launcher waits in neutral before enabling the coupled driver, so the showcase begins straight instead of inheriting a previous target. The driver uses quintic waypoint transitions and no per-frame nonlinear optimizer.
- The batched visual runtime preserves the 50 source surfaces, 28 shared vertices, and 76 crease edges. A 1,800-update conservative coupled full-cycle probe measured 54.93 FPS and commanded fold 0…60°, lateral −16.5…+16.5°, and twist −22.5…+22.5° simultaneously during the coupled portions; the separate physical stops remain ±22°/±30°.
- This is a smooth presentation envelope near the physical paper stop, not an FEA-calibrated material/failure limit or an ML training baseline. Keep solver mode and the v8 one-actuator stage for validation/calibration.

The constraint-focused v2 pass was tested live in Isaac Sim 6.0.1 from the uncommitted `8fcbb672` worktree:

- The stage contains the original 50 triangles, 28 shared vertices, and 76 source-edge constraints; the red/blue debug axis bars and center marker are absent.
- Upper and lower interface vertices remain hard-anchored with measured maximum anchor error `0.0 m` in all tested poses.
- The bounded solver uses 32 iterations, relaxation `0.16`, and a maximum per-edge correction of `0.002 m`; it has no explicit spring velocity integration, so the prior mass-spring explosion/jitter path is removed.
- Neutral: maximum edge error `1.184e-5 m`, maximum edge strain `0.001692`.
- Sagittal knee target `(X=0°, Z=0°, Y=42°)`: maximum edge error `0.00323038 m`, maximum edge strain `0.461671`.
- Combined positive `(X=14°, Z=16°, Y=42°)`: maximum edge error `0.00426583 m`, maximum edge strain `0.445082`.
- Combined negative `(X=-14°, Z=-16°, Y=24°)`: maximum edge error `0.002886489 m`, maximum edge strain `0.257888`.
- The combined-positive settled samples stopped changing after the first transition sample, with anchor error remaining `0.0 m`; this checks solver settling, not FEA accuracy.
- The residual strain is expected from the closed-topology rank result and is the calibration signal for the next FEA displacement/torque handoff. This v2 stage is still an experiment, not the ML baseline.

The global least-strain v3 pass was then rebuilt and tested live in Isaac Sim 6.0.1:

- The stage still contains the original 50 triangles, 28 shared vertices, and 76 source-edge records; debug axis bars and the center marker remain absent.
- Upper/lower interface anchor error remained `0.0 m` in every tested pose.
- V3 uses normalized edge strain, objective power `4`, four damped Gauss-Newton iterations, line search, and a `0.003 m` maximum per-vertex solver step.
- Neutral after 120 updates: maximum edge error `4.2312e-5 m`, maximum edge strain `0.002044`.
- Sagittal `(X=0°, Z=0°, Y=42°)` after 240 updates: maximum edge error `0.002944924 m`, maximum edge strain `0.183173`.
- Combined positive `(X=14°, Z=16°, Y=42°)` after 240 updates: maximum edge error `0.003722613 m`, maximum edge strain `0.209054`.
- Combined negative `(X=-14°, Z=-16°, Y=24°)` after 240 updates: maximum edge error `0.002796492 m`, maximum edge strain `0.135110`.
- At a fixed combined-positive pose, the solver settled to approximately `0.003812 m` maximum edge error and `0.20906` maximum strain without oscillation; anchor error stayed `0.0 m`.
- Compared with v2's combined-positive maximum strain of approximately `0.445`, v3 distributes the deformation more evenly. FEA displacement/torque data is still required before treating this as a calibrated ML environment.

The facet-rigid v4 pass was rebuilt and tested live in Isaac Sim 6.0.1:

- The stage contains the original 50 source surfaces, 52 internal triangular facet records, 28 shared vertices, and 76 source-edge records; no debug axis bars or center marker were added.
- Neutral after 120 updates: maximum edge/facet distortion `0.021253`, maximum facet-fit residual `0.0001666 m`, maximum anchor error `0.0 m`.
- Sagittal `(X=0°, Z=0°, Y=42°)`: maximum facet edge distortion `0.177980`, maximum facet-fit residual `0.0022739 m`, maximum anchor error `0.0 m`.
- Combined positive `(X=14°, Z=16°, Y=42°)`: maximum facet edge distortion `0.219584`, mean facet edge distortion `0.114243`, maximum facet-fit residual `0.0024504 m`, maximum anchor error `0.0 m`.
- Combined negative `(X=-14°, Z=-16°, Y=24°)`: maximum facet edge distortion `0.150100`, maximum facet-fit residual `0.0016232 m`, maximum anchor error `0.0 m`.
- At the fixed combined-positive pose, four 100-update samples settled from `0.217563` to `0.218550` maximum facet distortion and then remained unchanged; anchor error stayed `0.0 m` throughout. This checks numerical settling and shared-interface continuity, not physical accuracy.
- V4 is closer to the reference geometry in how it defines the deformation: triangular facets are the local rigid units and their common vertices are the crease lines. The remaining 15–22% worst-case facet-edge distortion under combined independent end poses shows that the current X/Z/Y driver limits are not yet a calibrated reproduction of the physical videos.

The coupled video-profile v5 pass was rebuilt and tested live in Isaac Sim 6.0.1:

- The active demo has one command path. The independent multi-DOF motion task is disabled when the v5 `motionDriver` is `coupled_fold_profile`.
- Profile mapping was confirmed live: fold `30°` → lateral `2.0°`, twist `1.6°`; fold `60°` → lateral `4.0°`, twist `3.2°`; fold `90°` → lateral `6.0°`, twist `4.8°`.
- The sweep reached fold `0°`, `30°`, `60°`, and `90°` with hard anchor error `0.0 m` at every sample. The static 60° settling samples remained stable after the transition, with anchor error `0.0 m`.
- The fold-90 sample reached maximum facet edge distortion `0.363797` and maximum facet-fit residual `0.0037709 m`. This is a useful coupled-motion baseline, not proof of physical accuracy; the profile must be calibrated against actual measured interface poses and FEA response before ML training.

The canonical reference-crease v6 pass was rebuilt and tested live in Isaac Sim 6.0.1:

- The legacy `input.json` and canonical `input_improved.json` have identical geometric edge sets after the documented millimetre-to-metre conversion: 76 lines and 76 unique edges.
- The generated v6 stage contains 76 crease cylinders under `OriginalJointVisual/CreaseLines`, each with one source edge key; the stage contains no duplicate edge keys and no non-canonical `Crease_*` visual layer.
- The coupled fold smoke test reached fold commands `0°`, `30°`, and `60°` with zero maximum anchor error. The settling samples at `60°` remained finite and stable.
- V6 is a visual-topology correction/guard, not a claim that the provisional video profile or facet distortion is FEA-calibrated.

The corrected coupled-fold v7 safe-range pass was rebuilt and tested live in Isaac Sim 6.0.1:

- A requested 90° command returned the stage-limited coupled targets `fold=60°`, `lateral=4°`, `twist=3.2°`, `compression=0.5`.
- The live stage contained exactly 76 crease cylinders, 76 unique source edges, and no duplicate source edge keys.
- On a continuous 0→60° path, the closest measured non-adjacent crease-segment pair remained outside the combined 0.52 mm crease-cylinder diameter at the validated 60° endpoint; the bounded stage reported zero self-clearance violation.
- The v7 midpoint barrier is a reduced-order visual/contact guard, not an FEA-calibrated self-contact law. The stage is intentionally limited until measured contact, displacement, and torque data are available.

The neutral-start twist pass was tested live in Isaac Sim 6.0.1:

- `panel_crease_leg_constraints_v7_safe.usd` opens with all three physical targets at `0°`; the prior inherited 30° knee target is removed.
- The default `twist_sweep` driver holds fold and lateral bend at `0°` and traverses the authored `TwistJoint` range `−30°…+30°`. Settled static endpoint checks reached approximately `−29°` and `+30°` without changing the fold target.
- The live stage retained 76 canonical crease cylinders/edges, `constraintMaxAnchorErrorM = 0.0`, and `constraintMaxSelfClearanceViolationM = 0.0`; console error scan returned zero matching lines.
- This is a full supported-range cycle (360° of sweep phase), not a claim of 360° physical joint rotation. The current joint stops do not authorize a 360° twist.

The prior v7 facet-rigid visual path was corrected on 2026-08-25 after the linear interpolation experiment flattened the visible fold structure. It is historical; the geometry-coupled v8 experiment above is the current live pass:

- `panel_crease_leg_constraints_v7_showcase_rigid_mesh3.usd` is the prior live showcase stage; it is superseded by `panel_crease_leg_constraints_v8_geometry_fold.usd`.
- The default launcher mode is `baked`, using the 50 source surfaces, 52 internal solver triangles, 28 shared vertices, and the original 76-edge crease registry through a deterministic rigid-facet projection.
- The numeric facet solve remains available as an explicit validation mode. It runs asynchronously, but its Python constraint loops still starve the Kit update path on this machine; active live timing was about 3–4 FPS versus about 41–43 FPS for the deterministic runtime.
- The fast runtime projects each source triangle from the physical top/bottom interface poses, welds shared vertices, and performs no independent gimbal motion. It is a reduced-order display model; use the solver/FEA path for calibrated strain and torque work.
- `BakedCreaseMesh` renders all 76 authored source fold lines in one point-updated mesh. Its endpoints use the same welded map as the colored surfaces, so the top and bottom interfaces stay connected while the line structure follows every fold.
- The current geometry-fold showcase runs at `0.025 Hz` with a smooth cosine command from exact neutral through the 15° provisional envelope. The latest builder removes the unused fallback geometry instead of relying on visibility overrides.
- The prior `showcase` mode visited independent lateral/twist stops and is retained only for historical comparison. The current `geometry_fold` mode starts at exactly `0°`, drives only the physical knee, and is limited to the provisional 15° shell envelope. The model is still a reduced-order visual/physics baseline, not an FEA-calibrated material or torque model.

The first FEA compression replay was also run against the v8 physical interface bodies:

- 52 Ansys samples from 0.01 to 1.0 seconds were replayed at full force scale.
- Peak applied total reaction force was 639.69 N.
- The replay completed at neutral and at a 45-degree knee target.
- Measured top/bottom interface translation stayed below `1e-12 m` in both runs.
- Equivalent stress is recorded as reference telemetry; it is not yet a calibrated material or failure model.
- The workbooks contain no displacement or knee-angle channel, so angle-dependent stiffness, damping, and torque are not identified yet.

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

No trained machine-learning model, weights, policy checkpoint, dataset, or RL training run has been created in this conversation. The current progress is a deterministic physics/visual-controller simulation with a first Ansys compression load replay. The shared rules and handoff templates are in `PROJECT_RULES.md`, `ML_PROGRESS.md`, `fea/`, and `ml/`. The next calibration input is the missing displacement/angle and reaction-torque data, followed by non-ML response validation and then stabilization learning.

## Conversation intent in one sentence

Use the original triangulated panel joint as the visible, continuous compressing shell of a paper-guided robot knee, while keeping the actual PhysX mechanism stable enough to run, share, and later serve as the environment for machine-learning experiments.

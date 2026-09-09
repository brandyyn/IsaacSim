# Force-driven knee workshop: status and demonstration

This is an **uncalibrated nonlinear shell experiment**, not a validated prediction
of the physical joint's full range or drop survival. The original small-strain
solid FEM remains available separately. Neither is promoted to the ML baseline.

## What changed

- Original JSON: 28 source vertices, 50 panels, 76 crease paths, unchanged at
  neutral. Uniform width 30 mm; neutral height 26.4 mm; one knee only.
- Only the upper/lower square **perimeters** are rigid. Roof interiors and side
  panels have free deformation nodes; the rigid leg links attach to perimeter yokes.
- Each side-panel boundary has a finite-width **PET-only** strip. PLA is inset
  by half the exposed gap on each adjacent face. PLA 0.4 mm and PET 80 micrometres
  are retained. The default mesh has 458 nodes / 912 triangular surface elements.
- Original crease paths are subdivided, allowing different fold angles along a
  crease and distributed twisting. These subdivisions are not additional cuts or
  a changed manufacturing pattern. Full travel has not been established.
- Applied cable tensions enter the energy as tension times current cable length;
  gradients provide the forces and moments. No target angle drives Apply/demo.
- The same deformable shell and perimeter frames have inertia in the drop model.
  Accepted calculated states drive the visible leg at **1x displacement**.

The formulation uses finite-rotation membrane elements and discrete bending
hinges. This class of reduced model can include panel stretching and bending,
but implementing it is not physical validation: see the primary
[non-rigid origami mechanics paper](https://paulino.scholar.princeton.edu/document/1566).

## Open the correct workshop

Use the installed Isaac Kit `--exec` launcher with
`exact_joint/open_nonlinear_gui.py` and `PANEL_CREASE_PROJECT_ROOT` set to the
checkout. The existing `open_gui.py` deliberately still opens the rigid-roof
reference. From that window, choose **Nonlinear frames + flexible panels**.

The new window is **Nonlinear knee - rigid frames / flexible panels**. Its heading
says rigid square frames / flexible interiors. The original reference window is
hidden while it is active. Generic USD material edits and generic PhysX forces
do not feed this custom CPU solver; use its own controls.

Portable PowerShell launcher: `exact_joint/launch.ps1 -RuntimeRoot <built-release> -Nonlinear`.
Close or return from another workshop first; the launcher does not kill existing Kit sessions.

## Demonstrate the response

1. **Neutral / cancel** removes loads and cancels this workshop's calculation.
2. Select a cable family, enter **Cable tension (N)**, press Enter, then
   **Apply cable + force**. The input is tension *per active strand*. The load
   ramps through eight equilibrium solves, ending at your requested tension.
   Gold overlays identify loaded cables; cable endpoints follow the solved frames.
3. **Cable demo** ramps compression, both X bends, both Y bends and both twists.
   It uses the entered cable tension, unlike the older reference demo. Zero
   tension produces no cable-driven deformation. It is a quasistatic load sweep,
   not a calibrated motor-speed or spool-displacement simulation.
4. For an external load, set cable tension to zero and enter **Lower frame force
   X/Y/Z (N)**. Press Apply. The resultant is distributed over the lower perimeter;
   the upper perimeter is fixed. Positive Z compresses the neutral module.
   X/Y are transverse forces, not imposed bend angles. Force lines use 2 mm/N.
5. **Drop 70 mm / recompute** releases the whole leg upright, foot-first, with
   unloaded cables. It shows analytic free fall, then computes shell deformation,
   rigid-frame inertia and four compliant normal foot contacts. The camera closes
   in at contact. The ground-force and physical-time readings follow accepted states.
6. Use **Pause / resume** to freeze the accepted geometry and calculation record.
   **Show peak bending** selects the recorded frame with its matching force and
   time. **Replay recorded impact** replays calculated frames without solving again.
7. **Save calculation** writes a timestamped `exact_joint/results/` folder with
   states, actual loads, material settings, impact trace and any rejected candidate.
   It does not save a movie or declare survival. Use a screen recorder for video.

Native timeline Play resumes the active calculation or repeats the last selected
workshop action. Pause/Stop freezes it. Return to reference FEM restores the
original stage/model; it does not overwrite the v8 asset.

## Tweak meanings

Commit field edits with Enter. Stiffness/gap edits need **Rebuild stiffness**;
force edits need Apply, and drop inputs need **Drop / recompute**.

| Input | Effect / limitation |
|---|---|
| Panel / crease bending ratio | Laminate flexural rigidity divided by effective PET-strip rigidity. Larger = easier strip bending relative to panels. Does **not** reduce PET membrane stretching stiffness. Default 100 is an assumption, not a measurement. |
| Crease twist coupling | Penalizes differences between adjacent sections' fold-angle changes. Smaller lets the crease warp more independently. It is not a torsional motor command. |
| Panel bending scale | Multiplies laminate flexural rigidity; also changes strip rigidity at a fixed ratio. Does not change membrane modulus or the printed thickness. Values other than 1 are effective-model experiments. |
| Membrane stiffness scale | Multiplies in-plane stretching/shear stiffness. Lowering it is not equivalent to making a crease easier to rotate; it changes the material model and needs calibration. |
| PET exposed gap (mm) | Actual total PET-only width between adjacent PLA panels. Rebuild changes PLA coverage, strip compliance and shell mass while preserving the neutral midsurface. Too-wide gaps for a source facet are rejected. |
| Membrane strain guard (%) | Model-use limit, not yield strain or fracture. A rejected state is not applied. Raising this does not make a result validated or a design safe. |
| Upper/lower-side mass (g) | Total assembly masses including the assigned half of the shell. Defaults 30 g / 20 g are assumptions, with no payload. Remaining mass is represented by rigid-link inertia. |
| Contact stiffness (N/m) | Total four-corner normal penalty stiffness. A softer support absorbs more travel; a stiffer support raises/delays different impact components and needs smaller timesteps. It is not the joint stiffness. |
| Contact damping (N s/m) | Total normal closing-contact damping. No tensile ground force. No tangential friction in this experiment. |
| Drop timestep (us) | Physical integration step, not playback speed. Default GUI 25 us. Compare with 12.5 us; convergence must be checked again after changing masses, stiffness, geometry or contact. |
| Drop duration (ms) | Calculated time **after first contact**. Default 15 ms. Completion means the window ended, not that the leg survived or all later motion was resolved. |
| Roof test force (N) | Concentrated downward force at the top roof's free center with its perimeter fixed. Tests that the roof interior can deform. A point-load peak is mesh-sensitive. |
| Bend/twist/compression targets | Used only by the separately labelled **displacement studies**. These prescribe one coordinate and report reaction loads. They are not cable-force predictions or proof of actuator capacity. |

Colors show the largest absolute principal **Green membrane strain**, not stress,
plastic damage or a safety factor. Roof warp is distance out of its moving frame
plane. Reported frame rotations use a relative rotation vector; components are
not independent Euler angles at large rotations. Numeric values are not magnified.

## Why the full motions are still not demonstrated

A nonzero force is not a command to reach a motion limit. Its response depends on
the load direction, lever arm, crease law, material stiffness and constraints.
Compression can remain nearly symmetric; an upright frictionless normal impact
has no applied ground torque about world Z. Twist must not be added cosmetically
to that test. The off-center lower assembly can excite Y bending.

The supplied videos show real folding but do not provide calibrated dimensions,
cable forces, camera geometry or stress. Their physical module width is still
needed: shrinking only the width to 30 mm while retaining 0.4 mm PLA / 80 um PET
does not preserve the larger prototype's compliance.

Initial force studies of this explicit-strip model still predict small motions.
Even changing the panel/crease ratio from 100 to 10,000 did not establish full
travel; larger loads encountered the membrane-strain guard. These are model
results, **not** evidence that the physical prototype cannot move as shown.

The two-level discretization check also **fails** the 5% response gate: at
0.25 N/strand, the finer boundary/centroid mesh increases compression about 28%,
Y bending about 40%, and twist about 12%. This is a measured numerical limitation,
not just missing material data. Further uniform interior refinement and bending/
crease benchmark work are needed; the current boundary-fan subdivisions alone
are not a complete uniform mesh-convergence study.

Open validation gates:

- Measured force-versus-motion curves for compression, both bend axes and twist;
  exposed PET width, manufactured module dimensions, pre-crease rest state and
  hysteresis. A measured load–motion pair is a useful first calibration check.
- Spatial mesh convergence, bending/crease benchmark validation and large-fold
  path/buckling continuation. Original neutral geometry alone does not validate
  every deformed configuration.
- PET/PLA anisotropy, plasticity, rate effects, adhesive slip/delamination and
  failure data. Effective hinges do not resolve through-thickness PET stress.
- Finite-thickness self-contact, coplanar overlap, friction and continuous
  collision detection. The current midsurface guard is only a partial check.
- Measured leg mass/inertia and landing-surface contact/friction. The current
  backward-Euler integrator adds numerical damping, so rebound is timestep-sensitive.

Until these gates pass, do not use this experiment as a strength rating, a Sony
test reproduction, or an ML-calibrated joint model. The saved results deliberately
contain `survives: null`.

## Reproduce numerical studies

Run in a Python environment with the installed Isaac NumPy/SciPy/Torch libraries:

```text
python -m unittest discover -s exact_joint -t . -p "test_*.py"
python -m exact_joint.run_shell_study --mode cables
python -m exact_joint.run_shell_study --mode cables --ratio 10000
python -m exact_joint.run_shell_study --mode drop --steps-us 100 50 25 12.5
python -m exact_joint.run_shell_study --mode cables --subdivision 2 --forces .25
```

Each study saves source/code hashes, assumptions, SI results and acceptance status.
Run `validate_shell_live.py` through the project's Isaac remote sender for actual
control checks on an already opened nonlinear workshop. This temporarily operates
that workshop, records an evaluation and returns it to neutral.

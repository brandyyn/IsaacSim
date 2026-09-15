# Frame-to-plate PET hinges

Both square roof plates now have a continuous PET-only border between the rigid
perimeter and the inset PLA/PET plate. The border is a finite-stiffness flexure,
not a frictionless pin joint and not a disconnected surface. All four sides of
each roof are connected. Side-panel folds keep their existing exposed PET strips.

The original JSON's 28 vertices, 50 panel regions and 76 edges are unchanged.
Adding the roof border changes the **PLA coverage/manufacturing layout**, not the
original neutral shape. PLA remains 0.4 mm; PET remains 80 um. Only the square
perimeter nodes are constrained to rigid frames; the plate and hinge nodes are
free. Both cable actuation and impact use this same mesh and material law.

## See and adjust it in Isaac

Launch the nonlinear workshop (`launch.ps1 -RuntimeRoot <built-release>
-Nonlinear`). Its new default includes 0.2 mm roof hinges. For an already-running
workshop, reload the code using `reload_shell_live.py`, then choose **Frame hinge
design**. Reload alone deliberately preserves the existing model/settings.

1. Click **Frame hinge design** to rebuild with 0.2 mm roof hinges, the actual
   thicknesses and unscaled assumed moduli. It returns the model to neutral.
2. Click **Knee close-up**. Gold outlines are rigid frames. Green outlines mark
   the inner PET/PLA boundary; the narrow region between green and gold is PET.
   The outlines follow the actual computed mesh. Their drawn thickness is only
   a visual aid, not a solid FEM or collision thickness.
3. Set **Roof test force (N)** to 1 and click **Roof flex test** to load the upper
   plate centre. Watch **Upper/lower roof warp** and **Roof PET local hinge
   rotation**. The latter is the largest local mesh-dihedral change, not a
   single global revolute angle or a measured joint limit.
4. Use the **Compression**, **Bend**, **Twist**, or **Cable demo** controls for
   cable loads. These solve immediately; Play is not required. **Drop 70 mm /
   recompute** includes the new flexures in the dynamic model.
5. Change **PET hinge (mm)** beside **Frame hinge design**, then click
   **Apply hinge width** (or **Rebuild stiffness**). The ACTIVE readout must show the new width before the
   next calculation. Rebuild invalidates old states because the mesh changed.

| Control/value | Meaning |
|---|---|
| Roof hinge 0 mm / Material reference | Legacy roof: laminate reaches the frame; no PET-only roof border |
| Roof hinge 0.2 mm / Frame hinge design | Starting design; full exposed width inside each roof |
| Roof hinge 0.4 mm | Wider test strip; more PLA setback, generally lower rotational restraint |
| PET exposed gap (mm) | Separate side-panel fold gap; does not set the roof hinge width |
| Thickness-derived PET bending | Uses the assumed PET modulus and actual thickness; ignores the panel/crease ratio |
| Apply hinge width / Rebuild stiffness | Applies geometry/material inputs and resets to neutral |
| Magnified response | Right-side displacement plot only; left leg always remains physical 1x |

The width input accepts 0-2 mm with an additional geometry guard. This is a
software study range, **not a fabrication recommendation** or guaranteed safe
range. A 0.2 mm strip is narrow relative to the 80 um film thickness; the shell
approximation and real creased-film response still need experimental validation.

## Measured numerical effect, 2026-09-15

All 12 separate static cases converged below the configured residual/strain
guards with no detected intersections. Geometry: 30 mm wide, 26.4 mm high;
boundary subdivision 1 / interior refinement 1; midsurface IPC enabled.
Assumed moduli: PLA 2.2 GPa, PET 3.5 GPa, Poisson ratios 0.35.

| Roof PET width | Compression at 3 N/strand | Bend magnitude at 3 N/strand | Twist magnitude at 3 N/strand | Roof-centre deflection at 1 N |
|---|---:|---:|---:|---:|
| 0 mm | 0.503% | 0.1411 deg | 0.09758 deg | 0.2471 mm |
| 0.2 mm | 0.527% | 0.1427 deg | 0.09770 deg | 0.2685 mm |
| 0.4 mm | 0.534% | 0.1432 deg | 0.09773 deg | 0.2753 mm |

Each column is a different load case, not simultaneous motion. The 0.2 mm design
has 1,890 nodes / 3,776 triangles, with 47.68 mm2 total roof PET-border area.
Eight new hinge tests verify material areas, source preservation at 10/30/40 mm,
free plate nodes, rigid-frame constraints, rigid-body energy invariance, physical
PET stiffness, sparse assembly, cable signs/unloading, and a short impact step.
The complete numerical/controller regression suite passes 75 tests.
Nine additional live Kit checks passed through the actual UI callback methods:
legacy/hinged presets, width rebuild, the eight-step cable ramp, a five-step
70 mm impact check covering only the first 1 ms after contact, roof loading,
correct force-arrow placement, mesh/interface coordinates at physical 1x, and
whole-form scrolling so the hinge inputs remain accessible in a short dock.
Native mouse clicks could not be checked because the desktop helper was
unavailable. The workshop remains open with the 1 N roof-flex result displayed.

This fixes the missing roof flexure connection, but **does not reproduce full
compression, bending or twisting**. Roof flexibility improves; the global
mechanism remains stiff. Mesh convergence, measured crease/plasticity response,
actual seams and guide routing, finite-thickness contact, adhesive failure and
strength remain unresolved. Strain colour is membrane strain, not a certified
failure map. No drop-survival verdict or ML baseline promotion is made.

Reproduce the comparison with `python -m exact_joint.probe_frame_hinges`
(`--output-dir`, `--widths-mm`, `--tension-n`, `--roof-force-n` are configurable).
Individual cable/drop studies accept `--frame-hinge-mm` in
`python -m exact_joint.run_shell_study`.

Case: `fea/exact_joint_frame_hinges_v1/manifest.json`.
Evaluation: `ml/runs/20260915-exact-knee-frame-hinges-v1/run_manifest.json`.

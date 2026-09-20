# Photo-based knee and smooth FEM display

This version corrects the end topology and display stepping. It **does not yet
reproduce the physical joint's full compression, bending or twisting**, and is
not a validated strength or drop-survival model.

## What changed

The six supplied photos show open square ends. The source JSON has two end-cap
quadrilaterals. **Photo joint** excludes those two regions from mechanics and
rendering, but keeps all 28 original vertices, the 48 side triangles and the
76-edge source registry. The source JSON itself is unchanged. The legacy capped
model remains available for comparison; neither changes the shared v8 baseline.

PLA remains **0.4 mm**, printed on continuous **80 um PET**. The PET strips are
explicit finite-width material regions, not cuts. Only the square perimeters
move rigidly. Side-panel interiors and strip nodes deform. The open edges have
no imposed clamped slope; PET/PLA bending is calculated inside the adjacent web.
The default side gap and frame-to-side-panel strip width are each **0.2 mm**,
both unmeasured assumptions. No stiffness scale reduction is used in this preset.

White mounting bands are schematic visual geometry, 2 mm tall and 0.4 mm thick;
they are not extra FEM caps. Actual lug/hole dimensions and cable-guide geometry
cannot be reconstructed exactly from these perspective photos. Current cables
still connect frame corners; guide contact and routing over panel lugs are not
resolved. Thus this is a photo-informed adaptation, **not an exact CAD replica**.

## Demonstrate it

Launch `exact_joint/launch.ps1 -RuntimeRoot <built-release> -Nonlinear`.
In an existing workshop, load `reload_shell_live.py`, then click **Photo joint**.
Reload alone preserves existing accepted results and settings.

1. Select **Photo joint**, then **Knee close-up**. The ends are open. White is
   PLA/PET laminate; blue-green is exposed PET; green lines mark frame-strip
   interfaces; black/red lines show cable routing. Loaded strands turn gold.
2. Set **Cable tension (N / active strand)**, initially 3 N for this numerical
   demonstration. Click **Compression**, **Bend X/Y**, **Twist**, or **Cable demo**.
   Each solves a force ramp; no target angle is imposed. Play is not required.
3. Leave **Smooth solved transitions** checked. Display easing runs while the
   next solve computes. The banner explicitly marks these intermediate poses
   as **display-only**. They are neither equilibria nor dynamic FEM timesteps;
   strain colours are hidden during them. They are not collision-certified.
4. Check **Strain colours (accepted frames)** to inspect calculated membrane
   strain once an accepted frame is held. This is not a stress/failure contour.
   Numbers during a transition refer to its accepted endpoint, not its
   interpolated geometry. Turn smoothing off for strictly solved-frame display.
5. Use **Smooth replay solved states** after a run for an eased replay. It does
   not recalculate mechanics. The final accepted state is restored afterward.
   **Pause / resume** holds both calculation application and display movement.
   **Neutral / cancel** cancels pending display and results and restores neutral.
6. **Drop 70 mm / recompute** uses the same open shell, unloaded cables and the
   assumed 30 g upper / 20 g lower assembly. The solver is slower than real time.
   The free-fall preview and display easing are not the impact integration clock.
   Impact time in the results is physical solver time.
7. **Save calculation** saves accepted states, forces, assumptions and traces.
   It never saves interpolated samples as solved FEM results.

For tiny responses, **Magnified response** shows a separate displacement-vector
plot. It is not a valid folded shape and must not be presented as full travel.
The main model is never magnified.

## Tweaks and their meaning

| Control | Effect |
|---|---|
| PET hinge (mm) / Apply hinge width | Full PET setback from a rigid frame into adjacent side panels; rebuilds mesh and clears old results |
| PET exposed gap (mm) | Total exposed width shared by two adjacent side-panel edges; changes PLA coverage |
| Thickness-derived PET bending | Computes strip bending from assumed modulus and actual film thickness; panel/crease ratio is ignored |
| Fold compliance (100 = PET reference) | Scales the PET fold-line bending law as `D_pet * 100 / control` in the physical mode; does not change PET membrane stiffness. Not measured material data. |
| Crease twist coupling | Extra penalty for different fold angles along a crease; photo preset sets this to zero, not zero PET torsional resistance |
| Panel bending / membrane scales | Experimental multipliers; keep at 1 for this material reference |
| PLA/PET thicknesses and moduli | Change constitutive response; thicknesses are supplied values, moduli remain assumed |
| Winch pull / stiffness / force cap | Cable rest-length take-up with tension-only elastic response and saturation; not a commanded joint angle |
| Lower frame force X/Y/Z | Additional force, included by **Apply cable + force**, not the individual cable-only buttons |
| Boundary subdivision / interior refinement | Mesh resolution; finer runs cost more and require a convergence study |
| Midsurface self-contact + CCD | Checks shell midsurface contact paths; does not model finite laminate thickness or cable contact |
| Membrane strain guard | Numerical acceptance limit, not a measured yield or failure strain |
| Drop timestep / duration | Physical integration resolution/window; repeat at smaller timestep before interpreting peaks |
| Upper/lower mass and contact stiffness/damping | Assumed impact inputs; measured robot and landing-pad values are required for prediction |
| Capped comparison | Earlier JSON roof experiment; enables **Roof flex test** |
| Relief experiment | A separate cut/softened model, NOT the supplied continuous-film joint |

The form scrolls if the dock is short. Material/mesh changes require **Rebuild
stiffness** or **Apply hinge width**; check the ACTIVE readout before applying loads.

## Numerical evidence and remaining issue

At 30 mm width / 26.4 mm height, assumed PLA modulus 2.2 GPa and PET 3.5 GPa,
thickness-derived bending, original side geometry, no cuts or stiffness scaling:

| Side PET gap | Compression at 10 N/strand | Y bend at 10 N/strand | CW twist at 10 N/strand |
|---|---:|---:|---:|
| 0.2 mm | 2.196% | 0.509 deg | 0.360 deg |
| 0.8 mm comparison | 4.468% | 0.746 deg | 0.502 deg |

Each column is a separate load case. These are exploratory calculated values,
not measured response or recommended operating loads. Widening PET helps but
does not explain the observed full travel. The narrower preset remains selected.

An independent linearized audit of the original 76 edge lengths finds rank
66 for 66 free coordinates with the upper frame fixed. That idealized
**rigid-facet / zero-width hinge** model has no infinitesimal folding mechanism
at neutral. This is not a proof that the real PET joint cannot fold, or that the
finite-width shell is locked identically. It isolates why adding ideal hinges
alone is insufficient: deformation, junction details, rest folds and/or a more
accurate manufacturing geometry matter. See `fold_compatibility.py`.

The suite covers 85 implementation/controller tests. Live checks exercise
open/capped controls, hinge-width rebuild, compression/bend/twist callbacks,
smooth rendering, pause, replay, cancellation and a five-step impact window.
The live impact check covers only **the first 1 ms after contact**, not the full
peak or rebound. No survival verdict is issued. Native mouse automation was
unavailable; actual callback methods and rendered coordinates were checked in Kit.

Remaining gates: updated flat manufacturing pattern and joint dimensions;
measured force-displacement and crease moment-angle behavior; finite-thickness
contact, wrinkling/creasing/plasticity and bonds; spatial/time convergence.
The custom triangular membrane/discrete-hinge solver runs alongside Isaac; it
is not native solid FEM inside PhysX and is not an Ansys calibration.
The [discrete-shell formulation](https://www.cs.columbia.edu/cg/pdfs/10_ds.pdf)
and [IPC Toolkit](https://github.com/ipc-sim/ipc-toolkit) describe the underlying
method families; neither validates this specific joint implementation.

Reproduce force checks with `python -m exact_joint.run_shell_study --open-ends
--frame-hinge-mm .2 --sparse --physical-strip --self-contact --interior-refinement 1
--crease-twist-ratio 0 --forces 3 10` (one shell command). Add `--gap-mm .8` for
the wider-strip comparison. Use `validate_photo_joint_live.py` inside Kit for UI QA.

Case: `fea/exact_joint_photo_open_v1/manifest.json`.
Evaluation: `ml/runs/20260916-exact-knee-photo-open-v1/run_manifest.json`.
No ML training or baseline promotion.

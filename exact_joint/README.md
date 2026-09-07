# Exact JSON joint: cable-driven live FEM workshop

The user's **original joint is now used at the knee only**, between rigid upper and
lower leg sections. Hip and ankle copies were removed at the user's request. The rejected
cylindrical/Kresling-inspired sleeve is not used. This is a working **small-deformation,
quasistatic structural prototype**, not a validated large-folding leg or an ML model.

## Start and operate

Run `exact_joint/launch.ps1 -RuntimeRoot <built-Isaac-release>` in PowerShell. The
release directory contains `kit/kit.exe` and `apps/isaacsim.exp.full.kit`. The launcher
sets `PANEL_CREASE_PROJECT_ROOT` automatically. It needs the built Isaac runtime's
NumPy and SciPy. With Isaac already open and its Python server enabled, send
`exact_joint/open_live.py` using `skills/isaac-sim-remote/scripts/isaacsim_send.py`.

Inside **Exact knee - cable FEM**:

1. **Compare FEM** shows the actual knee on the left and a separate magnified
   displacement plot on the right. The right plot is diagnostic, not another leg joint.
   Use **Exact knee close-up** for the original shape, or **Whole leg** for the assembly.
2. Select a cable pattern and tension for the Knee, then click **Apply**.
   Tension is **newtons per active cable**, not a servo position. Start at 0.1 N;
   the default input is 0.1 N. These are experimental simulation inputs, not hardware recommendations.
   Apply first validates the full target, then ramps the cable load over about 2.4 s,
   recomputing equilibrium and stress at intermediate steps. Changing families first
   unloads the old cables, then loads the new ones. **APPLIED** confirms the completed target; **NOT APPLIED** explains
   a rejected load. **LAST ACCEPTED** identifies the result still displayed after rejection.
3. **Cable demo** cycles compression, positive/negative X/Y bending and both twist
   directions (eight seconds per mode, 0-0.25 N per active cable). Gold cable highlights,
   direction chevrons and twelve live tension bars identify the pulling family. Moving
   chevrons indicate opposing force directions, **not cable speed or simulated spools**.
4. **Stress / material** switches between the actual laminate and FEM stress colors.
   The diagnostic plot always shows stress. **Stress max MPa / Update display** changes
   the color scale; the viewport legend states its current maximum. Red is not failure.
5. **Play / replay load** replays the selected input from zero, or resumes a paused
   ramp/demo. The native Isaac timeline **Play** does the same; **Pause/Stop** freezes
   the custom FEM. Panel **Pause** freezes both FEM and glyphs; **Neutral** removes loads.
6. Change gap, PET/PLA modulus or refinement and **Rebuild** at zero load. Save each
   comparison with **Save FEM results and inputs**. Results are immutable timestamped
   directories under `exact_joint/results/`.

The actual leg remains **true scale (1x)**. The diagnostic view defaults to explicitly
labelled **500x displacement**: `x_plot = x_reference + gain * u_FEM`. **Display gain /
Update display** accepts 1-1000x without changing a force, stiffness, stress or solver
result. This standard deformation plot may exaggerate/distort small displacements;
it is not a physical large-fold prediction. Grey edges mark its neutral reference.
Record the full application/viewport overlay to retain the gain and true-response
legend; a bare USD render does not include the UI legend. Do not present that isolated
magnified mesh as a physically simulated large fold.

If Apply appears to do nothing, check the message immediately beneath it. For
example, **40 N per cable exceeds this model's small-strain guard** and is rejected;
it does not produce a 40 N result. Return to 0.1 N and Apply. At the default mesh,
compression at 0.1/0.2 N per active cable is approximately 2.52/5.04 micrometres,
so read the numerical outputs and stress field instead of expecting a large fold.
The material/mesh fields require **Rebuild**; **Apply** changes cable loading only.
Saved records distinguish accepted `commands` from `requested_commands` and record
the rejection error. Never interpret an old displayed result as an accepted new load.

While the workshop is running, reopening a USD now pauses with an explicit
disconnection message instead of silently killing the controller. **Reconnect opened
knee** binds a matching original-knee USD to the **current workshop FEM settings**;
it does not infer material properties from a legacy USD. Source identity, mesh topology,
rigid-roof size, units and available saved configuration are checked before attachment.
New scenes embed FEM configuration; legacy scenes still require the matching settings.
It retains the scene, camera, lights and rigid leg objects, and resumes updating the
controller-owned knee meshes after attachment. An unrelated scene is rejected.

`refresh_ui_live.py` updates trusted UI/render code through the Python server without
replacing the current stage; it can attach a matching reopened scene. The reproducible
button-click test is `validate_apply_live.py` (run inside Kit with `isaacsim.test.utils`
enabled). It exercises accepted/rejected loads and recovery through the real Apply button.
`validate_visible_live.py` additionally tests load ramps, all seven cable families,
1x versus magnified coordinates, live demo, panel and native Play/Pause, and reconnect.
Run it on a fresh generated workshop: it exports/reopens its own timestamped test USD.

## Practical controls and demonstration guide

### What each visible module represents

| Part | Meaning / what it does |
|---|---|
| Left knee and attached leg | The original JSON knee at true scale. The thigh/mount support the fixed top plate; the solved bottom plate carries rigid shank/foot display geometry. No additional origami hip or ankle. |
| Two square roof plates | Rigid boundary constraints; they do not bend. The bottom plate can translate/rotate according to cable/FEM equilibrium. |
| PLA triangular panels | The original 48 side facets with 0.4 mm PLA, represented by elastic finite elements rather than perfectly rigid panels. |
| PET film / exposed borders | The 80 um backing film and PET-only gaps between PLA facets. These provide the modeled compliant regions. |
| White crease lines | The original 76-edge registry. They show panel boundaries; they are not separate actuators or an independently calibrated hinge model. |
| Black/red routing | Black top X is the guide/return reference. Axial and red diagonal spans between plates are the modeled ideal pull-only actuators; crossings are not welded. |
| Gold cables and chevrons | A currently loaded cable family and opposing pull directions. Chevron animation is a visual cue, not motor speed or material travelling along a cable. |
| Right-hand FEM plot | A separate diagnostic copy, normally at 500x displacement, with unscaled stress colors and grey neutral-reference edges. Not a second physical knee. |
| Stage / Property / Content panels | General Isaac scene tree, selected-prim attributes and asset browser. Editing these is not the same as changing this custom FEM's inputs. Use the workshop controls below. |

### Every workshop control

Enter a value, press Enter to commit the field, then use the appropriate button.
Scroll **inside Exact knee - cable FEM** to reveal the display and material fields.

| Control | What changes | How to use / what to expect |
|---|---|---|
| Cable pattern dropdown | Selects the tension family for the next Apply | Does not immediately change the current solve. Compression loads A0-A3; Bend X+ loads A0/A1, X- A2/A3, Y+ A1/A2, Y- A0/A3; Twist CW/CCW loads the corresponding four diagonal cables. |
| N / cable | Force in each active strand, not total force, displacement or spool rotation | Start at 0.10 N, then compare 0.20 N. Compression has four axial strands, so these correspond approximately to 0.40/0.80 N total axial pull near neutral. Diagonal resultants require vector projection. |
| Apply | Full-target validity check followed by a roughly 2.4 s load ramp | Watch LAST ACCEPTED, compression/angles and stresses change. Identical already-applied inputs need not produce a new deformation; use Play / replay load to replay from zero. |
| Play / replay load | Resume a paused ramp/demo, or replay the selected manual load from zero | A convenience visualization of equilibrium states, not a calibrated loading rate. |
| Native timeline Play/Pause/Stop | Workshop replay/resume/freeze through its Python controller | Does not convert this model to native PhysX deformable dynamics. If the timeline is already stopped, use the workshop Pause to freeze a demo started independently. |
| Cable demo | Automatic compression, X+/X-/Y+/Y- bending, CW and CCW twist | Approximately 8 s per mode, 56 s per full cycle; 0-0.25 N per active cable. It ignores the manual input amplitude until Apply is pressed. |
| Pause | Freeze load ramp, FEM output and chevrons | Useful for explaining a frame or saving a steady result. It does not remove cable tension. |
| Neutral | Remove all tensions and return to zero-load equilibrium | Stops the demo. The dropdown may retain its prior label; zero force is what makes the state neutral. |
| Whole leg | Camera only | Shows assembly context. The diagnostic may be less prominent at this zoom. |
| Exact knee close-up | Camera only | Compare the original panel/roof shape at true scale. |
| Compare FEM | Camera/diagnostic visibility only | Frames true-scale knee and magnified plot together. |
| Stress / material | Actual knee's shading only | Switch PLA/PET material appearance versus stress colors. The right diagnostic always shows stress. No force/stiffness changes. |
| Display gain + Update display | Right-hand displacement magnification only, 1-1000x | Compare 500 and 1000: apparent displacement doubles, but all physical results stay the same. This field is not a bending-angle limit. |
| Stress max MPa + Update display | Color-map range only; must be positive | Compare 5 and 20 MPa: the same stress gets a different color. Peak stress numbers remain unchanged. Red means saturation of this display scale, not material failure. |
| PET exposed gap (mm) + Rebuild | PLA inset, exposed PET border and associated discretization | Baseline 0.20 mm total gap, about 0.10 mm setback per adjacent panel edge. Try 0.10/0.20/0.40 as numerical studies, not a manufacturing recommendation. See the warning below. |
| PET E (GPa) + Rebuild | Film elastic stiffness, not its thickness or strength | Baseline 3.5 GPa. Lower E generally increases displacement under the same force. E remains an assumed isotropic constant until measured. |
| PLA E (GPa) + Rebuild | Panel elastic stiffness | Baseline 2.2 GPa. Useful to distinguish film-dominated flexibility from panel deformation. Changing E alone does not identify a new printable material. |
| Refinement 1-4 + Rebuild | Numerical mesh density, not physical joint size | Use whole numbers. Baseline 1 has 1,152 elements/2,616 nodes; 2 has 4,608 elements/9,552 nodes. Larger values cost more memory and rebuild time and are for convergence studies. |
| Rebuild exact joint FEM | Reassemble mesh/material matrices and create a new neutral workshop scene | Save scene edits and results first. Current camera/display choices may reset. Then re-enter the same load and Apply for a fair comparison. Invalid settings retain the old model. |
| Reconnect opened knee | Attach a matching saved USD to the running workshop | Uses the current FEM settings; does not recover unknown material constants from an old USD. Rejects incompatible geometry/configuration. |
| Save FEM results and inputs | Timestamped JSON snapshot | Saves configuration, accepted/requested loads, results and display settings under `exact_joint/results/`. It is not a movie, full nodal stress export, or saved USD. Use File > Save As separately for scene edits. |

There is currently **no GUI slider for joint width or laminate thickness**. The
30 mm width, 0.4 mm PLA and 80 um PET are active configuration values. Width
(10-40 mm), thickness and Poisson ratios can be changed in `JointConfig` through
a controlled code/config rebuild, but require new validation and a matching case
record. Scaling the USD with a gizmo will not correctly rescale the FEM.

The workshop accepts cable tensions, not arbitrary external loads. Dragging the
mesh, adding a generic PhysX force, or changing a USD physics material does not feed
that force/property into this custom FEM. Payload, gravity and ground reaction are
not included. Multiple motion components can arise from one family, but the GUI
does not currently expose independently adjustable arbitrary 12-cable mixtures.

### What each output means

| Output | Interpretation |
|---|---|
| A0-A3 / CW0-CW3 / CCW0-CCW3 | Actual solved per-strand tensions. Bars use 0.25 N as 100%; that is a display range, not cable rated capacity. Numeric labels remain authoritative if a valid load exceeds that bar range. |
| RAMPING TO / LAST ACCEPTED | Requested endpoint versus most recently solved intermediate load. During a ramp, these need not match yet. During the demo, the dropdown may differ from the active family. |
| APPLIED / NOT APPLIED | New target accepted versus rejected. A rejection preserves the last accepted geometry/results. Never describe the old frame as the rejected force case. |
| Bend X/Y, twist (degrees) | Solved signed bottom-cap orientation components. Mode +/- names label cable families; read the result rather than assuming the same Euler-angle sign. |
| Compression (um) | True reduction in plate separation. 1 um = 0.001 mm. This value is never multiplied by display gain. |
| PET / PLA peak (MPa) | Maximum modeled von Mises stress for each material, derived from element integration-point stresses. These peaks are mesh-sensitive and are not strength/fatigue allowables. |
| Max principal strain (%) | Largest absolute principal strain used for the small-deformation check. The 1% strain/2 degree cap-rotation guards are model-use limits, not material failure criteria. |
| Update count / update ms | Number of accepted FEM updates and solve/update timing. Not policy-training steps, motor frequency, physical timestep accuracy or viewport FPS. |

### Measured tweak examples (unconverged, simulation only)

All rows use Compression at **0.10 N per active cable**, unchanged 0.4 mm PLA /
80 um PET, 30 mm source width, and baseline values except the named edit. Each
configuration was rebuilt using the real UI button before Apply.

| One experiment | Compression (um) | What it demonstrates |
|---|---:|---|
| Baseline: PET E 3.5 GPa, PLA E 2.2 GPa, gap 0.20 mm, refinement 1 | 2.521 | Reference numerical state |
| PET E 1.75 GPa | 4.402 | Film stiffness strongly influences compliance |
| PLA E 1.1 GPa | 2.817 | Panels are elastic too, but this response is less sensitive to this edit |
| PET E 7.0 GPa and PLA E 4.4 GPa | 1.261 | Doubling both moduli approximately halves displacement under force control; stress need not halve |
| PET gap 0.10 mm | 2.569 | Gap changes both physical discretization and element layout |
| PET gap 0.40 mm | 1.673 | Counterintuitive stiffening on this unconverged formulation; not evidence that a wider physical PET hinge is stiffer |
| Refinement 2 | 4.984 | Nearly doubles the response without changing the design: mesh convergence is still inadequate |
| Original baseline restored | 2.521 | Repeatable recovery of the original configuration |

**Do not use the gap table to choose the best physical crease.** Mesh/element
effects are still entangled with the geometry study. The non-monotonic gap response
and existing failed convergence require formulation/convergence work before an
optimization or engineering claim. These tests confirm that controls affect the
calculation, not that the resulting predictions match hardware.

### A repeatable 3-minute demonstration

1. **Neutral > Whole leg:** explain that there is one original origami knee between
   rigid leg sections. Then **Exact knee close-up** and material shading show the
   triangular facets, roof plates and PLA/PET construction.
2. **Compare FEM:** set gain **500**, stress max **5 MPa**, and **Update display**.
   Say: "Left is true scale; right is a magnified displacement plot."
3. Choose **Compression, 0.10 N/cable > Apply**. Watch the ramp and read about
   **2.52 um** true compression. Change to **0.20 N > Apply** and read about
   **5.04 um**. There are four active axial cables; this is not 0.20 N total.
4. Choose **Bend X+**, then **Bend X-**, each at **0.10 N > Apply**. Point out the
   switched axial pair, signed bend output and magnified shape. Repeat **Twist CW**
   and **Twist CCW** to show the red diagonal families and reversed twist.
5. **Cable demo** for one full roughly 56 s cycle. **Pause** on a useful phase;
   point at active cable numbers, stress peaks and the true-versus-display legend.
   Record the full app view so these qualifiers remain visible.
6. Optionally change gain **500 > 1000** or stress maximum **5 > 20** and press
   **Update display**: prove that a larger-looking deformation/redder picture does
   not change the physical numbers. Restore **500 / 5** afterward.
7. For a separate stiffness demonstration, **Neutral**, save results/scene edits,
   set PET E **7.0**, PLA E **4.4**, and **Rebuild**. Reapply **Compression 0.10 N**:
   expect about **1.26 um**. Restore PET **3.5**, PLA **2.2**, gap **0.20**, refinement
   **1**, Rebuild and repeat the same load. Do this outside the main 3-minute clip
   if rebuild time interrupts the presentation.
8. **Pause > Save FEM results and inputs** for a reproducible screenshot/result.
   Finish with **Cable demo** if you want the model to keep moving.

Suggested narration: "This is our exact original knee geometry with a trial cable
routing and live small-deformation quasistatic FEM. Loads produce calculated
compression, bending and twist. Displacement is magnified for inspection; the
material/crease model and mesh are not yet validated for large folding, strength
or walking."

The repeatable real-control checks are `validate_apply_live.py`,
`validate_visible_live.py`, and `validate_tweaks_live.py`. The last one backs up
test-owned stages, exercises the material/display controls, restores the baseline
and leaves Cable demo active. Do not run rebuild tests on an unsaved design scene.

## What is exact, and what is assumed

| Item | Implementation / provenance |
|---|---|
| Source | Byte-preserved `source_joint.json`, SHA256 `d578618a0f90342bd0093b4febe130dc0b07c18f1b223c4d6bf02c9f3d93ce35` |
| Topology | 28 original vertices, 48 triangular side panels, 2 quadrilateral roofs, 76 unique edges per module |
| Geometry identity | Original panel coordinates also match the repository's `input_improved.json`; no shape-fitting or substitute pattern |
| Mapping | Uniform source-coordinate scale 0.00012 m/unit, +90 degree X rotation, lower roof shifted to local Z=0 |
| Neutral size | 30 x 30 x 26.4 mm original surface envelope; outward 80 um PET / 0.4 mm PLA adds finite thickness |
| End plates | Both original square roofs are exactly rigid constraints; their physical flexure is excluded |
| Materials | User-confirmed PLA 0.4 mm on PET film 80 um |
| Elastic properties | Provisional isotropic PET E=3.5 GPa, PLA E=2.2 GPa, nu=0.35 each; not measured from this stock |
| Exposed PET gap | 0.2 mm total between PLA facets, initially 0.1 mm inset on each side; 0.1/0.2/0.4 mm candidates tested numerically |
| Creases / bonds | PET-only borders and perfect PLA/PET bonds; initial folded shape is taken as stress free |
| Videos | All three supplied clips inspected as motion/shape references, not force/strain measurements |

The original vertices and triangle surfaces are preserved as the laminate reference.
Additional FEM nodes and thickness are discretization/manufacturing assumptions, not
a replacement fold pattern. The smallest gap that can be meshed is **not** automatically
the smallest printable or durable crease. No optimum gap or scale is established.

## Cable layout and available motions

Each module has twelve ideal, pull-only cable spans between its two rigid plates:
four same-corner axial spans, four clockwise perimeter diagonals and four reverse
diagonals. The red crosses run **between plates**, not across the lower plate itself.
The black top-plane X remains the user's routing reference; its guide/spool return
path is not specified and does not independently actuate a rigid plate.

| Pattern | Nonzero cable tensions | Response tested |
|---|---|---|
| Compression | Four axial strands | Shortening between the plates |
| Bend X+ / X- | Opposing pairs of axial strands | Opposite bending responses |
| Bend Y+ / Y- | The other opposing pairs | Bending in the other plane |
| Twist CW / CCW | One or the other four diagonal families | Opposite twist with coupled compression |

Pattern names identify cable families, not a guaranteed positive Euler-angle sign.
The UI reports the signed result. Bending, twist, compression and small lateral
translations are coupled by the source geometry; they are not artificially separated.
Crossings have no weld or mechanical connection. Cable stretch, slack dynamics,
friction, guides, spools, routing contact and motor torque are not modeled. The expanded
12-span routing is an experimental actuator layout, not an as-built cable survey.

## What the live FEM actually computes

This is **custom finite-element analysis inside Isaac Sim's Python runtime**, not
native PhysX deformable dynamics, an Ansys replay, or a pre-baked animation.

The laminate uses quadratic ten-node tetrahedra (TET10), four Gauss integration points,
and material-specific 3D isotropic elasticity. The neutral interactive mesh has 2,616
nodes and 1,152 elements per module. The 80 um film is modeled at its physical thickness;
all exchanged quantities are SI. Second-order elements reduce the bending locking
seen with first-order thin solid elements, but do not remove the need for convergence.

Element matrices are assembled as `K_e = integral(B^T D B dV)`. The upper roof is fixed
in the module's local frame. Free interior coordinates are eliminated with a sparse
factorization: `u_f = -K_ff^-1 K_fb u_b`. This static condensation is algebraically
exact for the assembled linear-elastic continuum model. It is **not** a learned or
fitted stiffness curve. The lower roof has six rigid-body coordinates; exact rotation
matrices keep its shape rigid. The online solve balances condensed FEM reactions with
the twelve current cable forces, minimizing elastic energy plus `sum(T_i * L_i)`.

Changing tension therefore changes the solved cap pose. Changing E, gap or mesh changes
the assembled response. Nodal displacements are recovered after every solve; strains
and stresses follow from `epsilon = B u` and `sigma = D epsilon`. The displayed element
von Mises value is the maximum over its four Gauss points, not a native PhysX stress.

The single knee connects rigid unloaded thigh/shank sections and a rigid foot. Its
actuation system is self-equilibrated locally. This excludes gravity/payload transmission, inertia,
foot contact, walking and balance. It is not a whole-leg dynamic load case. No policy
has been trained, and the canonical v8/author v9 assets remain unchanged.

## Validation: passed implementation checks, failed convergence

The original seven tests pass: exact source identity; quadratic patch and rigid-motion invariance;
zero load; cable/FEM force equilibrium and rigid roofs; twist reversal; material/load
scaling; rejection of negative tension. The single-knee live validation asserts that
no hip or ankle origami module exists and exercises all seven cable patterns. Exact
timings, rendered roof-rigidity errors and source hashes are in the linked run report.
A 10.004 s single-knee probe completed **45 live FEM updates** with no error. Isolated
mode updates took **14.6-15.7 ms**; the renderer runs independently. The maximum
rendered roof-rigidity error was **4.2e-9 m**, consistent with float32 display rounding.

At **0.1 N per active cable**, with the 0.2 mm gap:

| Refinement | Elements/module | Compression pattern shortening (um) | Bend-X pattern X angle (deg) | CW pattern twist (deg) |
|---|---:|---:|---:|---:|
| 1 | 1,152 | 2.521 | -0.004972 | -0.003275 |
| 2 | 4,608 | 4.984 | -0.008937 | -0.005009 |
| 3 | 10,368 | 6.207 | -0.010905 | -0.006089 |

From refinement 2 to 3, these movement magnitudes change by about **24.6%, 22.0% and
21.6%**, measured relative to refinement 2. They fail the 5% convergence criterion.
Peak stress also changes with refinement. These are unconverged model outputs, **not
measured performance, capacity or engineering predictions**. Gap comparisons cannot
establish the best design until the mesh and crease formulation are reliable.

The UI rejects a result above 1% maximum principal strain or 2 degrees cap rotation and
keeps the last valid geometry. Those are model-use guards, not material allowables.
Small cap rotation can still create large local crease strain. Increasing tension
until a dramatic fold appears would leave this model's scope; reproducing the larger
video motions needs nonlinear crease/buckling/contact analysis and calibration.

## Reproduce and next work

Run `open_live.py`, then `validate_live.py` through the Python server. The latter wraps
its async work in a function, checks all seven expected live modes, records source
hashes, compares three meshes and gap candidates, and saves its own immutable report.
Offline tests need Python with NumPy and SciPy: `python -m unittest exact_joint.test_exact_joint -v`.
After restarting Isaac, use the launcher to start the Python controller; opening a
USD alone still cannot start Python. For a saved scene, then open it and Reconnect.

The versioned case is `fea/exact_joint_cable_v1/manifest.json`. The implementation
commit, immutable validation report and case checksum are linked in
`ml/runs/20260906-exact-knee-cable-v1/run_manifest.json` (evaluation only; no training).
Source bytes are preserved across checkouts so the recorded hashes remain comparable.
The visible-control follow-up is recorded separately in
`ml/runs/20260906-exact-knee-visible-fem-v1/`; it changes no structural formulation,
material, small-strain guard or convergence claim.

Next, still on the knee alone: converge a crease-appropriate formulation, characterize PET crease rest angles
and hysteresis, measure cable force versus cap displacement/rotation, and verify guide
clearance and bond behavior. Then add nonlinear folding/contact and loaded leg validation.
Repeating the origami module at other joints is deferred, not part of the active scene.
Only after those non-ML checks pass should this become a calibrated ML environment.

## 70 mm drop: pre-impact preview, survival indeterminate (2026-09-07)

Click **70 mm drop preview (no impact FEM)** in the cable workshop. It uses the
original knee and whole rigid-link leg, upright and foot first. **Replay 70 mm**
starts an analytic gravity release at 0.05x playback (20 times slower than real
time). **Pause / resume fall** freezes/resumes it. The preview stops exactly at
first foot contact; **Return to cable FEM** removes the preview translation,
restores the stress diagnostic and returns to unloaded, paused cable FEM. Click
Cable demo or Apply to run the structural experiment again.

The user specified 70 mm and authorized example masses. We assume **50 g total**:
30 g upper-side and 20 g lower-side, including knee/plates, no payload. This is not
a measured or optimized mass distribution, and it is not used to fabricate a
dynamic mass matrix. The floor is an assumed rigid horizontal surface. Height is
measured from the lowest foot point to the floor top. Cables are unloaded.

The preview evaluates `distance = g*t^2/2`, `speed = g*t`, `energy = m*g*h`,
with `g = 9.81 m/s^2`. At contact: **119.46 ms**, **1.17192 m/s**, **0.034335 J**.
For 25/50/100 g, incident energies are 0.0171675/0.034335/0.06867 J; fall time and
speed are unchanged. These are incident whole-leg quantities, not peak force,
absorbed knee energy, a strength limit or a survival verdict.

**No impact FEM is running in this preview.** The assembly stays undeformed
during ideal uniform-gravity free fall, and no continuation after contact is
computed. A visually intact joint does not show that it survived. The existing
cable FEM has no inertia, contact, plasticity, rate-dependent failure or adhesive
separation model. A credible survival assessment needs those models and measured
material/crease/bond data, as well as mesh convergence and mass distribution.
Native timeline Play does not turn this preview into an impact calculation;
use the labelled preview controls.

**Save drop assumptions** exports a JSON report with assumptions, ballistics,
current preview time and explicit null impact-force/stress/survival fields. It
does not export a fracture prediction. `drop_test.py` holds the SI assumptions
and analytic calculation; `drop_preview.py` owns the visible replay. To explore
different assumed masses programmatically, create a `DropCase`; the GUI button
uses the documented default 70 mm / 50 g case.

### Defects and remaining validation gates

- Fixed: a fractional UI refinement such as 2.9 was silently truncated to 2.
  It is now rejected before Rebuild, preserving the scene and assembled FEM.
- Fixed: malformed or nonfinite initial six-DOF solver states are explicitly
  rejected. These add two numerical regression tests (nine FEM tests total).
- Fixed: maintenance refresh during the preview could cancel the FEM updater
  before its first frame and abort the refresh. Cancellation is now handled by
  the caller, and the active-preview refresh is covered by a regression check.
- **Still open:** PET outer-layer normals are regenerated when mesh refinement
  inserts vertices. PET reference volume increases **0.766%** from refinement
  1 to 4, while PLA volume stays constant. Thus the refinement sweep does not
  hold the entire laminate reference geometry fixed. Original JSON source
  vertices remain exact, but this generated-volume defect needs a new mesh
  revision and a fresh FEA case, not a silent edit of earlier results.
- **Still unconverged:** compression at 0.1 N per active cable rises from
  6.207 um (refinement 3) to 7.181 um (refinement 4): **15.68%**, failing 5%.
  Neither stresses nor gap optimization nor impact survival are validated.

Run `validate_drop_live.py` through the Python server for real-button checks:
fractional-refinement rejection, world-space 70 mm/zero contact clearance,
pause/resume, hard stop, hidden diagnostics, no FEM updates during the fall,
Save, restoration and working cable demo. It also runs nine FEM and four
ballistic tests. `audit_drop_mesh_live.py` reproduces the four-level mesh audit
without replacing the displayed model (the large assemblies can briefly stall
the viewport). The immutable case/run package is
`fea/exact_joint_drop_70mm_v1/manifest.json` and
`ml/runs/20260907-exact-knee-drop-readiness-v1/`.

## Technical references

- [FEBio Theory Manual: quadratic tetrahedral elements](https://help.febio.org/docs/FEBioTheory-4-7/TM47-Subsection-4.1.4.html): TET10 and four-point integration.
- [Autodesk: tetrahedral elements](https://help.autodesk.com/cloudhelp/2015/ENU/SimMech/files/GUID-EB10FAC1-1CCC-4EB4-A3AD-C7DE7F3B8CEC.htm): quadratic interpolation for bending with thin solid meshes.
- [SOLIDWORKS: large-displacement solution](https://help.solidworks.com/2022/English/SolidWorks/cworks/c_Large_Displacement_Solution.htm): limits of small-displacement stiffness assumptions.
- [Mylar A example material data](https://mylar.com/wp-content/uploads/2025/08/mylar_a-1.pdf) and [Polymaker Draft PLA example data](https://polymaker.com/wp-content/uploads/lana-downloads/Polymaker-Draft-PLA_TDS_EN_V5.4.pdf): starting modulus examples only; not the user's identified grades.

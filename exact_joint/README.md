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

1. Use **Exact knee close-up** to compare the shape, or **Whole leg** for the assembly.
2. Select a cable pattern and tension for the Knee, then click **Apply**.
   Tension is **newtons per active cable**, not a servo position. Start at 0.1 N;
   the default input is 0.1 N. These are experimental simulation inputs, not hardware recommendations.
   **APPLIED** beside the button confirms a completed solve; **NOT APPLIED** explains
   a rejected load. **LAST ACCEPTED** identifies the result still displayed after rejection.
3. **Cable demo** cycles compression, positive/negative X/Y bending and both twist
   directions. The single knee solves its cable-loaded FEM equilibrium.
4. **Stress / material** switches between the actual laminate and FEM stress colors.
   Red means 20 MPa or above on the present display scale, not yield or failure.
5. **Pause** freezes this custom solver; **Neutral** removes cable loads. The Isaac
   timeline Play button is not this quasistatic workshop's controller.
6. Change gap, PET/PLA modulus or refinement and **Rebuild** at zero load. Save each
   comparison with **Save FEM results and inputs**. Results are immutable timestamped
   directories under `exact_joint/results/`.

The small model-predicted movement is shown at **true scale**. There is no hidden
deformation amplification, enlarged motion envelope or softened material for video.

If Apply appears to do nothing, check the message immediately beneath it. For
example, **40 N per cable exceeds this model's small-strain guard** and is rejected;
it does not produce a 40 N result. Return to 0.1 N and Apply. At the default mesh,
compression at 0.1/0.2 N per active cable is approximately 2.52/5.04 micrometres,
so read the numerical outputs and stress field instead of expecting a large fold.
The material/mesh fields require **Rebuild**; **Apply** changes cable loading only.
Saved records distinguish accepted `commands` from `requested_commands` and record
the rejection error. Never interpret an old displayed result as an accepted new load.

`refresh_ui_live.py` can update trusted UI code through the Python server without
replacing an already connected scene. It preserves the FEM and camera; it is not
a loader for saved USD files or a repair for a disconnected stage. The reproducible
button-click test is `validate_apply_live.py` (run inside Kit with `isaacsim.test.utils`
enabled). It exercises accepted/rejected loads and recovery through the real Apply button.

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

Seven tests pass: exact source identity; quadratic patch and rigid-motion invariance;
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
Do not merely open an exported USD and expect the Python FEM controller to run.

The versioned case is `fea/exact_joint_cable_v1/manifest.json`. The implementation
commit, immutable validation report and case checksum are linked in
`ml/runs/20260906-exact-knee-cable-v1/run_manifest.json` (evaluation only; no training).
Source bytes are preserved across checkouts so the recorded hashes remain comparable.

Next, still on the knee alone: converge a crease-appropriate formulation, characterize PET crease rest angles
and hysteresis, measure cable force versus cap displacement/rotation, and verify guide
clearance and bond behavior. Then add nonlinear folding/contact and loaded leg validation.
Repeating the origami module at other joints is deferred, not part of the active scene.
Only after those non-ML checks pass should this become a calibrated ML environment.

## Technical references

- [FEBio Theory Manual: quadratic tetrahedral elements](https://help.febio.org/docs/FEBioTheory-4-7/TM47-Subsection-4.1.4.html): TET10 and four-point integration.
- [Autodesk: tetrahedral elements](https://help.autodesk.com/cloudhelp/2015/ENU/SimMech/files/GUID-EB10FAC1-1CCC-4EB4-A3AD-C7DE7F3B8CEC.htm): quadratic interpolation for bending with thin solid meshes.
- [SOLIDWORKS: large-displacement solution](https://help.solidworks.com/2022/English/SolidWorks/cworks/c_Large_Displacement_Solution.htm): limits of small-displacement stiffness assumptions.
- [Mylar A example material data](https://mylar.com/wp-content/uploads/2025/08/mylar_a-1.pdf) and [Polymaker Draft PLA example data](https://polymaker.com/wp-content/uploads/lana-downloads/Polymaker-Draft-PLA_TDS_EN_V5.4.pdf): starting modulus examples only; not the user's identified grades.

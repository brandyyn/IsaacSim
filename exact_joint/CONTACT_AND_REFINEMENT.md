# PLA/PET knee: refined shell and contact investigation

This is an **uncalibrated structural experiment**, not a validated full-travel
joint, a solid-stress FEM result, or a drop-survival calculation. The custom
CPU shell solver runs inside the Isaac workshop; PhysX is not calculating these
shell stresses. The v8 machine-learning baseline is unchanged.

## What changed on 2026-09-14

- Every triangular cell can now be subdivided into four, including panel and
  roof interiors. The original 28 vertices, 50 panels and 76 crease paths remain
  unchanged at neutral. This fixes the previous boundary-only refinement.
- Sparse element assembly replaces the dense global residual Jacobian for the
  new preset. It uses the same elastic energy and force-residual acceptance.
  The search tangent is not a new material law or a physical stability test.
- `Material reference` now uses 0.4 mm PLA on 80 um PET, intact connections,
  unit membrane/panel-bending scales, and PET bending rigidity derived from
  thickness and modulus. The extra empirical crease-twist coupling is zero;
  actual strip bending/warping DOFs remain free. No fold is a one-axis pin joint.
- Roof interiors and panel interiors deform. Only the two square perimeter
  frames are rigid. Only the knee module is compliant in the whole leg.
- Optional IPC barriers provide actual **midsurface** contact forces. Continuous
  collision checks include a conservative bound on the curved rigid-frame path.
  This is not finite-thickness PLA/PET clearance, friction or cable-guide contact.
- Refined drop solves use sparse inertia assembly, avoiding the previous dense
  memory allocation. Accepted physical steps are shown as they finish; the solve
  is slower than real time. Recorded replay is explicitly labelled replay.

## What the investigation found

With intact connections and unscaled assumed material properties, the current
model still does **not** reproduce the large motion in the supplied videos.
An illustrative refinement-1 sweep (0.2 mm PET gap, extra crease coupling 0.1)
at 10 N **per active strand** produced approximately 1.90% compression,
0.492 degrees bend, and 0.351 degrees twist in separate load cases. At 20 N it
produced 4.92%, 1.063 degrees and 0.804 degrees respectively. These are numerical
diagnostics, not recommended hardware loads; no cable/anchor strength is known.

Refinement 1 to 2 changes the 3 N predictions by about 15.9% in compression,
42.6% in Y bend and 16.8% in twist, relative to refinement 1. This fails the
5% spatial-convergence gate. A small equilibrium residual does not establish
mesh-independent accuracy. Contact was inactive in these small-motion cases:
missing contact was a large-fold safety gap, not the cause of small-load stiffness.

The live material-reference demo (zero extra crease coupling, 3 N/active strand)
completed all 42 load/unload steps across seven cable patterns in 194.93 s, with
no rejected candidate. Median solve time was 4.81 s, maximum 5.37 s. Maximum
compression was 0.503%, bend about 0.141 degrees, twist 0.098 degrees. Median app
update interval was 10.05 ms across 11,133 updates; this is **not GPU timing or
real-time FEM speed**. Kit used 20 Torch threads; offline studies used 4, so their
wall times are not directly comparable. 53 numerical checks and six live
controller checks passed; none is a physical material calibration.

A 0.8 mm exposed PET-gap experiment, with unchanged material moduli/thicknesses,
increased compression to about 0.944% at 3 N and 4.21% at 10 N. It still did not
establish full motion. This is a different PLA inset/fabrication width, not the
default 0.2 mm gap.

The 70 mm / assumed 50 g impact benchmark covered only the first 1 ms after
contact. With 50 us steps it reached 20.27 N ground force, 0.466% compression and
0.066 degrees bend by that point. This is **not the full impact peak**. An overly
strict internal optimizer stopping target caused one earlier step to spend 180
iterations at an already small residual. The new internal target is 1e-6
J/scaled coordinate; the physical residual rejection guard remains 3e-5. The
rerun used at most a few iterations per step and changed the window's maximum
ground force by less than one part per million. No material stiffness or strain
guard was relaxed. A complete converged transient remains outstanding.

The neutral JSON supplies a 3D surface, not a flat fabrication pattern, material
seams, cable-guide locations, or a crease constitutive law. Some internal source
vertices have nonzero angle defects. That is a reason to verify film continuity
at those junctions, **not proof that the physical joint cannot fold**.
The current PET is elastic about the imported neutral shape; pre-creasing,
plasticity, hysteresis, adhesive compliance/damage and print anisotropy are not
calibrated. Merely reducing PLA stiffness until a video-like motion appears
would not validate the actual material.

## Use the updated workshop

### Seeing very small calculated movements

The nonlinear workshop now has a separate cyan **magnified displacement plot**
on the right. The actual leg on the left always remains at 1x. The checkbox
shows/hides the plot; the adjacent number changes displacement gain from 1 to
500 (default 100). Grey lines mark the neutral reference. Gain applies only to
displayed displacement vectors, never to cable force, material stiffness,
strain, contact, solver state or the saved physical calculation.

**Replay response** loops neutral and recorded accepted states on the right at
two steps per second. **Pause / resume** pauses this replay when no solve is
active. **Hold** returns that plot to the current solved result.
No new FEM solve or physical time integration occurs during this replay. A new
cable/drop command stops diagnostic replay and updates the plot as new accepted
states arrive. The right plot removes upper-frame rigid-body motion, so whole-leg
fall translation is not magnified as deformation. **Knee close-up** frames both
views while the plot is enabled.

This is a vector visualization, not a mechanically admissible large-fold shape:
linear magnification can distort rigid-frame lengths and create apparent
intersections. Do not infer travel, strain or survival from it. In the user's
0.5 mm twist-winch example, the actual maximum displacement was only 19.34 um
and twist 0.0155 degrees; x100 shows a 1.934 mm vector-plot change. That makes
the numerical response visible but does not fix the model's failure to reproduce
the physical joint's full folding. The physical-response problem remains open.

### Calculate a response

Open the nonlinear launcher as usual, or send `reload_shell_live.py` through the
project's running-Isaac remote helper. Do not start a second Kit instance.

1. Choose **Material reference**, then **Knee close-up**.
2. Set **Cable tension (N / active strand)**, then click a top **Compression**,
   **Bend X+/X-**, **Bend Y+/Y-**, or **Twist CW/CCW** button. Each starts an
   eight-step cable-only ramp immediately; it does not need Play or Apply.
   For combined external forces, choose the dropdown pattern, set the lower-frame
   force inputs, and press **Apply cable + force**. **Cable demo** loads/unloads
   all seven patterns. No mode prescribes a final cable-driven angle.
3. For displacement-controlled cable take-up, set **Winch pull (mm)** and use
   **Apply winch pull**. The cable rest length changes; joint displacement is
   still solved. Tension is `min(k * max(length-rest_length, 0), force_cap)`.
   Gold strands show loaded cables; inactive strands carry no force.
4. Change material/mesh settings, then press **Rebuild stiffness**. The ACTIVE
   readout lists the settings actually used. Invalid settings preserve the last
   accepted model. Rebuilding returns the new model to neutral.
5. **Drop 70 mm / recompute** uses the whole leg with one compliant knee,
   assumed 30 g upper and 20 g lower assemblies, foot-first normal contact and
   unloaded cables. It is a custom implicit impact calculation, not a hardware
   survival test. Inspect solved time and ground force, not wall-clock time.
6. **Save calculation** stores the source hash, active material/configuration,
   accepted geometry, load histories, contact diagnostics and rejected candidate.
   **Replay recorded impact** and **Show peak bending** only use saved solved frames.

### What the new settings mean

| Setting | Meaning / effect |
|---|---|
| PLA thickness (mm), PET thickness (um) | Physical layer thicknesses. Affect membrane/bending stiffness and shell mass. User-specified values are 0.4 mm and 80 um. |
| PLA/PET modulus (GPa, assumed) | Elastic assumptions: 2.2 / 3.5 GPa. These have not been measured on the user's samples. |
| Thickness-derived PET bending | On: `E*t^3/[12*(1-nu^2)]`; the ratio field is deliberately ignored. Off: effective panel/crease ratio experiment. |
| Panel / crease bending ratio | Only active with thickness-derived PET bending off. Larger = softer effective PET bending; not a measured crease law. |
| Crease twist coupling | Additional empirical penalty between neighbouring crease-segment rotations. Material reference uses zero, retaining the strip's own elasticity. |
| PET exposed gap (mm) | Wider PET-only strip between inset PLA facets. Changes the fabrication geometry and folding compliance, not PET modulus. Confirm a buildable width. |
| Panel bending / membrane scales | Experimental multipliers. Leave both at 1 for the unscaled material reference. |
| Interior refinement 0/1/2 | 912 / 3,648 / 14,592 triangles at boundary subdivision 1. Higher levels are slower; current results are not yet spatially converged. |
| Sparse element solver | Required for interior refinement and IPC. Numerical method, not a stiffness scale. |
| Midsurface self-contact + CCD | Barrier forces plus swept-path checks. Does not enforce the 0.48 mm laminate clearance or friction. |
| PET junction relief (%) | Actual PET connection cuts near source vertices. Zero is intact; nonzero changes the manufacturing design. |
| Strain / height guards | Stop checks, not yield/fracture criteria. Raising them is not evidence that the material can survive. |

The status line updates while the numerical worker runs: **SOLVING** shows the
movement, accepted step count and elapsed wall time. At 3 N the previously
measured solve took roughly five seconds per step; it is not a real-time animation.
**Pause / resume** holds publication of the next candidate. Choosing a different
movement replaces the previous job after its current solver step; the latest
button wins, without another click. **Neutral / cancel** restores the source
shape immediately and discards the old candidate. A worker already running must
finish before a new numerical worker can start, so cancellation can leave a
short delay before the new movement begins.

The old **Bend/Twist/Compression range** controls were prescribed-displacement
studies, not cable patterns. They are now inside a collapsed **Advanced
prescribed-pose studies** group and disabled with IPC self-contact active. Use
the top cable buttons. Explicitly disabling contact and rebuilding enables
separately labelled displacement studies, not evidence of achievable travel.

**A completed calculation can still look almost stationary at 1x.** Check the
numeric angles/compression and loaded gold strands. The button fix does not
change material stiffness, magnify geometry, establish full physical folding,
or validate FEM accuracy. A numerical guard remains a STOPPED result, not COMPLETE.

## Reproduce / dependencies

Use a CPython 3.12 numerical environment with the project's NumPy, SciPy and
Torch dependencies. The optional Windows contact wheel is pinned. From the
project root, using that environment's Python:

```powershell
python -m pip install --no-deps --target tmp/knee_contact_deps -r exact_joint/requirements-contact.txt
python -m exact_joint.run_shell_study --sparse --physical-strip --self-contact --interior-refinement 1 --crease-twist-ratio 0 --forces 1 3 5
python -m unittest exact_joint.test_shell_sparse
```

No installed Isaac runtime file needs modification. The dependency loader uses
the project-local target only when `ipctk` is otherwise unavailable and rejects
versions other than 1.6.0. Contact-off legacy calculations do not require IPC.

IPC is MIT-licensed; see `IPC_LICENSE.txt`. API/source references:
[IPC Toolkit v1.6.0](https://github.com/ipc-sim/ipc-toolkit/tree/v1.6.0),
[Python package](https://pypi.org/project/ipctk/1.6.0/).

## Remaining evidence needed before claiming full travel

Confirm PET continuity/slits/overlaps at the central fold junctions and the
actual cable guide/attachment paths. Measure at least one load/unload curve for
compression, bend and twist, including cable take-up or tension. Then fit the
creased-PET response, resolve spatial convergence, add finite-thickness contact
and compare a complete impact transient at converged timesteps. Until those
checks pass, full compression/bending/twisting and survival remain unestablished.

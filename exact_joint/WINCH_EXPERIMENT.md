# Cable-pull and fold-junction experiment

This improves the experiment controls and demonstrates a larger calculated
response, but **does not yet reproduce the full bend, twist and compression in
the supplied videos**. It is not validated FEM strength or a survival test.

## What to try in the open workshop

1. Click **Material reference** for the intact JSON pattern, nominal assumed
   material moduli and default crease law. It also rebuilds the mesh.
2. Select Compression, either bend direction, or either twist direction.
3. Enter **Winch pull (mm)**, press Enter, then **Apply winch pull**. This shortens
   the selected cables' rest lengths, not a joint angle. Actual cable tensions
   and the resulting mesh are calculated. Start at 0.1 mm on the reference.
4. Click **Relief experiment** to compare a deliberately modified PET-cut design:
   0.8 mm exposed gap; relief over 20% of each crease edge at both ends; panel
   bending scale 0.01; panel/crease ratio 1000; membrane scale still 1. The
   cyan boundaries show actual missing PET connections. This is NOT just a softer
   value for the user's existing intact joint, nor a production recommendation.
5. Start at 0.5 mm pull. In the recorded single-direction sweeps, 4 mm compression
   take-up and 3 mm bend take-up passed the current guards. A larger value is
   **not** assured safe or even numerically admissible. Other paths/load histories
   must be checked separately. Rejected states stay out of the viewport.
6. **Winch demo** cycles all seven cable families, loading and unloading in at
   most 0.25 mm take-up increments. It computes equilibria, so wall-clock playback
   can be slow. It is not a calibrated winch speed or transient cable simulation.
7. **Save calculation** preserves accepted states, actual cable tensions, rest
   lengths, material/settings and any rejection. Use **Neutral / cancel** to
   clear the load. **Material reference** returns to the intact pattern.

The ACTIVE readout shows the settings actually assembled into the model. Edits
to stiffness, relief or gap need **Rebuild stiffness**; Apply alone changes loads.
The old **Apply cable + force** and **Cable demo** still use constant tension;
they are separate from the new winch buttons. External frame-force fields apply
to manual Apply/winch, not the seven-family demos. Generic PhysX force edits do
not enter this custom CPU shell solver.

## What each new control means

| Control | Meaning |
|---|---|
| Winch pull (mm) | Shortens active cables relative to their neutral lengths. Positive take-up does not prescribe compression or an angle. |
| Winch stiffness (N/m) | Effective cable/drive series stiffness. Default 1000 N/m = 1 N/mm; provisional. A softer drive can stretch more at the same take-up. |
| Winch force cap (N) | Maximum tension per active strand, default 10 N. Not the actual tension: the solver reports that separately. Once capped, the cable acts as a constant-force pull in this quasistatic model. |
| PET junction relief (%) | Removes PET border segments near original crease endpoints. This changes manufacturing topology and shell mass. Zero restores intact film. It does not shorten PLA facets. |
| Static solver iterations | Maximum optimization effort, default 900 / relief preset 3000. Raising it does not soften anything or bypass strain/intersection checks. |
| Active settings | Values currently used by the solver, which may differ from un-applied field edits. |

The pull-only actuator law is `T = min(k * max(L - L0, 0), force_cap)`;
unselected strands are slack. The energy is the exact integral of this law.
There is no imposed joint angle, no hidden displacement magnification, and no
measured guide friction, pretension or spool dynamics. Cable routing remains the
earlier assumed four axial/eight crossed perimeter strands, not a verified
reconstruction of every physical guide in the video.

At the coarse relief resolution there is one remaining connected segment on
each original crease. The adjacent-section **Crease twist coupling** term is
therefore inactive in that cut mesh; it remains active on the intact subdivided
mesh. Joint yaw can still arise from shell deformation and crossed cable loads.

## What the experiments establish—and do not

- Reducing bending alone by 100x with a larger panel/crease ratio still produced
  small rotations with the intact film. Reducing membrane stiffness too produced
  a collapsed, intersecting candidate, not a usable full-motion result.
- PET junction reliefs greatly reduce the calculated membrane resistance without
  lowering its modulus. With reduced bending stiffness and elastic take-up, the
  modified design reached about **15.1% compression** and **3.73 degrees Y bend**
  and **1.66 degrees twist** in separate accepted sweeps. Twist was strongly
  coupled with compression. The next steps encountered PLA-to-PLA midsurface
  intersections. Exact numbers and twist results are in the linked run metrics.
- The compression reaction curve softens along part of this calculated path.
  Constant tension and displacement of an elastic winch do not trace the same
  stable path. This is a model result, not a calibrated physical actuator curve.
- The modified 70 mm/50 g drop diagnostic stopped at **0.85 ms after contact**
  on the solver-residual guard. Its recorded maximum force is only the maximum
  before that stop—not the full impact peak. No new drop survival result exists.
- Forty-one numerical implementation tests pass. The live button checks are
  recorded separately. These do not close the previous spatial-convergence
  failure, material calibration, large-fold/contact or strength gates.

The next model work is contact-aware large-fold continuation and appropriate
spatial refinement, followed by calibration against measured cable take-up,
tension and motion for the actual module size. Weakening the material further
or accepting crossed panels would not resolve those missing checks. A reference
for the membrane/bend/fold distinction is
[Liu and Paulino's nonlinear non-rigid origami model](https://paulino.scholar.princeton.edu/document/1566).

## Reproduce the larger-range study

Run using the installed Isaac NumPy/SciPy/Torch environment from the repository:

```text
python -m exact_joint.run_shell_study --mode winch --ratio 1000 --panel-bending-scale .01 --membrane-scale 1 --gap-mm .8 --vertex-relief-fraction .2 --max-iterations 3000 --pulls-mm .5 1 1.5 2 2.5 3 4 5 6 8 10 12 16 20
python -m exact_joint.run_shell_study --mode drop --ratio 1000 --panel-bending-scale .01 --gap-mm .8 --vertex-relief-fraction .2 --steps-us 25 --duration-ms 3
python -m unittest discover -s exact_joint -t . -p "test_*.py"
```

Every direction starts at neutral; each stops at its first rejected candidate.
Rejected candidate arrays are diagnostic evidence, never accepted playback.
Case: `fea/exact_joint_winch_relief_v1/manifest.json`.
Evaluation: `ml/runs/20260910-exact-knee-winch-relief-v1/`.
No training, baseline promotion or external publication was performed.

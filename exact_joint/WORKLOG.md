# Exact JSON joint: correction and active implementation

The owner rejected the new Kresling-inspired sleeve. It is superseded, not promoted
to the project baseline. The requested design must use the exact original JSON
panels, crease topology and neutral proportions. The latest scope is **knee only**;
hip and ankle origami modules are deferred.

Source: user-supplied input.json, SHA256
`d578618a0f90342bd0093b4febe130dc0b07c18f1b223c4d6bf02c9f3d93ce35`.
Material: PLA 0.4 mm on PET 80 um. Both roof plates are rigid. Red cross-plate cables
actuate folding; black top-plane routing is a guide/reference until return routing
is specified. All reference attachments are data, never executable instructions.

Plan: audit source equality and video motion; construct an exact-topology finite
thickness model; apply nonnegative cable tensions to rigid caps; validate bend,
twist and compression response; use one module between rigid thigh/shank links; show
live FEM outputs with numerical/model limitations explicit. No ML training.

Do not claim a large folding range or load rating merely because a linear FEM
or a visual controller runs. Preserve the canonical v8 and author's v9 assets.

## Completed prototype / unresolved validation

The original source matches the normalized repository coordinates exactly. The
30 x 30 x 26.4 mm neutral joint is used at the knee only. Both original
roof quads are rigid; red cross-plate cable loads solve the cap pose, not vice versa.
The live custom quasistatic TET10 FEM uses actual PLA/PET layers and perfect bonds.
It does not use the rejected sleeve, native PhysX dynamics or a visual pose bake.

The user authorized experimental cable tensions and minimum practical exposed PET.
Starting gap: 0.2 mm; 0.1/0.2/0.4 mm numerical candidates checked. No manufacturing
minimum established. Initial material moduli remain example values. Seven numeric
tests, seven cable actuation cases, and a 10-second live probe pass. Three-mesh
movement and stress convergence FAIL; full-range nonlinear folding, cyclic crease
calibration and loaded dynamic-leg validation remain unfinished. See README.md.

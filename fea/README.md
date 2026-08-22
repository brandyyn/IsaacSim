# FEA Handoff Contract

This directory stores reviewed Ansys FEA cases that calibrate the Isaac Sim knee. Create one immutable case directory per geometry/load/solver revision:

```text
fea/<case-id>/
  manifest.json          # copy fea/manifest.template.json and complete it
  README.md              # optional human summary and review notes
  raw/                   # solver exports; use Git LFS or the approved artifact store
  processed/             # versioned CSV/JSON tables used by Isaac Sim
```

## Required handoff information

The manifest must identify the exact geometry revision and include:

- unit system and coordinate frames;
- neutral pose, joint axis, angle sign, and angle range;
- material model and material parameters;
- mesh size, element type, contacts, constraints, loads, and solver settings;
- convergence evidence and known limitations;
- hashes and paths for raw and processed files.

## Minimum useful result set

The first handoff should provide enough data to derive and check an angle-dependent mechanical model, ideally including:

- joint angle versus actuator/reaction torque;
- restoring torque or force versus displacement;
- stiffness and damping assumptions or estimates;
- stress/strain and displacement fields for limit/load cases;
- reaction forces, contact state, and any failure or buckling indicators;
- mesh/convergence comparisons for the reported curves.

Use SI units in files and make the FEA-to-Isaac frame transform explicit. Do not send a plot without its source table and metadata. Do not train directly on unversioned solver output.

The Isaac Sim integration should consume a processed, versioned response model rather than silently embedding one engineer's local path or an unreviewed fitted curve. The conversion must record its source case ID and checksum.


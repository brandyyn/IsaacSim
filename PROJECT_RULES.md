# Project Rules: FEA-Calibrated Isaac Sim Knee and ML

These rules keep the Ansys FEA work, the Isaac Sim model, and machine-learning experiments synchronized across machines.

## Sources of truth

- `main` is the shared integration branch. Every machine pulls it before starting work.
- `PROJECT_CONTEXT.md` records the technical history and current simulation architecture.
- `ML_PROGRESS.md` records the current phase, owners, commits, FEA case, and next action.
- `fea/<case-id>/` contains the versioned FEA handoff package.
- `ml/runs/<run-id>/` contains one immutable record for each training or evaluation run.
- The current simulation baseline is the commit that contains `panel_crease_leg_v8.usd`. Do not silently replace it with an older prototype.

## Synchronization rules

1. Before work, run `git pull --ff-only` and record the starting commit in the experiment manifest.
2. Make one logical change at a time. Do not rewrite or delete another member's FEA, ML, or validation artifacts.
3. After a simulation change, rebuild or validate the stage as appropriate and update `PROJECT_CONTEXT.md` or `ML_PROGRESS.md`.
4. Commit code, configuration, manifests, metrics, and small reproducibility assets. Push the commit before handing work to another machine.
5. Never use `git add .` in this Isaac Sim source tree. Stage the project files explicitly so caches, `_build/`, IDE state, logs, and unrelated Isaac Sim files are not committed.
6. Do not commit secrets, personal absolute paths, or machine-local launch settings. Use `PANEL_CREASE_PROJECT_ROOT` for a cloned project on a different machine.
7. Never overwrite an ML run folder. If a run changes, create a new run ID and record the parent run or commit.

## FEA-to-Isaac rules

- The FEA case is authoritative for the structural response of the supplied geometry and boundary conditions. The source joint geometry and its revision remain explicit and must match the FEA case.
- Every FEA handoff must include units, coordinate frames, zero-angle definition, positive rotation direction, material data, mesh/convergence information, boundary conditions, loads, solver settings, and file hashes. Start from `fea/manifest.template.json`.
- Use SI units in exchanged data: metres, kilograms, seconds, newtons, newton-metres, pascals, and radians. Degrees may be used in human-facing tables only when clearly labelled.
- Do not silently rescale, mirror, or rotate a result. Put the mapping from the FEA frame to the Isaac frame in the manifest and validate it with a known neutral pose.
- Do not use a deformed FEA mesh as an unvalidated rigid-body or collision model. First reduce the FEA results to a versioned mechanical model: for example angle-dependent restoring torque, stiffness, damping, friction, force limits, displacement limits, and any required mass/inertia or modal information.
- Keep raw FEA results separate from processed lookup tables or fitted models. Raw solver files may be stored with Git LFS or an approved artifact store; the repository must always retain the manifest, provenance, and checksum.
- Before ML training, the FEA-calibrated Isaac model must pass neutral-pose, static torque-angle, joint-limit, and dynamic-response checks without a policy. A policy must not be used to hide a physics or frame-mapping error.

## ML experiment rules

- Every run gets a unique directory under `ml/runs/<run-id>/` and starts from `ml/run_manifest.template.json`.
- Record at minimum: baseline Git commit, FEA case ID and checksum, Isaac Sim version, stage name, environment version, algorithm, seed, observation/action definitions, reward version, command/config, training duration, evaluation metrics, and checkpoint location/hash.
- Keep training and evaluation separate. Report both raw metrics and the test conditions used to obtain them.
- Use deterministic seeds where the simulator and algorithm allow it. If randomization is enabled, record every randomized parameter and its range.
- Large checkpoints and datasets belong in Git LFS or the agreed artifact store. Git tracks the manifest and a stable pointer/checksum even when the binary is stored elsewhere.
- A result is not considered reproducible until another machine can check out the recorded commit, obtain the referenced FEA/data artifacts, launch the same environment, and reproduce the validation command.

## Definition of done for the next phase

1. Receive an Ansys export package in `fea/<case-id>/` using the handoff template.
2. Verify geometry revision, units, frame mapping, zero pose, and sign convention.
3. Convert FEA results into a versioned calibrated knee model and record the conversion method.
4. Validate the calibrated model in Isaac Sim against the FEA reference curves and the existing v8 physics smoke test.
5. Create the stabilization environment and its first ML run manifest.
6. Train only after the non-ML validation is passing, then update `ML_PROGRESS.md` with the commit and metrics.


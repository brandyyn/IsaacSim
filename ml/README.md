# ML Experiment Contract

Keep the simulator baseline and the learning result connected. Each run is immutable and lives under:

```text
ml/runs/<run-id>/
  manifest.json          # copy ml/run_manifest.template.json
  config.json            # exact training configuration
  metrics.json           # machine-readable results
  README.md              # short human summary
  checkpoints/           # large files via Git LFS or the approved artifact store
```

Before training, record the exact Git commit, FEA case ID/checksum, Isaac Sim version, stage, environment version, seed, and action/observation/reward definitions. Evaluation must state the disturbance and test conditions. Update the root `ML_PROGRESS.md` index after the run is reviewed.

The first ML environment should be trained against the FEA-calibrated, non-ML-validated knee. If the policy is compensating for a coordinate, unit, joint-limit, or solver error, stop and fix the simulation instead of treating the policy as a workaround.


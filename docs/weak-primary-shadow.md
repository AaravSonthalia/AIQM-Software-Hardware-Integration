# Weak Primary λ=0.1 Shadow Integration

The optional **Weak primary shadow** checkbox runs all 36 λ=0.1 evaluation
checkpoints. It never selects one fold as “the model” and never replaces the
existing live classifier.

This branch ships a self-contained runtime under
`models/weak_primary_lambda_0_1/RHEEDClassify/` containing:

- `Classifier2/weak_primary_model.py`
- `Classifier2/artifacts/weak_primary_four_class_20260803/encoder/`
- `Classifier2/artifacts/weak_primary_four_class_20260803/full_benchmark/checkpoints/`

The bundled package is the default, so ChMBE only needs this branch.
`AI_REPO_ROOT` remains an explicit override for a
different RHEEDClassify checkout. Alternatively,
`AIQM_WEAK_PRIMARY_SHADOW_ROOT` can point directly to another
`weak_primary_four_class_20260803` artifact directory. Device selection is
automatic; `AIQM_WEAK_PRIMARY_DEVICE=cpu` or `cuda` overrides it.
Before loading, the GUI also requires at least 1536 MB of currently available
physical memory. Set `AIQM_WEAK_PRIMARY_MIN_AVAILABLE_MB` only when a measured
workstation-specific limit is justified; setting it to `0` disables the guard.

The package contains the shared registered encoder and all 36 evaluation
heads. It is intentionally not a final fixed-split three-seed deployment
ensemble. Do not copy these weights over a production classifier checkpoint.

At ARM, the loader verifies the registered encoder SHA-256 and requires the
complete 4-fold × 3-pair-run × 3-seed collection. Each checkpoint must declare
`lambda_pair=0.1`, `execution_scope=weak_shadow_only`, and
`deployment_eligible=false`.

The monitor shows four probabilities conditioned on a visible superstructure:
Twinned (2×1), c(6×2), RT13, and HTR. There is no 1×1/none class, so these
values are not surface fractions and are never actionable. Results, ensemble
identity, disagreement, capture sequence, and inference timing are appended to
`temporal_trace.jsonl` as `weak_primary_shadow_state` events.

## Workstation acceptance

Run the model benchmark before enabling the checkbox on an older workstation:

```powershell
python scripts/weak_primary_shadow_benchmark.py --device cpu `
  --iterations 60 --output logs/weak-primary-benchmark.json
```

It is a separate process, so a failed load cannot terminate Growth Monitor.
The default gate requires load time <=120 s, p95 inference <=1 s, peak working
set <=2500 MB, and steady RSS growth <=128 MB. The stress loop runs without the
normal two-second worker pause, making it more demanding than live shadow use.
Do not enable the GUI option unless the report says `PASS`. Repeat with
`--device cuda` only when that workstation has a CUDA-enabled PyTorch build.

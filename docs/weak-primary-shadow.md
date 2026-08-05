# Weak Primary λ=0.1 Shadow Integration

The optional **Weak primary shadow** checkbox runs all 36 λ=0.1 evaluation
checkpoints. It never selects one fold as “the model” and never replaces the
existing live classifier.

The runtime requires an adjacent `AI_for_quantum`/RHEEDClassify checkout with:

- `Classifier2/weak_primary_model.py`
- `Classifier2/artifacts/weak_primary_four_class_20260803/encoder/`
- `Classifier2/artifacts/weak_primary_four_class_20260803/full_benchmark/checkpoints/`

Set `AI_REPO_ROOT` when that checkout is not at a known workstation path.
Alternatively, set `AIQM_WEAK_PRIMARY_SHADOW_ROOT` directly to the
`weak_primary_four_class_20260803` artifact directory. Device selection is
automatic; `AIQM_WEAK_PRIMARY_DEVICE=cpu` or `cuda` overrides it.

At ARM, the loader verifies the registered encoder SHA-256 and requires the
complete 4-fold × 3-pair-run × 3-seed collection. Each checkpoint must declare
`lambda_pair=0.1`, `execution_scope=weak_shadow_only`, and
`deployment_eligible=false`.

The monitor shows four probabilities conditioned on a visible superstructure:
Twinned (2×1), c(6×2), RT13, and HTR. There is no 1×1/none class, so these
values are not surface fractions and are never actionable. Results, ensemble
identity, disagreement, capture sequence, and inference timing are appended to
`temporal_trace.jsonl` as `weak_primary_shadow_state` events.

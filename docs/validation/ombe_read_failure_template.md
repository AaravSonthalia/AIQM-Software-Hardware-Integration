# O-MBE Read-Failure Validation

## Run Identity

- Date/operator:
- Workstation: Bulbasaur
- Recorder commit and dirty state: see `run_info.json`
- Software/WGC commit and dirty state: see `run_info.json`
- Windows/DPI:
- Vendor software versions and window title:
- Raw run directories and `sha256_manifest.json`:

## Safety Boundary

The probe is read-only. The operator may cover, minimize, close, or restore a
software window. Do not unplug serial cables, power-cycle devices, change
setpoints, move shutters, or stop hardware controllers for this validation.
Use a new output directory for every run; the recorder refuses to overwrite one.

## Durable Run

Commands longer than ten seconds must use `managed-long-jobs`. Keep job status
and raw evidence outside Git. Use the explicit GUI Python environment:

```powershell
$manager = "$env:USERPROFILE\.codex\skills\managed-long-jobs\scripts\managed_job.cmd"
$python = "C:\Users\Yao_Yufan\.conda\envs\ai4mbe-gui\python.exe"
$checkout = "D:\path\to\failure-validation-checkout"
$run = "D:\O-MBE-validation\failure\mistral_20260727_120000"
$job = "D:\O-MBE-validation\jobs\mistral_20260727_120000"

& $manager launch --output-dir $job --cwd $checkout -- $python `
  scripts\ombe_failure_probe.py run --source mistral --mode screengrab `
  --duration-s 300 --baseline-duration-s 30 --output-dir $run

& $manager status --output-dir $job
```

Poll `status` at least every 30 seconds until `result_available=true`, then read
its stdout/stderr and the run's `summary.json`. Do not reuse `$job` or `$run`.

Allow at least 30 healthy seconds before the first failure marker, unless a
healthy p95 from the timing branch is supplied with `--baseline-p95-s`.
Timestamp manual actions from another terminal:

```powershell
& $python "$checkout\scripts\ombe_failure_probe.py" mark `
  --output-dir $run --kind failure-start --label cover `
  --note "MISTRAL fully covered"

& $python "$checkout\scripts\ombe_failure_probe.py" mark `
  --output-dir $run --kind recovery --label uncover
```

Use `--kind failure-start` and `--kind recovery` for every failure window.
Ordinary notes may use `--kind action`. Add the recovery marker before the
managed run ends so the final SHA manifest includes the complete marker file.

## WGC Cross-Checkout Run

The failure branch starts at `b24ceb1` and does not contain WGC. For the RHEED
case, keep the recorder in this checkout but load workers from the committed WGC
checkout. The resulting `run_info.json` records both `recorder_git_commit` and
`software_git_commit`, both dirty flags, driver identity, and reported backend.

```powershell
$failureCheckout = "D:\path\to\failure-validation-checkout"
$wgcCheckout = "D:\path\to\wgc-rheed-live-test-checkout"
$run = "D:\O-MBE-validation\failure\rheed_wgc_20260727_130000"
$job = "D:\O-MBE-validation\jobs\rheed_wgc_20260727_130000"

& $manager launch --output-dir $job --cwd $failureCheckout -- $python `
  scripts\ombe_failure_probe.py run --source camera --mode screengrab `
  --software-root $wgcCheckout --expected-capture-backend wgc `
  --duration-s 300 --baseline-duration-s 30 --output-dir $run
```

The command fails closed if `CameraState.capture_backend` is not `wgc` or the
requested mode resolves to an unexpected/Dummy driver.

## Test Matrix

| Source | Mode | Manual actions |
|---|---|---|
| RHEED | WGC `screengrab` | cover, minimize/restore, close/reopen |
| MISTRAL | `screengrab` | cover, minimize/restore, close/reopen |
| EvapControl | `screengrab` | cover, minimize/restore, close/reopen |
| EvapControl | `elog` | approved stop/restart of file updates |
| Pyrometer | TemperaSure `screengrab` | minimize/restore, close/reopen |

This standalone recorder measures worker emissions. Run Growth Monitor
interactively for the separate GUI checks: whether session logging continues,
whether stale values remain visible, and whether recovery requires re-ARM.
Record those observations below; do not put the interactive GUI under the job
manager.

## Results

| Source/mode | Detection latency | Old valid value retained? | GUI logging continued? | Recovery/re-ARM | Stale candidates |
|---|---:|---|---|---|---:|
| RHEED WGC | | | | | |
| MISTRAL OCR | | | | | |
| Evap OCR | | | | | |
| Evap Elog | | | | | |
| TemperaSure UIA | | | | | |

`summary.json` treats all-`None` readings as invalid even if a driver still says
connected. A stale candidate is a value that remains valid and unchanged for
longer than `max(2 × healthy p95 update interval, 3 s)` inside an
operator-marked failure window. Constant readings outside a marked failure
window are not labeled stale. `data_age_ms` is populated only when the State
provides source, capture, or receive provenance; `unavailable` is explicit.

## Findings

- Confirmed behavior:
- Ambiguities:
- Recommended hardening:

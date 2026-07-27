# O-MBE Timing Observability Live Test

This test measures the timing of values already read by Growth Monitor. It is
passive: do not change instrument setpoints, disconnect cables, or power down
equipment for this branch.

## Before the run

Record:

- Operator and date:
- Bulbasaur Windows version and display DPI:
- GUI commit:
- Pyrometer, MISTRAL, and EvapControl modes:
- Exact window titles:
- GUI logging interval and worker settings:
- EvapControl/MISTRAL/TemperaSure versions:
- Session directory:

Confirm the GUI is using real O-MBE modes rather than `dummy`. ARM the
instruments, START a session, and locate its live `sensor_log.csv`.

## One-hour probe

Run the passive watcher in a second terminal. Long runs must use the repository's
managed-job launcher so they survive a Codex tool timeout:

```powershell
$manager = "$env:USERPROFILE\.codex\skills\managed-long-jobs\scripts\managed_job.cmd"
$python = "C:\Users\Yao_Yufan\.conda\envs\ai4mbe-gui\python.exe"

& $manager launch `
  --output-dir D:\AIQM-timing-job `
  --cwd D:\AI4MBE\AIQM-Software-Hardware-Integration `
  -- $python scripts\ombe_timing_probe.py `
  D:\path\to\growth_SESSION\sensor_log.csv `
  --duration-seconds 3600 `
  --output-dir D:\AIQM-timing-results `
  --operator "NAME" `
  --windows-dpi "Windows BUILD; DPI SCALE" `
  --gui-modes "pyrometer=MODE;mistral=MODE;evap=elog" `
  --window-titles "TemperaSure=...;MISTRAL=...;EvapControl=..." `
  --sampling-settings "sensor_log=1Hz;pyrometer=0.5s/5 reads;mistral=1s;evap=1s" `
  --software-versions "Growth Monitor=...;EvapControl=..."
```

Poll `& $manager status --output-dir D:\AIQM-timing-job` about every 30
seconds. Read stdout/stderr as soon as `result_available=true`. Use new job and
result directories for every run; the probe refuses to overwrite results.
Do not move generated raw logs into Git.

## Evidence review

Keep `timing_summary.json`, `timing_report.md`, the original `sensor_log.csv`,
and its SHA-256 together on the workstation. Review:

- sequence-normalized interval p50/p95/p99;
- read-duration and logged-age p50/p95/p99;
- rows missing provenance or all instrument values;
- repeated sequence rows and unobserved intermediate samples;
- Elog source-to-Python-receive offset.
- Elog embedded-record update interval and repeated source timestamps.

Do not declare synchronization sufficient or insufficient from a single scalar.
Document pauses, window changes, clock adjustments, and any application errors.
The generated report intentionally applies no pass/fail threshold.

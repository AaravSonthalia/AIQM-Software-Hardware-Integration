# O-MBE GUI Temporal Performance Validation

This protocol is read-only. Growth Monitor records when data reaches Python;
it does not change setpoints or instrument polling rates. WGC time is a Python
callback time, not camera exposure time. Only Elog currently supplies a native
source timestamp.

## Run manifest

- Operator/date:
- GUI commit and branch:
- Windows build, timezone, DPI and `w32tm /query /status` result:
- RHEED/Pyrometer/MISTRAL/EvapControl modes:
- Default-software versions and exact window titles:
- Worker and logging intervals:
- Session directory:
- External result directory:

Use `D:\Environment_Cache\conda_envs\ai4mbe-gui\python.exe`. Keep raw images,
CSV and JSONL on the workstation; commit only small summaries and SHA-256.

## Local preflight (no instruments)

Run this before moving the branch to Bulbasaur:

```powershell
$python = 'D:\Environment_Cache\conda_envs\ai4mbe-gui\python.exe'
& $python scripts\ombe_temporal_offline_smoke.py `
  --output-dir D:\OMBE_Temporal\offline_smoke_001
```

This creates a deterministic four-source session, analyzes it, and runs the
durability validator. `tests\test_modbus_pyrometer.py` and
`tests\test_exactus_pyrometer.py` use fakes and are safe offline.
`scripts\test_pyrometer.py` opens a real serial port and is live-only.

## A. Per-interface baselines

Warm up each mode for two minutes, then record at least ten minutes. Serial
pyrometer modes are mutually exclusive.

1. RHEED WGC; repeat with `screengrab_mss` only as a diagnostic.
2. Pyrometer Exactus, Modbus and TemperaSure UIA.
3. MISTRAL OCR; test JSON-RPC separately and record connected-but-None.
4. EvapControl Elog and OCR. If possible, compare overlapping Elog/OCR values.

Do not treat a row timestamp as simultaneous acquisition. Review sequence
interval, read duration, worker-to-GUI delay, logged age, missing/invalid rate,
repeated sequence, skipped samples and longest gap.

## B. Combined one-hour run

Start Growth Monitor interactively in production modes and START a session.
Launch the passive analyzer with the managed-job wrapper:

```powershell
$manager = 'C:\Users\Yao_Yufan\.codex\skills\managed-long-jobs\scripts\managed_job.cmd'
$python = 'D:\Environment_Cache\conda_envs\ai4mbe-gui\python.exe'
& $manager launch --output-dir D:\OMBE_Temporal\job_001 `
  --cwd D:\AI4MBE\AIQM-Software-Hardware-Integration -- `
  $python scripts\ombe_timing_probe.py `
  D:\path\to\session\sensor_log.csv --duration-seconds 3600 `
  --output-dir D:\OMBE_Temporal\run_001 --operator NAME `
  --windows-dpi 'BUILD; SCALE' --gui-modes 'rheed=wgc;pyro=...;mistral=...;evap=...'
```

Poll the managed job about every 30 seconds. Exercise tab changes, buttons,
window moves, foreground/background, resize and DPI during labeled segments.
Record classifier and continuous-capture on/off intervals in the notes. Review
cross-source `sync_span_ms`, nearest-neighbor delta to RHEED, Qt next-turn
latency, classifier capture-to-complete, CSV/image write time, RSS and threads.

## C. Fault and recovery markers

The operator performs all physical actions under the instrument safety
procedure. Immediately bracket each action with markers on the same computer:

```powershell
$python scripts\ombe_temporal_marker.py `
  D:\path\to\session\operator_actions.jsonl mistral_close `
  --source mistral --phase before
# Manually close the MISTRAL window.
$python scripts\ombe_temporal_marker.py `
  D:\path\to\session\operator_actions.jsonl mistral_reopen `
  --source mistral --phase after
```

Repeat as authorized for window occlusion/minimize/close/frozen display,
serial/network disconnection, instrument power loss and recovery. Never change
setpoints. Test normal GUI close, forced process termination, and—only with
explicit facility approval—computer power interruption. Record whether recovery
requires ARM/reconnect and whether classification or image writes continued.

The audit flag for retained stale data is
`max(2 * measured p95 update interval, 3 s)`. It is not a production threshold.

## D. Durability check and evidence

After every run, including a forced exit:

```powershell
$python scripts\validate_temporal_session.py D:\path\to\session `
  --output D:\OMBE_Temporal\run_001\session_validation.json
```

The validator fails on malformed CSV/JSONL, negative ages, valid samples without
provenance, inconsistent RHEED capture identity or orphan heartbeat images.
Retain `timing_summary.json`, `timing_report.md`, session validation, raw files
and hashes together.

No 500 ms or 1 s synchronization gate is assumed in the first run. Choose the
final permissible skew only after relating measured p95/p99 span to the maximum
physical change rate relevant to the experiment.

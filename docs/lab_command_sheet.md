# Lab Command Sheet — Bulbasaur / Ch-MBE

Copy-pasteable PowerShell commands for the standard lab-session precheck
sequence. Every command below is **READ-ONLY** unless explicitly noted;
none change device or PLC state.

Referenced scripts live in `scripts/`; each has a `--help` for its own
flag reference. Sequencing follows the [Codex-suggested lab day
order](#recommended-day-order) — deviate as needed for the day's
priorities.

---

## 0. Environment setup (once per session)

```powershell
# From the AIQM repo root
cd C:\path\to\AIQM-Software-Hardware-Integration
git pull
```

Confirm the active chamber:

```powershell
$env:AIQM_CHAMBER          # blank → get_active_config() defaults to O-MBE
$env:AIQM_CHAMBER = "ombe"  # explicit O-MBE
$env:AIQM_CHAMBER = "chmbe" # explicit Ch-MBE — MUST set at shell scope, not inline
```

If working on Ch-MBE, set `$env:AIQM_CHAMBER = "chmbe"` **at the PowerShell
prompt** before invoking any script. Inline `python -c "os.environ['AIQM_CHAMBER']='chmbe'; ..."`
can silently default to O-MBE because `get_active_config()` may already have
run at import time (module-level cache). Setting the env var at the shell
scope, before Python is invoked, avoids that hazard.

Choose an evidence directory once per session so scripts pipe their JSON
into a common location:

```powershell
$evidence = "C:\lab_evidence\{{yyyy-MM-dd}}"          # substitute date
New-Item -ItemType Directory -Path $evidence -Force
```

---

## 1. Fast Mac-side sanity checks (optional, if AJ pulled fresh code)

Runnable from Mac dev env too — no lab hardware needed:

```powershell
python scripts\test_vimba_camera.py                    # VmbCamera + Read fallback
python scripts\test_pyrometer_physical_debug.py        # pyrometer debug script
python scripts\test_pyrometer_worker.py                # pyrometer worker + has_valid_reading
python scripts\test_pyrometer_downstream_consumers.py  # log_sensors / heartbeat / commit
python scripts\test_pyrometer_modbus_discover.py       # baud × device_id discovery sweep
python scripts\test_pyrometer_force_modbus.py          # expert-gated mode-switch write
python scripts\test_precheck_mistral_ads.py            # MISTRAL ADS precheck
python scripts\test_audit_session_sensor_log.py        # sensor-log audit
```

All should exit `0` with green summaries. Failures here mean something
regressed in the pulled code — do NOT proceed until fixed.

---

## 2. O-MBE camera — VmbCamera / kSA coexistence

### 2a. kSA closed — regression guard for the Full path

```powershell
# With kSA 400 fully closed:
python scripts\precheck_direct_camera.py --access-mode auto
```

Expected: `Access mode: full` in the output; verdict READY. If it says
`read`, kSA is still holding the camera despite appearing closed —
check Task Manager for `kSA400.exe` and end it.

### 2b. kSA closed — explicit Full (regression guard)

```powershell
python scripts\precheck_direct_camera.py --access-mode full
```

Same expected behavior; different code path.

### 2c. kSA open — auto (requires Task #187 multicast enabled)

```powershell
# With kSA 400 open on the Live Video window:
python scripts\precheck_direct_camera.py --access-mode auto
```

Two possible outcomes:

- **Task #187 landed**: `Access mode: read`, frames flowing → **PASS**
- **Task #187 NOT landed**: either "Read opened but no frames" or
  "Full denied AND Read denied" — both are useful signal per the
  Read-fallback plan. Read the recovery hint in the verdict block
  and follow it.

### 2d. kSA open — forced Read (isolates Read code path)

```powershell
python scripts\precheck_direct_camera.py --access-mode read
```

Bypasses the auto-fallback. If test 2c is ambiguous, this tells you
whether Read itself is fine and only the fallback path is misbehaving.

---

## 3. O-MBE MISTRAL ADS direct read

### 3a. Direct precheck (with evidence bundle)

```powershell
python scripts\precheck_mistral_ads.py --output-dir $evidence\mistral_ombe
```

Expected: `State: ok` with `6/6 cells populated` (Bulbasaur has 6 cells;
cell7 is not physically present). Any other state — read the recovery
hint.

Common failure hints:
- `connect_failed` → check TwinCAT System Service is running (Windows
  Services), ADS route configured (TwinCAT XAE / SysMan → Routes)
- `provenance_violation` → chamber config mismatch; check
  `drivers/config.py` `ads_cell_count`

### 3b. Short GUI session (for the audit script)

Launch Growth Monitor for O-MBE:

```powershell
python growth_monitor_ombe.py
```

Arm a session, let it collect ≥30s of data, disarm, note the session
directory (printed on session start, format `sessions/YYYY-MM-DD_HH-MM-SS`).

### 3c. Run the sensor-log audit

```powershell
python scripts\audit_session_sensor_log.py path\to\session_dir
```

Expected: `VERDICT: PASS` — the CSV has all 56 ADS union columns,
cells 1-6 populated, cell7 blank, session_metadata.json provenance
consistent.

---

## 4. Pyrometer (Task #196) — BEFORE opening TemperaSure

Critical: TemperaSure grabs COM4 **exclusively**. None of the pyrometer
scripts below can share the port with it. The rule is:

> **TemperaSure MUST be CLOSED while these scripts run.** Keep
> TemperaSure available to open BEFORE (verify probe is alive) and
> AFTER (re-verify probe is still alive), but never concurrently. The
> two apps cannot own COM4 at the same time.

### 4.0 Decision flowchart

```
Pyrometer silent when TemperaSure closed?
  │
  ├─ Run pyrometer_physical_debug.py first (READ-ONLY diagnostic)
  │    ├─ Verdict = exactus_streaming / modbus_responsive → use the
  │    │  matching driver in the GUI. STOP.
  │    └─ Verdict = silent / transmitting_unknown / port_down → continue.
  │
  ├─ Run pyrometer_modbus_discover.py (READ-ONLY, broader sweep)
  │    ├─ identity_found_at_default → device is fine at (115200, 1).
  │    │  ModbusPyrometer will read it. STOP.
  │    ├─ identity_found_at_non_default → device is at unexpected
  │    │  (baud, device_id). Reconfigure driver OR use TemperaSure to
  │    │  reconfigure device back to default. STOP (do NOT force).
  │    └─ silent_all_combos → device isn't answering Modbus at any
  │       combo. If TemperaSure just confirmed the probe is alive,
  │       continue to force-Modbus experiment. Otherwise physical
  │       inspection / power-cycle.
  │
  └─ Run pyrometer_force_modbus.py (EXPERIMENTAL WRITE, expert-gated)
       ├─ Requires --i-am-doing-a-write (safety gate; --dry-run
       │  previews the bytes without touching the port)
       ├─ Re-run pyrometer_modbus_discover.py to confirm the switch
       │  worked. wrote_with_response from force_modbus is NOT success
       │  by itself — see Section 4.3 "Interpreting the verdict."
       └─ If still silent after force-Modbus: fall back to Path A
          (physical power-cycle the probe, needs Jiangang's OK). Do
          NOT loop force-Modbus retries.
```

### 4.1 `pyrometer_physical_debug.py` (READ-ONLY, always safe)

```powershell
python scripts\pyrometer_physical_debug.py --output-dir $evidence\pyrometer_debug
```

Or with a longer Exactus listen if the device is expected to be
streaming slowly:

```powershell
python scripts\pyrometer_physical_debug.py --exactus-listen-s 10 `
    --output-dir $evidence\pyrometer_debug
```

Verdict states and next actions:

| State | Next action |
|-------|-------------|
| `exactus_streaming` | Use `ExactusSerialPyrometer` in the GUI. Done. |
| `modbus_responsive` | Use `ModbusPyrometer` in the GUI. Done. |
| `both_active_unexpected` | Either driver works; report to group. |
| `transmitting_unknown` | Continue to 4.2 (discover) — different baud may match. |
| `silent` | Continue to 4.2 (discover) — device may be at non-default (baud, id). |
| `port_down` | Check Device Manager COM assignment + hoggers in Phase 0 output. |

### 4.2 `pyrometer_modbus_discover.py` (READ-ONLY, broader sweep)

Broadens `pyrometer_physical_debug`'s search: 5 baud rates × 11 device
IDs (default) instead of 3 bauds × 1 device_id. Opens the port ONCE per
baud (inner device-ID loop shares the same client), so the full 55-combo
sweep takes ~500 ms of port overhead instead of ~11 s.

**Data-only reads**: touches REG_VER / REG_NAME0 / REG_SN0 / REG_CH1_TEMP
only. Never reads config registers (address, baud, rate, cmd), never
writes anything. Broadcast address 0 is intentionally excluded (Modbus
spec §4.1.1 — write-only convention).

```powershell
python scripts\pyrometer_modbus_discover.py --output-dir $evidence\pyrometer_discover
```

After a probe power-cycle (BASF manual p. 51 documents 5-10 s self-init),
give the device time to boot with a **one-shot** wait — this is NOT
per-combo, just the initial pre-sweep pause:

```powershell
python scripts\pyrometer_modbus_discover.py --initial-wait-s 8 `
    --output-dir $evidence\pyrometer_discover
```

For debugging multiple devices on the bus, walk the full matrix
instead of early-stopping on the first hit per baud:

```powershell
python scripts\pyrometer_modbus_discover.py --exhaustive `
    --output-dir $evidence\pyrometer_discover
```

Verdict states and next actions:

| State | Next action |
|-------|-------------|
| `identity_found_at_default` | Device is at (115200, 1) — the driver default. `ModbusPyrometer` will read it. Done. |
| `identity_found_at_non_default` | Device answered at a non-default (baud, id). **This script does NOT reconfigure** — either update `ModbusPyrometer` construction to match OR use TemperaSure to reconfigure the device back. |
| `silent_all_combos` | Device isn't answering Modbus at any combo. If TemperaSure confirms the probe is alive, force-Modbus experiment (4.3) is a candidate. Otherwise physical inspection. |
| `pymodbus_missing` | Install `pymodbus`: `pip install pymodbus pyserial`. Env issue, not a device issue. |

### 4.3 `pyrometer_force_modbus.py` (EXPERIMENTAL WRITE, expert-gated)

**Read Section 4.0 flowchart before considering this script.** This is
the ONLY write-capable pyrometer script in the repo. It sends Jacques's
candidate Exactus→Modbus command `02 4D 4D 03` (STX + "MM" + ETX, from
`JacquesPyrometerTesting.ipynb`). The command is NOT independently
confirmed by any BASF manual page we've read — treat as
candidate-experimental.

**Pre-flight**:

1. Run `pyrometer_modbus_discover.py` first (Section 4.2). Only proceed
   if it returned `silent_all_combos`.
2. TemperaSure MUST be **closed**. Open TemperaSure briefly BEFORE
   running this script to confirm the probe is alive → close TemperaSure
   → run force_modbus → close force_modbus → optionally reopen
   TemperaSure to confirm the probe is still alive.
3. Have Path A fallback authorization from Jiangang before starting
   (in case force-Modbus doesn't work and physical power-cycle is
   needed).

**Preview the bytes without touching the port** (always safe, works
without pyserial):

```powershell
python scripts\pyrometer_force_modbus.py --dry-run
```

**Actual write** (requires the explicit ack flag):

```powershell
python scripts\pyrometer_force_modbus.py --i-am-doing-a-write `
    --output-dir $evidence\pyrometer_force_modbus
```

Add `--verify` to print the next-step command (the discover invocation)
after the run — the script does NOT auto-invoke it:

```powershell
python scripts\pyrometer_force_modbus.py --i-am-doing-a-write --verify `
    --output-dir $evidence\pyrometer_force_modbus
```

**Interpreting the verdict** — repeat this to yourself before acting:

| State | Meaning | Success? |
|-------|---------|----------|
| `dry_run` | Preview only, nothing sent | N/A |
| `wrote_no_response` | Write completed, no bytes echoed. Neutral outcome — matches BASF's "no response expected" documentation. | **Unknown**. Verify via 4.2. |
| `wrote_with_response` | Write completed, some bytes came back. | **NOT SUCCESS.** Jacques's notebook observed echo-like bytes; could equally be Prolific PL2303 local echo. Verify via 4.2. |
| `port_error` | Serial open or write raised. | Failure. Check TemperaSure isn't running, cable is seated, port matches Device Manager. |
| `safety_denied` | Neither `--i-am-doing-a-write` nor `--dry-run` was set. Nothing sent. | N/A |

**Success is proven ONLY by re-running `pyrometer_modbus_discover.py`
and getting `identity_found_at_default`.** Never treat any verdict from
this script — including `wrote_with_response` — as evidence the mode
switch worked.

If verification returns `silent_all_combos` again, **do NOT retry the
force write in a loop**. Fall back to Path A (physical power-cycle,
requires Jiangang's OK). Record the outcome in the day's memory update
so the next session inherits the finding.

---

## 5. Ch-MBE regression (if working on Ch-MBE that session)

Set the chamber and re-do the MISTRAL ADS precheck:

```powershell
$env:AIQM_CHAMBER = "chmbe"
python scripts\precheck_mistral_ads.py --output-dir $evidence\mistral_chmbe
```

Expected: `State: ok` with `7/7 cells populated` (Ch-MBE has 7 cells)
and `ads_display_confirmed: True` in the Phase 1 config summary.

Then a short GUI session + audit:

```powershell
python growth_monitor_chmbe.py
# Arm, collect ≥30s, disarm, note the session dir
python scripts\audit_session_sensor_log.py path\to\chmbe_session_dir
```

Audit `VERDICT: PASS` should show 7/7 cells populated (no blank cell7
this time).

---

## 6. Opportunistic Ch-MBE tasks (if time remains)

None of these have precheck scripts yet — they're pending investigations.
Include here so nothing gets forgotten:

- **Task #189** — LakeShore 335 SCPI direct-read on COM4
- **Task #190** — kSA SQL query interface for RHEED metrics
- **Task #195** — Diagnose why MISTRAL screengrab failed on Ch-MBE

If any of these get exploratory work at the lab, capture findings in a
dated `docs/lab_validation_YYYY_MM_DD_*.md` file.

---

## Recommended day order

Per Codex Jul 28 sequencing:

1. Section 0 + 1 (pull, sanity, environment)
2. Section 2 (O-MBE camera coverage)
3. Section 3 (O-MBE MISTRAL ADS end-to-end)
4. Section 4 (pyrometer — BEFORE anything TemperaSure-related)
5. Section 5 (Ch-MBE regression, if applicable)
6. Section 6 (opportunistic — only if time)

---

## Post-lab

- Push any evidence bundles to Slack or archive under
  `docs/lab_evidence/YYYY-MM-DD/` (SOP TBD — for now, keep JSONs local
  and paste relevant excerpts).
- Update the Claude memory `bulbasaur_lab_day_YYYY_MM_DD.md` with the
  day's outcomes (Claude will do this on request — say something like
  "update the memory files with today's work").
- If any code changes were pushed from the lab, note them in the memory
  cross-referenced to their commit hashes.

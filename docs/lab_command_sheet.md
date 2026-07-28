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
python scripts\test_vimba_camera.py               # VmbCamera + Read fallback
python scripts\test_pyrometer_physical_debug.py   # pyrometer debug script
python scripts\test_precheck_mistral_ads.py       # MISTRAL ADS precheck
python scripts\test_audit_session_sensor_log.py   # sensor-log audit
```

All four should exit `0` with green summaries. Failures here mean
something regressed in the pulled code — do NOT proceed until fixed.

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

Critical: run this diagnostic **before** launching TemperaSure or any
other pyrometer-facing tool. TemperaSure grabs COM4 exclusively and
mode-switches can put the device in a state where this diagnostic
would misread.

```powershell
python scripts\pyrometer_physical_debug.py --output-dir $evidence\pyrometer
```

Or with a longer Exactus listen if the device is expected to be
streaming slowly:

```powershell
python scripts\pyrometer_physical_debug.py --exactus-listen-s 10 --output-dir $evidence\pyrometer
```

Verdict states and next actions:

| State | Next action |
|-------|-------------|
| `exactus_streaming` | Use `ExactusSerialPyrometer` in the GUI |
| `modbus_responsive` | Use `ModbusPyrometer` in the GUI |
| `both_active_unexpected` | Either driver works; report to group |
| `transmitting_unknown` | Capture raw bytes; wrong baud or unknown protocol |
| `silent` | Physical inspection per verdict hint (power / cable / IFD-5) |
| `port_down` | Check Device Manager COM assignment + hoggers in Phase 0 output |

Do NOT run any write-capable pyrometer scripts (mode switches, reboots,
rate changes) without explicit sign-off — get Jiangang confirmation
first if hardware state needs changing.

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

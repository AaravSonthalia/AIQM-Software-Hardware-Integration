# Growth Session Schema — Proposal for Group Review

**Status:** Draft for discussion at the next group meeting · Last updated 2026-04-28

This document proposes a frozen schema for everything Growth Monitor writes
to disk per session. It exists because (a) the Apr 10 group-meeting
follow-ups list "Data schema — is everyone aligned on what gets logged?"
as an open question, and (b) recent feature additions (auto-capture,
heartbeat anchors, MISTRAL set-change events, pyrometer multi-reading)
have grown the per-session output significantly. We should agree on the
shape now, before it diverges across machines or future contributors add
ad-hoc fields.

---

## 1. Per-session directory layout

A growth session writes to:

```
logs/growths/{prefix}_{sample_id}_{YYYYMMDD_HHMMSS}/
├── sensor_log.csv              ← polled instrument readings
├── commit_log.csv              ← grower-pressed LOG ENTRY events
├── auto_capture_events.csv     ← detector-flagged change events
├── heartbeat_log.csv           ← periodic anchor frames (10-min cadence)
├── set_change_events.csv       ← MISTRAL Set V/I operator changes
├── ocr_debug/                  ← (when AIQM_OCR_DEBUG=1) raw OCR crops
├── frames/                     ← all saved RHEED frames
│   ├── entry_NNN_HHMMSS.png            ← from manual commits
│   ├── heartbeat_NNN_HHMMSS.png        ← from heartbeat timer
│   └── auto_event_NNN/                 ← from auto-capture trigger
│       └── buf_MM_HHMMSS.png           ← N=20 ring-buffer dump per event
├── session_metadata.json       ← summary + grower notes (closed at session end)
└── growth_log.xlsx             ← grower-readable export of commits
```

**Naming convention:** `{prefix}` defaults to `growth`; `{sample_id}` is
free-form ASCII (spaces → `_`). The 6-digit `HHMMSS` timestamps inside
filenames are local time at the moment of writing; the ISO timestamp in
each CSV row is authoritative.

---

## 2. CSV schemas (current, after this proposal accepted)

### `sensor_log.csv` — polled at 1 Hz (configurable)

| Column | Type | Notes |
|---|---|---|
| `timestamp` | ISO 8601 | Local time, microsecond precision |
| `elapsed_s` | float | Seconds since session start |
| `pyrometer_temp_C` | float | Mean of `pyrometer_temp_n` rapid sub-readings |
| `pyrometer_temp_std_C` | float | Sample std of those sub-readings (0.0 if n=1) |
| `pyrometer_temp_n` | int | Sub-reading count (default 5) |
| `mistral_v_set_V` | float | OCR'd from MISTRAL window |
| `mistral_v_actual_V` | float | OCR'd |
| `mistral_i_set_A` | float | OCR'd |
| `mistral_i_actual_A` | float | OCR'd |
| `chamber_pressure_mbar` | float (sci) | OCR'd from Evap Control |

**Why mean ± std on temp:** Polybot-inspired statistical consistency.
Per-poll spread is a cheap signal of pyrometer health (large std =
signal interference) without requiring a second analysis pass over
the log.

### `commit_log.csv` — grower-driven (one row per LOG ENTRY click)

| Column | Type | Notes |
|---|---|---|
| `timestamp` | ISO 8601 | |
| `time_display` | string | Human-readable mm:ss for grower review |
| `elapsed_s` | float | |
| `sample_id` | string | Redundant w/ session dir, useful for cross-session pivots |
| `grower` | string | Same as above |
| `pyrometer_temp_C` | float | Snapshot at commit time |
| `voltage_V` | float | MISTRAL v_actual at commit |
| `current_A` | float | MISTRAL i_actual at commit |
| `recon_1x1` | int (0–100) | Reconstruction confidence sliders |
| `recon_Twinned (2x1)` | int | |
| `recon_c(6x2)` | int | |
| `recon_rt13xrt13` | int | |
| `recon_HTR` | int | |
| `note` | string | Free-form grower text |
| `frame_path` | string | Relative path to entry_NNN.png |

### `auto_capture_events.csv` — detector-driven

| Column | Type | Notes |
|---|---|---|
| `timestamp` | ISO 8601 | When the trigger fired |
| `elapsed_s` | float | |
| `event_idx` | int | Monotonic per session |
| `change_score` | float | Smoothed mean abs pixel diff at trigger |
| `pyrometer_temp_C` | float | Snapshot at trigger |
| `buffer_count` | int | Frames saved to `buffer_dir` (≤ 20) |
| `buffer_dir` | string | Relative path: `frames/auto_event_NNN/` |

### `heartbeat_log.csv` — timer-driven (every 10 min)

| Column | Type | Notes |
|---|---|---|
| `timestamp` | ISO 8601 | |
| `elapsed_s` | float | |
| `heartbeat_idx` | int | Monotonic per session |
| `pyrometer_temp_C` | float | Snapshot |
| `frame_path` | string | Relative path to heartbeat_NNN.png |

### `set_change_events.csv` — MISTRAL operator action

| Column | Type | Notes |
|---|---|---|
| `timestamp` | ISO 8601 | |
| `elapsed_s` | float | |
| `event_idx` | int | |
| `channel` | string | `"voltage"` or `"current"` |
| `old_value` | float | Last seen before change |
| `new_value` | float | First reading at new value |
| `delta` | float | Signed `new - old` |
| `pyrometer_temp_C` | float | Snapshot |

---

## 3. `session_metadata.json`

Written at session end (so it's only present on clean stops; sessions
killed mid-run leave the directory with logs but no metadata).

```json
{
  "date": "2026-04-28",
  "grower": "...",
  "sample_id": "...",
  "session_start": "2026-04-28T14:00:00.123456",
  "session_end":   "2026-04-28T18:30:00.456789",
  "total_entries": 24,
  "schema_version": "v2"
}
```

**Proposal:** add `schema_version` (currently absent) so downstream
consumers can branch on the layout. Bump it whenever any CSV header
changes.

---

## 4. Cross-stream consistency

Five different streams (`sensor_log`, `commit_log`, `auto_capture_events`,
`heartbeat_log`, `set_change_events`) all carry `timestamp` and
`elapsed_s`. They are *not* row-aligned — each has its own cadence.
Joining for analysis means matching on nearest `elapsed_s`.

`pyrometer_temp_C` appears in four of the five files. **Convention:** in
non-`sensor_log` files, it is a *snapshot of the most recent
sensor_log reading at the moment of the event*, not a fresh hardware
read. This keeps event-stream latency predictable (no extra serial
round-trip per event) and makes the value join-consistent.

---

## 5. Open questions for the meeting

1. **Per-event subdir vs flat naming?** Currently auto-capture buffer
   dumps into `frames/auto_event_NNN/`. Heartbeat and manual commits
   are flat in `frames/`. A flat layout is easier to glob; a subdir
   layout scales better to many events. Currently mixed — pick one.
2. **`schema_version` bookkeeping** — agree on a numbering scheme
   (v2 = current; v3 = whenever this proposal lands?) and a place
   to document it (this doc? a changelog?).
3. **Lab-wide adoption hooks.** When the schema is stable, future
   work (ML models, BO loops, dashboards) should consume *this
   schema*, not parse one-off log formats from individual sessions.
   Worth committing the schema definition into the repo (`docs/`)
   so external code can pin a version.
4. **Synology migration.** End-of-year goal is Synology becomes
   primary storage. Logger paths are configurable via the
   `base_dir` constructor arg — agree that downstream tooling
   should read from the configurable path, not hardcode
   `logs/growths/`.
5. **Failed/interrupted sessions.** Polybot's recommendation is to
   log failures with the same structure as successes (so the model
   can learn from what *didn't* work). Currently a hard-killed
   session leaves an incomplete directory with no `session_metadata.json`.
   Proposal: write a placeholder metadata file at session start
   (mark `incomplete: true`), update at clean stop.
6. **Recipe parameters.** Open Apr 10 follow-up #2: what defines
   a "recipe" for each growth stage? Should there be a `recipe.json`
   alongside `session_metadata.json` capturing intended setpoints
   vs. observed? Out of scope for this schema doc but adjacent.

---

## 6. Migration / compatibility notes

- The 2026-04-28 changes (multi-reading pyrometer columns,
  auto-capture buffer columns) are pure column *additions* —
  any reader that uses `csv.DictReader` will pick up the new columns
  transparently. Readers that rely on column *index* will need to
  be updated.
- No existing column has been renamed or removed. No reader breaks.
- Session directories from before today carry the older schema (no
  `pyrometer_temp_std_C`, no `buffer_count`/`buffer_dir`). Downstream
  tools should default missing columns to empty string.

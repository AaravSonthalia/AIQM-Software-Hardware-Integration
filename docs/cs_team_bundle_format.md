# AIQM dataset bundle — schema reference

**Audience:** CS-team collaborators consuming Growth Monitor session
data for classifier training, active-comparisons pipelines
(Yuxin's #1), and pseudo-labeling experiments (Chen 2025 style).

**Bundle producer:** `scripts/build_cs_dataset.py` in the AIQM repo.

**Bundle root layout:**

```
<bundle_name>/
  catalog.json              # index of all included sessions
  schema.md                 # this file
  README.md                 # one-page usage summary
  sessions/
    <session_id>/           # e.g. growth_STO-ideal_20260710_161714
      sensor_log.csv        # ~1 Hz sensor snapshot
      commit_log.csv        # grower LOG ENTRY commits
      auto_capture_events.csv  # auto-fired capture events
      manual_events.csv     # grower MARK EVENT clicks (Jul 10+)
      heartbeat_log.csv     # continuous-capture frames
      events_labels.csv     # per-event labels (Events tab)
      live_labels.csv       # Live Equalizer save labels (Jul 10+)
      set_change_events.csv # PSU / valve setpoint changes
      session_metadata.json # from GrowthLogger.start_session
      session_meta.json     # bundle-writer's summary + quality flags
      frame_manifest.json   # frame paths (if --include-frames not passed)
      frames/*.bmp          # RHEED frame BMPs (if --include-frames passed)
```

---

## 1. `catalog.json`

Top-level bundle index. Consume this first to filter which sessions
you want to load.

```json
{
  "generated_iso": "2026-07-14T15:30:00",
  "generator": "scripts/build_cs_dataset.py",
  "session_count": 16,
  "sessions": [
    {
      "session_id": "growth_STO-ideal_20260710_161714",
      "sample_id": "STO-ideal",
      "grower": "Jiangang",
      "start_iso": "2026-07-10T16:17:14",
      "duration_s": 2467.32,
      "duration_str": "41m 7s",
      "counts": {
        "sensor_rows": 250,
        "commit_rows": 8,
        "auto_capture_rows": 42,
        "manual_event_rows": 3,
        "heartbeat_rows": 249,
        "event_label_rows": 15
      },
      "quality_flags": ["real_growth", "has_labels", "long_duration",
                         "has_classifier_data"],
      "has_classifier_data": true,
      "has_grower_labels": true,
      "path_rel": "growth_STO-ideal_20260710_161714"
    }
  ]
}
```

Fields:

| Field | Type | Notes |
|---|---|---|
| `session_id` | str | Directory name; also the join key across CSVs |
| `sample_id` | str | Grower-entered sample tag from Session tab. Empty on legacy sessions |
| `grower` | str | Grower name from Session tab. Empty on legacy sessions |
| `start_iso` | str | Earliest timestamp seen in any CSV, ISO-8601 |
| `duration_s` | float | Last elapsed_s across sensor / heartbeat / commit |
| `duration_str` | str | Human "Nm Ns" render (informational) |
| `counts` | dict | Row counts per CSV — useful for filtering |
| `quality_flags` | list[str] | See §5 |
| `has_classifier_data` | bool | Any commit has non-empty `classifier_recon_1x1` |
| `has_grower_labels` | bool | Any of: events_labels rows, live_labels rows, `grower_corrected=True` commit |
| `path_rel` | str | Path relative to `sessions/` in the bundle |

---

## 2. Per-session CSVs

Every CSV uses `csv.DictReader`-compatible headers (comma-separated,
newline-terminated). Fields with no value are empty strings, not
`null`. All timestamps are naive ISO-8601 (no timezone; assume local
lab time / Chicago).

### 2.1 `sensor_log.csv`

Written every ~1 second by `SensorLogWriter`. See
`GrowthLogger.SENSOR_FIELDS` in `gui/growth_logger.py:35-49`.

| Column | Type | Meaning |
|---|---|---|
| `timestamp` | ISO-8601 | Write time |
| `elapsed_s` | float | Seconds since START |
| `pyrometer_temp_C` | float | Substrate temp from Exactus/Modbus/screengrab pyrometer |
| `pyrometer_temp_std_C` | float | 5-sample rolling std |
| `pyrometer_temp_n` | int | Rolling window size |
| `mistral_v_set_V` | float | MISTRAL PSU V setpoint (OCR from screengrab) |
| `mistral_v_actual_V` | float | MISTRAL PSU V actual |
| `mistral_i_set_A` | float | MISTRAL PSU I setpoint |
| `mistral_i_actual_A` | float | MISTRAL PSU I actual |
| `chamber_pressure_mbar` | float | Thyracont gauge via Moxa NPort |
| `substrate_temp_pv_C` | float | EvapControl `.elo` substrate thermocouple PV |
| `substrate_temp_setpoint_C` | float | EvapControl substrate SP |
| `cell_HTEC2_pv_C` | float | HTEC2 cell temp PV |
| `cell_Y_pv_C` | float | Y cell PV |
| `cell_Sr_pv_C` | float | Sr cell PV |
| `cell_Eu_pv_C` | float | Eu cell PV |
| `cell_Er_pv_C` | float | Er cell PV |
| `plasma_dc_bias_V` | float | Plasma source DC bias |
| `plasma_forward_W` | float | Plasma forward power |
| `plasma_reflected_W` | float | Plasma reflected power |

Elog-direct columns (`substrate_temp_*`, `cell_*`, `plasma_*`) are
blank in sessions where `evap_control_mode = "screengrab"` (they only
populate under `mode = "elog"`).

### 2.2 `commit_log.csv`

Written once per LOG ENTRY click. See
`GrowthLogger.COMMIT_FIELDS` in `gui/growth_logger.py:50-112`.

Key columns:

| Column | Type | Meaning |
|---|---|---|
| `timestamp`, `time_display`, `elapsed_s` | | Standard timing |
| `sample_id`, `grower` | str | Copied from Session tab |
| `pyrometer_temp_C`, `voltage_V`, `current_A` | float | Sensor snapshot at commit |
| `psu_source` | enum | `"mistral"` / `"direct"` / `"none"` — how V/I were sourced |
| `recon_1x1`, `recon_Twinned (2x1)`, `recon_c(6x2)`, `recon_rt13xrt13`, `recon_HTR` | float [0,100] | GROWER's slider values at commit. Sum to 100 iff `grower_corrected=True` |
| `classifier_recon_*` | float [0,100] | CLASSIFIER's smoothed_percent — one column per class. Always populated if classifier was OK for the session |
| `grower_corrected` | str "True"/"False"/"" | Was `✎ Correct` active? Empty = classifier disabled entirely |
| `classifier_status` | enum | `OK` / `LOADING` / `ERROR` / `DISABLED` at commit time |
| `note` | str | Grower's free-text note |
| `frame_path` | str | Absolute path to the LOG-ENTRY frame BMP (`frames/log_<n>.bmp`) |
| `frame_quality_pass` | str "True"/"False"/"" | Did the frame pass the black/saturation/uniform gate? |

**The `recon_*` / `classifier_recon_*` pairing is the primary
training signal for Yuxin's #1 active-comparisons pipeline** — for
every `grower_corrected=True` commit, the two vectors disagree, and
that disagreement is a labeled training example.

### 2.3 `auto_capture_events.csv`

Written when the auto-capture engine fires. See
`GrowthLogger.AUTO_CAPTURE_FIELDS` in `gui/growth_logger.py:113-118`.

| Column | Type | Meaning |
|---|---|---|
| `timestamp`, `elapsed_s`, `event_idx` | | Standard timing + integer event key |
| `change_score` | float | Trigger score (higher = more visual change) |
| `pyrometer_temp_C` | float | Sensor snapshot |
| `buffer_count` | int | Frames captured to the buffer at fire time |
| `buffer_dir` | str | Path to buffer BMPs (`frames/auto_event_<n>/`) |
| `event_state` | enum | `pending` / `kept_explicit` / `kept_default` / `discarded` / `auto_skipped` |
| `state_changed_at` | ISO-8601 | When the state transitioned |

### 2.4 `manual_events.csv` *(Jul 10+ sessions only)*

Written once per MARK EVENT click. See `MANUAL_EVENT_FIELDS`.

Sparse row: `timestamp`, `elapsed_s`, `event_idx`, `pyrometer_temp_C`,
`voltage_V`, `current_A`, `psu_source`, `frame_path`, `note`.

### 2.5 `heartbeat_log.csv`

Continuous-capture surface. One row per heartbeat frame (~5s cadence
configurable). See `HEARTBEAT_FIELDS`.

`timestamp`, `elapsed_s`, `heartbeat_idx`, `pyrometer_temp_C`,
`frame_path`.

### 2.6 `events_labels.csv`

Grower labels applied via Events tab (post-hoc). See
`EVENT_LABEL_FIELDS` in `gui/growth_logger.py:166-182`.

| Column | Type | Meaning |
|---|---|---|
| `event_idx` | int | Join key to `auto_capture_events.csv` / `manual_events.csv` |
| `primary_reconstruction` | enum | One of RECON_LABELS + `"unknown"` / `"artifact"` |
| `change_from`, `change_to` | enum | Transition labels (Jul 15+) — see §4 |
| `notes` | str | Grower free-text |
| `label_timestamp_iso` | ISO-8601 | When the label was applied |
| `recon_1x1`, `recon_tw`, `recon_c6x2`, `recon_rt13`, `recon_HTR` | float [0,1] | Equalizer mixture (May 19+, if grower used Label with Equalizer) |

### 2.7 `live_labels.csv` *(Jul 10+ sessions only)*

Live Equalizer tab save events. See `LIVE_LABEL_FIELDS`.

`timestamp`, `elapsed_s`, `label_idx`, `recon_1x1`, `recon_tw`,
`recon_c6x2`, `recon_rt13`, `recon_HTR`, `pyrometer_temp_C`,
`voltage_V`, `current_A`, `psu_source`, `frame_path`.

Slider values are in `[0, 1]`, not `[0, 100]` like commit_log —
the grower isn't required to Normalize before Save.

---

## 3. Frames

RHEED frames are 656×492 12-bit-mono BMPs from an Allied Vision Manta
G-033B (kSA k700-12) — see `memory/ref_rheed_camera_k700.md` in the
AIQM repo for the vendor spec.

**When `--include-frames` was passed to the bundler:** every
referenced BMP lives at `sessions/<id>/frames/`. Basenames come from
the original session (`log_<n>.bmp`, `auto_event_<n>_<n>.bmp`,
`heartbeat_<n>.bmp`, `manual_<n>.bmp`, `live_label_<n>_<HHMMSS>.bmp`).

**When it was NOT passed:** each session dir carries
`frame_manifest.json` mapping the CSV frame_path references back to
their locations in the source repo. CS-team consumers with
filesystem access to the AIQM `logs/growths/` tree can resolve paths
themselves.

---

## 4. Reconstruction class labels

The five canonical class strings (defined in
`gui/recon_labels.py`):

```
["1x1", "Twinned (2x1)", "c(6x2)", "rt13xrt13", "HTR"]
```

Notes on conventions:
- ASCII `x` (not Unicode `×`)
- lowercase `rt13` (not `RT13`)
- `rt13xrt13` = √13 × √13
- Full `Twinned (2x1)` (not the internal Equalizer's `Tw(2x1)`)

**`live_labels.csv` uses shorter keys:** `recon_1x1`, `recon_tw`,
`recon_c6x2`, `recon_rt13`, `recon_HTR`. Same 5 classes, different
column-name convention — join by class name manually if you're
merging with `commit_log.csv`.

**`change_from` / `change_to`** in `events_labels.csv` capture
grower-annotated reconstruction transitions ("this frame was 1x1
becoming Twinned (2x1)"). Values are canonical class strings, or
`RECON_UNLABELED` (empty) if the grower didn't annotate the
transition. These feed Yuxin's #1 active-comparisons pipeline as
labeled transition examples.

---

## 5. Quality flags

Applied by `assess_quality()` in `scripts/build_cs_dataset.py`.

| Flag | Meaning | Threshold |
|---|---|---|
| `empty` | No sensor readings, no events | `sensor_rows==0` AND no grower interaction |
| `dummy` | Sensor readings look synthetic | All pyrometer readings within 0.5°C (DummyPyrometer emits fixed values) |
| `startup_test` | Short session with no engagement | Duration < 2 min AND no grower interaction |
| `real_growth` | Grew for a while OR grower interacted | Duration ≥ 2 min OR any commit/manual/auto-capture/label |
| `long_duration` | Sustained session | Duration ≥ 10 min |
| `has_labels` | Grower produced any reconstruction label | Any events_labels rows OR live_labels rows OR `grower_corrected=True` commits |
| `has_classifier_data` | Classifier ran successfully | Any commit has non-empty `classifier_recon_1x1` |

Flags are **tags, not filters** — every session gets zero or more,
and the CS team decides which combinations mean "include" for their
pipeline. `real_growth + has_labels` is a strong training-data
combination; `dummy + startup_test` is what you probably want to
exclude.

---

## 6. Common join patterns

**Labeled frames for classifier training:**

```python
# Frame + primary reconstruction label per auto-capture event
labels = read_csv("events_labels.csv")
events = read_csv("auto_capture_events.csv")
frames_by_event = {}  # event_idx → list of BMP paths
for e in events:
    frames_by_event[e["event_idx"]] = sorted(
        glob(f"{e['buffer_dir']}/*.bmp")
    )

for label in labels:
    for frame_path in frames_by_event.get(label["event_idx"], []):
        yield frame_path, label["primary_reconstruction"]
```

**Grower-vs-classifier disagreement (Yuxin's #1):**

```python
# Every grower_corrected=True commit is one training example
for commit in read_csv("commit_log.csv"):
    if commit["grower_corrected"] != "True":
        continue
    grower_vec = [commit[f"recon_{c}"] for c in RECON_LABELS]
    classifier_vec = [commit[f"classifier_recon_{c}"] for c in RECON_LABELS]
    yield commit["frame_path"], grower_vec, classifier_vec
```

**Reconstruction transitions for active-comparisons:**

```python
# change_from → change_to pairs
for label in read_csv("events_labels.csv"):
    if not label["change_from"] or not label["change_to"]:
        continue
    yield label["event_idx"], label["change_from"], label["change_to"]
```

---

## 7. Loader stub — PyTorch `Dataset`

```python
import csv, json
from pathlib import Path
from PIL import Image
import torch
from torch.utils.data import Dataset

RECON_LABELS = ["1x1", "Twinned (2x1)", "c(6x2)", "rt13xrt13", "HTR"]

class AIQMLabeledFrameDataset(Dataset):
    """Frame + primary reconstruction label from an AIQM bundle."""
    def __init__(self, bundle_root, quality_include=("real_growth", "has_labels")):
        self.root = Path(bundle_root)
        catalog = json.loads((self.root / "catalog.json").read_text())
        self.pairs = []
        for entry in catalog["sessions"]:
            if not all(f in entry["quality_flags"] for f in quality_include):
                continue
            session_dir = self.root / "sessions" / entry["session_id"]
            self._collect_pairs(session_dir, entry)

    def _collect_pairs(self, session_dir, entry):
        labels_path = session_dir / "events_labels.csv"
        if not labels_path.exists():
            return
        events_path = session_dir / "auto_capture_events.csv"
        events_by_idx = {}
        with events_path.open() as f:
            for row in csv.DictReader(f):
                events_by_idx[row["event_idx"]] = row
        with labels_path.open() as f:
            for label in csv.DictReader(f):
                if not label["primary_reconstruction"]:
                    continue
                event = events_by_idx.get(label["event_idx"])
                if not event:
                    continue
                buffer_dir = Path(event["buffer_dir"])
                # If --include-frames was passed, frames live under
                # session_dir / "frames". Else use frame_manifest.json
                # to resolve to the source repo.
                for bmp in sorted(buffer_dir.glob("*.bmp")):
                    class_idx = RECON_LABELS.index(label["primary_reconstruction"])
                    self.pairs.append((bmp, class_idx))

    def __len__(self):
        return len(self.pairs)

    def __getitem__(self, idx):
        path, class_idx = self.pairs[idx]
        img = Image.open(path).convert("L")  # 12-bit mono
        return torch.tensor(list(img.getdata()), dtype=torch.float32), class_idx
```

Substitute your preferred transforms; the above just reads raw
grayscale bytes into a flat tensor so you can see the pipe.

---

## 8. Change log

| Bundle date | Change |
|---|---|
| Jul 14 2026 | v1 — initial CS-team schema doc, matches AIQM commit `9125c9d` |

For newer schema additions, see `gui/growth_logger.py`'s FIELDS
lists — they're the source of truth. This doc is regenerated from
those on demand.

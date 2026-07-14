# RHEED Equalizer labeling — grower SOP

**Audience:** Growers using the AIQM Growth Monitor to label RHEED
reconstructions during (or after) an OMBE growth.

**Scope:** The three Equalizer surfaces + how to choose between them.

**Last updated:** Jul 14 2026 (v1 — Live Equalizer tab pause button
shipped).

---

## The three labeling surfaces

The Growth Monitor exposes three distinct places to label a RHEED
reconstruction. They cover different moments in the workflow:

| Surface | When to use | What it writes |
|---------|-------------|----------------|
| **Live Equalizer tab** | Live growth, mid-session, seeing an interesting reconstruction unfold | `live_labels.csv` + `live_label_NNN_*.bmp` per Save |
| **Retrospective Equalizer** (Events tab → "Label with Equalizer…") | Auto-capture buffer review after the moment passed | `events_labels.csv` row keyed by `event_idx` |
| **Monitor tab sliders** (LOG ENTRY with grower-correction toggle) | Deliberate log-entry moment; typing a note anyway; one row per commit | `commit_log.csv` `recon_*` + `classifier_recon_*` paired columns |

Choose the surface by asking **when the labeling happens relative to
the moment being labeled**:

- **During:** Live Equalizer (real-time slider game against the live
  frame)
- **Just after:** Monitor tab LOG ENTRY (slider snapshot + note in one
  atomic commit)
- **Post-hoc:** Events tab → "Label with Equalizer…" against the
  auto-capture buffer frames

All three write different CSV files so you never overwrite one path's
label with another's. Downstream ML consumers join across files on
timestamp / `event_idx` as needed.

---

## Live Equalizer tab — walkthrough

Location: `Live Equalizer` tab (between `Scrubber` and `Session`).

**The layout (2×2):**

- **Top-left — Selected (live RHEED):** the current camera frame,
  downsampled + rendered in the same phosphor-green palette as the
  Monitor tab RHEED display.
- **Top-right — Constructed (grower blend):** what the current slider
  mix would look like as a reconstruction. Updates live as you drag.
- **Bottom-left — Classifier %:** what Classifier2 currently thinks
  (5 reconstruction classes, same values shown on the Monitor tab
  sliders).
- **Bottom-right — Grower %:** your 5 sliders + Auto-fit / Normalize /
  Reset / Freeze frame / Save label buttons.

**Workflow:**

1. **Wait for a session to be running.** The Save button stays disabled
   in idle / armed so nothing writes to disk until the logger's
   `live_labels.csv` is open.
2. **Watch the Selected pane.** Frames arrive at the camera worker's
   rate (typically 1-2 Hz).
3. **Click Auto-fit** as a starting point. Least-squares fit of the
   5-class basis onto the current Selected image. Usually gets you
   within 10-20% of the "right" answer per class.
4. **Refine by dragging sliders.** Watch the Constructed pane rebuild
   as you drag. Goal: Constructed matches Selected.
5. *(Optional)* **Click Normalize** to rescale sliders to sum to 100%.
   Purely a display convenience — the ratio is what matters for
   downstream training, not the absolute magnitudes.
6. **Click Save label.** Snapshots the current live frame + writes a
   row to `live_labels.csv` with your slider weights + sensor state
   (pyro / V / I / psu_source).

---

## Freeze frame — the pause pattern (Jul 14 2026)

**Problem it solves:** the live camera stream updates every ~500 ms.
If you're mid-drag and the frame changes underneath you, the
Constructed pane's target has moved. You end up chasing.

**How it works:**

- Click **Freeze frame** — the Selected pane stops updating. The
  cached frame is preserved.
- The button turns amber and its label switches to **Resume live** —
  a reminder that pressing it now will unfreeze.
- While frozen, Auto-fit / slider drags / Save all operate on the
  frozen frame. `live_labels.csv` gets a row keyed to the frozen
  frame's timestamp.
- Click **Resume live** to unfreeze; the Selected pane starts updating
  again with the next incoming frame.

**When to freeze:**

- You want to label a specific reconstruction and the growth is moving
  faster than your slider work
- You want to compare multiple slider mixtures against a single stable
  target

**When NOT to freeze:**

- You want to catch a transition as it happens — leave live so
  Selected keeps updating. Save right after the transition and the
  timestamp will match reality.

---

## Auto-fit vs manual sliders

**Auto-fit** does a least-squares fit of the Selected image onto the
5-class basis, then clips negatives to zero and normalizes. It's a
reasonable starting point but has known limitations:

- **Basis limits:** the basis is 5 class-mean images. Real frames
  contain features (contamination, over-flash, out-of-frame beam) not
  well-represented in the basis. Auto-fit distributes those artifacts
  across the 5 classes somewhat arbitrarily.
- **Non-uniqueness:** several mixtures can produce visually similar
  Constructed images. Auto-fit picks one; you may prefer another.

Treat Auto-fit as a first-pass suggestion, not ground truth. If the
Constructed pane doesn't match the Selected pane after Auto-fit,
adjust manually.

---

## Save timing

**When you click Save, this is what happens:**

1. Current slider weights are read (as fractions in [0, 1])
2. Current sensor snapshot is captured: pyro temp, V, I, psu_source,
   elapsed_s
3. Current live frame (or frozen frame if paused) is snapshotted to
   `live_label_NNN_HHMMSS.bmp` under the session's `frames/`
   subdirectory
4. A row is appended to `live_labels.csv` with the timestamp + all of
   the above

**Idempotency:** every Save is a new row. Clicking Save five times in
30 seconds gives you five rows with five distinct `label_idx` values.
There's no "update the last save" — every click is a new event.

**Verify a save landed:**
- Status bar shows `Live label #N saved`
- Session tab → open `logs/growths/<session_id>/live_labels.csv` and
  the newest row should have your weights

---

## CSV schema — `live_labels.csv`

Written by `GrowthLogger.record_live_label`. Full schema in
`gui/growth_logger.py::LIVE_LABEL_FIELDS`.

Columns:

- `timestamp`, `elapsed_s`, `label_idx` — identity
- `recon_1x1`, `recon_tw`, `recon_c6x2`, `recon_rt13`, `recon_HTR` —
  slider weights as floats in [0, 1]
- `pyrometer_temp_C`, `voltage_V`, `current_A`, `psu_source` — sensor
  snapshot
- `frame_path` — path to the BMP snapshot

For downstream training: join on `timestamp` with
`heartbeat_log.csv` (continuous capture) or `sensor_log.csv` (fuller
sensor trajectory) for context around the labeled moment.

---

## Reconstruction transition labeling (Jul 15 2026)

**Location:** Events tab → labeling form → "Change (from → to):" row
(two dropdowns with an arrow between them).

**What it captures:** the moment a growth transitions between two
reconstructions. Single-class labeling (via "Primary reconstruction")
answers *what is this frame?*; transition labeling answers *what did
this frame CHANGE from and to?* The two coexist — you can set
primary_reconstruction *and* change_from/change_to on the same event,
and downstream analysis reads whichever it needs.

**Workflow:**

1. In the Events tab, select an auto-capture event you believe
   captured a reconstruction transition
2. In the labeling form, set "Change (from):" to the reconstruction
   you saw before the event, and "Change (to):" to the one you saw
   after
3. Selection is atomic-per-dropdown: each change writes to
   `events_labels.csv` immediately (no separate Save button)
4. Both default to "(unlabeled)" — leave them there if the event
   isn't a transition (grower didn't see a clear before/after)

**Which fields to use:**

- **Just primary_reconstruction:** steady-state frame ("this event
  captured a good example of 1x1")
- **Just change_from / change_to:** transition frame ("this event
  captured 1x1 → Twinned")
- **All three:** both signals for the same frame (rare but legit —
  useful when the frame is at the crest of a transition and the
  primary label captures the dominant class)
- **None of them:** event doesn't need a label (mislabeled auto-
  capture, artifact, out-of-frame beam). Discard it via the banner
  instead of leaving the labels blank.

**Downstream ML tie-in:** the change_from / change_to columns are the
primary signal for Yuxin's #1 active-comparisons pipeline, which
trains a model to discriminate transition frames from steady-state
frames. High-quality transition labels are more scarce than
steady-state ones, so growers filling these dropdowns during a
labeling session directly increases the pool of transition-training
data.

---

## Common pitfalls

1. **Save button greyed out:** the session must be running. Arm →
   Start on the Session tab first.
2. **Constructed pane empty:** the basis images failed to load. Check
   `data/equalizer_class_means.npz` is present and readable. See the
   Constructed pane's placeholder text for the specific error.
3. **Sliders won't sum to exactly 100:** they don't have to. Save
   accepts any weights; downstream normalizes at read time. Click
   Normalize if you want the sum locked to 100 for a specific row.
4. **Camera stream too fast to label:** click Freeze frame. Or drop
   the camera worker's trigger rate on the Config tab (Session tab
   config) before arming.
5. **Auto-fit gives weird numbers on a bad frame:** don't Save. Wait
   for a better frame, or freeze on a good one, then Auto-fit.

---

## Cross-references

- `gui/live_equalizer_tab.py` — implementation
- `gui/growth_logger.py::LIVE_LABEL_FIELDS, record_live_label` —
  schema + writer
- `scripts/equalizer_ui.py` — shared basis-loading + reconstruction
  math (used by both the Live Equalizer tab and the retrospective
  Events-tab launcher)
- `scripts/test_live_equalizer_tab.py` — tests

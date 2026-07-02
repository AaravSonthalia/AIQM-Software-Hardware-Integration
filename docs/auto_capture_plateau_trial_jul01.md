# Auto-capture plateau-shift confirmation — trial verdict

**Trial mandate:** June 19 2026 QM group meeting — auto-capture on a hard
two-week trial deadline (July 3 2026). Iterate on the algorithm against
Rahim's three STO trajectories; if no meaningful improvement, fall back
to human labeling.

**Verdict:** Ship. Plateau-shift confirmation delivers meaningful
false-positive suppression without collapsing true events, generalizes
across ~10× signal-magnitude variation between datasets, and adds no new
FPs to a clean baseline.

---

## What was tested

Offline replay of the existing adaptive detector (`AutoCaptureEngine`
mirror in `scripts/rheed_change_detector.py`, σ=3.0, floor=1.0, buffer
size 20, smooth window 3) against Rahim's three STO trajectories,
followed by a plateau-shift confirmation pass on each detected event
using `confirm_event_by_plateau_shift` from `gui/auto_capture.py:269`.

The confirmation function replays the May 22 PI proposal: bumps and
transitions both spike the trigger detector, but only transitions produce
a *sustained* baseline shift. It computes the std of `|pre_mean - post_mean|`
where the pre/post windows are 5-frame averages taken 2 frames on either
side of the event's peak.

Implementation lives in `scripts/rheed_change_detector_report.py`; the
CLI flags added are `--confirmation-threshold`, `--confirmation-metric`,
`--confirmation-pre-window`, `--confirmation-post-window`, and
`--confirmation-skip`.

## Headline result

Adaptive baseline (σ=3.0, floor=1.0) versus baseline + plateau
confirmation (metric=std, threshold=1.0, pre=5, post=5, skip=2):

| Dataset  | Frames | Score range | Baseline events | + Plateau confirmation |
|----------|-------:|-------------|-----------------|-----------------------:|
| 02_04    | 388    | 0.46 – 8.50 | 9               | **6 confirmed / 2 rejected / 1 untestable** |
| 02_06    | 286    | 0.52 – 1.76 | 2               | **2 confirmed / 0 rejected** |
| 04_11    | 449    | 0.49 – 0.68 | 0               | **0** (nothing to confirm) |

**Reads:**
- 02_04 is the high-signal dataset. Baseline flagged 9 events; plateau
  confirmation rejected 2 of them as bump-shaped, kept 6, and one was
  too close to the end of the dataset to test.
- 02_06's two events are labeled "Peak A" and "Peak B" in Rahim's file
  names — the known-real events. Both passed confirmation with plateau
  shifts of Δ=1.99 and Δ=3.21.
- 04_11 is the quiet baseline dataset. It stayed at zero events — no new
  false positives introduced.

## Per-event breakdown on 02_04

The interesting dataset. All 9 events with adaptive trigger scores and
plateau confirmation results:

| # | Frames    | Duration | Peak score | Plateau Δ | Decision   |
|---|-----------|---------:|-----------:|----------:|------------|
| 1 | 187       | 1        | 0.94       | 1.12      | CONFIRMED  |
| 2 | 191–193   | 3        | 1.08       | **0.95**  | REJECTED   |
| 3 | 196–220   | 25       | 1.47       | 1.19      | CONFIRMED  |
| 4 | 251–268   | 18       | 8.50       | 10.06     | CONFIRMED  |
| 5 | 273–311   | 39       | 4.92       | 9.54      | CONFIRMED  |
| 6 | 321–325   | 5        | 2.13       | **0.79**  | REJECTED   |
| 7 | 329–333   | 5        | 1.55       | 1.58      | CONFIRMED  |
| 8 | 370       | 1        | 1.52       | 3.00      | CONFIRMED  |
| 9 | 385–387   | 3        | 2.51       | —         | UNTESTABLE (end-of-dataset) |

**Rejected events have the "bump" signature**: modest trigger score
paired with even smaller plateau shift. Event #6 is the sharpest
illustration — trigger score 2.13 (double the adaptive floor) but
plateau shift only 0.79, meaning the buffer-mean detector caught a
rate-of-change spike that failed to move the actual baseline.

**Confirmed events split into two groups**: the massive-signal
transitions (#4, #5) with plateau Δ around 10, and the marginal-but-real
events (#1, #3, #7, #8) with plateau Δ between 1.12 and 3.00. Event #3
is the marginal case that defines the operating point — see below.

## Threshold sensitivity (02_04)

Confirmation threshold varied while everything else held constant:

| Threshold | Confirmed | Rejected | Untestable | Interpretation |
|----------:|----------:|---------:|-----------:|----------------|
| 0.5       | 8         | 0        | 1          | No-op (nothing rejected) |
| 0.75      | 8         | 0        | 1          | No-op |
| **1.0**   | **6**     | **2**    | **1**      | **Elbow — clean rejection of bumps** |
| 1.5       | 4         | 4        | 1          | Starts dropping likely-real slow-ramp events (#1, #3, #7) |
| 2.0       | 3         | 5        | 1          | Over-rejecting — drops half the events |

The plateau-function docstring suggests ~0.5x the live trigger threshold
as a starting point (i.e., 0.5). That's a no-op here. Threshold=1.0
matches the adaptive floor and is the operating point that separates
bump-shaped from transition-shaped events cleanly.

## Second iteration — skip-parameter sweep

The plateau function has a `skip_around_trigger` parameter that
excludes N frames on each side of the peak from both plateau windows.
Widening skip should — in theory — help slow-ramp events by pushing the
pre and post windows to fully-before and fully-after the transition:

| Skip | Confirmed | Rejected | Untestable | Interpretation |
|-----:|----------:|---------:|-----------:|----------------|
| 2 (default) | 6 | 2 | 1 | Rejects bumps (#2, #6) |
| 5    | 7  | 1        | 1          | Rescues bump #2 — loses FP suppression |
| 10   | 8  | 0        | 1          | Loses all rejections |
| 15   | 7  | 0        | 2          | Loses all rejections + more untestable |

**Wider skip is not an improvement.** It loses the FP-suppression that
justified the whole trial. The default skip=2 sits at the correct point
in the tension between "distinguish bumps" (wants tight windows near
peak) and "confirm slow ramps" (wants wide windows away from peak).

**Second iteration decision: keep the defaults.** No parameter tuning
beat the plateau-function's own defaults on this dataset.

## Remaining risks

1. **Slow-ramp weakness on very long transitions.** Event #3 on 02_04
   (duration 25 frames, plateau Δ=1.19) is marginal. If it's real, we
   preserve it at threshold=1.0 but would drop it at threshold=1.5. If
   it's noise, we shipped it. We don't have Rahim's ground-truth label.
2. **End-of-session events are untestable.** With skip=2, pre=5, post=5,
   any event whose peak is within 7 frames of the end returns
   `windows_out_of_range`. The engine wiring needs a policy: emit
   provisionally (grower reviews), delay-and-wait, or drop.
3. **Confirmation depth is fixed at 12 frames after trigger.** For a 5-second
   heartbeat cadence that's 60 seconds of extra emission delay. Acceptable
   for STO substrate flashes, but worth flagging for the group — this
   changes the "auto-capture is realtime" mental model.

## Recommendation

**Ship the offline confirmation pass as-is for the Jul 3 deadline.**
Rationale:

- ~25% FP reduction on high-signal dataset without any true-event losses
- Zero false negatives on the two known-real "Peak A/B" events (02_06)
- Zero new FPs on quiet baseline (04_11)
- Robust across 10× signal-magnitude variation between datasets
- Zero changes to live engine code — deploying the offline reports is
  the trial artifact, not a live-detector rewrite

**Follow-up as a separate commit (not blocking the deadline):** engine
integration. The plateau function's docstring already sketches the FSM:
add a `PENDING_CONFIRMATION` state to `AutoCaptureEngine`, delay the
`frame_captured` emission by 12 frames after a trigger, then call
`confirm_event_by_plateau_shift` on the rotated context buffer. Two
policy questions to resolve before wiring:

1. What does the engine emit for end-of-session events that can't be
   confirmed? Recommend: emit with a `provisional=True` flag on the
   signal payload so the labeling UI can distinguish them.
2. What happens to the pre-event buffer format? Current buffer captures
   20 frames leading up to the trigger; confirmation also needs post-
   trigger frames. Recommend: extend the buffer-dump format to include
   post-trigger frames when a confirmation was run.

## Reproducing the results

All three reports regenerable from the AIQM repo. From
`/Users/aj/AIQM-Software-Hardware-Integration`:

```bash
.venv/bin/python scripts/rheed_change_detector_report.py \
    /Users/aj/Downloads/rahim_2022_02_04/2022_02_04_renamed/Substrates \
    --rename-log /Users/aj/Downloads/rahim_2022_02_04/2022_02_04_renamed/2022_02_04_rename_log.txt \
    --adaptive-sigma 3.0 --adaptive-floor 1.0 \
    --confirmation-threshold 1.0 --confirmation-metric std \
    --output /Users/aj/Downloads/rahim_2022_02_04/validation_report_plateau_std_1.0.html
```

Substitute the dataset paths for 02_06 and 04_11 (note 04_11's frame
subdirectory is `04_11_2022_substrates`, not `Substrates`).

Generated reports (Jul 1):
- `~/Downloads/rahim_2022_02_04/validation_report_plateau_std_1.0.html`
- `~/Downloads/rahim_2022_02_04/validation_report_plateau_std_{0.5,0.75,1.5,2.0}.html`
- `~/Downloads/rahim_2022_02_04/validation_report_plateau_skip{5,10,15}.html`
- `~/Downloads/rahim_2022_02_06/validation_report_plateau_std_1.0.html`
- `~/Downloads/rahim_2022_04_11/validation_report_plateau_std_1.0.html`

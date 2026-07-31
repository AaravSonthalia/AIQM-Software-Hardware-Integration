# RHEED Equalizer Labeling - Grower SOP

## Scope

Use this procedure to label RHEED reconstruction mixtures during a growth or
from an auto-capture event. Equalizer alignment maps the simulator basis into
the camera orientation; it does not align the physical gun, change classifier
output, or determine film quality.

## Labeling Surfaces

| Surface | Use | Output |
|---|---|---|
| **Live Equalizer** | Label the current or deliberately frozen RHEED frame | `live_labels.csv` and one BMP per save |
| **Events -> Label with Equalizer...** | Label one selected auto-capture frame | `events_labels.csv`, keyed by `event_idx` |
| **Monitor sliders + LOG ENTRY** | Record the existing classifier/grower correction with a note | `commit_log.csv` |

Only Live and Events use the camera-aligned Equalizer workflow described
below. The Monitor sliders remain a separate logging surface.

## Basis and Class Policy

The canonical coordinate system is the simulator's 128 x 96 orientation. The
active Equalizer classes are `1x1`, `Tw(2x1)`, `c(6x2)`, and `RT13`. Their
source transforms are identity; an accepted camera calibration supplies the
single geometric transform used for comparison and fitting.

`HTR` is disabled and displayed as **N/A - canonical basis pending**. Auto-fit
excludes it, and `recon_HTR` is saved as an empty CSV value, never a synthetic
zero.

## Live Workflow

1. Start a logging session. Confirm that RHEED is connected, the physical gun
   is aligned, no realignment is active, and the displayed frame is fresh.
2. Select **Calibrate**. This copies one immutable calibration snapshot,
   including image, WGC sequence/time, HWND/backend, dimensions, session,
   `view_segment_id`, and `visual_history_generation`. Acquisition may
   continue, but it cannot change this snapshot.
3. If automatic 1x1 landmark detection succeeds, inspect its candidates. If
   it fails, click the left, centre, and right 1x1 points on the frozen image.
   Do not accept low-signal, collinear, or incorrectly ordered points.
4. Review the normal and mirrored candidates. Green is the camera snapshot;
   magenta is the warped basis chosen in **Overlay basis**. Switch among
   `1x1`, `Tw(2x1)`, `c(6x2)`, and `RT13`, then inspect the three points,
   residual vectors, rotation, scale, parity, and valid coverage.
5. Select the candidate that matches both the landmarks and the observed
   handedness. Choose the naturally present asymmetric class (or a clear
   streak/tail) in **Handedness evidence**, check the explicit confirmation,
   then choose **Request acceptance**. The request stays disabled until both
   evidence fields are complete. Acceptance is always a grower decision;
   candidate ranking never accepts automatically.
6. Use **Auto-fit** as a four-class starting point, then refine the sliders.
   **Normalize** rescales the four active weights to sum to one.
7. Optionally select **Freeze frame** to hold one labeling target. Calibration
   and label freezes are independent: Calibrate freezes its own source frame;
   Freeze frame selects which current snapshot Auto-fit and Save use.
8. Select **Save label**. The application rechecks session, calibration,
   physical alignment, capture continuity, and frame age before any image or
   CSV row is written.

Every Save creates a new `label_idx`; it does not overwrite an earlier label.

## Calibration Acceptance and Lifecycle

`GrowthApp` is the sole owner of an accepted calibration. The Equalizer tab
only proposes a candidate. Accepted and invalidated records are appended to
`equalizer_calibrations.jsonl`; Live and Events resolve calibration IDs from
that same journal.

Auto-fit and Save fail closed until a compatible calibration is accepted. A
session reset, gun realignment, camera disconnect/reconnect, backend or HWND
change, resolution/ROI/DPI change, visual-history generation change, or basis
hash change invalidates it immediately. The same discontinuities cancel a
candidate still under review, so moving away and back cannot revive it.
Reconnect or start a new session, then
calibrate and accept again; never copy an old matrix into a new context.

Three nearly symmetric 1x1 points cannot establish handedness. During initial
Bulbasaur commissioning, confirm normal versus mirrored using a naturally
asymmetric Tw/c(6x2)/RT13 frame or a clear streak/tail. On acceptance the
application saves the immutable calibration snapshot and records the evidence
kind, image path, and SHA-256 as one required provenance unit.

## Events Workflow

1. Select an event and the exact buffer frame to label.
2. Choose **Label with Equalizer...**. The frame must have an unambiguous row in
   that event's `capture_manifest.csv` with capture, session, view-segment,
   visual-history, and gun-alignment provenance.
3. Events first checks the calibration ID already saved for that exact frame
   in `events_labels.csv`, then the manifest ID, and resolves it from the
   append-only calibration history. Otherwise calibrate and accept against
   this exact event frame before fitting or saving.
4. Adjust the four active sliders and save. The Equalizer update references
   the already captured frame and is written to the existing
   `events_labels.csv` row for that `event_idx`; the logger never creates an
   untracked fallback image.

Legacy buffers without a complete manifest, stale/incompatible calibration
IDs, ambiguous filenames, or invalid QC context are rejected for Equalizer
labeling. The application must not silently use raw or previously aligned
basis images. Ordinary event notes and reconstruction dropdowns remain
available separately, including `change_from` and `change_to` for transitions.

## Saved Provenance

Live and Events rows retain the reconstruction fields and append:

- `calibration_id`, `basis_bundle_id`, and `equalizer_active_classes`;
- `view_segment_id` and `visual_history_generation`;
- `capture_backend`, `captured_at_utc`, `capture_sequence`, `frame_age_ms`,
  `source_hwnd`, and `capture_geometry_id`;
- the saved or selected `frame_path`.

The label snapshot is an immutable image-plus-metadata unit. This prevents the
image from one callback being combined with the timestamp or sequence of
another. Capture timestamps are validated as explicit UTC ISO-8601 values.
The calibration journal additionally stores the full matrix, parity,
landmarks, residuals, basis hash, the complete basis manifest once per bundle,
evidence kind/path/SHA-256, acceptance, and
invalidation reason.

Live label image/CSV writes and calibration evidence/journal writes are
recoverable transactions. If Windows, Python, or workstation power stops
between commits, the next logger startup uses the pending WAL to complete the
exact committed pair or remove the uncommitted image. Malformed, conflicting,
or journal-untrusted state remains fail-closed. Never delete an unresolved WAL
by hand; preserve it for diagnosis.

## Disabled Legacy Entry Point

`scripts/equalizer_ui.py` remains importable for canonical basis loading,
display palettes, and fitting helpers. Its legacy standalone Save path is
disabled, and direct execution returns a non-zero status with instructions to
use Growth Monitor. Create labels only through the Live or Events workflow so
camera alignment, QC, and capture provenance are enforced.

## Common Fail-Closed Messages

- **Save disabled:** start a session and accept a calibration on a fresh frame.
- **Automatic landmarks failed:** use manual left-centre-right selection; do
  not invent fallback points.
- **Candidate cannot be accepted:** correct its residual, scale, coverage, or
  point-order failure, then select and explicitly confirm asymmetric evidence.
- **Calibration invalidated:** restore camera/QC continuity and recalibrate.
- **Event unavailable to Equalizer:** inspect `capture_manifest.csv`; do not
  bypass missing provenance.
- **HTR cannot be moved:** expected until a canonical, view-provenanced HTR
  basis is commissioned.

## References

- `gui/equalizer_alignment.py` - canonical basis, snapshots, candidates, and
  calibration records
- `gui/live_equalizer_tab.py` - Live and retrospective Equalizer component
- `gui/growth_logger.py` - CSV schemas and calibration journal
- `docs/equalizer_camera_alignment.md` - algorithm and acceptance overview
- `docs/validation/ombe_equalizer_alignment_template.md` - Bulbasaur report

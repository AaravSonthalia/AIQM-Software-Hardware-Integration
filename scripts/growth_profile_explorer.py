#!/usr/bin/env python3
"""Growth Profile Explorer — post-hoc session analysis.

Reads a Growth Monitor session directory and renders diagnostic charts
that would otherwise require manual CSV wrangling. Day 5 (Jul 17 2026)
ships 5 panels total — Day 4 shipped the first two, Day 5 adds three:

  - ``temperature_profile_annotated.png`` — T vs t with commit / manual /
    auto-capture event overlays  [Day 4]
  - ``pyro_stability.png`` — pyrometer mean ± std over time  [Day 4]
  - ``classifier_trajectory.png`` — 5 lines, classifier smoothed_percent
    per class over the session  [Day 5]
  - ``score_distribution.png`` — auto-capture change_score histogram
    with threshold reference + per-state count summary  [Day 5]
  - ``grower_vs_classifier.png`` — 5-panel scatter of grower slider %
    vs classifier % per LOG ENTRY, one panel per class  [Day 5]

Day 6 (Jul 20) will wrap all five into an HTML report with base64-
embedded PNGs + session-metadata header.

Usage:
    python scripts/growth_profile_explorer.py \\
        logs/growths/<session_dir>/ \\
        --output-dir logs/growths/<session_dir>/analysis/

Consumed CSVs (all optional except ``sensor_log.csv`` — missing files
yield empty row lists so the tool works against pre-Jul-10 sessions
that predate ``manual_events.csv`` / ``events_labels.csv`` /
``live_labels.csv``):

    sensor_log.csv          — GrowthLogger.SENSOR_FIELDS       (always)
    commit_log.csv          — GrowthLogger.COMMIT_FIELDS       (always)
    auto_capture_events.csv — GrowthLogger.AUTO_CAPTURE_FIELDS (always)
    manual_events.csv       — GrowthLogger.MANUAL_EVENT_FIELDS (Jul 10+)
    heartbeat_log.csv       — GrowthLogger.HEARTBEAT_FIELDS    (always)
    events_labels.csv       — GrowthLogger.EVENT_LABEL_FIELDS  (post-label)

See ``gui/growth_logger.py`` for schema definitions and
``gui/recon_labels.py`` for canonical class labels.
"""
from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# gui.recon_labels is PyQt-free (pure module) so importing it here
# doesn't pull PyQt6 into an analysis-only script. Using the canonical
# label list (not a re-declared local copy) means if the class set
# ever changes, both the GUI and the analysis tool update in lockstep.
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from gui.recon_labels import RECON_LABELS  # noqa: E402


# Event marker colors on temperature charts. First three keys mirror
# gui/scrubber_tab.py:_SOURCE_COLORS (heartbeat cyan / manual amber /
# auto grey) so the visual identity carries from the live scrubber into
# post-hoc charts. The fourth key ("commit") is chart-scope only — LOG
# ENTRY events don't need their own scrubber marker but deserve a
# distinct color here. Duplicated (not imported from gui.scrubber_tab)
# to keep this analysis script free of PyQt6 imports; if the scrubber's
# three colors change, update both places.
EVENT_MARKER_COLORS = {
    "heartbeat": "#0891b2",  # cyan — continuous capture
    "manual":    "#d97706",  # amber — MARK EVENT
    "auto":      "#888888",  # grey — auto-capture engine
    "commit":    "#059669",  # emerald — LOG ENTRY (chart-only)
}

# Per-class line colors for the classifier-trajectory + grower-vs-
# classifier charts. Deliberately DIFFERENT from EVENT_MARKER_COLORS so
# the reader doesn't conflate "which source fired this event" with
# "which class did the classifier assign." Sourced from matplotlib's
# tab10 qualitative palette — well-tested for perceptual distinction
# across 5+ categories. Keyed by RECON_LABELS to prevent drift if the
# canonical label set ever changes.
CLASS_LINE_COLORS: dict[str, str] = {
    "1x1":            "#1f77b4",  # tab10 blue
    "Twinned (2x1)":  "#ff7f0e",  # tab10 orange
    "c(6x2)":         "#2ca02c",  # tab10 green
    "rt13xrt13":      "#d62728",  # tab10 red
    "HTR":            "#9467bd",  # tab10 purple
}

# Default event-fire threshold as of the last AutoCaptureEngine ctor
# (see gui/auto_capture.py:394). Used only as a reference line on the
# score-distribution chart. The CSV doesn't record the effective
# threshold — if the grower changed it mid-session, the recorded
# scores were compared against a different bar. Chart title text
# calls this caveat out.
DEFAULT_CHANGE_THRESHOLD: float = 0.20

# Auto-capture event_state values written to auto_capture_events.csv
# (see gui/growth_logger.py:26-30). Locked in this order for
# consistent legend / summary sort across all sessions.
EVENT_STATE_ORDER: list[str] = [
    "pending", "kept_explicit", "kept_default", "discarded",
    "auto_skipped",
]

# Matches plot_temperature.py's line color for cross-tool visual
# consistency. If both scripts render side-by-side (e.g. Day 6 HTML
# report), pyro traces look the same.
_TRACE_COLOR = "#0d9488"  # teal


def _safe_float(value) -> Optional[float]:
    """Parse a string-like to float; return None on empty/missing/invalid.

    Mirrors ``plot_temperature.py:37-53``'s tolerance for messy CSV rows
    (blank cells, wrong dtypes, ``None`` when a key is absent). Callers
    check the return for ``None`` and skip the row.
    """
    if value is None:
        return None
    s = str(value).strip()
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        return None


def _read_csv_rows(path: Path) -> list[dict[str, str]]:
    """Read a CSV into a list-of-dicts. Missing file → empty list.

    Values stay as strings (``csv.DictReader`` default). Per-field type
    coercion is caller-side via ``_safe_float`` — keeps this reader
    schema-agnostic so it survives future column additions.
    """
    if not path.exists():
        return []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        return list(reader)


@dataclass
class SessionArtifacts:
    """All CSV data from one Growth Monitor session, decoded once.

    One field per CSV — each is a list of dicts (raw string cells as
    read from disk). Typed accessors on the dataclass coerce columns to
    float when needed. Missing optional CSVs materialize as empty
    lists, so downstream chart renderers can rely on iteration
    protocols regardless of which files the session logged.
    """
    session_dir: Path
    sensor_rows: list[dict[str, str]] = field(default_factory=list)
    commit_rows: list[dict[str, str]] = field(default_factory=list)
    auto_capture_rows: list[dict[str, str]] = field(default_factory=list)
    manual_event_rows: list[dict[str, str]] = field(default_factory=list)
    heartbeat_rows: list[dict[str, str]] = field(default_factory=list)
    event_label_rows: list[dict[str, str]] = field(default_factory=list)

    @classmethod
    def from_session_dir(cls, session_dir: Path) -> "SessionArtifacts":
        """Read all six CSVs from a Growth Monitor session directory.

        Raises ``FileNotFoundError`` only if ``session_dir`` doesn't
        exist or isn't a directory. Individual CSVs missing from a
        valid session dir do NOT raise — they yield empty lists so
        pre-schema sessions still open cleanly.
        """
        session_dir = Path(session_dir)
        if not session_dir.is_dir():
            raise FileNotFoundError(
                f"Session directory not found: {session_dir}"
            )
        return cls(
            session_dir=session_dir,
            sensor_rows=_read_csv_rows(session_dir / "sensor_log.csv"),
            commit_rows=_read_csv_rows(session_dir / "commit_log.csv"),
            auto_capture_rows=_read_csv_rows(
                session_dir / "auto_capture_events.csv"
            ),
            manual_event_rows=_read_csv_rows(
                session_dir / "manual_events.csv"
            ),
            heartbeat_rows=_read_csv_rows(
                session_dir / "heartbeat_log.csv"
            ),
            event_label_rows=_read_csv_rows(
                session_dir / "events_labels.csv"
            ),
        )

    def temperature_series(self) -> tuple[list[float], list[float]]:
        """Return ``(elapsed_s, pyrometer_temp_C)`` from ``sensor_rows``.

        Skips rows where either column is missing or unparseable — same
        policy as ``plot_temperature.py:35-53``. Empty return
        ``([], [])`` means no chartable data (empty session, or all
        rows had bad temp reads).
        """
        elapsed: list[float] = []
        temps: list[float] = []
        for row in self.sensor_rows:
            t = _safe_float(row.get("elapsed_s"))
            temp = _safe_float(row.get("pyrometer_temp_C"))
            if t is None or temp is None:
                continue
            elapsed.append(t)
            temps.append(temp)
        return elapsed, temps

    def temperature_std_series(
        self,
    ) -> tuple[list[float], list[float], list[float]]:
        """Return ``(elapsed_s, mean_T, std_T)`` from ``sensor_rows``.

        Skips rows missing any of the three columns. Pre-Apr-2026
        sessions predate ``pyrometer_temp_std_C`` and return empty
        lists here (chart renderers detect this and skip PNG output).
        """
        elapsed: list[float] = []
        temps: list[float] = []
        stds: list[float] = []
        for row in self.sensor_rows:
            t = _safe_float(row.get("elapsed_s"))
            temp = _safe_float(row.get("pyrometer_temp_C"))
            std = _safe_float(row.get("pyrometer_temp_std_C"))
            if t is None or temp is None or std is None:
                continue
            elapsed.append(t)
            temps.append(temp)
            stds.append(std)
        return elapsed, temps, stds

    def event_overlays(self) -> list[tuple[float, str]]:
        """List of ``(elapsed_s, source)`` for chart vertical-line overlays.

        Sources come from three discrete-event CSVs:

          - ``commit_log.csv``        → ``"commit"``  (LOG ENTRY)
          - ``manual_events.csv``     → ``"manual"``  (MARK EVENT)
          - ``auto_capture_events.csv`` → ``"auto"``  (auto-capture engine)

        Heartbeat rows are intentionally excluded — continuous capture
        would swamp any chart with hundreds of overlays. Heartbeats
        form the background timeline (visible via density of the
        temperature trace itself), not discrete events.

        Result is sorted by ``elapsed_s``.
        """
        overlays: list[tuple[float, str]] = []
        for source, rows in (
            ("commit", self.commit_rows),
            ("manual", self.manual_event_rows),
            ("auto", self.auto_capture_rows),
        ):
            for row in rows:
                t = _safe_float(row.get("elapsed_s"))
                if t is not None:
                    overlays.append((t, source))
        overlays.sort(key=lambda pair: pair[0])
        return overlays

    def label_for_event(self, event_idx: int) -> Optional[dict[str, str]]:
        """Look up the ``events_labels.csv`` row for a given event_idx.

        Returns the row dict (string cells) or ``None`` if that event
        wasn't labeled. Callers use this to correlate an auto-capture
        or manual event with the grower's post-hoc reconstruction
        label — the primary join Day 5's grower-vs-classifier chart
        will need.
        """
        target = str(event_idx)
        for row in self.event_label_rows:
            if row.get("event_idx") == target:
                return row
        return None


def render_temperature_chart(
    artifacts: SessionArtifacts,
    output_path: Path,
    dpi: int = 150,
    title: Optional[str] = None,
) -> Optional[Path]:
    """Render T vs t chart with event overlays.

    Returns ``output_path`` on success, or ``None`` (and writes no
    file) if the session has no chartable temperature data. Callers
    should check the return before advertising the file exists.
    """
    elapsed, temps = artifacts.temperature_series()
    if not temps:
        return None

    # Import matplotlib lazily so ``SessionArtifacts.from_session_dir``
    # stays cheap for callers that only need the data layer (Day 7's
    # batch inference doesn't render anything, and Day 4's tests only
    # need matplotlib for the 3 chart tests, not the 7 reader tests).
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    elapsed_min = [t / 60.0 for t in elapsed]

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(elapsed_min, temps, color=_TRACE_COLOR, linewidth=1.2,
            zorder=3, label="Pyro T")
    ax.set_xlabel("Elapsed time (min)", fontsize=11)
    ax.set_ylabel("Pyrometer T (°C)", fontsize=11)
    ax.set_title(
        title or f"Temperature profile — {artifacts.session_dir.name}",
        fontsize=13,
    )
    ax.grid(True, alpha=0.3)

    # Overlay commit/manual/auto events as vertical lines. Dedup the
    # legend by only attaching a label on the first line per source
    # (matplotlib would render N separate legend entries otherwise).
    seen_sources: set[str] = set()
    for t_s, source in artifacts.event_overlays():
        color = EVENT_MARKER_COLORS.get(source, "#000000")
        label = source if source not in seen_sources else None
        ax.axvline(
            x=t_s / 60.0, color=color, alpha=0.55, linewidth=0.8,
            linestyle="--", label=label, zorder=1,
        )
        seen_sources.add(source)

    ax.legend(loc="best", fontsize=9, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(str(output_path), dpi=dpi)
    plt.close(fig)
    return output_path


def render_pyro_stability_chart(
    artifacts: SessionArtifacts,
    output_path: Path,
    dpi: int = 150,
    title: Optional[str] = None,
) -> Optional[Path]:
    """Render pyrometer mean ± std band over time.

    Uses ``pyrometer_temp_std_C`` from the 5-sample rolling window
    that ``PyrometerWorker`` writes with each reading. Returns
    ``output_path`` on success; returns ``None`` (writes no file)
    when the session has no std data — either a pre-Apr-2026 session
    predating the column, or a session where every row failed the
    rolling window.
    """
    elapsed, temps, stds = artifacts.temperature_std_series()
    if not stds:
        return None

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    elapsed_min = [t / 60.0 for t in elapsed]
    upper = [m + s for m, s in zip(temps, stds)]
    lower = [m - s for m, s in zip(temps, stds)]

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.fill_between(
        elapsed_min, lower, upper, color=_TRACE_COLOR, alpha=0.25,
        label="±1σ (5-sample rolling)",
    )
    ax.plot(elapsed_min, temps, color=_TRACE_COLOR, linewidth=1.2,
            label="Mean T")
    ax.set_xlabel("Elapsed time (min)", fontsize=11)
    ax.set_ylabel("Pyrometer T (°C)", fontsize=11)
    ax.set_title(
        title or f"Pyro stability — {artifacts.session_dir.name}",
        fontsize=13,
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9, framealpha=0.9)

    fig.tight_layout()
    fig.savefig(str(output_path), dpi=dpi)
    plt.close(fig)
    return output_path


def render_classifier_trajectory(
    artifacts: SessionArtifacts,
    output_path: Path,
    dpi: int = 150,
    title: Optional[str] = None,
) -> Optional[Path]:
    """Render classifier smoothed_percent per class over the session.

    One line per RECON_LABELS class, sourced from ``commit_log.csv``'s
    ``classifier_recon_<CLASS>`` columns (see ``growth_logger.py:80-83``
    for the field spec). Each LOG ENTRY commit contributes one point
    per class; the resulting 5-line chart shows how the classifier's
    class-belief drifted across the session.

    Returns ``output_path`` on success; ``None`` (writes no file) if
    the session has no commits or all classifier cells are blank
    (session logged before any frame was classified, or classifier
    was DISABLED for the whole session — see ``classifier_status``
    column semantics in ``growth_logger.py:90-100``).
    """
    if not artifacts.commit_rows:
        return None

    # Extract per-class series. Each series is (elapsed_min, percent);
    # rows with missing/blank classifier cells for that class are
    # dropped from that specific class's series but not from others.
    per_class: dict[str, tuple[list[float], list[float]]] = {}
    total_classifier_points = 0
    for label in RECON_LABELS:
        col = f"classifier_recon_{label}"
        xs: list[float] = []
        ys: list[float] = []
        for row in artifacts.commit_rows:
            t = _safe_float(row.get("elapsed_s"))
            v = _safe_float(row.get(col))
            if t is None or v is None:
                continue
            xs.append(t / 60.0)
            ys.append(v)
        per_class[label] = (xs, ys)
        total_classifier_points += len(xs)

    if total_classifier_points == 0:
        # commits exist but classifier never emitted — skip render.
        return None

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(11, 5))
    for label in RECON_LABELS:
        xs, ys = per_class[label]
        if not xs:
            continue
        ax.plot(
            xs, ys, color=CLASS_LINE_COLORS[label], linewidth=1.4,
            marker="o", markersize=3, label=label, alpha=0.85,
        )
    ax.set_xlabel("Elapsed time (min)", fontsize=11)
    ax.set_ylabel("Classifier smoothed % per class", fontsize=11)
    ax.set_title(
        title or f"Classifier trajectory — {artifacts.session_dir.name}",
        fontsize=13,
    )
    ax.set_ylim(0, 100)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9, framealpha=0.9, ncol=3)

    fig.tight_layout()
    fig.savefig(str(output_path), dpi=dpi)
    plt.close(fig)
    return output_path


def render_score_distribution(
    artifacts: SessionArtifacts,
    output_path: Path,
    dpi: int = 150,
    title: Optional[str] = None,
    threshold: float = DEFAULT_CHANGE_THRESHOLD,
) -> Optional[Path]:
    """Render histogram of auto-capture change_score values.

    Bins the ``change_score`` column from ``auto_capture_events.csv``
    (see ``growth_logger.py:113-118``) and overlays:

      - a vertical dashed line at ``threshold`` (default
        ``DEFAULT_CHANGE_THRESHOLD``, matching
        ``AutoCaptureEngine.__init__``'s default at code freeze)
      - an anchored text box with per-event_state counts (pending /
        kept_explicit / kept_default / discarded / auto_skipped),
        so a grower can see at a glance whether their labels
        skewed toward keep vs discard on this session

    Returns ``output_path`` on success; ``None`` if the session
    fired no auto-capture events (no rows in the CSV → nothing to
    histogram).
    """
    scores: list[float] = []
    state_counts: dict[str, int] = {s: 0 for s in EVENT_STATE_ORDER}
    for row in artifacts.auto_capture_rows:
        s = _safe_float(row.get("change_score"))
        if s is not None:
            scores.append(s)
        state = row.get("event_state", "").strip() or "pending"
        if state not in state_counts:
            # Future-proof: unknown states get their own bucket so
            # we don't silently drop them. Order becomes append.
            state_counts[state] = 0
        state_counts[state] += 1

    if not scores:
        return None

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(11, 5))
    n_bins = min(30, max(5, len(scores) // 3 or 5))
    ax.hist(
        scores, bins=n_bins, color=_TRACE_COLOR, alpha=0.75,
        edgecolor="white",
    )
    ax.axvline(
        x=threshold, color="#dc2626", linestyle="--", linewidth=1.2,
        label=f"threshold ≈ {threshold:.2f}",
    )
    ax.set_xlabel("change_score (higher = more visual change)",
                  fontsize=11)
    ax.set_ylabel("Event count", fontsize=11)
    ax.set_title(
        title or (
            f"Auto-capture score distribution — "
            f"{artifacts.session_dir.name}"
        ),
        fontsize=13,
    )
    ax.grid(True, alpha=0.3)

    # State-count summary box in the upper right. Bare text is faster
    # to scan than a second legend section.
    summary_lines = [
        f"{s}: {state_counts[s]}"
        for s in state_counts
        if state_counts[s] > 0
    ]
    if summary_lines:
        ax.text(
            0.98, 0.97, "\n".join(summary_lines),
            transform=ax.transAxes, va="top", ha="right", fontsize=9,
            bbox={
                "boxstyle": "round,pad=0.4", "facecolor": "white",
                "edgecolor": "#888888", "alpha": 0.9,
            },
        )
    ax.legend(loc="upper left", fontsize=9, framealpha=0.9)

    fig.tight_layout()
    fig.savefig(str(output_path), dpi=dpi)
    plt.close(fig)
    return output_path


def render_grower_vs_classifier_agreement(
    artifacts: SessionArtifacts,
    output_path: Path,
    dpi: int = 150,
    title: Optional[str] = None,
) -> Optional[Path]:
    """Render 5-panel scatter: grower slider % vs classifier smoothed %.

    One subplot per RECON_LABELS class. X axis = classifier %,
    Y axis = grower slider % at LOG ENTRY. Perfect agreement is the
    y=x diagonal (drawn dashed grey for reference). Grower-corrected
    commits deviate from the diagonal; uncorrected commits sit right
    on it (sliders mirror classifier when ``✎ Correct`` is off).

    Column pairing comes from ``growth_logger.py:73-83``:
    ``recon_<CLASS>`` (grower's belief) with
    ``classifier_recon_<CLASS>`` (classifier's smoothed_percent),
    per commit row.

    Returns ``output_path`` on success; ``None`` when no commit has
    both a grower and classifier reading for any class (session with
    classifier DISABLED or a session with no LOG ENTRY commits).
    """
    if not artifacts.commit_rows:
        return None

    # Per-class list of (classifier_pct, grower_pct, grower_corrected).
    per_class_points: dict[str, list[tuple[float, float, bool]]] = {
        label: [] for label in RECON_LABELS
    }
    total_points = 0
    for row in artifacts.commit_rows:
        corrected = row.get("grower_corrected", "").strip() == "True"
        for label in RECON_LABELS:
            grower = _safe_float(row.get(f"recon_{label}"))
            classifier = _safe_float(row.get(f"classifier_recon_{label}"))
            if grower is None or classifier is None:
                continue
            per_class_points[label].append((classifier, grower, corrected))
            total_points += 1

    if total_points == 0:
        return None

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 5, figsize=(16, 3.5), sharey=True)
    for ax, label in zip(axes, RECON_LABELS):
        pts = per_class_points[label]
        # Diagonal reference y=x (0-100).
        ax.plot(
            [0, 100], [0, 100], color="#888888", linestyle="--",
            linewidth=0.8, alpha=0.6, zorder=1,
        )
        # Uncorrected commits (sliders mirror classifier — sit ON diag).
        uncorr = [(x, y) for (x, y, c) in pts if not c]
        # Corrected commits (grower deviated — off-diag = disagreement).
        corr = [(x, y) for (x, y, c) in pts if c]
        if uncorr:
            xs, ys = zip(*uncorr)
            ax.scatter(
                xs, ys, s=18, color="#888888", alpha=0.5,
                label="uncorrected", zorder=2,
            )
        if corr:
            xs, ys = zip(*corr)
            ax.scatter(
                xs, ys, s=28, color=CLASS_LINE_COLORS[label], alpha=0.85,
                label="corrected", zorder=3, edgecolor="white",
                linewidth=0.5,
            )
        ax.set_xlim(-2, 102)
        ax.set_ylim(-2, 102)
        ax.set_title(label, fontsize=10)
        ax.set_xlabel("Classifier %", fontsize=9)
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel("Grower slider %", fontsize=9)
    # One shared legend, above.
    handles, labels_ = axes[-1].get_legend_handles_labels()
    if handles:
        fig.legend(
            handles, labels_, loc="upper right", fontsize=9,
            framealpha=0.9, bbox_to_anchor=(0.995, 0.98),
        )
    fig.suptitle(
        title or (
            f"Grower vs classifier — {artifacts.session_dir.name} "
            f"({total_points} points across {len(artifacts.commit_rows)} commits)"
        ),
        fontsize=12,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(str(output_path), dpi=dpi)
    plt.close(fig)
    return output_path


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Render post-hoc analysis charts for a Growth Monitor session."
        ),
    )
    parser.add_argument(
        "session_path", type=Path,
        help="Path to session directory (contains sensor_log.csv, etc.)",
    )
    parser.add_argument(
        "--output-dir", "-o", type=Path, default=None,
        help=(
            "Directory to write PNGs (default: <session_path>/analysis/)"
        ),
    )
    parser.add_argument("--dpi", type=int, default=150)
    args = parser.parse_args()

    try:
        artifacts = SessionArtifacts.from_session_dir(args.session_path)
    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    out_dir = args.output_dir or (args.session_path / "analysis")
    out_dir.mkdir(parents=True, exist_ok=True)

    written: list[Path] = []
    renderers = [
        (render_temperature_chart,
         "temperature_profile_annotated.png"),
        (render_pyro_stability_chart,
         "pyro_stability.png"),
        (render_classifier_trajectory,
         "classifier_trajectory.png"),
        (render_score_distribution,
         "score_distribution.png"),
        (render_grower_vs_classifier_agreement,
         "grower_vs_classifier.png"),
    ]
    for renderer, filename in renderers:
        p = renderer(artifacts, out_dir / filename, dpi=args.dpi)
        if p is not None:
            written.append(p)

    if not written:
        print(
            f"No chartable data in session {args.session_path.name}",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"Rendered {len(written)} chart(s) to {out_dir}:")
    for p in written:
        print(f"  {p.name}")


if __name__ == "__main__":
    main()

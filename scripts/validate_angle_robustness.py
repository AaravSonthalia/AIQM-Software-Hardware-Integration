#!/usr/bin/env python3
"""Angle / signal-magnitude robustness validation for Classifier2.

Runs Classifier2 over every frame in each of Rahim's 3 archived STO
trajectories, aggregates quality-distribution statistics per session,
and writes a markdown report comparing the sessions.

Purpose: quantify whether Classifier2 stays consistent when the input
signal magnitude varies by ~10× across sessions of the same substrate
material. This is the practical equivalent of the 15°–60° angle-range
test in Chen 2025 — same idea (robustness to input-space variation),
different axis (magnitude vs geometry).

The three trajectories:
  2022-02-04  388 frames, RR220204A
  2022-02-06  287 frames, RR220206A
  2022-04-11  449 frames, RR220411A

Filenames encode temperature: RR<date>A_<temp>C_<hhmm>[-XX].bmp. We
harvest that to bucket frames by pyrometer temperature so the report
can show class-vs-temperature trajectories per session.

Deliverable-#4-adjacent — the classifier code is validated on Bulbasaur
(Jul 8 lab session, screenshots + CSV per bulbasaur_lab_day_jul07). This
script closes the "does it generalize across sessions?" loop by running
the same weights over historical sessions from a different sample.

Usage:
    python scripts/validate_angle_robustness.py \\
        --data-root ~/Downloads \\
        --out ~/Downloads/angle_robustness_report.md

    # Faster iteration during development — subsample each session
    python scripts/validate_angle_robustness.py --stride 5

    # Skip inference, just render report from cached CSV
    python scripts/validate_angle_robustness.py --skip-inference
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


# --- Session catalog -------------------------------------------------------

@dataclass(frozen=True)
class Session:
    name: str
    frames_dir: Path
    sample_id: str


def _default_sessions(data_root: Path) -> list[Session]:
    return [
        Session(
            name="2022-02-04",
            frames_dir=data_root / "rahim_2022_02_04" / "2022_02_04_renamed" / "Substrates",
            sample_id="RR220204A",
        ),
        Session(
            name="2022-02-06",
            frames_dir=data_root / "rahim_2022_02_06" / "2022_02_06_renamed" / "Substrates",
            sample_id="RR220206A",
        ),
        Session(
            name="2022-04-11",
            frames_dir=(
                data_root / "rahim_2022_04_11" / "2022_04_11_renamed"
                / "04_11_2022_substrates"
            ),
            sample_id="RR220411A",
        ),
    ]


# --- Filename → temperature extractor -------------------------------------

_TEMP_RE = re.compile(r"_([0-9]+)C_")


def extract_temp_C(path: Path) -> Optional[int]:
    """Pull the pyrometer temperature (°C) out of the filename.
    Returns None if the pattern doesn't match — script tolerates that."""
    m = _TEMP_RE.search(path.name)
    return int(m.group(1)) if m else None


# --- Inference over a session ---------------------------------------------

def infer_session(
    bridge, session: Session, stride: int = 1,
) -> list[dict]:
    """Run Classifier2 over every ``stride``-th frame in the session.

    Returns a list of per-frame result dicts with:
      filename, temp_C, predicted_class, quality, is_bad, bad_confidence,
      score_1x1, score_Twinned (2x1), score_c(6x2), score_rt13xrt13, score_HTR
    """
    from PIL import Image

    frames = sorted(session.frames_dir.glob("*.bmp"))
    if stride > 1:
        frames = frames[::stride]

    print(f"[{session.name}] {len(frames)} frames (stride={stride})...")
    results: list[dict] = []
    for i, path in enumerate(frames):
        try:
            img = np.array(Image.open(path).convert("RGB"))
        except Exception as e:  # noqa: BLE001 — corrupt file, skip and note
            print(f"  ⚠ skip {path.name}: {e}", file=sys.stderr)
            continue
        r = bridge.classify(img)
        row = {
            "session": session.name,
            "sample_id": session.sample_id,
            "filename": path.name,
            "temp_C": extract_temp_C(path),
            "predicted_class": r.get("predicted_class", ""),
            "quality": float(r.get("quality") or 0.0),
            "is_bad": bool(r.get("is_bad", False)),
            "bad_confidence": float(r.get("bad_confidence", 0.0)),
        }
        for cls, score in (r.get("classification_scores") or {}).items():
            row[f"score_{cls}"] = float(score)
        results.append(row)
        if (i + 1) % 50 == 0:
            print(f"  ... {i+1}/{len(frames)}")
    return results


# --- Statistics + markdown ------------------------------------------------

def _percentile(a: np.ndarray, p: float) -> float:
    return float(np.percentile(a, p)) if a.size else 0.0


def session_summary(rows: list[dict]) -> dict:
    q = np.array([r["quality"] for r in rows])
    bad_frac = float(np.mean([r["is_bad"] for r in rows]))
    class_counts: dict[str, int] = {}
    for r in rows:
        cls = r["predicted_class"]
        class_counts[cls] = class_counts.get(cls, 0) + 1
    total = max(1, len(rows))
    class_frac = {c: n / total for c, n in class_counts.items()}
    return {
        "n_frames": len(rows),
        "quality_mean": float(np.mean(q)) if q.size else 0.0,
        "quality_std": float(np.std(q)) if q.size else 0.0,
        "quality_p50": _percentile(q, 50),
        "quality_p10": _percentile(q, 10),
        "quality_p90": _percentile(q, 90),
        "bad_frame_fraction": bad_frac,
        "class_distribution": class_frac,
    }


def ks_matrix(rows_by_session: dict[str, list[dict]]) -> dict:
    """Kolmogorov-Smirnov two-sample test on the quality distributions
    for every pair of sessions. Returns {(A, B): (D, p)} for A < B."""
    from scipy import stats

    names = sorted(rows_by_session)
    out: dict[tuple[str, str], tuple[float, float]] = {}
    for i, a in enumerate(names):
        qa = np.array([r["quality"] for r in rows_by_session[a]])
        for b in names[i + 1:]:
            qb = np.array([r["quality"] for r in rows_by_session[b]])
            D, p = stats.ks_2samp(qa, qb)
            out[(a, b)] = (float(D), float(p))
    return out


def write_markdown_report(
    out_path: Path,
    summaries: dict[str, dict],
    ks: dict,
    stride: int,
) -> None:
    lines: list[str] = []
    lines.append("# Classifier2 angle / signal-magnitude robustness report")
    lines.append("")
    lines.append(
        "Ran Classifier2 (best_model.pth as of Jul 8 2026 Bulbasaur "
        f"validation) over Rahim's 3 archived STO trajectories with "
        f"stride={stride}."
    )
    lines.append("")

    # Per-session summary table
    lines.append("## Per-session quality summary")
    lines.append("")
    lines.append(
        "| Session | N frames | Q mean ± std | Q p10 / p50 / p90 | Bad frame % |"
    )
    lines.append(
        "|---|---|---|---|---|"
    )
    for name in sorted(summaries):
        s = summaries[name]
        lines.append(
            f"| {name} | {s['n_frames']} | "
            f"{s['quality_mean']:.3f} ± {s['quality_std']:.3f} | "
            f"{s['quality_p10']:.3f} / {s['quality_p50']:.3f} / {s['quality_p90']:.3f} | "
            f"{s['bad_frame_fraction']*100:.1f}% |"
        )
    lines.append("")

    # KS test pairwise
    lines.append("## Pairwise quality-distribution KS test")
    lines.append("")
    lines.append(
        "Null: two sessions' quality distributions are drawn from the same "
        "underlying distribution. Small p-value → distributions differ "
        "significantly → classifier's confidence depends on which session "
        "(i.e., NOT robust to signal-magnitude variation)."
    )
    lines.append("")
    lines.append("| Session A | Session B | KS D-stat | p-value | Verdict |")
    lines.append("|---|---|---|---|---|")
    for (a, b), (D, p) in sorted(ks.items()):
        verdict = (
            "distributions differ (p<0.05)" if p < 0.05
            else "no significant difference"
        )
        lines.append(f"| {a} | {b} | {D:.3f} | {p:.3g} | {verdict} |")
    lines.append("")

    # Class distribution
    lines.append("## Class distribution per session")
    lines.append("")
    all_classes = set()
    for s in summaries.values():
        all_classes.update(s["class_distribution"].keys())
    ordered_classes = sorted(all_classes)
    header = "| Session | " + " | ".join(ordered_classes) + " |"
    sep = "|---|" + "---|" * len(ordered_classes)
    lines.append(header)
    lines.append(sep)
    for name in sorted(summaries):
        cd = summaries[name]["class_distribution"]
        cells = [
            f"{cd.get(c, 0.0)*100:.1f}%" for c in ordered_classes
        ]
        lines.append(f"| {name} | " + " | ".join(cells) + " |")
    lines.append("")

    # Interpretation
    lines.append("## Interpretation")
    lines.append("")
    lines.append(
        "- **Q mean > 0.5 across all sessions** would indicate the classifier "
        "is confidently classifying most frames across the signal-magnitude "
        "range — the robustness we hoped for."
    )
    lines.append(
        "- **KS p > 0.05 on all pairs** would confirm no statistically "
        "significant difference in quality distribution across sessions — "
        "the strongest positive result available from this test."
    )
    lines.append(
        "- **Class distribution similarity** across sessions of the same "
        "STO substrate is a weaker but complementary signal — same physics, "
        "should produce similar recon-class fractions modulo temperature "
        "trajectory differences."
    )
    lines.append("")
    lines.append(
        "Chen 2025 tested robustness across 15°–60° camera angle (a "
        "different input-space axis). This script's analog test — "
        "robustness across ~10× signal-magnitude variation — is the "
        "practically-relevant equivalent for our archive."
    )
    lines.append("")

    out_path.write_text("\n".join(lines))
    print(f"Wrote report: {out_path}")


# --- Main -----------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data-root",
        type=Path,
        default=Path.home() / "Downloads",
        help="Directory containing the three rahim_2022_* session dirs",
    )
    parser.add_argument(
        "--ai-repo-root",
        default="/Users/aj/test-claude/projects/ai-for-quantum",
        help="AI_for_quantum repo root for ClassifierBridge",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Subsample every Nth frame (default 1 = every frame)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path.home() / "Downloads" / "angle_robustness_report.md",
        help="Path for the markdown report",
    )
    parser.add_argument(
        "--csv-out",
        type=Path,
        default=None,
        help="Optional path for per-frame CSV (defaults to alongside the report)",
    )
    parser.add_argument(
        "--skip-inference",
        action="store_true",
        help="Skip inference; render report from cached CSV",
    )
    args = parser.parse_args()

    csv_out = args.csv_out or args.out.with_suffix(".csv")

    # Bridge setup
    if not args.skip_inference:
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
        from gui.classifier_bridge import ClassifierBridge
        print(f"Loading ClassifierBridge from {args.ai_repo_root}...")
        bridge = ClassifierBridge(ai_repo_root=args.ai_repo_root)

        # Run inference over all sessions
        all_rows: list[dict] = []
        for session in _default_sessions(args.data_root):
            if not session.frames_dir.exists():
                print(f"⚠ skip {session.name}: {session.frames_dir} missing",
                      file=sys.stderr)
                continue
            all_rows.extend(infer_session(bridge, session, stride=args.stride))

        # Persist to CSV — take the union of keys across all rows so
        # rows with missing score fields (first frame's classify() may
        # return an empty classification_scores dict, later frames
        # populate them) don't crash DictWriter.
        if all_rows:
            base_fields = [
                "session", "sample_id", "filename", "temp_C",
                "predicted_class", "quality", "is_bad", "bad_confidence",
            ]
            score_fields = sorted({
                k for r in all_rows for k in r if k.startswith("score_")
            })
            fields = base_fields + score_fields
            with open(csv_out, "w", newline="") as f:
                writer = csv.DictWriter(
                    f, fieldnames=fields, extrasaction="ignore",
                )
                writer.writeheader()
                writer.writerows(all_rows)
            print(f"Wrote per-frame CSV: {csv_out}")
    else:
        # Load cached CSV
        with open(csv_out) as f:
            all_rows = [dict(r) for r in csv.DictReader(f)]
        # Coerce numeric fields
        for r in all_rows:
            r["quality"] = float(r["quality"])
            r["is_bad"] = r["is_bad"] in ("True", "true", "1")
            for k in list(r):
                if k.startswith("score_"):
                    r[k] = float(r[k])
        print(f"Loaded {len(all_rows)} cached rows from {csv_out}")

    # Group by session
    by_session: dict[str, list[dict]] = {}
    for r in all_rows:
        by_session.setdefault(r["session"], []).append(r)

    # Compute summaries + KS
    summaries = {name: session_summary(rows) for name, rows in by_session.items()}
    ks = ks_matrix(by_session) if len(by_session) >= 2 else {}

    write_markdown_report(args.out, summaries, ks, stride=args.stride)
    return 0


if __name__ == "__main__":
    sys.exit(main())

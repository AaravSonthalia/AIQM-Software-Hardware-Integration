"""Find MISTRAL log files on Bulbasaur.

MISTRAL (Scienta Omicron) most likely writes a continuous log file to
disk somewhere. If we can locate it, the OCR-based driver
(`drivers/mistral.py`) can be replaced with file watching — robust to
window positioning, no Tesseract dependency, no "Set Voltage" vs
"Actual Voltage" parse confusion.

This script searches likely Windows locations for recently-modified
.txt / .log / .csv / .dat files and surfaces candidates with metadata
plus an optional content preview.

Scienta Omicron data-storage convention (from SES Software Manual v5.0,
page 6, Table 1): data is stored INSIDE the install directory in
sub-folders /data (binary .dat + .bin), /ini (.ini, .json), and
/sequences (.seq, .ini) — NOT in standard Windows locations like
ProgramData or AppData. Worth assuming MISTRAL follows the same pattern.
SES also uses a custom top-level directory (e.g.
`C:\\Scienta Omicron\\SES_1.8.0_Win64\\`) per the File Options dialog on
page 27, so both `C:\\Scienta Omicron\\` and the standard install paths
are searched.

Usage on Bulbasaur:
    python scripts/find_mistral_logs.py              # default search
    python scripts/find_mistral_logs.py --peek       # also show first lines
    python scripts/find_mistral_logs.py --max-age 1  # last 24 hours only
    python scripts/find_mistral_logs.py --wide       # broader AppData / ProgramData
    python scripts/find_mistral_logs.py --name-search  # match files named *mistral*

What to look for in candidate files:
- Numeric readings that change over time (temperature, voltage, current,
  power, pressure)
- Timestamps
- Cell names ("Mani", etc., matching MistralGui display)
- A header row identifying columns

Once a likely file is found, share path + first ~20 lines so we can
design the file-watching driver around its format.
"""

import argparse
import os
import sys
import time
from datetime import datetime
from typing import Optional

# Most-likely roots, in order of priority. `C:\Scienta Omicron` is the
# top-level data directory pattern Scienta Omicron uses per the SES
# Software Manual v5.0 (page 27, File Options dialog) — separate from the
# Program Files install location.
KNOWN_ROOTS: tuple[str, ...] = (
    r"C:\Scienta Omicron",
    r"C:\Program Files\Scienta Omicron",
    r"C:\Program Files (x86)\Scienta Omicron",
    r"C:\ProgramData\Scienta Omicron",
    r"C:\Users\Lab10\AppData\Local\Scienta Omicron",
    r"C:\Users\Lab10\AppData\Roaming\Scienta Omicron",
    r"C:\Users\Lab10\Documents\Scienta Omicron",
    r"C:\Users\Lab10\Documents\MistralGui",
    r"C:\Users\Lab10\Documents",
    r"C:\Mistral",
    r"C:\MistralGui",
    r"C:\_Omicron_Software",
)

# Used only when --wide is passed (these trees are large and noisy).
WIDE_ROOTS: tuple[str, ...] = (
    r"C:\ProgramData",
    r"C:\Users\Lab10\AppData",
)

LOG_EXTS: frozenset[str] = frozenset({".txt", ".log", ".csv", ".dat"})
NAME_KEYWORDS: tuple[str, ...] = ("mistral", "omicron", "scienta")


def _exists(path: str) -> bool:
    try:
        return os.path.exists(path)
    except OSError:
        return False


def find_log_files(
    roots: tuple[str, ...],
    exts: frozenset[str],
    max_age_days: Optional[float] = None,
) -> list[tuple[float, int, str]]:
    """Walk ``roots`` and return matching files as ``[(mtime, size, path)]``,
    newest first."""
    cutoff = time.time() - max_age_days * 86400 if max_age_days else 0.0
    out: list[tuple[float, int, str]] = []
    for root in roots:
        if not _exists(root):
            continue
        for dirpath, _dirnames, filenames in os.walk(root):
            for fname in filenames:
                ext = os.path.splitext(fname)[1].lower()
                if ext not in exts:
                    continue
                fpath = os.path.join(dirpath, fname)
                try:
                    st = os.stat(fpath)
                except OSError:
                    continue
                if cutoff and st.st_mtime < cutoff:
                    continue
                out.append((st.st_mtime, st.st_size, fpath))
    out.sort(reverse=True)
    return out


def find_named(roots: tuple[str, ...], keywords: tuple[str, ...]) -> list[str]:
    """Find files/dirs under ``roots`` whose name contains any keyword (CI)."""
    keywords_lower = tuple(k.lower() for k in keywords)
    matches: list[str] = []
    for root in roots:
        if not _exists(root):
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            for name in dirnames + filenames:
                lname = name.lower()
                if any(k in lname for k in keywords_lower):
                    matches.append(os.path.join(dirpath, name))
    return matches


def peek(path: str, n_lines: int = 10, max_chars: int = 180) -> list[str]:
    """Return up to ``n_lines`` lines from ``path``, each clipped to ``max_chars``."""
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            lines: list[str] = []
            for i, line in enumerate(f):
                if i >= n_lines:
                    break
                lines.append(line.rstrip()[:max_chars])
            return lines
    except OSError as e:
        return [f"<cannot read: {e}>"]


def format_size(size: int) -> str:
    if size < 10 * 1024:
        return f"{size:>9d} B "
    if size < 10 * 1024 * 1024:
        return f"{size / 1024:>9.1f} KB"
    return f"{size / 1024 / 1024:>9.1f} MB"


def format_mtime(mtime: float) -> str:
    return datetime.fromtimestamp(mtime).strftime("%Y-%m-%d %H:%M:%S")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Find MISTRAL log files on Bulbasaur",
    )
    parser.add_argument(
        "--peek", action="store_true",
        help="Show first 10 lines of each candidate (helpful for identification)",
    )
    parser.add_argument(
        "--max-age", type=float, default=14,
        help="Max age in days (default: 14)",
    )
    parser.add_argument(
        "--wide", action="store_true",
        help=r"Also search C:\ProgramData and C:\Users\Lab10\AppData broadly",
    )
    parser.add_argument(
        "--name-search", action="store_true",
        help="Also search for files/dirs named *mistral* / *omicron* / *scienta*",
    )
    parser.add_argument(
        "--limit", type=int, default=30,
        help="Max number of candidates per phase (default: 30)",
    )
    args = parser.parse_args()

    print("=" * 72)
    print("Searching for MISTRAL log files")
    print("=" * 72)

    # Phase 1: which known roots actually exist?
    print("\n--- Phase 1: known roots (existence check) ---")
    found_roots: list[str] = []
    for root in KNOWN_ROOTS:
        if _exists(root):
            print(f"  [OK ] {root}")
            found_roots.append(root)
        else:
            print(f"  [   ] {root}")
    if not found_roots:
        print(
            "\nNo standard Scienta Omicron paths found. "
            "MISTRAL may be installed elsewhere — try --wide for broader search."
        )

    # Phase 2: log files in known roots.
    print(f"\n--- Phase 2: recent log files in known roots (last {args.max_age}d) ---")
    candidates = find_log_files(KNOWN_ROOTS, LOG_EXTS, max_age_days=args.max_age)
    if not candidates:
        print("  (none)")
    else:
        print(f"  {len(candidates)} candidate(s) — showing up to {args.limit}:\n")
        for mtime, size, path in candidates[: args.limit]:
            print(f"  [{format_mtime(mtime)}] {format_size(size)}  {path}")
            if args.peek:
                for line in peek(path):
                    print(f"      | {line}")
                print()

    # Phase 3 (optional): broader sweep, filtered by name keyword in path.
    if args.wide:
        print(
            f"\n--- Phase 3: wide sweep filtered by keyword in path "
            f"(last {args.max_age}d) ---"
        )
        wide = find_log_files(WIDE_ROOTS, LOG_EXTS, max_age_days=args.max_age)
        wide = [
            c for c in wide
            if any(k in c[2].lower() for k in NAME_KEYWORDS)
        ]
        if not wide:
            print("  (none with 'mistral'/'omicron'/'scienta' in path)")
        else:
            print(f"  {len(wide)} candidate(s) — showing up to {args.limit}:\n")
            for mtime, size, path in wide[: args.limit]:
                print(f"  [{format_mtime(mtime)}] {format_size(size)}  {path}")
                if args.peek:
                    for line in peek(path):
                        print(f"      | {line}")
                    print()

    # Phase 4 (optional): filename keyword search.
    if args.name_search:
        print("\n--- Phase 4: filename keyword search ---")
        named = find_named(WIDE_ROOTS, NAME_KEYWORDS)
        if not named:
            print("  (none)")
        else:
            print(f"  {len(named)} match(es) — showing up to {args.limit}:\n")
            for p in named[: args.limit]:
                print(f"  {p}")

    print("\nDone. Share interesting candidate paths back for driver design.")


if __name__ == "__main__":
    main()

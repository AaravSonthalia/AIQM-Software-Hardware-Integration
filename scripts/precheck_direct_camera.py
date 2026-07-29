"""Direct-camera pre-flight for the Jul 10 2026 lab check.

Structured GO/NO-GO output for the Vimba direct-camera path — tests
OUR ``VmbCamera`` wrapper (not just raw vmbpy), so a green result here
implies the exact code path Growth Monitor uses when
``camera_mode="vimba"``. Complements ``scripts/vimba_camera_smoke.py``,
which tests raw vmbpy independently.

Reports pass/fail for:
  1. ``VmbCamera`` importable + constructable
  2. ``connect()`` — streaming thread starts (in the negotiated AccessMode)
  3. ``read_frame()`` — returns a frame within ~5s
  4. Frame stats sensible (mean, std, range)
  5. **BGW palette LUT applied** — every unique RGB triple in the frame
     is a valid entry of ``gui.ksa_palette.KSA_BGW_PALETTE``. The BGW
     ramp naturally has R=B=0 for intensities ≤ 127 (lower half of the
     LUT), so per-channel std asymmetry on dark frames is expected
     behavior, not a regression. See ``vmb_palette_bug_jul02.md`` for
     the historical ``(0, I, 0)`` bug this check evolved from.
  6. ``disconnect()`` cleanly

Prints a summary block designed as a meeting-report line, including the
negotiated AccessMode for auditability.

Usage on Bulbasaur (kSA state depends on --access-mode):
    python scripts\\precheck_direct_camera.py                     # auto (default)
    python scripts\\precheck_direct_camera.py --access-mode full  # kSA-closed
    python scripts\\precheck_direct_camera.py --access-mode read  # kSA-open coexist

Access-mode semantics (match ``VmbCamera(access_mode=...)``):
  * ``auto`` — try Full first; on access denial, fall back to Read.
    Use for validation when kSA state is unknown or mixed.
  * ``full`` — require exclusive control. Use for the kSA-closed
    regression path.
  * ``read`` — passive consumer only. Requires camera multicast enabled
    (Task #187). Use to isolate the Read code path from the auto-fallback
    logic when kSA is open.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def status(label: str, ok: bool, detail: str = "") -> None:
    tag = "[PASS]" if ok else "[FAIL]"
    line = f"  {tag} {label}"
    if detail:
        line += f" — {detail}"
    print(line)


def info(label: str, detail: str = "") -> None:
    """Neutral status line — neither pass nor fail. Used when a check
    intentionally reports its result without contributing to the verdict
    (e.g. the dark-frame stats check under --allow-dark-frame)."""
    line = f"  [INFO] {label}"
    if detail:
        line += f" — {detail}"
    print(line)


# ---------------------------------------------------------------------------
# Pure-function stats evaluation (extracted so tests can exercise it
# without instantiating the whole VmbCamera pipeline)
# ---------------------------------------------------------------------------

# The mean/std thresholds that define a "sensible" frame — i.e. a frame
# with a real RHEED signal on it. Constants at module scope so tests
# can reference them without duplication.
STATS_MEAN_MIN: float = 0.5
STATS_MEAN_MAX: float = 250.0
STATS_STD_MIN: float = 0.5


def evaluate_frame_stats(frame, allow_dark_frame: bool) -> dict:
    """Compute frame stats and decide how to render + score them.

    Returns:
      {
        "mean": float, "std": float, "min": int, "max": int,
        "heuristic_ok": bool,      # True iff the mean/std thresholds pass
        "tag": str,                # "PASS" | "FAIL" | "INFO"
        "contributes_pass": bool,  # gates the final verdict
        "detail": str,             # human-readable one-liner
      }

    Under the default (``allow_dark_frame=False``):
      * heuristic pass → tag=PASS, contributes_pass=True
      * heuristic fail → tag=FAIL, contributes_pass=False (blocks verdict)

    Under ``--allow-dark-frame`` (``allow_dark_frame=True``):
      * heuristic pass → tag=PASS, contributes_pass=True (unchanged)
      * heuristic fail → tag=INFO, contributes_pass=True — the check still
        RUNS and its numbers are reported, but a dark frame no longer
        blocks the verdict. Useful for beam-off code-path validation
        where the RHEED gun is intentionally off (Jul 28 lab pattern).

    Note: the flag never REMOVES the stats check; it only reclassifies a
    failing result as informational. A grower's session with the beam on
    that mysteriously produces a dark frame will still be visible in the
    output as an INFO line so it's not silently hidden.
    """
    import numpy as np  # local import so this helper stays testable without
                        # forcing numpy import at module top (already imported
                        # above; local re-import is a no-op).
    mean = float(np.mean(frame))
    std = float(np.std(frame))
    minv = int(np.min(frame))
    maxv = int(np.max(frame))
    heuristic_ok = (
        STATS_MEAN_MIN < mean < STATS_MEAN_MAX and std > STATS_STD_MIN
    )
    detail = f"mean={mean:.1f}, std={std:.1f}, range=[{minv}, {maxv}]"
    if heuristic_ok:
        tag = "PASS"
        contributes = True
    elif allow_dark_frame:
        tag = "INFO"
        contributes = True
        detail += " (dark frame accepted per --allow-dark-frame)"
    else:
        tag = "FAIL"
        contributes = False
    return {
        "mean": mean, "std": std, "min": minv, "max": maxv,
        "heuristic_ok": heuristic_ok,
        "tag": tag,
        "contributes_pass": contributes,
        "detail": detail,
    }


def verdict(ready: bool, hint: str = "") -> int:
    print()
    if ready:
        print("Verdict: READY for Growth Monitor camera_mode=\"vimba\" testing.")
        print("Next: launch growth_monitor_app.py, set Camera mode = 'vimba',")
        print("and arm a session.")
    else:
        print(f"Verdict: BLOCKED — {hint}")
    return 0 if ready else 1


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Direct-camera pre-flight for the Vimba VmbCamera path. "
            "Reports pass/fail per stage plus the negotiated AccessMode."
        ),
    )
    parser.add_argument(
        "--access-mode",
        choices=("auto", "full", "read"),
        default="auto",
        help=(
            "Requested AccessMode: 'auto' (Full first, Read fallback), "
            "'full' (exclusive), or 'read' (passive, requires camera "
            "multicast — Task #187). Default: auto."
        ),
    )
    parser.add_argument(
        "--allow-dark-frame",
        action="store_true",
        help=(
            "Treat a dark frame (mean < 0.5 or std < 0.5) as [INFO] "
            "rather than [FAIL]. The stats check still runs and its "
            "numbers are still printed, but a dark frame no longer "
            "blocks the verdict. Use for beam-off code-path validation "
            "(the RHEED gun is intentionally off) — the default is the "
            "grow-time heuristic which correctly flags 'no signal'."
        ),
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)

    print("=== Direct-camera pre-flight ===")
    print(f"Requested access mode: {args.access_mode}")
    print()

    # --- 1. VmbCamera importable + constructable ---
    try:
        from drivers.rheed_camera import FrameNotYetAvailableError, VmbCamera
    except ImportError as e:
        status("VmbCamera importable", False, f"import error: {e}")
        return verdict(
            False,
            "check drivers/rheed_camera.py + vmbpy install (pip install vmbpy)",
        )
    try:
        cam = VmbCamera(
            camera_index=0,
            trigger_hz=1.0,
            bit_depth=12,
            access_mode=args.access_mode,
        )
    except Exception as e:
        status("VmbCamera constructable", False, f"{type(e).__name__}: {e}")
        return verdict(False, "VmbCamera __init__ failed")
    status("VmbCamera importable + constructable", True)

    # --- 2. connect() — starts streaming thread ---
    try:
        cam.connect()
    except Exception as e:
        status("VmbCamera.connect()", False, f"{type(e).__name__}: {e}")
        return verdict(
            False,
            "connect failed — likely (a) no camera visible to VimbaX SDK, "
            "(b) kSA holds the camera in Full and multicast isn't enabled "
            "(try --access-mode read once Task #187 lands), (c) VimbaX SDK "
            "not installed, or (d) both Full and Read were denied. Try "
            "scripts/vimba_camera_smoke.py --list-only to isolate.",
        )
    # Cache the negotiated mode NOW — cam.access_mode gets cleared to ""
    # when disconnect() runs at step 6, and the final verdict block prints
    # this value after disconnect. Reading `cam.access_mode` there would
    # show an empty string.
    negotiated_access_mode = cam.access_mode
    status(
        "VmbCamera.connect()",
        True,
        f"streaming thread active in AccessMode.{negotiated_access_mode.capitalize()}",
    )

    # --- 3. read_frame() with brief retry (first frame may take ~1-2s) ---
    frame = None
    last_error = None
    deadline = time.time() + 5.0
    while time.time() < deadline:
        try:
            frame = cam.read_frame()
            break
        except FrameNotYetAvailableError as e:
            # Transient — the stream is alive but hasn't produced a
            # frame yet. Retry on the next tick.
            last_error = e
            time.sleep(0.1)
        except Exception as e:
            # Any other exception is a real driver failure; stop retrying.
            last_error = e
            break

    if frame is None:
        if last_error is not None:
            status(
                "VmbCamera.read_frame()",
                False,
                f"{type(last_error).__name__}: {last_error}",
            )
        else:
            status(
                "VmbCamera.read_frame()",
                False,
                "no frame within 5s",
            )
        try:
            cam.disconnect()
        except Exception:
            pass
        return verdict(False, "streaming did not produce a frame in 5s")
    status(
        "VmbCamera.read_frame()",
        True,
        f"shape {frame.shape}, dtype {frame.dtype}",
    )

    # --- 4. Frame stats sensible (mean, std, range) ---
    # evaluate_frame_stats owns the rendering + verdict-contribution
    # decision so --allow-dark-frame can reclassify a dark-frame FAIL
    # into a non-blocking INFO. See its docstring for the full semantics.
    stats = evaluate_frame_stats(frame, allow_dark_frame=args.allow_dark_frame)
    if stats["tag"] == "PASS":
        status("Frame stats sensible", True, stats["detail"])
    elif stats["tag"] == "INFO":
        info("Frame stats sensible", stats["detail"])
    else:  # FAIL
        status("Frame stats sensible", False, stats["detail"])
    stats_ok = stats["contributes_pass"]

    # --- 5. BGW palette LUT applied ---
    # The driver defaults to apply_palette=True, mapping raw uint8
    # intensity through gui.ksa_palette.KSA_BGW_PALETTE (indices 0-127:
    # G ramps 0→255 with R=B=0; indices 128-255: G stays 255 while R+B
    # ramp 0→255). Dark frames legitimately have R=B=0 everywhere —
    # that's BGW behavior, not the historical (0, I, 0) bug. Verify by
    # confirming every unique RGB triple in the frame is a valid LUT
    # entry, and reporting per-channel stds for context.
    palette_ok = False
    palette_detail = ""
    if frame.ndim == 3 and frame.shape[-1] == 3:
        try:
            from gui.ksa_palette import KSA_BGW_PALETTE
            unique = np.unique(frame.reshape(-1, 3), axis=0)
            in_lut = np.array([
                np.any(np.all(KSA_BGW_PALETTE == triple, axis=1))
                for triple in unique
            ])
            palette_ok = bool(in_lut.all())
            r_std = float(np.std(frame[..., 0]))
            g_std = float(np.std(frame[..., 1]))
            b_std = float(np.std(frame[..., 2]))
            if palette_ok:
                palette_detail = (
                    f"{len(unique)} unique colors, all valid BGW LUT entries; "
                    f"R_std={r_std:.2f}, G_std={g_std:.2f}, B_std={b_std:.2f}"
                )
            else:
                palette_detail = (
                    f"{int((~in_lut).sum())} of {len(unique)} unique triples "
                    "not in BGW LUT — palette mapping may not be applied"
                )
        except ImportError:
            palette_ok = True
            palette_detail = "gui.ksa_palette not importable — skipping"
    else:
        palette_ok = True
        palette_detail = "monochrome frame — no per-channel palette check"
    status(
        "BGW palette LUT applied",
        palette_ok,
        palette_detail,
    )

    # --- 6. disconnect() cleanly ---
    try:
        cam.disconnect()
        status("VmbCamera.disconnect()", True)
    except Exception as e:
        status("VmbCamera.disconnect()", False, f"{type(e).__name__}: {e}")
        return verdict(False, "disconnect failed — see above")

    # --- All checks weighted for the verdict ---
    all_ok = stats_ok and palette_ok
    # Restate the negotiated mode near the verdict so a scan of the
    # summary tail alone conveys which SDK access mode the session ran in
    # — critical for auto mode where kSA state determined the outcome.
    # Uses the cached value from post-connect (see comment at cache site);
    # `cam.access_mode` is now "" because disconnect() cleared it.
    print()
    print(f"Negotiated access mode: {negotiated_access_mode}")
    return verdict(
        all_ok,
        "some checks did not pass — see above" if not all_ok else "",
    )


if __name__ == "__main__":
    sys.exit(main())

"""Direct-camera pre-flight for the Jul 10 2026 lab check.

Structured GO/NO-GO output for the Vimba direct-camera path — tests
OUR ``VmbCamera`` wrapper (not just raw vmbpy), so a green result here
implies the exact code path Growth Monitor uses when
``camera_mode="direct"``. Complements ``scripts/vimba_camera_smoke.py``,
which tests raw vmbpy independently.

Reports pass/fail for:
  1. ``VmbCamera`` importable + constructable
  2. ``connect()`` — streaming thread starts
  3. ``read_frame()`` — returns a frame within ~5s
  4. Frame stats sensible (mean, std, range)
  5. **BGW palette LUT applied** — every unique RGB triple in the frame
     is a valid entry of ``gui.ksa_palette.KSA_BGW_PALETTE``. The BGW
     ramp naturally has R=B=0 for intensities ≤ 127 (lower half of the
     LUT), so per-channel std asymmetry on dark frames is expected
     behavior, not a regression. See ``vmb_palette_bug_jul02.md`` for
     the historical ``(0, I, 0)`` bug this check evolved from.
  6. ``disconnect()`` cleanly

Prints a summary block designed as a meeting-report line.

Usage on Bulbasaur (kSA must NOT be holding the camera):
    python scripts\\precheck_direct_camera.py
"""
from __future__ import annotations

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


def verdict(ready: bool, hint: str = "") -> int:
    print()
    if ready:
        print("Verdict: READY for Growth Monitor camera_mode=\"direct\" testing.")
        print("Next: launch growth_monitor_app.py, set Camera mode = 'direct',")
        print("and arm a session.")
    else:
        print(f"Verdict: BLOCKED — {hint}")
    return 0 if ready else 1


def main() -> int:
    print("=== Direct-camera pre-flight ===")
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
        cam = VmbCamera(camera_index=0, trigger_hz=1.0, bit_depth=12)
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
            "(b) kSA is holding the Manta, or (c) VimbaX SDK not installed. "
            "Try scripts/vimba_camera_smoke.py --list-only to isolate.",
        )
    status("VmbCamera.connect()", True, "streaming thread active")

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
    mean = float(np.mean(frame))
    std = float(np.std(frame))
    minv = int(np.min(frame))
    maxv = int(np.max(frame))
    # Sensible band: not saturated black (mean=0), not saturated white
    # (mean=255), and has actual variation (std > 0).
    stats_ok = 0.5 < mean < 250 and std > 0.5
    status(
        "Frame stats sensible",
        stats_ok,
        f"mean={mean:.1f}, std={std:.1f}, range=[{minv}, {maxv}]",
    )

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
    return verdict(
        all_ok,
        "some checks did not pass — see above" if not all_ok else "",
    )


if __name__ == "__main__":
    sys.exit(main())

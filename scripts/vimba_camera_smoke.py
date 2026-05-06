"""Vimba SDK direct-camera smoke test.

Verifies that vmbpy can enumerate and open the Allied Vision Manta camera
on Bulbasaur, then grab a single frame and save it as PNG. The smallest-
commitment validation of the Tier-1 direct-camera path that would
eventually replace the screengrab pipeline (which is vulnerable to
kSA window-state changes — see Apr 29 event_008 finding).

Adapted from the camera-handling kernel in Jacques' OxideMBE_RHEED_GUI.py
(lines 489-573), simplified to a single-frame synchronous grab.

Usage on Bulbasaur (kSA must NOT be holding the camera):
    python scripts/vimba_camera_smoke.py            # open + grab + save
    python scripts/vimba_camera_smoke.py --list-only  # just enumerate

Three failure modes worth knowing:
    1. Import fails → vmbpy not installed; ``pip install vmbpy``
    2. Import succeeds, no cameras found → VimbaX SDK absent, or kSA
       holds the camera. Close kSA and try again. Verify VimbaX install
       at ``C:\\Program Files\\Allied Vision\\Vimba_X\\``.
    3. Camera opens but get_frame() hangs/errors → driver / camera
       state issue; check Allied Vision Vimba Viewer if available.

Prereqs:
    - VimbaX SDK installed on Bulbasaur
    - ``pip install vmbpy`` in the AIQM venv
    - kSA closed (or otherwise not currently streaming from the Manta)
"""
from __future__ import annotations

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from PIL import Image

try:
    from vmbpy import VmbSystem
except ImportError as exc:  # pragma: no cover
    print(
        f"Missing vmbpy: {exc}\n"
        "Install with: pip install vmbpy\n"
        "(Requires VimbaX SDK to be installed on the system.)",
        file=sys.stderr,
    )
    raise


def normalize_to_uint8(arr: np.ndarray) -> np.ndarray:
    """Coerce a frame array to uint8 for PNG saving."""
    if arr.dtype == np.uint8:
        return arr
    a_min, a_max = float(arr.min()), float(arr.max())
    if a_max <= a_min:
        a_max = a_min + 1.0
    return ((arr - a_min) / (a_max - a_min) * 255.0).astype(np.uint8)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="Vimba SDK direct-camera smoke test"
    )
    p.add_argument(
        "--output",
        type=Path,
        default=Path("vimba_smoke.png"),
        help="Output PNG path (default: vimba_smoke.png in cwd)",
    )
    p.add_argument(
        "--list-only",
        action="store_true",
        help="Enumerate cameras without opening one (safer first run)",
    )
    args = p.parse_args(argv)

    print("Opening VmbSystem...")
    with VmbSystem.get_instance() as vmb:
        cams = vmb.get_all_cameras()
        print(f"Found {len(cams)} camera(s)")
        for i, c in enumerate(cams):
            try:
                print(
                    f"  [{i}] id={c.get_id()}  model={c.get_model()}  "
                    f"serial={c.get_serial()}"
                )
            except Exception as e:
                print(f"  [{i}] (could not read identity: {e})")

        if not cams:
            print(
                "\nNo cameras visible to vmbpy. Verify (a) VimbaX SDK is "
                "installed, and (b) no other process (kSA, Vimba Viewer) "
                "currently holds the Manta.",
                file=sys.stderr,
            )
            return 2

        if args.list_only:
            print("\n--list-only set; not opening camera.")
            return 0

        cam = cams[0]
        print(f"\nOpening camera {cam.get_id()}...")
        with cam:
            print("Grabbing frame (synchronous get_frame)...")
            frame = cam.get_frame()
            arr = frame.as_numpy_ndarray()
            arr_2d = np.squeeze(arr)

            print(
                f"Got frame: shape={arr_2d.shape}, dtype={arr.dtype}, "
                f"range=[{arr.min()}, {arr.max()}], mean={float(arr.mean()):.2f}"
            )

            img = normalize_to_uint8(arr_2d)
            ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
            out = args.output.with_stem(f"{args.output.stem}_{ts}")
            Image.fromarray(img).save(out)
            print(f"Saved: {out}")

    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

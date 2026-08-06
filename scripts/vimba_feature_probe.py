"""Vimba X / vmbpy camera feature probe — discovery for GUI exposure control.

Answers one question the Vimba X Viewer cannot: what are the *GenICam feature
names* behind the Viewer's Brightness sliders, and are they writable from
vmbpy on this camera's firmware?

The Manta G-033B is a legacy AVT GigE camera. Its classic feature set uses
``ExposureTimeAbs`` (float, us) and ``GainRaw``/``Gain``; the SFNC standard
that newer cameras follow uses ``ExposureTime``/``Gain``. Vimba X may present
either, depending on the GenTL producer and firmware (this camera reports
firmware 00.01.44.18241). Hardcoding the wrong name in drivers/rheed_camera.py
would fail at runtime on the lab machine only, so this script establishes the
truth before any driver change is written.

SAFETY LADDER (same pattern as scripts/pyrometer_force_modbus.py):

  Phase 1 (default)  READ-ONLY. Enumerates every feature with its type,
                     access mode, range, increment, unit and current value.
                     Touches nothing. Safe to run any time the camera is free.

  Phase 2 (gated)    Requires --i-am-doing-a-write. Sets exposure to a test
                     value, reads it back, then RESTORES the original in a
                     finally block. Never calls UserSetSave/UserSetStore, so
                     every write is RAM-only and reverts on camera power-cycle
                     even if this script is killed mid-run.

PREREQUISITE: close the Vimba X Viewer first. It holds the camera in Full
access; while it is open this script can only get Read access and every
feature will report writable=False, which looks identical to "the firmware
forbids it". The script warns when it lands in Read mode for this reason.

Usage:
    python scripts/vimba_feature_probe.py
    python scripts/vimba_feature_probe.py --output logs/vimba_features.json
    python scripts/vimba_feature_probe.py --filter exposure
    python scripts/vimba_feature_probe.py --i-am-doing-a-write --test-exposure-us 150000
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from typing import Any, Optional

# Feature-name candidates for the two knobs the growers asked for. Ordered
# most-likely-first for this camera family; the probe reports which exist
# rather than assuming any of them do.
EXPOSURE_CANDIDATES = (
    "ExposureTimeAbs",      # legacy AVT GigE (expected on Manta)
    "ExposureTime",         # SFNC standard
    "ExposureTimeRaw",
    "ExposureAuto",
    "ExposureAutoTarget",
    "ExposureMode",
)
GAIN_CANDIDATES = (
    "Gain",
    "GainRaw",
    "GainAuto",
    "GainSelector",
)
# Read for context: these shape how a raw pixel value maps to the intensity
# the GUI plots. BlackLevel is an additive pedestal (35 DN on this camera as
# of 2026-08-06) and Gamma a non-linearity — both matter for whether the
# RHEED intensity trend can ever be a physical quantity.
CONTEXT_CANDIDATES = (
    "BlackLevel",
    "BlackLevelRaw",
    "Gamma",
    "PixelFormat",
    "Width",
    "Height",
    "AcquisitionFrameRateAbs",
    "AcquisitionFrameRate",
    "DeviceFirmwareVersion",
    "DeviceModelName",
    "DeviceSerialNumber",
)


def _safe(fn, default=None):
    """Call a vmbpy accessor, returning `default` if the SDK refuses.

    vmbpy raises for accessors that do not apply to a given feature type
    (get_range on a string, get() on a command). Probing must never abort
    on one awkward feature when the point is to survey all of them.
    """
    try:
        return fn()
    except Exception as exc:  # noqa: BLE001 — surveying unknown SDK surface
        return default if default is not None else f"<unavailable: {exc}>"


def describe_feature(feature) -> dict[str, Any]:
    """Collect everything interesting about one feature, defensively."""
    name = _safe(feature.get_name, "<unknown>")
    info: dict[str, Any] = {
        "name": name,
        "display_name": _safe(feature.get_display_name, ""),
        "type": type(feature).__name__,
        "unit": _safe(feature.get_unit, ""),
        "tooltip": _safe(feature.get_tooltip, ""),
    }

    access = _safe(feature.get_access_mode, None)
    if isinstance(access, tuple) and len(access) == 2:
        info["readable"], info["writable"] = bool(access[0]), bool(access[1])
    else:
        info["readable"], info["writable"] = None, None
        info["access_error"] = str(access)

    # Command features have no value; reading one would execute nothing but
    # does raise. Skip by type name rather than by try/except so the report
    # distinguishes "command" from "read failed".
    if "Command" not in info["type"]:
        value = _safe(feature.get, None)
        info["value"] = value if _is_jsonable(value) else str(value)

    rng = _safe(feature.get_range, None)
    if isinstance(rng, tuple) and len(rng) == 2:
        info["min"], info["max"] = _jsonable(rng[0]), _jsonable(rng[1])

    inc = _safe(feature.get_increment, None)
    if inc is not None and not isinstance(inc, str):
        info["increment"] = _jsonable(inc)

    # Enum features carry the legal strings — needed to know whether
    # ExposureAuto accepts "Off"/"Once"/"Continuous" verbatim.
    entries = _safe(feature.get_available_entries, None)
    if isinstance(entries, (list, tuple)):
        info["entries"] = [str(e) for e in entries]

    return info


def _is_jsonable(value: Any) -> bool:
    return isinstance(value, (int, float, str, bool, type(None)))


def _jsonable(value: Any) -> Any:
    return value if _is_jsonable(value) else str(value)


def open_camera(vmbpy, vmb, index: int, mode: str):
    """Open camera `index` in `mode`, returning (camera, negotiated_mode).

    Mirrors drivers/rheed_camera.py: Full is tried first unless the caller
    pins a mode, because feature writes need Full and a silent Read fallback
    would make a writable camera look read-only.
    """
    cams = vmb.get_all_cameras()
    if not cams:
        raise RuntimeError(
            "No cameras found. Check the GigE link (this camera is on a "
            "link-local 169.254.x.x address, so it needs a direct NIC) and "
            "that the Vimba X transport layer is installed."
        )
    if index >= len(cams):
        raise RuntimeError(f"camera_index {index} out of range; found {len(cams)}")

    cam = cams[index]
    attempts = ("full", "read") if mode == "auto" else (mode,)
    last_error: Optional[Exception] = None
    for attempt in attempts:
        try:
            cam.set_access_mode(getattr(vmbpy.AccessMode, attempt.capitalize()))
            cam.__enter__()
            return cam, attempt
        except Exception as exc:  # noqa: BLE001 — SDK raises several types
            last_error = exc
            continue
    raise RuntimeError(f"Could not open camera in {attempts}: {last_error}")


def find_first_present(features: dict[str, dict], names) -> Optional[str]:
    for candidate in names:
        if candidate in features:
            return candidate
    return None


def write_test(feature, test_value: float) -> dict[str, Any]:
    """Set → read back → restore. The restore runs even on failure.

    Deliberately does NOT call UserSetSave: the write stays in volatile
    camera RAM, so a power-cycle is a guaranteed escape hatch regardless of
    how this script exits.
    """
    original = feature.get()
    result: dict[str, Any] = {
        "original": _jsonable(original),
        "requested": test_value,
        "restored": None,
        "readback": None,
        "accepted": False,
        "error": "",
    }
    try:
        feature.set(test_value)
        readback = feature.get()
        result["readback"] = _jsonable(readback)
        # GenICam quantises to the feature increment, so exact equality is
        # the wrong test — accept anything within one increment.
        increment = _safe(feature.get_increment, 1.0)
        tolerance = abs(float(increment)) if isinstance(increment, (int, float)) else 1.0
        result["accepted"] = abs(float(readback) - float(test_value)) <= max(tolerance, 1.0)
    except Exception as exc:  # noqa: BLE001
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            feature.set(original)
            result["restored"] = _jsonable(feature.get())
        except Exception as exc:  # noqa: BLE001
            result["restored"] = f"<RESTORE FAILED: {exc}>"
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument(
        "--access-mode", choices=("auto", "full", "read"), default="auto",
    )
    parser.add_argument(
        "--filter", default="",
        help="Case-insensitive substring; limits the printed table only. "
             "The JSON report always contains every feature.",
    )
    parser.add_argument("--output", default="", help="Write full JSON report here")
    parser.add_argument(
        "--i-am-doing-a-write", action="store_true",
        help="Enable Phase 2: the set/readback/restore exposure test.",
    )
    parser.add_argument(
        "--test-exposure-us", type=float, default=150000.0,
        help="Exposure to set during Phase 2 (default 150000 us = 150 ms, "
             "half the 2026-08-06 observed value of 300000 us).",
    )
    args = parser.parse_args()

    try:
        import vmbpy
    except ImportError:
        print(
            "vmbpy not installed. On the lab machine this ships with the "
            "Vimba X SDK; confirm the venv can see it.", file=sys.stderr,
        )
        return 1

    report: dict[str, Any] = {
        "probed_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "vmbpy_version": getattr(vmbpy, "__version__", "<unknown>"),
        "phase_2_attempted": bool(args.i_am_doing_a_write),
    }

    with vmbpy.VmbSystem.get_instance() as vmb:
        cam, mode = open_camera(vmbpy, vmb, args.camera_index, args.access_mode)
        report["access_mode"] = mode
        try:
            report["camera"] = {
                "id": _safe(cam.get_id, ""),
                "model": _safe(cam.get_model, ""),
                "serial": _safe(cam.get_serial, ""),
                "interface": _safe(cam.get_interface_id, ""),
            }
            print(f"Camera : {report['camera']['model']} ({report['camera']['serial']})")
            print(f"Access : {mode.upper()}")
            if mode != "full":
                print(
                    "\n  !! Opened in READ mode. Every feature below will report\n"
                    "     writable=False regardless of what the firmware allows.\n"
                    "     Close the Vimba X Viewer and re-run for a valid answer.\n"
                )

            features: dict[str, dict] = {}
            for feature in cam.get_all_features():
                info = describe_feature(feature)
                features[info["name"]] = info
            report["feature_count"] = len(features)
            report["features"] = features

            exposure_name = find_first_present(features, EXPOSURE_CANDIDATES)
            gain_name = find_first_present(features, GAIN_CANDIDATES)
            report["resolved"] = {
                "exposure_feature": exposure_name,
                "gain_feature": gain_name,
            }

            print(f"\n{len(features)} features enumerated.\n")
            _print_group("EXPOSURE", EXPOSURE_CANDIDATES, features)
            _print_group("GAIN", GAIN_CANDIDATES, features)
            _print_group("CONTEXT", CONTEXT_CANDIDATES, features)

            if args.filter:
                needle = args.filter.lower()
                matches = {
                    n: f for n, f in features.items() if needle in n.lower()
                }
                _print_group(f"FILTER '{args.filter}'", tuple(matches), features)

            print("\n--- VERDICT ---")
            print(f"Exposure feature : {exposure_name or 'NOT FOUND'}")
            print(f"Gain feature     : {gain_name or 'NOT FOUND'}")
            if exposure_name:
                writable = features[exposure_name].get("writable")
                print(f"Exposure writable: {writable}")
                if writable and not args.i_am_doing_a_write:
                    print(
                        "\nExposure appears writable. Re-run with "
                        "--i-am-doing-a-write to verify a real set/readback "
                        "(the value is restored afterwards)."
                    )

            if args.i_am_doing_a_write:
                if not exposure_name:
                    print("\nPhase 2 skipped: no exposure feature found.")
                elif mode != "full":
                    print("\nPhase 2 skipped: needs Full access.")
                else:
                    print(f"\n--- PHASE 2: write test on {exposure_name} ---")
                    outcome = write_test(
                        getattr(cam, exposure_name), args.test_exposure_us,
                    )
                    report["write_test"] = outcome
                    for key, value in outcome.items():
                        print(f"  {key:10s}: {value}")
        finally:
            cam.__exit__(None, None, None)

    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, sort_keys=True)
        print(f"\nFull report written to {args.output}")

    return 0


def _print_group(title: str, names, features: dict[str, dict]) -> None:
    print(f"[{title}]")
    found = False
    for name in names:
        info = features.get(name)
        if info is None:
            continue
        found = True
        bounds = ""
        if "min" in info:
            bounds = f"  range=[{info['min']}, {info['max']}]"
            if "increment" in info:
                bounds += f" inc={info['increment']}"
        entries = f"  entries={info['entries']}" if "entries" in info else ""
        print(
            f"  {name:26s} {str(info.get('type', '')):18s} "
            f"r={info.get('readable')} w={info.get('writable')} "
            f"value={info.get('value')!r} {info.get('unit', '')}{bounds}{entries}"
        )
    if not found:
        print("  (none of these names exist on this camera)")
    print()


if __name__ == "__main__":
    sys.exit(main())

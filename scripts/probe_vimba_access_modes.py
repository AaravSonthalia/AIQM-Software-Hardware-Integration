#!/usr/bin/env python3
"""Probe VmbPy/Vimba X install state and cautious camera access modes.

Run this on Bulbasaur to answer two separate questions:

1. Is VmbPy importable, and can it see the Allied Vision camera?
2. If kSA is open, can VmbPy open the camera in AccessMode.Read and receive
   frames without taking full control?

The default run is inventory-only. It imports VmbPy, starts VmbSystem, lists
interfaces/cameras, and writes structured JSON. It does not open the camera
unless an explicit --attempt-* flag is supplied.
"""
from __future__ import annotations

import argparse
import importlib.metadata
import json
import platform
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable


@dataclass
class StepResult:
    name: str
    ok: bool
    elapsed_ms: float
    detail: dict[str, Any] = field(default_factory=dict)
    error: str = ""


def _now_utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def _safe_str(value: Any) -> str:
    try:
        return str(value)
    except Exception as exc:  # noqa: BLE001 - defensive evidence capture
        return f"<unprintable {type(exc).__name__}: {exc}>"


def _call(obj: Any, method_name: str) -> Any:
    method = getattr(obj, method_name)
    return method()


def _safe_method(obj: Any, method_name: str) -> dict[str, Any]:
    try:
        return {"ok": True, "value": _safe_str(_call(obj, method_name))}
    except Exception as exc:  # noqa: BLE001
        return {
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
        }


def _step(name: str, fn: Callable[[], dict[str, Any]]) -> StepResult:
    start = time.monotonic()
    try:
        detail = fn()
        return StepResult(
            name=name,
            ok=True,
            elapsed_ms=round((time.monotonic() - start) * 1000.0, 2),
            detail=detail,
        )
    except Exception as exc:  # noqa: BLE001
        return StepResult(
            name=name,
            ok=False,
            elapsed_ms=round((time.monotonic() - start) * 1000.0, 2),
            error=f"{type(exc).__name__}: {exc}",
        )


def _package_version(package_name: str) -> str | None:
    try:
        return importlib.metadata.version(package_name)
    except importlib.metadata.PackageNotFoundError:
        return None


def _import_vmbpy() -> tuple[Any | None, StepResult]:
    def run() -> dict[str, Any]:
        import vmbpy  # type: ignore

        return {
            "module_file": getattr(vmbpy, "__file__", ""),
            "package_version": _package_version("vmbpy"),
            "has_vmb_system": hasattr(vmbpy, "VmbSystem"),
            "has_access_mode": hasattr(vmbpy, "AccessMode"),
        }

    result = _step("import_vmbpy", run)
    if not result.ok:
        return None, result
    import vmbpy  # type: ignore

    return vmbpy, result


def _camera_identity(cam: Any) -> dict[str, Any]:
    methods = [
        "get_id",
        "get_extended_id",
        "get_name",
        "get_model",
        "get_serial",
        "get_interface_id",
        "get_permitted_access_modes",
    ]
    return {method: _safe_method(cam, method) for method in methods}


def _interface_identity(interface: Any) -> dict[str, Any]:
    methods = ["get_id", "get_name", "get_type"]
    return {method: _safe_method(interface, method) for method in methods}


def _frame_summary(frame: Any) -> dict[str, Any]:
    info: dict[str, Any] = {
        "frame": _safe_str(frame),
    }
    try:
        arr = frame.as_numpy_ndarray()
        info.update(
            {
                "array_shape": list(arr.shape),
                "array_dtype": str(arr.dtype),
                "array_min": float(arr.min()),
                "array_max": float(arr.max()),
                "array_mean": float(arr.mean()),
            }
        )
    except Exception as exc:  # noqa: BLE001
        info["array_error"] = f"{type(exc).__name__}: {exc}"
    return info


def _list_system(vmbpy: Any) -> StepResult:
    def run() -> dict[str, Any]:
        VmbSystem = vmbpy.VmbSystem
        with VmbSystem.get_instance() as vmb:
            detail: dict[str, Any] = {}
            try:
                detail["system_version"] = _safe_str(VmbSystem.get_version())
            except Exception as exc:  # noqa: BLE001
                detail["system_version_error"] = f"{type(exc).__name__}: {exc}"

            interfaces = []
            try:
                interfaces = vmb.get_all_interfaces()
            except Exception as exc:  # noqa: BLE001
                detail["interfaces_error"] = f"{type(exc).__name__}: {exc}"

            cameras = []
            try:
                cameras = vmb.get_all_cameras()
            except Exception as exc:  # noqa: BLE001
                detail["cameras_error"] = f"{type(exc).__name__}: {exc}"

            detail["interface_count"] = len(interfaces)
            detail["interfaces"] = [
                _interface_identity(interface) for interface in interfaces
            ]
            detail["camera_count"] = len(cameras)
            detail["cameras"] = [_camera_identity(cam) for cam in cameras]
            return detail

    return _step("inventory_vmb_system", run)


def _open_camera(
    vmbpy: Any,
    camera_index: int,
    access_mode_name: str,
    list_features: bool,
) -> StepResult:
    def run() -> dict[str, Any]:
        VmbSystem = vmbpy.VmbSystem
        AccessMode = vmbpy.AccessMode
        access_mode = getattr(AccessMode, access_mode_name)
        with VmbSystem.get_instance() as vmb:
            cameras = vmb.get_all_cameras()
            if not cameras:
                raise RuntimeError("No cameras visible to VmbPy.")
            cam = cameras[camera_index]
            cam.set_access_mode(access_mode)
            detail: dict[str, Any] = {
                "requested_access_mode": access_mode_name,
                "camera": _camera_identity(cam),
            }
            with cam:
                detail["current_access_mode"] = _safe_method(
                    cam, "get_access_mode"
                )
                detail["is_streaming"] = _safe_method(cam, "is_streaming")
                if list_features:
                    features = cam.get_all_features()
                    detail["feature_count"] = len(features)
                    detail["features"] = [
                        {
                            "name": _safe_method(feature, "get_name"),
                            "type": _safe_method(feature, "get_type"),
                            "access": _safe_method(
                                feature, "get_access_mode"
                            ),
                        }
                        for feature in features
                    ]
            return detail

    return _step(f"open_camera_{access_mode_name.lower()}", run)


def _read_stream_probe(
    vmbpy: Any,
    camera_index: int,
    duration_s: float,
) -> StepResult:
    def run() -> dict[str, Any]:
        VmbSystem = vmbpy.VmbSystem
        AccessMode = vmbpy.AccessMode
        frames: list[dict[str, Any]] = []
        handler_errors: list[str] = []

        def handler(cam: Any, _stream: Any, frame: Any) -> None:
            try:
                if not frames:
                    frames.append(_frame_summary(frame))
            except Exception as exc:  # noqa: BLE001
                handler_errors.append(f"{type(exc).__name__}: {exc}")
            finally:
                try:
                    cam.queue_frame(frame)
                except Exception as exc:  # noqa: BLE001
                    handler_errors.append(
                        f"queue_frame {type(exc).__name__}: {exc}"
                    )

        with VmbSystem.get_instance() as vmb:
            cameras = vmb.get_all_cameras()
            if not cameras:
                raise RuntimeError("No cameras visible to VmbPy.")
            cam = cameras[camera_index]
            cam.set_access_mode(AccessMode.Read)
            with cam:
                current_access_mode = _safe_method(cam, "get_access_mode")
                cam.start_streaming(handler)
                try:
                    time.sleep(duration_s)
                finally:
                    cam.stop_streaming()

        return {
            "requested_access_mode": "Read",
            "current_access_mode": current_access_mode,
            "duration_s": duration_s,
            "frame_count_observed": len(frames),
            "first_frame": frames[0] if frames else None,
            "handler_errors": handler_errors,
        }

    return _step("read_mode_stream_probe", run)


def build_payload(results: list[StepResult]) -> dict[str, Any]:
    return {
        "created_at_utc": _now_utc(),
        "platform": {
            "python": sys.version,
            "platform": platform.platform(),
            "machine": platform.machine(),
        },
        "results": [asdict(result) for result in results],
    }


def default_result_path() -> Path:
    if sys.platform == "win32":
        return Path(r"C:\temp\aiqm_vimba_access_probe\result.json")
    return Path("/tmp/aiqm_vimba_access_probe/result.json")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument(
        "--attempt-read-open",
        action="store_true",
        help=(
            "Open the camera in VmbPy AccessMode.Read. Use with kSA open to "
            "test whether read-only access is allowed."
        ),
    )
    parser.add_argument(
        "--attempt-read-stream",
        action="store_true",
        help=(
            "Open in AccessMode.Read and start a short passive stream. This "
            "is the multicast/coexistence probe."
        ),
    )
    parser.add_argument(
        "--attempt-full-open",
        action="store_true",
        help=(
            "Open the camera in AccessMode.Full. Only use when kSA is closed; "
            "this intentionally asks VmbPy to own the camera."
        ),
    )
    parser.add_argument(
        "--list-features",
        action="store_true",
        help="List camera features after a successful explicit open.",
    )
    parser.add_argument("--duration-s", type=float, default=3.0)
    parser.add_argument(
        "--result-json",
        type=Path,
        default=default_result_path(),
    )
    args = parser.parse_args(argv)

    results: list[StepResult] = []
    vmbpy, import_result = _import_vmbpy()
    results.append(import_result)

    if vmbpy is not None:
        results.append(_list_system(vmbpy))
        if args.attempt_read_open:
            results.append(
                _open_camera(
                    vmbpy=vmbpy,
                    camera_index=args.camera_index,
                    access_mode_name="Read",
                    list_features=args.list_features,
                )
            )
        if args.attempt_read_stream:
            results.append(
                _read_stream_probe(
                    vmbpy=vmbpy,
                    camera_index=args.camera_index,
                    duration_s=args.duration_s,
                )
            )
        if args.attempt_full_open:
            results.append(
                _open_camera(
                    vmbpy=vmbpy,
                    camera_index=args.camera_index,
                    access_mode_name="Full",
                    list_features=args.list_features,
                )
            )

    payload = build_payload(results)
    args.result_json.parent.mkdir(parents=True, exist_ok=True)
    args.result_json.write_text(json.dumps(payload, indent=2) + "\n")

    for result in results:
        status = "OK" if result.ok else "FAIL"
        print(f"[{status}] {result.name} ({result.elapsed_ms} ms)")
        if result.error:
            print(f"  {result.error}")
    print(f"\nWrote: {args.result_json}")

    return 0 if all(result.ok for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())

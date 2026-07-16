#!/usr/bin/env python3
"""Probe kSA TEXT_CMD commands for concurrent image export.

The goal is evidence, not assumptions. Run this on Bulbasaur with kSA open
and an acquisition/live image available. The script sends a small list of
candidate TEXT_CMD strings, records each reply, and checks whether kSA wrote
an image file for commands that include ``{path}``.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from drivers.ksa_comm import KsaCommClient, classify_error, err_name  # noqa: E402


DEFAULT_PROBES = [
    "help",
    "?commands",
    "roi list",
    'image save "{path}"',
    'save current image "{path}"',
    'roi save "{path}"',
    "get_pixel_data 0 0 100 100",
]


@dataclass
class ProbeResult:
    index: int
    command_template: str
    command: str
    err_code: int | None
    err_name: str
    error_category: str
    reply_text: str
    elapsed_ms: float
    output_path: str | None
    output_exists: bool
    output_size_bytes: int | None
    file_type: str | None
    image_mode: str | None
    image_width: int | None
    image_height: int | None
    image_dtype: str | None
    image_min: float | None
    image_max: float | None
    pil_bit_depth_hint: int | None
    observed_bit_depth: int | None
    image_error: str


def _slug(text: str) -> str:
    chars = []
    for ch in text.lower():
        if ch.isalnum():
            chars.append(ch)
        elif chars and chars[-1] != "_":
            chars.append("_")
    slug = "".join(chars).strip("_")
    return slug[:40] or "probe"


def _output_path(out_dir: Path, idx: int, command_template: str, ext: str) -> Path | None:
    if "{path}" not in command_template:
        return None
    return out_dir / f"probe_{idx:02d}_{_slug(command_template)}.{ext.lstrip('.')}"


def _pil_bit_depth_hint(mode: str) -> int | None:
    if mode == "1":
        return 1
    if mode in {"L", "P", "RGB", "RGBA"}:
        return 8
    if mode in {"I;16", "I;16B", "I;16L"}:
        return 16
    if mode in {"I", "F"}:
        return 32
    return None


def _observed_bit_depth(max_value: float | None) -> int | None:
    if max_value is None or max_value <= 0:
        return None
    return max(1, math.ceil(math.log2(max_value + 1)))


def inspect_image(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        from PIL import Image
        import numpy as np

        with Image.open(path) as img:
            file_type = img.format
            image_mode = img.mode
            width, height = img.size
            arr = np.array(img)
        image_min = float(np.min(arr)) if arr.size else None
        image_max = float(np.max(arr)) if arr.size else None
        return {
            "file_type": file_type,
            "image_mode": image_mode,
            "image_width": width,
            "image_height": height,
            "image_dtype": str(arr.dtype),
            "image_min": image_min,
            "image_max": image_max,
            "pil_bit_depth_hint": _pil_bit_depth_hint(image_mode),
            "observed_bit_depth": _observed_bit_depth(image_max),
            "image_error": "",
        }
    except Exception as exc:  # noqa: BLE001 - evidence capture, not control flow
        return {"image_error": f"{type(exc).__name__}: {exc}"}


def wait_for_file(path: Path, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.exists() and path.stat().st_size > 0:
            return True
        time.sleep(0.05)
    return path.exists() and path.stat().st_size > 0


def run_probe(
    client: KsaCommClient,
    idx: int,
    command_template: str,
    out_dir: Path,
    ext: str,
    file_wait_s: float,
) -> ProbeResult:
    path = _output_path(out_dir, idx, command_template, ext)
    if path is not None and path.exists():
        path.unlink()
    command = command_template.format(path=str(path)) if path else command_template

    t0 = time.monotonic()
    err_code = None
    reply_text = ""
    try:
        err_code, reply_text = client.send_text(command)
    except Exception as exc:  # noqa: BLE001
        reply_text = f"{type(exc).__name__}: {exc}"
    elapsed_ms = (time.monotonic() - t0) * 1000.0

    if path is not None and err_code == 0:
        wait_for_file(path, file_wait_s)

    output_exists = path.exists() if path is not None else False
    output_size = path.stat().st_size if output_exists else None
    image_info = inspect_image(path) if output_exists and path is not None else {}
    category = (
        classify_error(reply_text).value
        if err_code not in (None, 0)
        else ""
    )

    return ProbeResult(
        index=idx,
        command_template=command_template,
        command=command,
        err_code=err_code,
        err_name="exception" if err_code is None else err_name(err_code),
        error_category=category,
        reply_text=reply_text,
        elapsed_ms=round(elapsed_ms, 2),
        output_path=str(path) if path is not None else None,
        output_exists=output_exists,
        output_size_bytes=output_size,
        file_type=image_info.get("file_type"),
        image_mode=image_info.get("image_mode"),
        image_width=image_info.get("image_width"),
        image_height=image_info.get("image_height"),
        image_dtype=image_info.get("image_dtype"),
        image_min=image_info.get("image_min"),
        image_max=image_info.get("image_max"),
        pil_bit_depth_hint=image_info.get("pil_bit_depth_hint"),
        observed_bit_depth=image_info.get("observed_bit_depth"),
        image_error=image_info.get("image_error", ""),
    )


def build_payload(
    host: str,
    port: int,
    out_dir: Path,
    results: list[ProbeResult],
) -> dict[str, Any]:
    export_successes = [
        r for r in results if r.err_code == 0 and r.output_path and r.output_exists
    ]
    text_successes = [r for r in results if r.err_code == 0]
    return {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "host": host,
        "port": port,
        "output_dir": str(out_dir),
        "result_count": len(results),
        "text_success_count": len(text_successes),
        "export_success_count": len(export_successes),
        "results": [asdict(r) for r in results],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=1800)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(r"C:\temp\aiqm_ksa_image_probe")
        if sys.platform == "win32"
        else Path("/tmp/aiqm_ksa_image_probe"),
    )
    parser.add_argument("--ext", default="bmp")
    parser.add_argument("--file-wait-s", type=float, default=2.0)
    parser.add_argument(
        "--probe",
        action="append",
        default=[],
        help="Additional TEXT_CMD probe. Include {path} if it should write a file.",
    )
    parser.add_argument(
        "--result-json",
        type=Path,
        default=None,
        help="Optional explicit JSON result path.",
    )
    args = parser.parse_args(argv)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    commands = DEFAULT_PROBES + list(args.probe)

    results: list[ProbeResult] = []
    with KsaCommClient(args.host, args.port, args.timeout) as client:
        for idx, command_template in enumerate(commands, start=1):
            result = run_probe(
                client=client,
                idx=idx,
                command_template=command_template,
                out_dir=args.out_dir,
                ext=args.ext,
                file_wait_s=args.file_wait_s,
            )
            results.append(result)
            print(
                f"[{idx:02d}] err={result.err_code} "
                f"file={result.output_exists} cmd={result.command!r}"
            )

    payload = build_payload(args.host, args.port, args.out_dir, results)
    result_json = args.result_json
    if result_json is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        result_json = args.out_dir / f"ksa_image_export_probe_{stamp}.json"
    result_json.write_text(json.dumps(payload, indent=2) + "\n")

    print()
    print(f"Wrote: {result_json}")
    print(f"TEXT_CMD successes: {payload['text_success_count']}")
    print(f"Image export successes: {payload['export_success_count']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

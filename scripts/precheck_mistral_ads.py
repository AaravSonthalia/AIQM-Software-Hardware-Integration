"""MISTRAL ADS direct-read precheck — chamber-aware, READ-ONLY.

Purpose: quick pre-lab verification that the ADS path is live for the
active chamber (O-MBE or Ch-MBE per ``AIQM_CHAMBER`` env var). Bundles
three phases plus a chamber-specific-invariant check into a single
green/red output line and (optionally) a JSON evidence bundle.

Phases:

  1. Chamber config summary — dumps chamber_id / ads_netid / port_main /
     port_pid / ads_cell_count / ads_display_confirmed / mistral_mode_default
     via ``drivers.config.get_active_config()``.
  2. Connectivity — ``MistralAdsClient.connect()`` + one ``.read()`` +
     ``.disconnect()``, timed.
  3. Cell population summary — for cells 1..ads_cell_count, reports which
     ADS suffixes returned non-None values. Also checks the chamber-
     specific invariant that no cell > ads_cell_count returned data (for
     O-MBE this catches a driver bug that reads cell7 despite the 6-cell
     Bulbasaur PLC not having one).

Verdict states:
  * chamber_no_ads       — active chamber has no ads_netid configured
  * connect_failed       — MistralAdsClient.connect() raised
  * read_failed          — connect OK but read() raised
  * provenance_violation — cells above ads_cell_count returned data
                           (indicates chamber-config mismatch)
  * no_populated_cells   — read succeeded but every cell all-None
                           (cells likely idle / powered off on MistralGui side)
  * ok                   — at least one cell populated, no invariant violations

READ-ONLY invariant: this script uses only ``MistralAdsClient``, which
is invariantly read-only per its class docstring (only
``Connection.read_by_name`` is ever called; ``write_by_name`` is neither
imported nor used). See ``drivers/mistral_ads.py`` for the full guarantee.

Usage (O-MBE / Bulbasaur — default chamber):
    python scripts\\precheck_mistral_ads.py

Usage (Ch-MBE):
    $env:AIQM_CHAMBER="chmbe"          # PowerShell shell scope required
    python scripts\\precheck_mistral_ads.py

Usage with evidence bundle:
    python scripts\\precheck_mistral_ads.py --output-dir C:\\lab_evidence\\jul28
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Any, Optional

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from drivers.config import get_active_config  # noqa: E402


# The 8 per-cell ADS suffixes exposed by MistralAdsClient.read(). Mirrors
# gui/growth_logger.py's SENSOR_FIELDS + the driver's docstring. Kept as
# a module constant so tests can introspect the expected shape.
ADS_CELL_SUFFIXES: tuple[str, ...] = (
    "T", "T_set", "active_setpoint",
    "V", "I", "prog_V", "prog_A", "power",
)

# Maximum sample values to include per-cell in the human-readable output
# (all values still land in result.json — this is just visual noise control).
SAMPLE_VALUES_PER_CELL = 3


# ---------------------------------------------------------------------------
# Terminal helpers
# ---------------------------------------------------------------------------

def status(label: str, ok: bool, detail: str = "") -> None:
    tag = "[PASS]" if ok else "[FAIL]"
    line = f"  {tag} {label}"
    if detail:
        line += f" — {detail}"
    print(line)


def info(label: str, detail: str = "") -> None:
    line = f"  [INFO] {label}"
    if detail:
        line += f" — {detail}"
    print(line)


def section(title: str) -> None:
    print()
    print(f"--- {title} ---")


# ---------------------------------------------------------------------------
# Phase 1: Chamber config summary
# ---------------------------------------------------------------------------

def phase_config_summary(cfg) -> dict:
    """Print + return the active chamber's ADS-relevant config fields."""
    summary = {
        "chamber_id": cfg.chamber_id,
        "ads_netid": cfg.ads_netid,
        "ads_port_main": cfg.ads_port_main,
        "ads_port_pid": cfg.ads_port_pid,
        "ads_cell_count": cfg.ads_cell_count,
        "ads_display_confirmed": cfg.ads_display_confirmed,
        "mistral_mode_default": cfg.mistral_mode_default,
    }
    for k, v in summary.items():
        print(f"    {k}: {v!r}")
    return summary


# ---------------------------------------------------------------------------
# Phase 2: Connectivity + read
# ---------------------------------------------------------------------------

def phase_ads_read(cfg) -> dict:
    """Connect, read once, disconnect. Returns:

      { "connected": bool, "read_ok": bool,
        "keys": list[str], "read_dict": dict,
        "error": str, "duration_s": float }

    Uses the driver's own MistralAdsClient — which is invariantly
    read-only per its class docstring. Any error is captured into the
    dict; the script never propagates driver exceptions itself so the
    evidence bundle always gets written.
    """
    from drivers.mistral_ads import MistralAdsClient  # noqa: PLC0415 — lazy import

    result: dict = {
        "connected": False,
        "read_ok": False,
        "keys": [],
        "read_dict": {},
        "error": "",
        "duration_s": 0.0,
    }
    start = time.time()
    client = MistralAdsClient(
        netid=cfg.ads_netid,
        port_main=cfg.ads_port_main,
        port_pid=cfg.ads_port_pid,
        cell_count=cfg.ads_cell_count,
    )
    try:
        client.connect()
        result["connected"] = True
        r = client.read()
        result["read_ok"] = True
        result["read_dict"] = r
        result["keys"] = sorted(r.keys())
    except Exception as exc:  # noqa: BLE001
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            client.disconnect()
        except Exception:  # noqa: BLE001 — best-effort cleanup
            pass
        result["duration_s"] = round(time.time() - start, 3)
    return result


# ---------------------------------------------------------------------------
# Phase 3: Cell population summary
# ---------------------------------------------------------------------------

def _extract_cell_number(key: str) -> Optional[int]:
    """Return the cell index N from a ``cell{N}_*`` key, or None if it doesn't parse."""
    if not key.startswith("cell"):
        return None
    stem = key.split("_", 1)[0]  # "cell{N}"
    try:
        return int(stem[4:])
    except (ValueError, IndexError):
        return None


def phase_cell_summary(read_dict: dict, cell_count: int) -> dict:
    """Per-cell population analysis + chamber-invariant check.

    Returns:
      { "populated_cells":   list[int]  — cells with ≥1 non-None value,
        "unpopulated_cells": list[int]  — cells with all None values,
        "unexpected_cell_keys": list[str]
          — cell{N}_* keys where N > cell_count AND value is non-None;
            these violate the chamber-config invariant,
        "per_cell":          dict[int, dict]
          — { i: { "populated_suffixes": [...],
                   "sample_values": {suffix: value, ...} } } }
    """
    if not read_dict:
        return {
            "populated_cells": [],
            "unpopulated_cells": list(range(1, cell_count + 1)),
            "unexpected_cell_keys": [],
            "per_cell": {},
        }

    per_cell: dict[int, dict] = {}
    populated: list[int] = []
    unpopulated: list[int] = []
    for i in range(1, cell_count + 1):
        pop_suffixes: list[str] = []
        samples: dict[str, Any] = {}
        for sfx in ADS_CELL_SUFFIXES:
            key = f"cell{i}_{sfx}"
            val = read_dict.get(key)
            if val is None:
                continue
            pop_suffixes.append(sfx)
            if len(samples) < SAMPLE_VALUES_PER_CELL:
                samples[sfx] = val
        per_cell[i] = {
            "populated_suffixes": pop_suffixes,
            "sample_values": samples,
        }
        if pop_suffixes:
            populated.append(i)
        else:
            unpopulated.append(i)

    # Chamber invariant: no cell{N}_* key with N > cell_count should carry
    # non-None data. If it does, the driver just read a cell that doesn't
    # physically exist on this chamber's PLC — indicates a config mismatch.
    unexpected: list[str] = []
    for key, val in read_dict.items():
        n = _extract_cell_number(key)
        if n is None:
            continue
        if n > cell_count and val is not None:
            unexpected.append(key)

    return {
        "populated_cells": populated,
        "unpopulated_cells": unpopulated,
        "unexpected_cell_keys": sorted(unexpected),
        "per_cell": per_cell,
    }


# ---------------------------------------------------------------------------
# Verdict
# ---------------------------------------------------------------------------

def build_verdict(
    chamber_id: str,
    config_summary: dict,
    ads_result: dict,
    cell_summary: dict,
) -> tuple[str, str]:
    """Map phase outputs to (state, actionable recovery hint)."""
    if not config_summary.get("ads_netid"):
        return (
            "chamber_no_ads",
            f"Chamber {chamber_id!r} has no ads_netid configured in "
            "drivers/config.py. Either the chamber doesn't use ADS-mode "
            "MISTRAL, or the config needs an ads_netid field. Nothing to "
            "precheck — skip this script for this chamber.",
        )

    if not ads_result["connected"]:
        return (
            "connect_failed",
            f"MistralAdsClient could not connect to netid "
            f"{config_summary['ads_netid']}:{config_summary['ads_port_main']}"
            f"/{config_summary['ads_port_pid']}. Error: "
            f"{ads_result['error']}. Verify: (a) TwinCAT System Service is "
            "running on this machine (Windows Services → 'TwinCAT System "
            "Service'), (b) the ADS route to the PLC is configured (open "
            "TwinCAT XAE / SysMan → Routes), (c) the PLC is reachable on "
            "the network, (d) pyads is installed.",
        )

    if not ads_result["read_ok"]:
        return (
            "read_failed",
            f"Connected but the first read() raised: {ads_result['error']}. "
            "Likely causes: (a) ADS variable names on the PLC changed and "
            "drivers/mistral_ads.py needs updating, (b) PLC program not "
            "running, (c) transient ADS timeout — retry the script.",
        )

    if cell_summary["unexpected_cell_keys"]:
        return (
            "provenance_violation",
            f"Cells above ads_cell_count="
            f"{config_summary['ads_cell_count']} returned non-None data: "
            f"{cell_summary['unexpected_cell_keys']}. Chamber config "
            "mismatch — the driver read cells that don't physically exist "
            "on this chamber's PLC. Check drivers/config.py's ads_cell_count "
            f"for {chamber_id!r} and the driver's read() loop bound.",
        )

    if not cell_summary["populated_cells"]:
        return (
            "no_populated_cells",
            "Connected + read succeeded, but every cell returned all-None "
            "values. Likely cause: cells are idle on the MistralGui side "
            "(no active setpoint / powered off). Not necessarily an error "
            "for a cold-start chamber; open MistralGui to confirm the "
            "PLC state matches expectations.",
        )

    return (
        "ok",
        f"MISTRAL ADS live on chamber {chamber_id!r}: "
        f"{len(cell_summary['populated_cells'])}"
        f"/{config_summary['ads_cell_count']} cells populated. "
        f"Read completed in {ads_result['duration_s']}s. "
        "Ready for GUI integration testing.",
    )


# ---------------------------------------------------------------------------
# Evidence bundle
# ---------------------------------------------------------------------------

def write_evidence_bundle(
    output_dir: Path,
    args_summary: dict,
    env_summary: dict,
    config_summary: dict,
    ads_result: dict,
    cell_summary: dict,
    state: str,
    hint: str,
) -> dict[str, Path]:
    """Persist run to output_dir/result.json for share/paste-back."""
    import json
    from datetime import datetime, timezone

    output_dir.mkdir(parents=True, exist_ok=True)

    # Trim the full read_dict from the JSON to keep it scannable — retain
    # sorted key list + a first-20-key sample so the shape is still
    # documented. Full dict lives in the per_cell breakdown of cell_summary.
    ads_result_json = dict(ads_result)
    read_dict = ads_result_json.pop("read_dict", {})
    ads_result_json["read_dict_key_count"] = len(read_dict)
    ads_result_json["read_dict_sample"] = {
        k: read_dict[k] for k in list(sorted(read_dict.keys()))[:20]
    }

    result = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "args": args_summary,
        "environment": env_summary,
        "phase_1_config": config_summary,
        "phase_2_ads": ads_result_json,
        "phase_3_cells": cell_summary,
        "verdict": {"state": state, "recovery_hint": hint},
    }

    result_path = output_dir / "result.json"
    result_path.write_text(json.dumps(result, indent=2, default=str))
    return {"result.json": result_path}


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "MISTRAL ADS direct-read precheck — chamber-aware, READ-ONLY. "
            "Verifies the ADS path is live for the active chamber "
            "(get_active_config() per AIQM_CHAMBER env var) and reports "
            "per-cell population + chamber invariant compliance."
        ),
    )
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help=(
            "If set, write result.json (phase results + verdict) for "
            "paste-back / archival."
        ),
    )
    args = parser.parse_args(argv)

    print("═══ MISTRAL ADS precheck — READ-ONLY ═══")
    aiqm_chamber = os.environ.get("AIQM_CHAMBER", "")
    env_summary = {
        "AIQM_CHAMBER": aiqm_chamber if aiqm_chamber else "(unset)",
    }
    if aiqm_chamber:
        print(f"AIQM_CHAMBER: {aiqm_chamber!r}")
    else:
        print("AIQM_CHAMBER: (unset — get_active_config() defaults to O-MBE)")

    # --- Phase 1: Chamber config -----------------------------------------
    section("Phase 1: Active chamber config")
    cfg = get_active_config()
    config_summary = phase_config_summary(cfg)

    # Short-circuit for chambers with no ADS config: nothing to precheck.
    if not cfg.ads_netid:
        state, hint = build_verdict(cfg.chamber_id, config_summary, {}, {})
        section("Verdict")
        print(f"State:    {state}")
        print(f"Recovery: {hint}")
        if args.output_dir:
            written = write_evidence_bundle(
                args.output_dir, vars(args), env_summary,
                config_summary, {}, {}, state, hint,
            )
            print()
            print("Evidence bundle written:")
            for name, path in written.items():
                print(f"  {name}: {path}")
        return 1

    # --- Phase 2: Connect + read + disconnect ----------------------------
    section("Phase 2: MistralAdsClient connect + read + disconnect")
    info(
        f"target: netid={cfg.ads_netid} "
        f"port_main={cfg.ads_port_main} port_pid={cfg.ads_port_pid}"
    )
    ads_result = phase_ads_read(cfg)
    status("connect()", ads_result["connected"], ads_result["error"] or "")
    if ads_result["connected"]:
        status(
            "read()", ads_result["read_ok"],
            (
                f"{len(ads_result['keys'])} keys returned, "
                f"round-trip {ads_result['duration_s']}s"
            ) if ads_result["read_ok"] else ads_result["error"],
        )

    # --- Phase 3: Cell population ----------------------------------------
    if ads_result["read_ok"]:
        section("Phase 3: Cell population summary")
        cell_summary = phase_cell_summary(
            ads_result["read_dict"], cfg.ads_cell_count,
        )
        pop = cell_summary["populated_cells"]
        unpop = cell_summary["unpopulated_cells"]
        info(
            f"{len(pop)}/{cfg.ads_cell_count} cells populated: {pop}"
            + (f"; unpopulated: {unpop}" if unpop else "")
        )
        for i, cell in cell_summary["per_cell"].items():
            if cell["populated_suffixes"]:
                info(
                    f"cell{i}",
                    (
                        f"populated=[{', '.join(cell['populated_suffixes'])}]; "
                        f"sample={cell['sample_values']}"
                    ),
                )
        if cell_summary["unexpected_cell_keys"]:
            print()
            status(
                "chamber invariant",
                False,
                (
                    f"unexpected cells above ads_cell_count="
                    f"{cfg.ads_cell_count}: "
                    f"{cell_summary['unexpected_cell_keys']}"
                ),
            )
        else:
            status(
                "chamber invariant",
                True,
                f"no cell keys above ads_cell_count={cfg.ads_cell_count}",
            )
    else:
        cell_summary = {
            "populated_cells": [], "unpopulated_cells": [],
            "unexpected_cell_keys": [], "per_cell": {},
        }

    # --- Verdict ---------------------------------------------------------
    section("Verdict")
    state, hint = build_verdict(cfg.chamber_id, config_summary, ads_result, cell_summary)
    print(f"State:    {state}")
    print(f"Recovery: {hint}")

    # --- Evidence bundle (if requested) ----------------------------------
    if args.output_dir:
        written = write_evidence_bundle(
            args.output_dir, vars(args), env_summary,
            config_summary, ads_result, cell_summary, state, hint,
        )
        print()
        print("Evidence bundle written:")
        for name, path in written.items():
            print(f"  {name}: {path}")

    return 0 if state == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())

"""
MISTRAL PSU direct-read via Beckhoff TwinCAT ADS — per-chamber config.

Discovered Jul 22 2026 on Ch-MBE (NetID 10.0.42.112.1.1, 7 cells).
Extended Jul 27 2026 to O-MBE / Bulbasaur (NetID 10.0.42.111.1.1, 6 cells)
after the Kestrel JSON-RPC discovery on Bulbasaur exhausted every
introspection path and we bypassed it via direct pyads to the same
underlying Beckhoff PLC that MistralGui talks to through the gateway.

Two ADS ports (TwinCAT convention):
  - Port 851 = main PLC: pumps, valves, pressure gauges
  - Port 852 = PID program: effusion cells, shutters, EBVM

Full PLC symbol table on Ch-MBE: C:\\YangLab10 Gui\\VS\\Win32\\ModelDescription.xml
Full PLC symbol table on Bulbasaur: C:\\Program Files\\Scienta Omicron\\MistralGui\\ModelDescription.xml

Interface
---------
``MistralAdsClient`` matches ``drivers.mistral.MistralGui``:
  - ``connect()``      — open both ADS connections
  - ``read()``         — poll all cells + pressure; return superset dict
  - ``disconnect()``   — close both connections
  - ``connected``      — property
  - ``hwnd``           — property (always 0; no window)

Per-chamber config (see ``drivers/config.py``):
  - ``netid``       — PLC AmsNetId (Ch-MBE: 10.0.42.112.1.1, Bulbasaur: 10.0.42.111.1.1)
  - ``port_main``   — usually 851
  - ``port_pid``    — usually 852
  - ``cell_count``  — number of PID cells (Ch-MBE: 7, Bulbasaur: 6)

``read()`` output keys
-----------------------
Standard keys (consumed by existing ``MistralWorker``):
  v_set, v_actual, i_set, i_actual

  v_actual = Cell1 measured voltage (manipulator PSU)
  i_actual = Cell1 measured current
  v_set    = Cell1 programmed voltage setpoint (populated Jul 27 2026,
             previously None; sourced from cell1_prog_V)
  i_set    = Cell1 programmed current setpoint (sourced from cell1_prog_A)

Extended keys (per cell 1..cell_count):
  cell{i}_T                — actual temperature (°C)
  cell{i}_T_set            — setpoint temperature (°C)
  cell{i}_active_setpoint  — ramp active setpoint (°C) — added Jul 27 2026
  cell{i}_V                — measured voltage (V)
  cell{i}_I                — measured current (A)
  cell{i}_prog_V           — programmed voltage compliance (V) — added Jul 27 2026
  cell{i}_prog_A           — programmed current setpoint (A) — added Jul 27 2026
  cell{i}_power            — PID output power (appears to be watts based on
                             Jul 27 cross-check: V×I ≈ OutputPower for active
                             cells; not confirmed via vendor docs)
  cell{i}_state            — PID state (int enum)
  cell{i}_shutter_open     — bool
  cell{i}_shutter_closed   — bool

System keys:
  ebvm_shutter_open   — bool
  ebvm_shutter_closed — bool
  ebvm_coolant        — coolant flow
  ion_gauge_1_P  — IonGauge1 pressure (Torr)
  ion_gauge_2_P  — IonGauge2 pressure (Torr)
  pirani_1_P     — Pirani 1 foreline pressure
  pirani_2_P     — Pirani 2 foreline pressure
  turbo1_rpm     — Turbo1 speed (RPM)
  turbo2_rpm     — Turbo2 speed (RPM)
  service_mode   — bool: system in maintenance mode

READ-ONLY INVARIANT:
  Only ``Connection.read_by_name`` is ever called.
  ``write_by_name`` is never imported or called. Handles like
  ``Main.ValveUserCommand*``, ``PIDProgram.Cell{N}_Voltage``, and
  ``PIDProgram.Cell{N}_Current`` are write handles that would actuate
  hardware — this driver reads only the corresponding ``Programmed*``
  and ``Measured*`` observability handles instead. See test_ads_read.py
  for the full variable whitelist and safety rationale.
"""

from __future__ import annotations

import logging
from typing import Any, Optional

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Endpoint defaults — Ch-MBE Beckhoff PLC
# ---------------------------------------------------------------------------

DEFAULT_NETID = "10.0.42.112.1.1"
PORT_MAIN = 851   # main PLC: pumps, valves, gauges
PORT_PID  = 852   # PID program: cells, shutters, EBVM


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class AdsConnectionError(Exception):
    """Failed to open ADS connections to the Ch-MBE PLC."""


class AdsReadError(Exception):
    """One or more ADS reads failed during ``read()``."""


# ---------------------------------------------------------------------------
# Client
# ---------------------------------------------------------------------------

class MistralAdsClient:
    """Read-only ADS client for Ch-MBE MISTRAL / Beckhoff PLC.

    Drop-in for ``drivers.mistral.MistralGui`` and
    ``drivers.mistral_jsonrpc.MistralJsonRpcClient`` — same interface, richer
    ``read()`` output.

    Lazy pyads import: importing this module is safe on Mac/Linux where pyads
    is not installed. ``connect()`` is where the ImportError surfaces.
    """

    def __init__(
        self,
        netid: str = DEFAULT_NETID,
        port_main: int = PORT_MAIN,
        port_pid: int = PORT_PID,
        cell_count: int = 7,
    ):
        self._netid = netid
        self._port_main = port_main
        self._port_pid = port_pid
        self._cell_count = cell_count
        self._plc_main = None
        self._plc_pid = None
        self._connected = False

    # ------------------------------------------------------------------
    # MistralGui interface
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open both ADS connections to the Ch-MBE PLC.

        Raises:
            AdsConnectionError: pyads not installed, or ADS open() failed.
        """
        try:
            import pyads  # noqa: PLC0415 — intentional lazy import
        except ImportError as exc:
            raise AdsConnectionError(
                "pyads not installed; run: pip install pyads"
            ) from exc

        try:
            plc_main = pyads.Connection(self._netid, self._port_main)
            plc_pid  = pyads.Connection(self._netid, self._port_pid)
            plc_main.open()
            plc_pid.open()
        except Exception as exc:
            raise AdsConnectionError(
                f"ADS open() failed (netid={self._netid}, "
                f"ports={self._port_main}/{self._port_pid}): {exc}"
            ) from exc

        self._plc_main = plc_main
        self._plc_pid = plc_pid
        self._connected = True
        log.info(
            "MistralAdsClient connected: netid=%s ports=%d/%d",
            self._netid, self._port_main, self._port_pid,
        )

    def disconnect(self) -> None:
        """Close both ADS connections."""
        for plc in (self._plc_main, self._plc_pid):
            if plc is not None:
                try:
                    plc.close()
                except Exception:
                    pass
        self._plc_main = None
        self._plc_pid = None
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def hwnd(self) -> int:
        return 0  # no window; symmetric with MistralJsonRpcClient

    def read(self) -> dict[str, Optional[Any]]:
        """Poll all cells + pressure; return a superset dict.

        Standard keys (consumed by ``MistralWorker``):
          v_set, v_actual, i_set, i_actual

        v_actual = Cell1 voltage (manipulator — the substrate heater PSU).
        i_actual = Cell1 current.
        v_set / i_set = None (ADS exposes temperature setpoints, not V/I).

        Extended keys:
          cell{1..7}_{T, T_set, V, I, power, state, shutter_open, shutter_closed}
          ebvm_{shutter_open, shutter_closed, coolant}
          ion_gauge_{1,2}_P, pirani_{1,2}_P, turbo{1,2}_rpm, service_mode
        """
        import pyads  # already confirmed present from connect()

        result: dict[str, Optional[Any]] = {
            "v_set": None, "v_actual": None,
            "i_set": None, "i_actual": None,
        }

        def _read(plc, name: str, dtype: Any) -> Optional[Any]:
            try:
                return plc.read_by_name(name, dtype)
            except Exception as exc:
                log.debug("ADS read failed: %s — %s", name, exc)
                return None

        pm = self._plc_main
        pp = self._plc_pid

        # --- System / pressure (port 851) ---
        result["service_mode"]    = _read(pm, "Main.ServiceMode",       pyads.PLCTYPE_BOOL)
        result["ion_gauge_1_P"]   = _read(pm, "PVCXProgram.ionGauge1.IGC_ModbusCtrl.IonGaugePressure", pyads.PLCTYPE_LREAL)
        result["ion_gauge_2_P"]   = _read(pm, "PVCXProgram.ionGauge2.IGC_ModbusCtrl.IonGaugePressure", pyads.PLCTYPE_LREAL)
        result["pirani_1_P"]      = _read(pm, "PVCXProgram.ionGauge1.IGC_ModbusCtrl.PiraniPressure",   pyads.PLCTYPE_LREAL)
        result["pirani_2_P"]      = _read(pm, "PVCXProgram.ionGauge2.IGC_ModbusCtrl.PiraniPressure",   pyads.PLCTYPE_LREAL)
        result["turbo1_rpm"]      = _read(pm, "TurboProgram.pump1.spdRPM",              pyads.PLCTYPE_UINT)
        result["turbo2_rpm"]      = _read(pm, "TurboProgram.pump2.spdRPM",              pyads.PLCTYPE_UINT)

        # --- Cells 1..cell_count (port 852) ---
        for i in range(1, self._cell_count + 1):
            px = f"PIDProgram.Cell{i}"
            result[f"cell{i}_T"]                = _read(pp, f"{px}_pidTDK.ActualTemperature",           pyads.PLCTYPE_LREAL)
            result[f"cell{i}_T_set"]            = _read(pp, f"{px}_SetPoint",                           pyads.PLCTYPE_LREAL)
            result[f"cell{i}_active_setpoint"]  = _read(pp, f"{px}_pidTDK.ActiveSetPoint",              pyads.PLCTYPE_LREAL)
            result[f"cell{i}_V"]                = _read(pp, f"{px}_pidTDK.powerSupply.MeasuredVoltage",  pyads.PLCTYPE_LREAL)
            result[f"cell{i}_I"]                = _read(pp, f"{px}_pidTDK.powerSupply.MeasuredCurrent",  pyads.PLCTYPE_LREAL)
            result[f"cell{i}_prog_V"]           = _read(pp, f"{px}_pidTDK.powerSupply.ProgrammedVoltage", pyads.PLCTYPE_LREAL)
            result[f"cell{i}_prog_A"]           = _read(pp, f"{px}_pidTDK.powerSupply.ProgrammedCurrent", pyads.PLCTYPE_LREAL)
            result[f"cell{i}_power"]            = _read(pp, f"{px}_pidTDK.OutputPower",                  pyads.PLCTYPE_LREAL)
            result[f"cell{i}_state"]            = _read(pp, f"{px}_State",                               pyads.PLCTYPE_INT)
            result[f"cell{i}_shutter_open"]     = _read(pp, f"{px}_Shutter.StatusOpen",                  pyads.PLCTYPE_BOOL)
            result[f"cell{i}_shutter_closed"]   = _read(pp, f"{px}_Shutter.StatusClosed",                pyads.PLCTYPE_BOOL)

        # --- EBVM (port 852) ---
        result["ebvm_shutter_open"]   = _read(pp, "PIDProgram.EBVM_Shutter.StatusOpen",   pyads.PLCTYPE_BOOL)
        result["ebvm_shutter_closed"] = _read(pp, "PIDProgram.EBVM_Shutter.StatusClosed", pyads.PLCTYPE_BOOL)
        result["ebvm_coolant"]        = _read(pp, "PIDProgram.EBVM_FlowMeter.Flow",       pyads.PLCTYPE_LREAL)

        # --- Standard 4-key mapping (Cell1 = manipulator = substrate heater) ---
        # v_set / i_set populated Jul 27 2026 from Cell1 programmed values
        # (previously None because we hadn't yet read ProgrammedVoltage/Current).
        result["v_actual"] = result.get("cell1_V")
        result["i_actual"] = result.get("cell1_I")
        result["v_set"]    = result.get("cell1_prog_V")
        result["i_set"]    = result.get("cell1_prog_A")

        return result

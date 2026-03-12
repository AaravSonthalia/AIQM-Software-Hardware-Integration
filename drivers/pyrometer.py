"""
Pyrometer drivers — Modbus RTU (direct) and screen scrape (via TemperaSure).

Implements the TemperatureSensor ABC from heater_control.py so the pyrometer
can be used interchangeably with other temperature sensors in the system.

Two modes:
  1. ModbusPyrometer: Direct RS-422 via IFD-5 module (Exactus protocol)
  2. ScreenGrabPyrometer: pywinauto scrape of TemperaSure GUI window
"""

import struct
from typing import Optional

from heater_control import TemperatureSensor


# ─── Modbus register map (from Exactus manual) ───
REG_CH1_TEMP = 0x0000
REG_CH1_CURR = 0x0004
REG_AMBIENT = 0x0800

REG_ADDR = 0x1007
REG_BAUD = 0x1008
REG_RATE = 0x1011
REG_NAME0 = 0x1100  # 32 regs, 1 ASCII byte per reg
REG_VER = 0x1300
REG_BUILD = 0x1301
REG_SN0 = 0x1305  # 9 regs

REG_CMD = 0x8000
CMD_SAVE = 0x7001
CMD_REBOOT = 0x8080


def _regs_to_f32(hi: int, lo: int) -> float:
    """Convert two 16-bit holding registers to IEEE 754 float32."""
    raw = (hi << 16) | lo
    return struct.unpack(">f", raw.to_bytes(4, "big"))[0]


def _regs_to_ascii(regs: list[int]) -> str:
    """Convert register array (1 ASCII byte per reg) to string."""
    b = bytes(r & 0xFF for r in regs)
    return b.split(b"\x00", 1)[0].decode("ascii", errors="ignore")


class ModbusPyrometer(TemperatureSensor):
    """
    Direct Modbus RTU interface to BASF Exactus pyrometer via IFD-5 (RS-422).

    Ported from the existing pyrometer_script.py with pymodbus compatibility
    wrappers for different API versions.
    """

    def __init__(
        self,
        port: str = "COM4",
        baudrate: int = 115200,
        device_id: int = 1,
        timeout: float = 1.0,
        parity: str = "N",
        stopbits: int = 1,
        bytesize: int = 8,
    ):
        self._port = port
        self._baudrate = baudrate
        self._device_id = device_id
        self._timeout = timeout
        self._parity = parity
        self._stopbits = stopbits
        self._bytesize = bytesize
        self._client = None
        self._connected = False
        self._word_swap = False

    def connect(self) -> None:
        try:
            from pymodbus.client import ModbusSerialClient
        except ImportError:
            raise ImportError(
                "pymodbus not installed: pip install pymodbus pyserial"
            )

        self._client = ModbusSerialClient(
            port=self._port,
            baudrate=self._baudrate,
            parity=self._parity,
            stopbits=self._stopbits,
            bytesize=self._bytesize,
            timeout=self._timeout,
        )
        if not self._client.connect():
            raise RuntimeError(
                f"Could not open serial port {self._port}. "
                "Close TemperaSure or any serial monitor."
            )

        # Auto-detect word order
        self._detect_word_order()
        self._connected = True

    def _detect_word_order(self) -> None:
        """Read temperature and flip word order if result is implausible."""
        try:
            t = self._read_f32(REG_CH1_TEMP, word_swap=False)
            if not (-100.0 < t < 3000.0):
                t = self._read_f32(REG_CH1_TEMP, word_swap=True)
                if -100.0 < t < 3000.0:
                    self._word_swap = True
        except Exception:
            pass  # Will fail properly on first real read

    def disconnect(self) -> None:
        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass
            self._client = None
        self._connected = False

    def read_temperature(self) -> float:
        """Read channel 1 temperature in degrees C."""
        if not self._connected or self._client is None:
            raise RuntimeError("Pyrometer not connected.")
        return self._read_f32(REG_CH1_TEMP, self._word_swap)

    def read_current(self) -> float:
        """Read channel 1 signal current."""
        if not self._connected or self._client is None:
            raise RuntimeError("Pyrometer not connected.")
        return self._read_f32(REG_CH1_CURR, self._word_swap)

    def read_ambient(self) -> float:
        """Read ambient temperature (if supported by firmware)."""
        if not self._connected or self._client is None:
            raise RuntimeError("Pyrometer not connected.")
        return self._read_f32(REG_AMBIENT, self._word_swap)

    def get_info(self) -> dict:
        """Read device identity: name, serial, version, rate."""
        if not self._connected or self._client is None:
            return {}

        info = {}
        try:
            info["name"] = self._read_ascii(REG_NAME0, 32)
        except Exception:
            pass
        try:
            info["serial"] = self._read_ascii(REG_SN0, 9)
        except Exception:
            pass
        try:
            ver = self._read_u16(REG_VER)
            info["version"] = f"{ver >> 8}.{ver & 0xFF}"
        except Exception:
            pass
        try:
            info["rate_hz"] = self._read_u16(REG_RATE)
        except Exception:
            pass
        return info

    def set_rate(self, hz: int, persist: bool = False) -> None:
        """Set pyrometer sample rate. Optionally persist to EEPROM."""
        if not self._connected or self._client is None:
            raise RuntimeError("Pyrometer not connected.")

        self._write_register(REG_RATE, hz)
        if persist:
            try:
                self._write_register(REG_CMD, CMD_SAVE)
            except Exception:
                pass  # EEPROM save not supported on all firmware

    # ─── Low-level Modbus helpers ───

    def _read_holding(self, addr: int, count: int):
        """Read holding registers with pymodbus version compatibility."""
        for kw in ("device_id", "slave", "unit"):
            try:
                return self._client.read_holding_registers(
                    addr, count, **{kw: self._device_id}
                )
            except TypeError:
                continue
        return self._client.read_holding_registers(
            address=addr, count=count, device_id=self._device_id
        )

    def _write_register(self, addr: int, value: int):
        for kw in ("device_id", "slave", "unit"):
            try:
                return self._client.write_register(
                    addr, value, **{kw: self._device_id}
                )
            except TypeError:
                continue
        return self._client.write_register(
            address=addr, value=value, device_id=self._device_id
        )

    def _read_f32(self, addr: int, word_swap: bool = False) -> float:
        rr = self._read_holding(addr, 2)
        if hasattr(rr, "isError") and rr.isError():
            raise RuntimeError(f"Modbus read error at 0x{addr:04X}")
        regs = rr.registers
        if len(regs) < 2:
            raise RuntimeError(f"Short reply at 0x{addr:04X}")
        hi, lo = regs
        if word_swap:
            hi, lo = lo, hi
        return _regs_to_f32(hi, lo)

    def _read_u16(self, addr: int) -> int:
        rr = self._read_holding(addr, 1)
        if hasattr(rr, "isError") and rr.isError():
            raise RuntimeError(f"Modbus read error at 0x{addr:04X}")
        return rr.registers[0]

    def _read_ascii(self, addr: int, n_regs: int) -> str:
        rr = self._read_holding(addr, n_regs)
        if hasattr(rr, "isError") and rr.isError():
            raise RuntimeError(f"Modbus read error at 0x{addr:04X}")
        return _regs_to_ascii(rr.registers)


class ScreenGrabPyrometer(TemperatureSensor):
    """
    Reads pyrometer temperature by scraping the BASF TemperaSure GUI window.

    Uses pywinauto (Windows only) to find the TemperaSure window, locate
    ToolBar2, and extract the temperature from its Edit control.

    Two window title variants:
      - Oxide MBE:       'BASF TemperaSure 5.7.0.4 Advanced Mode'
      - Chalcogenide MBE: 'BASF TemperaSure 5.7.0.4'
    """

    def __init__(
        self,
        window_title: str = "BASF TemperaSure 5.7.0.4 Advanced Mode",
        exe_path: Optional[str] = None,
        auto_start: bool = False,
    ):
        self._window_title = window_title
        self._exe_path = exe_path
        self._auto_start = auto_start
        self._app = None
        self._connected = False

    def connect(self) -> None:
        try:
            from pywinauto.application import Application
        except ImportError:
            raise ImportError(
                "pywinauto not installed (Windows only): pip install pywinauto"
            )

        if self._auto_start and self._exe_path:
            self._app = self._start_temperasure(Application)
        else:
            # Connect to existing TemperaSure window
            try:
                self._app = Application(backend="uia").connect(
                    title=self._window_title
                )
            except Exception as e:
                raise RuntimeError(
                    f"Could not connect to TemperaSure window "
                    f"'{self._window_title}': {e}"
                )

        self._connected = True

    def _start_temperasure(self, Application) -> "Application":
        """Launch TemperaSure, click through Port Setup dialog, hit Start."""
        import time

        # Kill existing instance if running
        try:
            existing = Application(backend="uia").connect(
                title=self._window_title
            )
            existing.kill()
        except Exception:
            pass

        app = Application(backend="uia").start(self._exe_path)
        time.sleep(2)

        main = app.window(title=self._window_title)

        # Dismiss Port Setup dialog
        try:
            port_dlg = main.child_window(
                title="Port setup", control_type="Window"
            )
            port_dlg.child_window(title="OK", control_type="Button").click()
        except Exception:
            pass

        time.sleep(1)

        # Click Start
        try:
            main.child_window(title="Start", control_type="Button").click()
        except Exception:
            pass

        return app

    def disconnect(self) -> None:
        self._app = None
        self._connected = False

    def read_temperature(self) -> float:
        """Scrape temperature value from TemperaSure Edit control."""
        if not self._connected or self._app is None:
            raise RuntimeError("TemperaSure not connected.")

        dlg = self._app.window(title=self._window_title)
        toolbar2 = dlg.child_window(control_type="ToolBar", title_re="ToolBar2")
        edit_controls = toolbar2.children(control_type="Edit")

        if not edit_controls:
            raise RuntimeError("No Edit control found in ToolBar2")

        value_str = edit_controls[0].get_value()
        try:
            return float(value_str)
        except (ValueError, TypeError):
            raise RuntimeError(f"Could not parse temperature: '{value_str}'")

    def read_emissivity(self) -> Optional[float]:
        """Scrape emissivity value from TemperaSure, if available.

        Emissivity is typically in the second Edit control of ToolBar2,
        or in a separate toolbar. Returns None if not found.
        """
        if not self._connected or self._app is None:
            return None

        try:
            dlg = self._app.window(title=self._window_title)
            toolbar2 = dlg.child_window(control_type="ToolBar", title_re="ToolBar2")
            edit_controls = toolbar2.children(control_type="Edit")

            if len(edit_controls) >= 2:
                val = float(edit_controls[1].get_value())
                if 0.0 <= val <= 1.0:
                    return val

            # Fallback: search all toolbars for an Edit with a value in [0, 1]
            for tb in dlg.children(control_type="ToolBar"):
                for ed in tb.children(control_type="Edit"):
                    try:
                        val = float(ed.get_value())
                        if 0.01 <= val <= 1.0:
                            return val
                    except (ValueError, TypeError):
                        continue
        except Exception:
            pass

        return None


class DummyPyrometer(TemperatureSensor):
    """Test pyrometer that returns a slowly varying temperature."""

    def __init__(self, base_temp: float = 450.0, noise: float = 2.0):
        self._base_temp = base_temp
        self._noise = noise
        self._connected = False
        self._read_count = 0

    def connect(self) -> None:
        self._connected = True
        self._read_count = 0

    def disconnect(self) -> None:
        self._connected = False

    def read_temperature(self) -> float:
        if not self._connected:
            raise RuntimeError("Dummy pyrometer not connected.")

        import math
        import random

        self._read_count += 1
        # Slow sinusoidal drift + random noise
        drift = 5.0 * math.sin(self._read_count * 0.02)
        noise = random.gauss(0, self._noise)
        return self._base_temp + drift + noise

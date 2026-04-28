"""
State dataclasses for the hardware control GUI.
"""

from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional


@dataclass
class PowerSupplyState:
    """Current state of the power supply."""
    voltage_setpoint: float = 0.0
    current_setpoint: float = 0.0
    voltage_measured: float = 0.0
    current_measured: float = 0.0
    power_measured: float = 0.0
    output_enabled: bool = False
    ovp_limit: float = 0.0
    ocp_limit: float = 0.0
    connected: bool = False
    error: str = ""


@dataclass
class TemperatureState:
    """Current state of the thermocouple reader."""
    temperature: float = 0.0
    cold_junction: float = 0.0
    unit: str = "C"
    channel: str = "temperature"
    connected: bool = False
    error: str = ""
    device_info: str = ""
    product_id: str = ""
    serial_number: str = ""


@dataclass
class CameraState:
    """Current state of the RHEED camera."""
    frame: Optional[object] = None  # numpy ndarray (H, W, 3) uint8; typed as object for signal compat
    frame_number: int = 0
    fps: float = 0.0
    width: int = 0
    height: int = 0
    intensity: float = 0.0  # ROI mean intensity for oscillation tracking
    connected: bool = False
    error: str = ""
    mode: str = ""  # "direct", "screengrab", or "dummy"


@dataclass
class PyrometerState:
    """Current state of the pyrometer (separate from thermocouple).

    ``temperature`` is the mean of ``temperature_n`` rapid sub-readings
    taken within a single poll cycle. ``temperature_std`` is the sample
    standard deviation (0.0 when n=1). Polybot-inspired: a per-poll
    mean/std gives downstream consumers a cheap statistical-consistency
    check without requiring a second pass over the sensor log.
    """
    temperature: float = 0.0
    temperature_std: float = 0.0
    temperature_n: int = 1
    emissivity: Optional[float] = None  # 0.0-1.0, from TemperaSure or Modbus
    unit: str = "C"
    connected: bool = False
    error: str = ""
    device_info: str = ""
    mode: str = ""  # "modbus", "screengrab", or "dummy"


@dataclass
class MistralState:
    """Current state of the MistralGui cell V/I readout (OCR-scraped)."""
    v_set: Optional[float] = None
    v_actual: Optional[float] = None
    i_set: Optional[float] = None
    i_actual: Optional[float] = None
    connected: bool = False
    error: str = ""
    mode: str = ""  # "screengrab" or "dummy"


@dataclass
class EvapControlState:
    """Current state of the Evap Control MBE chamber pressure (OCR-scraped)."""
    chamber_pressure_mbar: Optional[float] = None
    connected: bool = False
    error: str = ""
    mode: str = ""  # "screengrab" or "dummy"


@dataclass
class ActionLogEntry:
    """A single entry in the action log."""
    timestamp: datetime = field(default_factory=datetime.now)
    category: str = ""
    action: str = ""
    details: str = ""
    psu_voltage: Optional[float] = None
    psu_current: Optional[float] = None
    psu_power: Optional[float] = None
    psu_output: Optional[bool] = None
    temperature: Optional[float] = None

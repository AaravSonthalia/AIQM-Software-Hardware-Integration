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
    mode: str = ""  # "screengrab", "jsonrpc", "ads", or "dummy"
    # Populated by MistralWorker when mode="ads" (Beckhoff TwinCAT ADS,
    # Ch-MBE only). Full read() output from MistralAdsClient — superset
    # of the 4 standard keys. Keys include cell{1..7}_T/V/I/power/state/
    # shutter_open/shutter_closed, ebvm_*, ion_gauge_*_P, pirani_*_P,
    # turbo*_rpm, service_mode. None in all other modes.
    ads_cells: Optional[dict] = None


@dataclass
class EvapControlState:
    """Current state from EvapControl / ElogReader.

    Field provenance:
    - ``screengrab`` mode (OCR): only ``chamber_pressure_mbar`` is populated.
    - ``elog`` mode (.elo binary log): all fields populated where the elog
      schema contains the underlying variable. Missing variables stay None
      (different MBE systems have different cells).

    Field-to-elog-variable mapping is defined in
    ``drivers.evap_control.ElogReader.DEFAULT_VAR_MAP``. To track different
    cells on a different system, pass a custom ``var_map`` when constructing
    the reader and extend this dataclass to match.
    """
    # Always populated (both modes)
    chamber_pressure_mbar: Optional[float] = None
    # Elog-mode only: substrate manipulator (the substrate temperature itself)
    substrate_temp_pv_C: Optional[float] = None
    substrate_temp_setpoint_C: Optional[float] = None
    # Elog-mode only: effusion cell process values (Bulbasaur OMBE config)
    cell_HTEC2_pv_C: Optional[float] = None
    cell_Y_pv_C: Optional[float] = None       # Yttrium
    cell_Sr_pv_C: Optional[float] = None      # Strontium
    cell_Eu_pv_C: Optional[float] = None      # Europium
    cell_Er_pv_C: Optional[float] = None      # Erbium
    # Elog-mode only: plasma source state (when in use)
    plasma_dc_bias_V: Optional[float] = None
    plasma_forward_W: Optional[float] = None
    plasma_reflected_W: Optional[float] = None
    connected: bool = False
    error: str = ""
    mode: str = ""  # "screengrab", "elog", or "dummy"


@dataclass
class ClassifierState:
    """Current state of the RHEED classifier worker.

    Lifecycle:
        - Initial: ``loading=True, ready=False, error=""``
        - After model loads: ``loading=False, ready=True``
        - Failure: ``loading=False, ready=False, error="..."``

    Score field provenance — per-cycle transformation chain:
        1. Bridge returns raw win-rates in ``raw_scores`` (0-1 per class,
           sum unconstrained — Bradley-Terry pairwise, not softmax).
        2. Equalizer recipe (clip negatives → divide by sum → uniform
           fallback) produces ``normalized_percent`` (0-100, sums to 100).
        3. EMA over successive cycles produces ``smoothed_percent`` —
           this is what the UI sliders display. Frozen while ``is_ood``.

    Confidence signals:
        - ``is_bad`` — classifier's own low-quality flag (bad-reference
          bank matched better than any real-class reference). Independent
          of ``is_ood``.
        - ``is_ood`` — derived by the worker from ``quality`` below the
          worker's OOD threshold. Signals the UI to grey out the sliders
          and stop advancing the EMA.

    Sentinel: ``last_frame_number == -1`` means no frame has been
    classified yet this session.
    """
    # Lifecycle
    loading: bool = True
    ready: bool = False
    error: str = ""

    # Latest inference — see class docstring for the transformation chain
    last_frame_number: int = -1
    raw_scores: dict[str, float] = field(default_factory=dict)
    normalized_percent: dict[str, int] = field(default_factory=dict)
    smoothed_percent: dict[str, int] = field(default_factory=dict)
    raw_sum: float = 0.0  # for the "Sum: X.XX" transparency label

    # Confidence / OOD
    quality: float = 0.0
    is_bad: bool = False
    bad_confidence: float = 0.0
    is_ood: bool = False
    # Latches True on the first non-OOD classification and stays True
    # for the rest of the session. Lets the UI distinguish "OOD, showing
    # last confident data" from "OOD, no confident data has ever
    # arrived" (in which case ``smoothed_percent`` is just the ready-time
    # uniform-20 placeholder and shouldn't be shown as if it were a
    # real prediction).
    has_confident_data: bool = False

    # Perf
    inference_ms: float = 0.0

    # Model identity — filename + mtime of best_model.pth, set once at
    # bridge-load time and repeated on every emission. Non-empty means
    # the bridge loaded successfully. Displayed in the UI's tooltip so
    # growers can tell at a glance which model checkpoint is running.
    model_version: str = ""


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

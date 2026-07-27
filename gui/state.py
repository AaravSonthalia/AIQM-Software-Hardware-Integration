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
    mode: str = ""  # "vimba", "screengrab", "screengrab_mss", or "dummy"
    capture_backend: str = ""
    captured_at_utc: str = ""
    capture_sequence: int = 0
    frame_age_ms: float = 0.0
    source_hwnd: int = 0
    captured_monotonic_ns: int = 0  # internal age calculation, not serialized
    capture_geometry_id: str = ""  # ROI/chrome-crop identity


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
    # Read-only sample provenance. UTC fields are ISO-8601 strings; the
    # monotonic timestamp is process-local and is used only to calculate age.
    source_at_utc: Optional[str] = None
    received_at_utc: Optional[str] = None
    sample_sequence: int = 0
    read_duration_ms: Optional[float] = None
    received_monotonic_ns: Optional[int] = field(
        default=None, repr=False, compare=False,
    )


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
    # ``source_at_utc`` is reserved for a future hardware/source timestamp;
    # current MISTRAL modes expose only the Python receive timestamp.
    source_at_utc: Optional[str] = None
    received_at_utc: Optional[str] = None
    sample_sequence: int = 0
    read_duration_ms: Optional[float] = None
    received_monotonic_ns: Optional[int] = field(
        default=None, repr=False, compare=False,
    )


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
    # Elog mode populates ``source_at_utc`` from the LabVIEW record. OCR and
    # dummy modes have no source clock and leave it None.
    source_at_utc: Optional[str] = None
    received_at_utc: Optional[str] = None
    sample_sequence: int = 0
    read_duration_ms: Optional[float] = None
    received_monotonic_ns: Optional[int] = field(
        default=None, repr=False, compare=False,
    )


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

    # Acquisition-side RHEED view state.  These fields are deliberately
    # separate from ``is_bad`` / ``is_ood`` above: alignment and history
    # readiness are operator-/pipeline-known facts, while Bad/OOD are model
    # predictions. ``None`` means that alignment has not yet been confirmed
    # for the current session.
    view_segment_id: Optional[int] = None
    # Monotonic token for any reset of pixel-coordinate-dependent state.
    # Unlike ``view_segment_id``, this also changes on a camera-continuity
    # reset within the same stable gun alignment.
    visual_history_generation: int = 0
    gun_aligned: Optional[bool] = None
    history_frame_count: int = 0
    # Zero means that the loaded runtime bridge is single-frame-only.  The
    # offline temporal experiments still use 32-frame causal histories.
    history_required: int = 0
    history_ready: bool = False
    prediction_actionable: bool = False
    model_input_mode: str = "unknown"


@dataclass
class RheedQcState:
    """Acquisition-side validity of the current RHEED view.

    ``view_segment_id`` changes only after a gun realignment is completed.
    Images are never deleted: frames captured while ``gun_aligned`` is False
    remain useful realignment/QC data, but must not be mixed into the visual
    history of the next stable segment. Temperature and process histories are
    intentionally outside this state and continue across segment boundaries.

    ``history_ready=False`` is not a Bad label. It only means that a temporal
    adviser has not yet accumulated enough same-segment visual observations.
    """

    session_active: bool = False
    view_segment_id: Optional[int] = None
    visual_history_generation: int = 0
    realignment_id: int = 0
    gun_aligned: Optional[bool] = None
    realignment_active: bool = False
    history_frame_count: int = 0
    history_required: int = 0
    history_ready: bool = False

    @property
    def prediction_actionable(self) -> bool:
        return (
            self.session_active
            and self.gun_aligned is True
            and not self.realignment_active
            and self.history_required > 0
            and self.history_ready
        )


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

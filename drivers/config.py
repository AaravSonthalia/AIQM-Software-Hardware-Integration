"""
System-specific configurations for Oxide MBE and Chalcogenide MBE setups.

Each MBE system has different:
  - TemperaSure window titles
  - TemperaSure exe paths
  - kSA window titles
  - COM ports for Modbus pyrometer
  - Data storage paths
  - MISTRAL / EvapControl driver mode defaults
  - Effusion cell display labels

Select the active config with ``get_active_config()``, which reads the
``AIQM_CHAMBER`` environment variable (default: ``"ombe"``). The two
chamber-specific launcher scripts set this before the app starts:

    growth_monitor_ombe.py   → AIQM_CHAMBER=ombe
    growth_monitor_chmbe.py  → AIQM_CHAMBER=chmbe
"""

import os
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class MBESystemConfig:
    """Configuration for a specific MBE system."""

    name: str

    # Chamber identifier — consumed by get_active_config() and the GUI
    # to branch on chamber-specific behaviour.
    chamber_id: str = "ombe"

    # Default driver mode selections for the Session config panel.
    # The GUI uses these as setCurrentText() seeds; the grower can still
    # override them manually before arming.
    mistral_mode_default: str = "screengrab"
    evap_mode_default: str = "elog"

    # EvapControl log directory passed to ElogReader.
    # Empty string = ElogReader uses its built-in multi-path auto-detect.
    evap_log_dir: str = ""

    # Effusion cell display entries for the Direct-read tab.
    # Each dict:
    #   "label"       — display name shown in the ValueDisplay widget
    #   "state_field" — EvapControlState attribute name to read; None
    #                   when the cell is not elog-sourced (e.g. Ch-MBE
    #                   cells are fed from ADS instead)
    cell_display: list = field(default_factory=lambda: [
        {"label": "HTEC2",       "state_field": "cell_HTEC2_pv_C"},
        {"label": "Y (Yttrium)", "state_field": "cell_Y_pv_C"},
        {"label": "Sr",          "state_field": "cell_Sr_pv_C"},
        {"label": "Eu",          "state_field": "cell_Eu_pv_C"},
        {"label": "Er",          "state_field": "cell_Er_pv_C"},
    ])

    # kSA 400 RHEED window title (for screen scraping)
    ksa_window_title: str = "AVT Manta_G Live Video"

    # TemperaSure pyrometer window title
    temperasure_title: str = "BASF TemperaSure 5.7.0.4 Advanced Mode"

    # TemperaSure executable path (for auto-start)
    temperasure_exe: str = ""

    # Modbus pyrometer serial port
    pyrometer_port: str = "COM4"
    pyrometer_baudrate: int = 115200
    pyrometer_device_id: int = 1

    # Data storage paths
    single_images_folder: str = ""
    stream_images_folder: str = ""

    # Camera settings
    camera_index: int = 0
    camera_fps: float = 1.0


# ---------------------------------------------------------------------------
# Pre-configured systems
# ---------------------------------------------------------------------------

OXIDE_MBE = MBESystemConfig(
    name="Oxide MBE",
    chamber_id="ombe",
    mistral_mode_default="screengrab",
    evap_mode_default="elog",
    # evap_log_dir left empty — ElogReader auto-detects the Bulbasaur path
    cell_display=[
        {"label": "HTEC2",       "state_field": "cell_HTEC2_pv_C"},
        {"label": "Y (Yttrium)", "state_field": "cell_Y_pv_C"},
        {"label": "Sr",          "state_field": "cell_Sr_pv_C"},
        {"label": "Eu",          "state_field": "cell_Eu_pv_C"},
        {"label": "Er",          "state_field": "cell_Er_pv_C"},
    ],
    temperasure_title="BASF TemperaSure 5.7.0.4 Advanced Mode",
    temperasure_exe=r"C:\Users\Lab10\Desktop\TemperaSure.exe",
    single_images_folder=(
        r"C:\Users\Lab10\Desktop\Automated RHEED Image Acquisition"
        r"\Acquiring Images Via Python Script Tests\Single Images"
    ),
    stream_images_folder=(
        r"C:\Users\Lab10\Desktop\Automated RHEED Image Acquisition"
        r"\Acquiring Images Via Python Script Tests\Stream Images"
    ),
)

CHALCOGENIDE_MBE = MBESystemConfig(
    name="Chalcogenide MBE",
    chamber_id="chmbe",
    mistral_mode_default="ads",
    # elog mode is left as screengrab default because Ch-MBE's elog
    # variable map differs from Bulbasaur's (different cell names).
    # Switch to "elog" once the Ch-MBE var_map is confirmed.
    evap_mode_default="screengrab",
    evap_log_dir=r"C:\evap_control_1.2.0.48\log",
    # Cell1 = manipulator (substrate heater — confirmed Jul 22 2026).
    # Cell2–7 physical mapping (Fe/Se/Te cracker) pending Jiangang
    # confirmation. state_field=None: these come from ADS, not elog.
    cell_display=[
        {"label": "Cell1 (Substrate)", "state_field": None},
        {"label": "Cell2",             "state_field": None},
        {"label": "Cell3",             "state_field": None},
        {"label": "Cell4",             "state_field": None},
        {"label": "Cell5",             "state_field": None},
        {"label": "Cell6",             "state_field": None},
        {"label": "Cell7",             "state_field": None},
    ],
    temperasure_title="BASF TemperaSure 5.7.0.4",
    temperasure_exe=r"C:\Users\Omicron\Desktop\TemperaSure.exe",
    single_images_folder=r"C:\Dropbox\Data\RHEED\RHEED_YangGroup\FeSeTe_STO",
    stream_images_folder=r"C:\Dropbox\Data\RHEED\RHEED_YangGroup\FeSeTe_STO",
)

# Available system configurations. "oxide" and "chalcogenide" are legacy
# aliases preserved for any callers that used the old SYSTEMS dict keys.
SYSTEMS: dict = {
    "ombe":          OXIDE_MBE,
    "oxide":         OXIDE_MBE,
    "chmbe":         CHALCOGENIDE_MBE,
    "chalcogenide":  CHALCOGENIDE_MBE,
}


def get_active_config() -> MBESystemConfig:
    """Return the chamber config selected by the ``AIQM_CHAMBER`` env var.

    Defaults to ``OXIDE_MBE`` (O-MBE / Bulbasaur) when the env var is
    absent or unrecognised. Recognised values (case-insensitive):
    ``ombe``, ``oxide``, ``chmbe``, ``chalcogenide``.
    """
    chamber = os.environ.get("AIQM_CHAMBER", "ombe").lower()
    return SYSTEMS.get(chamber, OXIDE_MBE)

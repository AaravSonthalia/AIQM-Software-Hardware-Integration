"""
System-specific configurations for Oxide MBE and Chalcogenide MBE setups.

Each MBE system has different:
  - TemperaSure window titles
  - TemperaSure exe paths
  - kSA window titles
  - COM ports for Modbus pyrometer
  - Data storage paths
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class MBESystemConfig:
    """Configuration for a specific MBE system."""

    name: str

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


# Pre-configured systems
OXIDE_MBE = MBESystemConfig(
    name="Oxide MBE",
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
    temperasure_title="BASF TemperaSure 5.7.0.4",
    temperasure_exe=r"C:\Users\Omicron\Desktop\TemperaSure.exe",
    single_images_folder=r"C:\Dropbox\Data\RHEED\RHEED_YangGroup\FeSeTe_STO",
    stream_images_folder=r"C:\Dropbox\Data\RHEED\RHEED_YangGroup\FeSeTe_STO",
)

# Available system configurations
SYSTEMS = {
    "oxide": OXIDE_MBE,
    "chalcogenide": CHALCOGENIDE_MBE,
}

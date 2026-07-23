"""Launch the Growth Monitor configured for the Chalcogenide MBE (Ch-MBE)."""
import os
os.environ.setdefault("AIQM_CHAMBER", "chmbe")
from growth_monitor_app import main  # noqa: E402
main()

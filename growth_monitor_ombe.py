"""Launch the Growth Monitor configured for the Oxide MBE (Bulbasaur)."""
import os
os.environ.setdefault("AIQM_CHAMBER", "ombe")
from growth_monitor_app import main  # noqa: E402
main()

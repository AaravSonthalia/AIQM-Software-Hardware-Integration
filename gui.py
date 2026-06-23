"""
OWON Hardware Control Dashboard (dummy-loop / heater-control GUI)

Slim shim — delegates to gui.heater_control.app.main().

This launcher targets the heater-control product line (OWON PSU + Dracal TC
+ PID), distinct from the OMBE Growth Monitor product (see
``growth_monitor_app.py``). The 9-tab heater UI lives entirely under
``gui/heater_control/`` so the two products share only ``state.py``,
``widgets.py``, and ``workers.py``.

Usage:
    python gui.py
    python gui.py /dev/tty.usbserial-XXXX  # specify port manually
"""

from gui.heater_control.app import main

if __name__ == "__main__":
    main()

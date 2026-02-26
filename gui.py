"""
OWON Hardware Control Dashboard

Slim shim — delegates to gui.app.main().

Usage:
    python gui.py
    python gui.py /dev/tty.usbserial-XXXX  # specify port manually
"""

from gui.app import main

if __name__ == "__main__":
    main()

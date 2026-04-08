"""Launch the OMBE Growth Monitor GUI."""
import os
import sys
from pathlib import Path

# Fix Qt platform plugin discovery for Anaconda environments where the compiled-in
# prefix path doesn't match the actual PyQt6 plugin location.
import PyQt6
_qt_plugins = Path(PyQt6.__file__).parent / "Qt6" / "plugins"
if _qt_plugins.exists() and "QT_QPA_PLATFORM_PLUGIN_PATH" not in os.environ:
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(_qt_plugins / "platforms")

from PyQt6.QtWidgets import QApplication
from gui.growth_app import GrowthApp


def main():
    app = QApplication(sys.argv)
    window = GrowthApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

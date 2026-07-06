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

# Windows workaround: preload torch on the MAIN thread so its C++
# runtime DLLs (c10.dll and friends) initialize before ClassifierWorker
# tries to import torch from its own QThread. Windows Python 3.12 +
# torch 2.12+cpu can fail with WinError 1114 on c10.dll if the first
# `import torch` happens on a non-main thread — c10.dll's DllMain does
# thread-local storage setup that fails outside the main thread. Once
# the DLLs are loaded on the main thread here, subsequent `import
# torch` from any worker thread just finds the already-initialized
# module and skips the DllMain call entirely.
#
# Ignored on macOS/Linux (dyld/ld.so don't have this issue). Best-effort
# — if torch isn't installed, the classifier will surface a clean
# "Failed to load classifier" error and the rest of the app runs
# unchanged, exactly as designed.
try:
    import torch  # noqa: F401 — imported for its DLL side effects
except ImportError:
    pass

from PyQt6.QtWidgets import QApplication
from gui.growth_app import GrowthApp


def main():
    app = QApplication(sys.argv)
    window = GrowthApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

"""Launch the OMBE Growth Monitor GUI."""
import sys
from PyQt6.QtWidgets import QApplication
from gui.growth_app import GrowthApp


def main():
    app = QApplication(sys.argv)
    window = GrowthApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

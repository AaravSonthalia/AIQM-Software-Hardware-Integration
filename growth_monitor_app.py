"""Launch the MBE Growth Monitor GUI."""
import sys
from PyQt6.QtWidgets import QApplication
from gui.growth_app import GrowthApp


def main():
    resource = f"ASRL{sys.argv[1]}::INSTR" if len(sys.argv) > 1 else None
    app = QApplication(sys.argv)
    window = GrowthApp(psu_resource=resource)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

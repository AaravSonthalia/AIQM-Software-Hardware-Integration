# AIQM Software-Hardware Integration — Project Instructions

## Hardware Ports (Mac)
| Device | Port | Baud |
|--------|------|------|
| OWON PSU | `/dev/cu.usbserial-120` | 115200 (SCPI over USB serial) |
| Dracal TC | `/dev/cu.usbmodemE297161` | VCP mode |

## Two Operating Modes
1. **Normal Mode:** Human controls heater via standard PSU, watches RHEED/pyrometer on screen
2. **AI-Scientist Mode:** OWON PSU, RHEED → Classifier2 → RL → PID → OWON → heater (closed-loop)

## Key Scripts
| Script | Purpose |
|--------|---------|
| `owon_power_supply.py` | OWON SPE interface (SCPI) |
| `dracal_thermocouple_reader.py` | Dracal TC reader (VCP) |
| `temperature_pid_control.py` | PID loop: OWON + Dracal |
| `scripts/heater_step_test.py` | Step response test for PID tuning |
| `scripts/psu_diagnostic.py` | PSU connectivity check |
| `scripts/owon_self_test.py` | OWON self-test |

## GUI (PyQt6) — Growth Monitor v1
Launch: `python growth_monitor_app.py` (optional PSU port arg)

**v1 = Display + Manual Logging MVP** (PI meeting Mar 6, 2026):
- ML disabled (classifier not loaded), AI widgets grayed out
- Human classification: **checkboxes** (not sliders) — multiple can be checked for transitional states
- SAVE OBSERVATION button (Ctrl+S) — saves frame PNG + commit_log.csv entry
- Collapsible Config panel: recording interval, save folder, filename prefix, camera/pyrometer mode, PSU resource
- Sensor logging at configurable interval (default 1s) to sensor_log.csv

| File | Purpose |
|------|---------|
| `growth_monitor_app.py` | Launcher |
| `gui/growth_monitor.py` | Dashboard widget: RHEED display, human checkboxes, AI widgets (disabled), config panel |
| `gui/growth_app.py` | Orchestrator: ARM/DISARM/START/STOP, worker lifecycle, config wiring |
| `gui/growth_logger.py` | Session logging: sensor CSV, commit CSV, auto-capture CSV, frame PNGs |
| `gui/classifier_bridge.py` | Loads Classifier2 model — NOT used in v1, kept for v3 |
| `gui/auto_capture.py` | Auto-capture engine — disabled in v1 |
| `gui/rheed_tab.py` | RHEED display tab (MainWindow, not Growth Monitor) |
| `gui/pyrometer_tab.py` | Pyrometer tab (MainWindow) |
| `gui/config_tab.py` | Config tab (MainWindow) |
| `gui/state.py` | Shared state dataclasses |
| `gui/workers.py` | Background worker threads |

### Phased Roadmap
- **v1:** Display + manual logging, dummy drivers, test on Mac — **DONE**
- **v2 (current):** Deploy to Bulbasaur, connect real hardware
  - [x] Python 3.12 installed on Bulbasaur
  - [x] GUI launches on Bulbasaur (`python gui.py`)
  - [x] RHEED screengrab connects to kSA 400 window (gun was off — needs live pattern test)
  - [x] Pyrometer screengrab reads live temp from TemperaSure — values verified
  - [x] Pyrometer Modbus on COM4 (blocked when TemperaSure holds port — expected)
  - [ ] OWON PSU on Bulbasaur (not yet plugged in / enumerated)
  - [ ] RHEED with gun on — verify live pattern displays
  - [ ] Full end-to-end growth session test (Growth Monitor app)
- **v3:** Re-enable Classifier2, live AI confidence, invite Justin
- **v4:** RL policy, PID loop, closed-loop AI-Scientist mode

## MBE Hardware Control
- **Scienta Omicron MISTRAL** controls all MBE hardware (pumps, valves, heaters, manipulators) via touch-screen panels. GUI-based, no serial API. Future interaction requires pywinauto/screen scraping. Not needed for v1-v2.
- **OWON PSU** is our controllable heater bypass (separate from MISTRAL) for AI-Scientist mode.
- CPU inference benchmarked at **13ms/frame** — no GPU/cluster needed.

## Deployment Target
- **Bulbasaur** (lab PC, Windows): Python 3.12.10 installed, GUI verified 2026-03-12
  - Displays: 1920x1200 + 1920x1080
  - COM4: Prolific PL2303GS → pyrometer RS-422 (IFD-5)
  - OWON PSU: not yet connected on Bulbasaur
  - TeamViewer: ID 1 756 871 318

## Safety
- PID has hard temperature limits (`--max-temp-hard`) and voltage step limits
- Always use safety shield when testing with real hardware
- Step test currently blocked on thermocouple contact with ceramic boat

## Setup
```bash
# Mac
python3 -m venv .venv && source .venv/bin/activate
pip install pyvisa pyvisa-py pyserial PyQt6 pyqtgraph numpy

# Bulbasaur (Windows) — no venv, system Python
pip install PyQt6 pyqtgraph numpy pyvisa pyvisa-py pyserial mss Pillow pywinauto pymodbus
```

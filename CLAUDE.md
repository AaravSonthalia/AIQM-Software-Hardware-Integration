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
- **v1 (current):** Display + manual logging, dummy drivers, test on Mac
- **v2:** Deploy to Bulbasaur, connect real hardware (kSA screengrab, COM4 pyrometer, OWON)
- **v3:** Re-enable Classifier2, live AI confidence, invite Justin
- **v4:** RL policy, PID loop, closed-loop AI-Scientist mode

## MBE Hardware Control
- **Scienta Omicron MISTRAL** controls all MBE hardware (pumps, valves, heaters, manipulators) via touch-screen panels. GUI-based, no serial API. Future interaction requires pywinauto/screen scraping. Not needed for v1-v2.
- **OWON PSU** is our controllable heater bypass (separate from MISTRAL) for AI-Scientist mode.
- CPU inference benchmarked at **13ms/frame** — no GPU/cluster needed.

## Deployment Target
- **Bulbasaur** (lab PC): Must run Growth Monitor GUI with all hardware connected
- No Python installed on Bulbasaur yet (blocker)
- TeamViewer: ID 1 756 871 318

## Safety
- PID has hard temperature limits (`--max-temp-hard`) and voltage step limits
- Always use safety shield when testing with real hardware
- Step test currently blocked on thermocouple contact with ceramic boat

## Setup
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install pyvisa pyvisa-py pyserial
```

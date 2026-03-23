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

## GUI (PyQt6) — OMBE Growth Log Assistant
Launch: `python growth_monitor_app.py`

**Current version (v2): Growth log assistant** — automates OMBE growth log data capture.
Reads RHEED from kSA 400 (screengrab) + pyrometer temp from TemperaSure (screengrab),
auto-timestamps grower notes, and exports partially-filled growth logs.

### Layout
- **Top bar (always visible):** Grower name, Sample ID, ARM / START / STOP
- **Monitor tab:** Value displays (elapsed, temp, V, I), live RHEED image, expanding note text box, LOG ENTRY button (Ctrl+S)
- **Session tab:** Config (upper-left), Sensor Log interval reads (upper-right), Growth Notes grower commits (bottom half), Export Growth Log button

### Key Files
| File | Purpose |
|------|---------|
| `growth_monitor_app.py` | Launcher |
| `gui/growth_monitor.py` | Two-tab UI: Monitor (RHEED + notes) and Session (config + logs + export) |
| `gui/growth_app.py` | Orchestrator: ARM/DISARM/START/STOP, worker lifecycle, sensor logging |
| `gui/growth_logger.py` | Session logging: sensor CSV, commit CSV, frame PNGs, JSON metadata, xlsx/CSV export |
| `gui/classifier_bridge.py` | Loads Classifier2 model — NOT used in v2, kept for v3 |
| `gui/auto_capture.py` | Auto-capture engine — NOT used in v2, kept for v3 |
| `gui/state.py` | Shared state dataclasses |
| `gui/workers.py` | Background worker threads (camera, pyrometer, PSU, thermocouple) |
| `gui/widgets.py` | Reusable widgets (ValueDisplay, ControlPanel, ProtectionPanel) |
| `drivers/` | Camera + pyrometer driver ABCs with dummy/screengrab/direct implementations |

### Session Output
Each session creates a directory under `logs/growths/` containing:
- `sensor_log.csv` — periodic temperature readings at configured interval
- `commit_log.csv` — timestamped grower entries with notes, temp, V, I
- `frames/` — RHEED frame PNGs captured at each LOG ENTRY
- `session_metadata.json` — grower, sample ID, date, entry count
- `growth_log.xlsx` — OMBE growth log export (on Export or session end)

### Config Defaults
- Recording interval: 1s minimum (prevents faster-than-hardware commits)
- Camera mode: dummy (Mac) / screengrab (Bulbasaur)
- Pyrometer mode: dummy (Mac) / screengrab (Bulbasaur)

### Phased Roadmap
- **v1:** Classification-centric display + manual logging — **DONE** (archived)
- **v2 (current):** Growth log assistant for OMBE
  - [x] Python 3.12 on Bulbasaur, GUI launches
  - [x] RHEED screengrab connects to kSA 400 (gun was off — needs live test)
  - [x] Pyrometer screengrab reads live temp from TemperaSure — verified
  - [x] Growth-log-centric UI with two-tab layout
  - [x] OMBE growth log export (xlsx + CSV fallback)
  - [ ] Full end-to-end growth session test on Bulbasaur
  - [ ] Investigate kSA window for V/I readings (may be in screengrab)
- **v3:** Add reconstruction classification checkboxes + Classifier2 ML confidence
- **v4:** RL policy, PID loop, closed-loop AI-Scientist mode
- **Deferred:** OWON PSU on Bulbasaur, PID step tests, auto-capture, ChMBE variant

## MBE Hardware Control
- **Scienta Omicron MISTRAL** controls all MBE hardware (pumps, valves, heaters, manipulators) via touch-screen panels. GUI-based, no serial API. Not needed for v2.
- **OWON PSU** is our controllable heater bypass for AI-Scientist mode (v4). Deprioritized per PI.
- CPU inference benchmarked at **13ms/frame** — no GPU/cluster needed.

## Deployment Target
- **Bulbasaur** (lab PC, Windows): Python 3.12.10 installed, GUI verified 2026-03-12
  - Displays: 1920x1200 + 1920x1080
  - COM4: Prolific PL2303GS → pyrometer RS-422 (IFD-5)
  - OWON PSU: not yet connected on Bulbasaur (deprioritized)
  - TeamViewer: ID 1 756 871 318

## Setup
```bash
# Mac
python3 -m venv .venv && source .venv/bin/activate
pip install pyvisa pyvisa-py pyserial PyQt6 pyqtgraph numpy

# Bulbasaur (Windows) — no venv, system Python
pip install PyQt6 pyqtgraph numpy pyvisa pyvisa-py pyserial mss Pillow pywinauto pymodbus openpyxl
```

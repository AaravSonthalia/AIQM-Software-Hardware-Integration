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

### Verified on Bulbasaur (Mar 23, 2026)
- **RHEED screengrab: WORKING** — captures detached kSA Live Video window
  - Requires kSA setting: Options → General → Images and Video → **uncheck "Keep Live Video inside application"**
  - Window found via 3-priority search: detached top-level → MDI child → main kSA fallback
- **Pyrometer screengrab: WORKING** — reads live temperature from TemperaSure (165°C at idle verified)
- **Full session test PASSED** — commit_log.csv, sensor_log.csv, session_metadata.json, growth_log.xlsx all generated correctly with correct data
- **Frame capture: WORKING** — RHEED PNGs saved to frames/ on each LOG ENTRY

### Layout
- **Top bar (always visible):** Grower name, Sample ID, ARM / START / STOP
- **Monitor tab:** Value displays (elapsed, temp, V, I), live RHEED image, expanding note text box, LOG ENTRY button (Ctrl+S)
- **Session tab:** Config (upper-left), Sensor Log interval reads (upper-right), Growth Notes grower commits (bottom half), Export Growth Log button

### Key Files
| File | Purpose |
|------|---------|
| `growth_monitor_app.py` | Launcher |
| `gui/growth_monitor.py` | Two-tab UI: Monitor (RHEED + notes) and Session (config + logs + export) |
| `gui/growth_app.py` | Orchestrator: ARM/DISARM/START/STOP, worker lifecycle, sensor logging, auto-export on STOP |
| `gui/growth_logger.py` | Session logging: sensor CSV, commit CSV, frame PNGs, JSON metadata, xlsx/CSV export |
| `gui/classifier_bridge.py` | Loads Classifier2 model — NOT used in v2, kept for v3 |
| `gui/auto_capture.py` | Auto-capture engine — NOT used in v2, kept for v3 |
| `gui/state.py` | Shared state dataclasses |
| `gui/workers.py` | Background worker threads (camera, pyrometer, PSU, thermocouple) |
| `gui/widgets.py` | Reusable widgets (ValueDisplay, ControlPanel, ProtectionPanel) |
| `drivers/rheed_camera.py` | RheedCamera ABC + DummyCamera, ScreenGrabCamera (3-priority kSA window search), VmbCamera |
| `drivers/pyrometer.py` | TemperatureSensor ABC + DummyPyrometer, ScreenGrabPyrometer, ModbusPyrometer |
| `drivers/config.py` | MBESystemConfig presets (OXIDE_MBE, CHALCOGENIDE_MBE) |

### Session Output
Each session creates a directory under the configured save folder containing:
- `sensor_log.csv` — periodic temperature readings at configured interval (default 10s)
- `commit_log.csv` — timestamped grower entries with notes, temp, V, I, frame paths
- `frames/` — RHEED frame PNGs captured at each LOG ENTRY
- `session_metadata.json` — grower, sample ID, date, entry count
- `growth_log.xlsx` — OMBE growth log export (auto-generated on STOP, also via Export button)

### Config Defaults
- Recording interval: 1s minimum, whole seconds (prevents faster-than-hardware commits)
- Camera mode: dummy (Mac) / screengrab (Bulbasaur)
- Pyrometer mode: dummy (Mac) / screengrab (Bulbasaur)

### ScreenGrabCamera — kSA Window Detection
The `ScreenGrabCamera` uses a 3-priority search to find the correct kSA window:
1. **Detached top-level window** — when kSA "Keep Live Video inside app" is unchecked (preferred)
2. **MDI child window** — searches inside the main kSA frame
3. **Main kSA window fallback** — captures the full application

Default search term: `"Live Video"`. Uses `EnumWindows` + `EnumChildWindows` with case-insensitive substring matching, excluding windows starting with "kSA 400" for priority 1.

### Known Issues / TODO
- RHEED capture includes kSA title bar + status bar — crop to image region only (cosmetic)
- Image very dim at idle (Pixel Int: 0.7%) — will be brighter during actual growth
- V/I displays show "---" — no source connected yet (investigate kSA Temperature Control or Analog Input)
- Edge case testing needed: empty fields, rapid commits, multiple sessions, close-while-running

### Phased Roadmap
- **v1:** Classification-centric display + manual logging — **DONE** (archived)
- **v2 (current):** Growth log assistant for OMBE — **CORE FUNCTIONALITY VERIFIED**
  - [x] Python 3.12 on Bulbasaur, GUI launches
  - [x] RHEED screengrab captures detached kSA Live Video window
  - [x] Pyrometer screengrab reads live temp from TemperaSure (verified 165°C)
  - [x] Growth-log-centric UI with two-tab layout (Monitor + Session)
  - [x] OMBE growth log export (xlsx matching lab template + CSV fallback)
  - [x] Auto-export on STOP + manual Export button
  - [x] Full end-to-end growth session test on Bulbasaur — PASSED
  - [x] Sensor log: newest-on-top, no scroll reset, 10s interval reads verified
  - [x] Frame PNGs saved on each LOG ENTRY
  - [ ] Crop kSA title bar / status bar from RHEED capture
  - [ ] Edge case testing (empty fields, rapid commits, close-while-running)
  - [ ] Investigate kSA for V/I data (Temperature Control, Analog Input, Automation interface)
  - [ ] Test during actual growth (brighter RHEED image, real temperature ramps)
  - [ ] ChMBE variant (similar template, different elements)
- **v3:** Add reconstruction classification checkboxes + Classifier2 ML confidence
- **v4:** RL policy, PID loop, closed-loop AI-Scientist mode
- **Deferred:** OWON PSU on Bulbasaur, PID step tests, auto-capture

## kSA 400 Configuration (Bulbasaur)
- **Camera:** AVT Manta_G-033B (E0022060) 10, model AVT Vimba
- **Live Video title:** `AVT Manta_G-033B (E0022060) 10 Live Video [live]`
- **Settings:** Exposure 700ms, Sum 1, Palette Green scale, Mapping Normal, A/D Black=30 White=0 Gain=1.0
- **Critical setting:** Options → General → Images and Video → "Keep Live Video inside application" must be **unchecked**
- **Chamber Interface:** Set to `<None>`, options available: kSAModBus32, kSAComm, Automation, kSADigitalControl
- **User manual:** `C:\Users\Lab10\Documents\kSA\kSA 400\Help\` (PDF + HTML)

## MBE Hardware Control
- **Scienta Omicron MISTRAL** controls all MBE hardware (pumps, valves, heaters, manipulators) via touch-screen panels. GUI-based, no serial API. Not needed for v2.
- **OWON PSU** is our controllable heater bypass for AI-Scientist mode (v4). Deprioritized per PI.
- CPU inference benchmarked at **13ms/frame** — no GPU/cluster needed.

## Deployment Target
- **Bulbasaur** (lab PC, Windows): Python 3.12.10 installed, GUI verified 2026-03-23
  - User: `Lab10`
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

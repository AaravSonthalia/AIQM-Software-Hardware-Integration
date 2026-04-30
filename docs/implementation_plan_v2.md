# AIQM Implementation Plan — v2 and Beyond

**Date:** 2026-03-07
**Author:** AJ
**Status:** DRAFT — for team review

---

## Context

Growth Monitor v1 (display + manual logging) is complete and was demoed at the Mar 6 PI meeting.
Python install on Bulbasaur is now approved. The next milestone is deploying a working,
non-AI version of the GUI on the lab computer with live hardware connections.

This plan reflects the staged roadmap agreed upon with the PI:
- v1: Display + manual logging (DONE)
- v2: Deploy to Bulbasaur, connect live hardware
- v3: Re-enable Classifier2, live AI confidence alongside human checkboxes
- v4: RL + PID closed-loop, full AI-Scientist mode

---

## v2: Live Hardware on Bulbasaur

**Goal:** Growth Monitor GUI running on Bulbasaur with live RHEED frames, pyrometer
temperature, and MISTRAL parameter display. No AI — humans classify and save observations.

**Priority:** ASAP (critical path for all downstream work)

### Phase 2.1: Bulbasaur Environment Setup

**Tasks:**
1. Install Python 3.12 on Bulbasaur (embeddable zip or winget — user-level, no admin needed)
2. Install minimal dependencies:
   - PyQt6 (GUI)
   - pyautogui + pywinauto (screen scraping)
   - Pillow (image processing)
   - minimalmodbus (pyrometer serial)
   - pyserial (OWON serial — for later, but install now)
   - numpy
   - Total footprint: ~100 MB without PyTorch (ML not needed for v2)
3. Clone/copy AIQM-Software-Hardware-Integration repo to Bulbasaur
4. Verify GUI launches with dummy drivers on Bulbasaur

**Estimated footprint:** ~100 MB (no PyTorch for v2)
**Blocker:** None (Python install approved)

### Phase 2.2: RHEED Camera Integration (kSA Screen Grab)

**Tasks:**
1. Identify kSA 400 window on Bulbasaur (window title: "AVT Manta_G Live Video" or similar)
2. Implement/test `ScreenGrabCamera` driver:
   - Use pyautogui or win32gui to locate and capture kSA window
   - Crop to RHEED image region (exclude kSA UI chrome)
   - Characterize the false-color LUT (Classifier2 was trained on these colors)
3. Verify frame rate: target 1 FPS minimum (screen grab ~10ms expected)
4. Test with RHEED gun UP and DOWN to confirm we get valid frames

**Key constraint:** Classifier2 was trained on kSA false-color screenshots, so screen
grabbing is the PRIMARY capture method. Direct camera access (vmbpy) produces different
images and would require retraining.

**Lab test checklist:**
- [ ] kSA 400 running and displaying live RHEED?
- [ ] Window title and dimensions confirmed?
- [ ] Screen grab captures correct region?
- [ ] False-color LUT matches training data appearance?
- [ ] Frame rate stable at 1+ FPS?

### Phase 2.3: Pyrometer Integration (COM4 Serial)

**Tasks:**
1. Confirm pyrometer is powered ON during test session
2. Test serial read on COM4 (PL2303GS adapter) at candidate baud rates (9600, 19200)
3. Implement Modbus RTU read using minimalmodbus:
   - Register map already documented in `pyrometer_script.py`
   - Read temperature register at 1 Hz
4. Wire `ModbusPyrometer` driver into Growth Monitor
5. Fallback: if Modbus fails, implement `ScreenGrabPyrometer` (scrape TemperaSure window)

**Lab test checklist:**
- [ ] Pyrometer powered on?
- [ ] COM4 accessible from Python?
- [ ] Baud rate determined? (try 9600 first, then 19200)
- [ ] Temperature reading matches TemperaSure display?
- [ ] Stable 1 Hz reads without timeouts?

### Phase 2.4: MISTRAL Data Access

**Tasks:**
1. **In-lab reconnaissance (required first):**
   - Document what parameters MISTRAL displays (temperature, voltage, current, pressure, etc.)
   - Check if MISTRAL writes log files to disk (look for .txt, .csv, .log in program directory)
   - Check if MISTRAL exposes any network ports (we saw 48898, 8080, 7070 on Bulbasaur)
   - Screenshot MISTRAL UI for screen-scraping target identification
2. **Determine data access strategy:**
   - **Option A (preferred): Log file tailing** — if MISTRAL writes growth logs, parse them in real time
   - **Option B: Network protocol** — if ports 48898/8080/7070 are MISTRAL-related, probe for data
   - **Option C: Screen scraping** — pywinauto to read displayed values from MISTRAL window
   - **Option D: Manual entry** — worst case, user types key values into config panel
3. Implement chosen strategy and wire into Growth Monitor dashboard

**Key parameters to capture (from meeting notes):**
- Substrate temperature (primary — may overlap with pyrometer)
- Heater voltage and current
- Chamber pressure
- Elapsed time / growth timer
- Sample ID

**Lab test checklist:**
- [ ] MISTRAL running and displaying data?
- [ ] Log files found on disk? (location, format, update frequency)
- [ ] Ports 48898/8080/7070 probed? (HTTP? custom protocol?)
- [ ] Screen scraping feasible? (window titles, control IDs via pywinauto inspect)
- [ ] Chosen strategy reads data reliably at 1+ Hz?

### Phase 2.5: Integration Testing on Bulbasaur

**Tasks:**
1. Run Growth Monitor with all live drivers simultaneously
2. Verify: RHEED frames display, pyrometer temp updates, MISTRAL data flows
3. Test SAVE OBSERVATION workflow end-to-end (frame + temp + human checkboxes → disk)
4. Stress test: run for 30+ minutes, check for memory leaks or frame drops
5. Test config panel: save folder selection, recording interval, filename prefix
6. Take screenshots and short screen recording for PI demo

**Success criteria:**
- GUI runs stably for 1+ hour on Bulbasaur
- RHEED frames at 1+ FPS
- Pyrometer temp at 1 Hz
- MISTRAL data at 0.1+ Hz (whatever is achievable)
- Observations save correctly with all metadata

---

## v2 Lab Visit Protocol

Since several v2 phases require in-lab testing, here is a checklist for the next lab visit:

### Pre-visit (can do remotely via TeamViewer)
- [ ] Install Python 3.12 on Bulbasaur
- [ ] Install pip packages (PyQt6, pyautogui, pywinauto, minimalmodbus, etc.)
- [ ] Copy repo to Bulbasaur
- [ ] Verify GUI launches with dummy drivers

### In-lab (requires physical presence or instruments ON)
- [ ] RHEED gun status — is it repaired and operational?
- [ ] Pyrometer status — is it powered on?
- [ ] kSA screen grab test
- [ ] COM4 pyrometer serial test
- [ ] MISTRAL reconnaissance (log files, network ports, UI inspection)
- [ ] SSH into SENA device at 10.0.42.231 to identify what it bridges
- [ ] Check http://localhost:8080 on Bulbasaur
- [ ] End-to-end integration test if all hardware is available

---

## v3: Classifier2 Re-enablement (after v2 is stable)

**Goal:** Live AI classification running alongside human checkboxes. Invite Justin.

### Phase 3.1: Deploy Model to Bulbasaur
1. Install PyTorch CPU-only on Bulbasaur (~250 MB additional)
2. Copy `best_model.pth` + ideal/bad reference images
3. Verify ClassifierBridge loads and runs inference (~13ms/frame benchmarked)

### Phase 3.2: Classifier Improvements (parallel work with Justin)
1. **More pairwise labels** — especially 1x1 (80% accuracy) and Twinned (only 6 ideal images)
2. **RT13 brightness cleanup** — 8 problematic images (1,3,5,6,7,8,9,10) need processing or removal
3. **Evaluate on live kSA frames** — do training-time screenshots match live screen grabs?
4. **Dedicated AI-MBE beamtime** — schedule data collection sessions for labeled images

### Phase 3.3: Live AI Display
1. Un-gray AI widgets in Growth Monitor (confidence bar, class label, quality score)
2. Run inference every Nth frame (configurable in config panel)
3. Log AI predictions alongside human observations for comparison
4. Auto-capture Tier 3 (ClassificationChangeDetector) enabled

### Phase 3.4: Human-AI Comparison Analysis
1. Collect dual human + AI labels over multiple growth sessions
2. Analyze agreement rates per reconstruction type
3. Identify systematic disagreements → feed back into labeling + retraining
4. This data is gold for online learning (mentioned in Feb 13 meeting)

---

## v4: RL + PID Closed-Loop (future, after v3 is validated)

**Goal:** Full AI-Scientist mode — RL policy controls substrate temperature.

### Prerequisites
- v3 running reliably with validated classifier accuracy
- OWON PSU connected and PID tuned (separate hardware workstream)
- RL policy trained on historical growth data (Anneal RL module)
- RHEED encoder stub connected to Classifier2

### Key integration points
- `anneal_rl/perceive/rheed_encoder.py` → connect to ClassifierBridge output
- RL policy output → PID setpoint → OWON serial command
- Safety shield: clamp actions to safe bounds (ΔT_max=200C per step)
- Human override capability in GUI (DISARM button)

---

## Parallel Workstreams

### FRHEED Evaluation (low priority, opportunistic)
- Test if FRHEED can connect to Allied Vision Manta G-033B via GigE
- If yes: evaluate as alternative/complement to kSA screen grabbing
- If no: borrow useful patterns from their camera capture architecture
- Lab tests needed (see separate FRHEED test plan)

### Literature & External Tools (ongoing)
- Contact authors of arXiv 2602.18243 (Feb 2026 CNN RHEED classifier) for code/data
- Study Harris et al. (Jan 2025) STO-specific RHEED deep learning
- Monitor awesome-self-driving-labs for new tools
- Evaluate NMF + classifier integration approach (pending discussion with Justin)

### Data Collection Strategy (ongoing)
- Schedule dedicated AI-MBE beamtime in next growth campaign
- Prioritize labeling 1x1 and Twinned ideal images
- Discuss additive component identification with Justin (RT13 + c6x2 coexistence)
- Expand pairwise comparison coverage (currently 170/1,124 = 15.1% labeled)

---

## Decision Log

| Date | Decision | Rationale |
|------|----------|-----------|
| Mar 6 | GUI runs on Bulbasaur (Option B) | All hardware (OWON USB, COM4, kSA) must be local |
| Mar 6 | v1 = display only, no AI | Step-by-step approach, validate basics first |
| Mar 6 | Checkboxes not sliders | Multiple reconstructions can coexist in transitions |
| Mar 6 | Invite Justin after working v2 | Need minimum viable hardware integration first |
| Mar 7 | Python install approved | User-level install, ~100 MB for v2 (no PyTorch) |
| Mar 7 | Remote cluster idea dropped | Inference is ~13ms/frame, not 30-60s (miscommunication) |
| Mar 7 | MISTRAL access strategy TBD | Requires in-lab reconnaissance — log files, ports, or screen scrape |

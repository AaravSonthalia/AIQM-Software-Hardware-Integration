# Bulbasaur Smoke Test — Auto-Capture Pre-Lab Checklist

Goal: confirm the auto-capture wiring (commits `4d408c2` → `9dcdc67` on
`v2-evap-mistral-capture`) launches and works correctly on Bulbasaur
*before* taking it into the lab on a real OMBE growth.

## 1. Pull the latest

```powershell
cd C:\path\to\AIQM-Software-Hardware-Integration
git fetch origin
git checkout v2-evap-mistral-capture
git pull
```

Expected: 5 commits since your last pull (auto-capture wiring, offline
script, buffer-mean mode, kSA crop, OCR debug).

## 2. Confirm dependencies

```powershell
.\.venv\Scripts\python.exe -m pip list | Select-String -Pattern "Pillow|matplotlib|pytesseract|mss|PyQt6"
```

Expected: all five present. If `Pillow` or `matplotlib` is missing on
Bulbasaur, install them (they were added to the Mac venv in this
session but Bulbasaur may need separate install):

```powershell
.\.venv\Scripts\python.exe -m pip install Pillow matplotlib
```

## 3. GUI launch — does it boot?

```powershell
.\.venv\Scripts\python.exe growth_monitor_app.py
```

Verify visually:
- [ ] Window opens with title "OMBE Growth Monitor"
- [ ] Top bar: Grower / Sample ID / ARM-START-STOP buttons present
- [ ] Monitor tab: value displays + RHEED area + recon sliders + LOG ENTRY button
- [ ] **NEW:** thin gray footer at bottom of Monitor tab reads `Auto-capture: idle`
- [ ] No crashes / Python tracebacks in the console

If the GUI doesn't launch, check the console output for the actual error.

## 4. Dummy session — does the auto-capture path run?

In the GUI:
- [ ] Set `Camera mode` = `dummy` (Session tab → Config)
- [ ] Set `Pyrometer mode` = `dummy`
- [ ] Set `MISTRAL mode` = `dummy`
- [ ] Set `Evap Control mode` = `dummy`
- [ ] Enter a Grower name and Sample ID like `smoketest_apr27`
- [ ] Click **ARM**
- [ ] Click **START**
- [ ] Wait ~30 seconds. The footer should change from `idle` to
      `Auto-capture: armed (warmup)` then to
      `Auto-capture: armed | score: X.XX | events: 0`.
- [ ] Click **STOP**

Verify the session output:
- [ ] Look in `logs/growths/growth_smoketest_apr27_<timestamp>/`
- [ ] File exists: `auto_capture_events.csv` (header at minimum, may be empty
      since dummy frames are random — events possible but not guaranteed)
- [ ] Existing files still produced: `sensor_log.csv`, `commit_log.csv`,
      `session_metadata.json`, `growth_log.xlsx`

If any of those are missing or there's a crash on STOP, capture the
console output and report back.

## 5. kSA window crop verification (when kSA is open)

The crop defaults to 75 px from the top, 30 px from the bottom of the
captured kSA window. These values were measured from a screenshot
(`/Users/aj/Downloads/entry_002_154807.png`); they may need adjustment
if Bulbasaur's kSA has a different DPI / theme.

To verify:
- [ ] Start a session with `Camera mode = screengrab` while kSA Live Video
      is open
- [ ] After a few seconds, look at the saved frame in
      `logs/growths/<session>/frames/` (LOG ENTRY to save one). The frame
      should contain *only* the green RHEED image — no title bar, no
      "Exposure: 999.00 ms" status text at the bottom.

If the crop is wrong:
- Too much cropped → set `chrome_top_px=60` (or similar) in
  `gui/workers.py` where `ScreenGrabCamera` is instantiated
- Bottom status text still visible → increase `chrome_bottom_px`
- Or disable cropping entirely with `crop_chrome=False`

## 6. OCR debug crop-save

Run the GUI with the debug env var:

```powershell
$env:AIQM_OCR_DEBUG="1"
.\.venv\Scripts\python.exe growth_monitor_app.py
```

(After this session, unset with `Remove-Item Env:AIQM_OCR_DEBUG`.)

Verify:
- [ ] After arming with screengrab modes for MISTRAL + Evap, check that
      `logs/ocr_debug/` exists and is filling with `mistral_*.png` and
      `evap_*.png` files (one per OCR call, ~1/sec each)
- [ ] Console shows `[OCR mistral] '...'` and `[OCR evap] '...'` lines
- [ ] Saved crops should *visually contain* the V/I or pressure value —
      this is the corpus for offline preprocessing experiments

## 7. Stress test (optional, ~5 min)

Run a 5-minute dummy session and confirm:
- [ ] No memory leak (GUI memory stays roughly flat in Task Manager)
- [ ] No crashes
- [ ] `auto_capture_events.csv` has a reasonable count (hopefully 0–3 events
      for dummy random data; lots of events would mean the threshold is
      too low for synthetic noise — expected, not a bug)
- [ ] All CSV files close cleanly on STOP (no "file in use" errors)

## What to report back

If anything fails:
1. The exact step that failed
2. Console output / traceback
3. Whether the GUI crashed or just behaved wrong
4. A screenshot of the failure if visual

If everything passes:
- "Smoke test passed, ready for lab" — and we're clear to plan the
  first OMBE session.

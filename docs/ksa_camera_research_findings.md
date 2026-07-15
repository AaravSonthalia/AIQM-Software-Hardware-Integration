# kSA direct camera reading — research findings

**Author:** AJ  \
**Date:** Jul 15 2026  \
**Scope:** Can AIQM read RHEED frames directly from the camera or via
kSA's TCP interface, alongside kSA continuing to operate its own live
view? Or must we keep relying on screengrab?

## TL;DR

The fundamental camera-lock constraint holds: **`VmbCamera` and
kSA cannot both hold the Manta G-033B at the same time.** Three
non-screengrab paths exist, each with a different cost:

| Path | State | Blocker |
|---|---|---|
| A — VmbCamera direct (`vmbpy`) | ✅ Shipped (`drivers/rheed_camera.py::VmbCamera`) | Exclusive lock — can't run alongside kSA |
| B — kSA TEXT_CMD image export | ❓ Unexplored | Need to probe kSA scripting language for `save current image` or `get frame` commands |
| C — GigE multi-cast shared stream | ❓ Requires network reconfig | Camera XML shows `StreamCount=1`; Manta hardware supports multi-cast but the current lab config doesn't use it |

**Recommendation:** Path B is the highest-value unexplored option
because it doesn't require the grower to stop kSA. Path C is
architecturally cleaner but requires network configuration changes
on Bulbasaur that need coordination with lab IT. Path A is our
current shipped fallback and stays.

---

## What already exists

### VmbCamera (`drivers/rheed_camera.py:63`)

Full streaming implementation, ~350 lines. Uses:
- `vmbpy` SDK (Allied Vision Vimba X)
- `TriggerSource='Software'` + `TriggerMode='On'` + `AcquisitionMode='Continuous'`
- Streaming callback pattern with threading (per-call trigger fails on Bulbasaur — May 8 2026 discovery)
- 12-bit mono normalized to uint8 via fixed 4095 denominator
- Frame handler wraps intensity in green channel to match AIQM's RGB convention
- Proper thread safety: `_frame_lock`, `_error_lock`, `_ready_event`, `_stop_event`

**Status:** Complete + shipped. Matches Jacques' `OxideMBE_RHEED_GUI.py`
reference pattern almost identically (both use `TriggerSoftware.run()`
in a while loop with `time.sleep(period)`). Adopted from his code
during path_a_vimba_integration effort.

**Limitation:** exclusive camera lock. When kSA is running,
`VmbCamera.connect()` will fail. Grower must stop kSA before AIQM
can take over the camera. Not viable for concurrent-with-kSA
operation.

### KsaCommClient (`drivers/ksa_comm.py:289`)

kSA v6 wire-protocol client. Reverse-engineered May 20 2026,
re-confirmed Jul 7. Ships 12 binary commands + text query interface:

```
CMD_INITIALIZE, SET_DATA_FIELDS, RUN, GET_DATA, STOP,
GET_DATA_SPECIFIC, RESTART_GROWTHRATE_FIT, OPEN_ACQUIRE,
CLOSE_ACQUIRE, GET_STATUS, GET_APP_VERSION, TEXT_CMD
```

The binary commands are all about **acquired data** (intensity time
series, growth-rate fits, status flags) — **not raw camera images.**
Nothing in the binary protocol says "give me the current frame."

The `TEXT_CMD` (1011) interface accepts kSA's SQL-like scripting
language. Probes tried so far:
- `"version"` → app version string
- `"app query SQL 'SELECT 1'"` → SESSION_REQUIRED error
- `"?"` → generic probe

**Unexplored TEXT_CMD space is where Path B lives.** kSA scripting
likely has commands like:
- `save current image "path"` — export the live-view frame
- `get_pixel_data(...)` — return array data
- `roi.SaveImage(...)` — ROI-scoped image export

None have been probed yet. Requires live Bulbasaur session.

### ScreenGrabCamera (`drivers/rheed_camera.py:360`)

Windows-only screen-capture fallback. Uses kSA's floating Live Video
window as the source. Lossy (JPEG-compressed screen buffer, kSA's
BGW-ramp false-color palette). Works fine when kSA is running, but
throws away 12-bit precision + adds palette-conversion overhead.

**Status:** Complete + shipped. Currently the default on Bulbasaur.

---

## Path A — VmbCamera (shipped, exclusive-lock only)

Already covered above. Not new research; documenting for
completeness.

**When to use:** kSA is intentionally not running (e.g., dedicated
AIQM acquisition session), OR the grower has stopped kSA for a
specific measurement.

**When it fails:** kSA holds the camera. `vmbpy.get_all_cameras()`
returns empty list, or `cam.open()` raises `VmbCameraError:
Camera in use.`

---

## Path B — kSA TEXT_CMD image export (unexplored)

kSA has its own scripting language (likely called "APS" or similar
from vendor context). Anything you can type into kSA's script window
should be reachable over `CMD_TEXT_CMD`.

**Concrete probes worth trying at Bulbasaur:**

```python
# Probe 1: does kSA expose a "save current image" command?
client.send_text('save current image "C:\\temp\\test.bmp"')

# Probe 2: does kSA have a pixel-data getter?
client.send_text('get_pixel_data 0 0 100 100')  # ROI xywh

# Probe 3: enumerate the available text commands
client.send_text('help')
client.send_text('?commands')

# Probe 4: kSA's ROI infrastructure
client.send_text('roi list')
client.send_text('roi save "C:\\temp\\roi_snap.bmp"')

# Probe 5: image object interrogation
client.send_text('image save "C:\\temp\\live.bmp"')
```

**If any of these succeed:** we have a path to read the current
kSA-displayed frame WITHOUT taking the camera lock. Would need to
handshake through a shared filesystem (kSA writes to disk, AIQM
polls the file), but that's a solved problem (`ScreenGrabCamera`
already polls kSA's own screen — polling a file is easier).

**Estimated Bulbasaur time to probe:** 30-45 minutes. Add as item
14 in the QA runner queue.

---

## Path C — GigE multi-cast (architectural)

Manta G-033B is a GigE Vision camera and supports multi-cast mode:
one camera can send frames to multiple hosts simultaneously (or the
same host multiple times). This would eliminate the exclusive-lock
constraint entirely.

**Current lab config (from `CameraSettings.xml`):**

```xml
<Feature Name="StreamCount" Value="1" Type="Int" />
<SelectorGroup Name="StreamSelector" Type="Int" Value="0">
    <Feature Name="StreamID" Value="DEV_000F314F7A86-Stream-000" Type="String" />
</SelectorGroup>
```

Single stream. To enable multi-cast:

1. Configure the camera for multi-cast (via Vimba Viewer or `vmbpy`):
   ```python
   cam.GevSCPSPacketSize.set(...)
   cam.GevSCPHostPort.set(...)
   cam.GevSCPDirection.set('Multicast')
   ```
2. Add multi-cast group + port to network config (Windows requires
   admin: `netsh interface add multicast ...`)
3. kSA and AIQM both `start_streaming(_handler)` — each receives its
   own copy of every frame

**Blocker:** requires network-level admin on Bulbasaur (or lab IT
coordination) + kSA itself would need to be configured to open a
multi-cast stream rather than an exclusive one. That last part is
NOT documented publicly for kSA and would need vendor consultation.

**Verdict:** worth pursuing IF Path B fails AND concurrent-with-kSA
operation is a hard requirement. Not the first thing to try.

---

## Path A vs B vs C — decision matrix

| Criterion | Path A (VmbCamera) | Path B (TEXT_CMD) | Path C (multi-cast) |
|---|---|---|---|
| Works alongside kSA | ❌ No | ✅ Yes (probably) | ✅ Yes (with config) |
| Full 12-bit precision | ✅ Yes | ❓ Unknown | ✅ Yes |
| Requires vendor docs | ❌ No | ⚠️ Some | ⚠️ Yes |
| Requires network admin | ❌ No | ❌ No | ✅ Yes |
| Latency | Low (~10ms) | Medium (disk poll) | Low (~10ms) |
| Effort to implement | ✅ Done | 1-2 days | 3-5 days |
| Bulbasaur session cost | 0 | 30-45 min | 2-4 hours |

---

## Next actions

Ordered by expected ROI:

1. **Add Path B probing to the Bulbasaur QA runner queue.** Item 14:
   "Probe kSA TEXT_CMD for image-export commands (5 probe strings)."
   30-45 min in-lab. Result determines whether Path B is viable.

2. **Clone Jacques' new Mini-MBE repo** (per
   `direct_read_research_jun23` memory) and grep for
   `SaveImage` / `save current` / `pixel_data` / `image save` —
   he may have already solved this and not told us. 15 min Mac-side.

3. **WebFetch kSA vendor docs** for the scripting language reference.
   Look for image-related commands. May require account login to
   kSA's customer portal.

4. **If B fails, prototype C on a Mac-side dev cam** first — Manta
   G-033B multi-cast can be tested without touching Bulbasaur if
   we have any GigE camera on hand.

---

## Cross-references

- Memory `path_a_vimba_integration_plan` — original Path A plan
- Memory `jacques_rheed_viewer_code` — three techniques inventory
- Memory `ksa_tcp_ip_breakthrough_may20` — protocol cracking notes
- Memory `direct_read_research_jun23` — Jacques Mini-MBE reference
- `drivers/rheed_camera.py::VmbCamera` — Path A implementation
- `drivers/ksa_comm.py::KsaCommClient` — Path B foundation (needs
  TEXT_CMD probes)
- `scripts/bulbasaur_qa_runner.py` — where to add the Path B probe
  item for next lab session

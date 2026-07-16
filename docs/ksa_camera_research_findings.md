# kSA direct camera reading — research findings

**Author:** AJ  \
**Date:** Jul 15 2026  \
**Scope:** Can AIQM read RHEED frames directly from the camera or via
kSA's TCP interface, alongside kSA continuing to operate its own live
view? Or must we keep relying on screengrab?

## TL;DR

The fundamental camera-lock constraint holds for the **full-owner**
mode: `VmbCamera` and kSA cannot both hold the Manta G-033B with
`AccessMode.Full` at the same time. **But** — per Codex's Jul 16
investigation — Allied Vision's VmbPy exposes `AccessMode.Read`,
which is documented for passive frame reception while another app
owns the camera. That's a fourth path.

| Path | State | Blocker |
|---|---|---|
| A — VmbCamera direct (`vmbpy`, `AccessMode.Full`) | ✅ Shipped (`drivers/rheed_camera.py::VmbCamera`) | Exclusive lock — can't run alongside kSA |
| B — kSA TEXT_CMD image export | ❓ Unprobed | Need to probe kSA scripting for `save current image` etc. Probe script ready (`scripts/probe_ksa_image_export.py`). **Caveat: kSA advertises 8-bit or 10-bit programmable output — exported files may NOT preserve 12-bit precision** |
| C — GigE multi-cast shared stream | ❓ Requires network reconfig | Camera XML shows `StreamCount=1`; Manta supports multi-cast but lab config doesn't |
| **D — VmbPy `AccessMode.Read` coexistence** | ❓ Unprobed (Codex finding Jul 16) | Simpler than B or C if it works. Probe script ready (`scripts/probe_vimba_access_modes.py`). Allied Vision docs describe this as the passive-read/multicast path |

**Recommendation:** probe B and D in the same lab session
(items #14 + #15 in the Bulbasaur QA runner queue). If either
works, we have concurrent-with-kSA operation. Path A stays as
the shipped fallback for dedicated AIQM sessions.

**Related Codex artifacts** (in `test-codex/codex-mirrors/AIQM-Software-Hardware-Integration copy/docs/`):
- `direct_camera_access_investigation.md` — Codex's parallel investigation with detailed outcome rules per probe
- `ksa_camera_research_findings.md` — Codex's version of THIS doc; overlaps but includes the kSA 8-bit output finding + `AccessMode.Read` discovery this doc now incorporates

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

## Path D — VmbPy `AccessMode.Read` (Codex discovery, Jul 16)

Allied Vision's VmbPy documents a `Camera.set_access_mode(...)`
API with `AccessMode.Read` as an explicit option. Per the VmbC
docs, Read access is the passive frame reception mode intended
for cases where another application (kSA in our case) already
owns the camera as the primary controller.

If this works, it's simpler than Path B (no vendor scripting
language reverse engineering) and simpler than Path C (no
network reconfig, no vendor coordination for kSA multi-cast
support).

**Codex has already scaffolded the probe.**

```
scripts/probe_vimba_access_modes.py --result-json <path>
# inventory only — no camera open

scripts/probe_vimba_access_modes.py \
  --attempt-read-open --attempt-read-stream --duration-s 5 \
  --result-json <path>
# opens in Read mode, tries to receive frames for 5s
```

**Expected results by outcome:**

- Read-mode opens + receives frames while kSA is running →
  build a `VmbCameraReadOnly` driver that consumes frames the
  same way `VmbCamera` does but with `AccessMode.Read`. Route
  through the same worker + state machinery. Grower workflow
  unchanged.
- Read-mode opens but no frames arrive → passive read requires
  the primary owner (kSA) to be configured for multi-cast
  streaming. Escalates to a kSA vendor conversation.
- Read-mode won't open at all → VmbPy install issue, or the
  camera firmware doesn't support Read mode for this vendor
  configuration.

**Blocker:** none Mac-side. The probe script is ready; needs a
Bulbasaur lab session with kSA running.

**Related Codex investigation:** see
`test-codex/.../docs/direct_camera_access_investigation.md`
for detailed outcome rules per probe and the vendor questions
Codex drafted for k-Space/Allied Vision.

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

## Path A vs B vs C vs D — decision matrix

| Criterion | A (Vmb Full) | B (TEXT_CMD) | C (multi-cast) | D (Vmb Read) |
|---|---|---|---|---|
| Works alongside kSA | ❌ No | ✅ Yes (probably) | ✅ Yes (with config) | ✅ Yes (per Allied Vision docs) |
| Full 12-bit precision | ✅ Yes | ⚠️ No (kSA outputs 8 or 10-bit) | ✅ Yes | ✅ Yes (native SDK path) |
| Requires vendor docs | ❌ No | ⚠️ Some | ⚠️ Yes | ✅ Standard Allied Vision docs |
| Requires network admin | ❌ No | ❌ No | ✅ Yes | ❌ Probably not |
| Latency | Low (~10ms) | Medium (disk poll) | Low (~10ms) | Low (~10ms) |
| Effort to implement | ✅ Done | 1-2 days after probe | 3-5 days | 1 day after probe |
| Bulbasaur session cost | 0 | 30-45 min | 2-4 hours | 30-45 min |
| Probe script ready | ✅ N/A | ✅ `probe_ksa_image_export.py` | ❌ | ✅ `probe_vimba_access_modes.py` |

**Path D leapfrogs B on precision** (full 12-bit vs kSA's 8/10-bit
programmable output) IF the read-mode probe returns frames.
Whichever probe hits green first at the next lab session wins.

---

## Next actions

Ordered by expected ROI:

1. **Bulbasaur QA queue items #14 + #15.** Both probes run in the
   same lab visit. ~1 hour combined. Result answers whether B or
   D (or both) are viable.

2. **Adopt Codex's `KsaImageExportCamera` stub** into
   `drivers/rheed_camera.py` if Path B probe succeeds. Codex has
   the stub in their mirror.

3. **Build `VmbCameraReadOnly`** if Path D probe succeeds.
   Trivial refactor of `VmbCamera` — swap `AccessMode.Full` for
   `AccessMode.Read` and skip the trigger commands.

4. **Clone Jacques' new Mini-MBE repo** (per
   `direct_read_research_jun23` memory) if BOTH probes fail —
   Jacques may have solved this with a different technique we
   haven't considered. 15 min Mac-side.

5. **k-Space vendor question** (Codex drafted the text) if paths
   B, C, and D all fail: does kSA 400 expose the current live
   RHEED image to another process while kSA owns an Allied
   Vision GigE camera?

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

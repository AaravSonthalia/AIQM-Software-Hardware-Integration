# Lab Validation — O-MBE (Bulbasaur), 2026-07-27

Durable record of the four-instrument direct-read validation session on
Bulbasaur (O-MBE / Yang Group Oxide MBE). Complements the source-tree
findings in `docs/ksa_camera_research_findings.md` and the vendor-side
discovery notes tracked in Claude memory.

## Session context

- **Chamber**: O-MBE / Bulbasaur (Windows 11, `10.0.42.254`)
- **Grower on-site**: AJ Bradshaw (validation session, no growth)
- **Goal**: end-to-end direct-read validation of the four instruments that
  populate the Growth Monitor GUI on Bulbasaur — camera (RHEED), elog
  (EvapControl), MISTRAL (cell PSU), and BASF Exactus pyrometer.
- **Outcome**: three of four instruments validated, one pinned for physical
  inspection, and one **architectural breakthrough** for MISTRAL (below).

## Instrument-by-instrument outcomes

### RHEED camera (Allied Vision Manta G-033B, GigE, `DEV_000F314F7A86`)

**Status**: Direct-read pipeline validated with kSA closed.

Three fixes shipped during the session:

| Commit | Fix |
|--------|-----|
| `5789f60` | Bump `VmbCamera.CONNECT_TIMEOUT_S` from 10 s → 45 s. Root cause: Bulbasaur has two Vimba transport-layer providers registered (old Vimba + Vimba X), so `get_all_cameras()` walks 9 network interfaces and takes ~15-20 s — well beyond the previous 10 s cap. |
| `016da9e` | `scripts/precheck_direct_camera.py` retry loop: replaced fragile string match (`"not yet available"`) with `isinstance(e, FrameNotYetAvailableError)`. The old check never triggered because the actual error message says `"no frame has arrived yet"` (no substring match). Deadline widened from 2.5 s → 5 s. |
| `fea7e93` | `scripts/precheck_direct_camera.py` palette check: rewritten to verify every unique RGB triple in the frame is a valid entry of `KSA_BGW_PALETTE`. The previous check enforced R-std ≈ G-std ≈ B-std (a `(I, I, I)` grayscale-stack assumption from the interim Jul 2 fix), which fails as expected under the current BGW LUT for dark frames. |

**Still open** (not shipped this session):
- kSA-open coexistence (needs multicast enabled via Vimba X Viewer — Task #187)
- Real RHEED signal quality validation with an active beam
- Display-only contrast stretch (max raw byte value ≈ 175 on ambient chamber → lime-green in BGW; visually dim vs. kSA)

### Elog reader (EvapControl `.elo` binary log)

**Status**: Direct-read validated. Live chamber data returned on first call:

```python
{
  'chamber_pressure_mbar': 3.24e-08,       # good UHV
  'substrate_temp_pv_C':   58.18,
  'substrate_temp_setpoint_C': 0.0,
  'cell_HTEC2_pv_C':       799.99,          # active source
  'cell_Y_pv_C':           149.98,          # warm standby
  'cell_Sr_pv_C':          28.82,           # idle
  'cell_Eu_pv_C':          149.99,          # warm standby
  'cell_Er_pv_C':          28.36,           # idle
  'plasma_dc_bias_V':      None,            # plasma off
  'plasma_forward_W':      None,
  'plasma_reflected_W':    None,
}
```

No code changes required. This continues the Jul 9 2026 direct-read
integration (`e6f5f5d` + `b157466`).

### MISTRAL (Beckhoff TwinCAT ADS via `pyads`)

**Status**: **Breakthrough — Path B (direct pyads) validated.**

The intended vendor architecture on Bulbasaur is a Kestrel/JSON-RPC gateway
at `http://10.0.42.231:9000/api` that forwards requests to a Beckhoff PLC.
Every JSON-RPC discovery path we could probe returned nothing useful:

- `rpc.discover`, `system.listMethods`, `system.describe` → all `-32601` (not implemented)
- No Swagger, no OpenAPI, no `/api/list`, no `/api/methods`, no `/help` (all return the 137-byte "Scandes NoName" catch-all placeholder)
- OPTIONS on `/api` times out (not handled)
- 30+ common PSU method-name guesses (getVoltage, ReadValue, PSU.List, etc.) → all `-32601`

The Kestrel service is a well-behaved but fully opaque JSON-RPC 2.0 black box.

**Path B (bypass)**: reading the ModelDescription.xml at
`C:\Program Files\Scienta Omicron\MistralGui\ModelDescription.xml`
revealed that the JSON-RPC gateway is a proxy in front of a Beckhoff PLC at
AmsNetId `10.0.42.111.1.1`, port 851/852. We can talk `pyads` directly to
that PLC — same driver shape as Ch-MBE (Task #191), just a different netId.

Validated reads on Bulbasaur:
- `Main.ServiceMode` (port 851) → `False`
- `PIDProgram.Cell1_pidTDK.powerSupply.MeasuredVoltage` (port 852) → `0.041`
- `PIDProgram.Cell1_pidTDK.powerSupply.MeasuredCurrent` (port 852) → `0.0`
- Full Cell1-Cell6 sweep confirmed all 6 cells readable; **Cell7 raises `ADSError(1808)`** (symbol not found — Bulbasaur has 6 cells, not 7)

### MISTRAL Bulbasaur architecture

```
MistralGui.exe ─(HTTP JSON-RPC)─> Kestrel service ─(ADS)─> Beckhoff PLC
                                  10.0.42.231:9000          10.0.42.111.1.1
                                                            :851 (Main.*)
                                                            :852 (PIDProgram.*)
                                                                  ^
                                                                  |
                                        AIQM MistralAdsClient ────┘
                                        (direct pyads, bypassing gateway)
```

The Kestrel gateway is left in place for MistralGui.exe; AIQM bypasses it.
Both clients are safe to run simultaneously (Beckhoff PLCs are designed
for multi-client ADS access).

### MISTRAL variable schema per cell (`PLCTYPE_LREAL` unless noted)

Validated at the lab on Jul 27 2026 by direct pyads reads on Bulbasaur.
Applies verbatim to Ch-MBE per Task #191.

| Key | ADS variable |
|-----|--------------|
| `cell{i}_T` | `PIDProgram.Cell{i}_pidTDK.ActualTemperature` |
| `cell{i}_T_set` | `PIDProgram.Cell{i}_SetPoint` |
| `cell{i}_active_setpoint` | `PIDProgram.Cell{i}_pidTDK.ActiveSetPoint` |
| `cell{i}_V` | `PIDProgram.Cell{i}_pidTDK.powerSupply.MeasuredVoltage` |
| `cell{i}_I` | `PIDProgram.Cell{i}_pidTDK.powerSupply.MeasuredCurrent` |
| `cell{i}_prog_V` | `PIDProgram.Cell{i}_pidTDK.powerSupply.ProgrammedVoltage` |
| `cell{i}_prog_A` | `PIDProgram.Cell{i}_pidTDK.powerSupply.ProgrammedCurrent` |
| `cell{i}_power` | `PIDProgram.Cell{i}_pidTDK.OutputPower` (appears to be watts based on Jul 27 cross-check — V×I ≈ OutputPower for active cells) |
| `cell{i}_state` (INT) | `PIDProgram.Cell{i}_State` |
| `cell{i}_shutter_open` (BOOL) | `PIDProgram.Cell{i}_Shutter.StatusOpen` |
| `cell{i}_shutter_closed` (BOOL) | `PIDProgram.Cell{i}_Shutter.StatusClosed` |

Per-chamber PLC config lives in `drivers/config.py` (`MBESystemConfig.ads_*`):

| Chamber | `ads_netid` | `ads_cell_count` | `ads_display_confirmed` |
|---------|-------------|------------------|-------------------------|
| O-MBE (Bulbasaur) | `10.0.42.111.1.1` | 6 | `False` (see note) |
| Ch-MBE | `10.0.42.112.1.1` | 7 | `True` |

### Cell → physical material mapping (TENTATIVE — not encoded in code)

Cross-referencing ADS ActualTemperature + OutputPower with elog cell names:

| ADS Cell | Temp | Power | Speculated identity |
|----------|------|-------|--------------------|
| Cell 1 | 57.5 °C | 0 W | Substrate heater (matches elog `substrate_temp_pv_C` = 58.18) |
| Cell 2 | 22.5 °C | 0 W | Unused / spare |
| Cell 3 | 28.7 °C | 0 W | Sr or Er (matches elog ~28 °C idle) |
| Cell 4 | 150.0 °C | 1.9 W | Y or Eu (matches elog ~150 °C warm standby) |
| Cell 5 | 28.2 °C | 0 W | Sr or Er (other one) |
| Cell 6 | 150.0 °C | 9.0 W | Y or Eu (other one) |

This mapping is **not encoded in code**. `drivers/mistral_ads.py` reads by
Cell{N}; the ADS-to-material mapping is a display concern that requires
Jiangang's confirmation of physical wiring before we label widgets.

`ads_display_confirmed=False` on O-MBE prevents the existing
`_cell_displays` widgets (labeled "Sr", "Eu", etc.) from being overwritten
with ADS Cell{N} temperatures. Data still flows to CSV; only widget
display is gated.

### BASF Exactus pyrometer

**Status**: Silent — pinned for physical inspection.

Both `ModbusPyrometer` and `ExactusSerialPyrometer` failed:

- `ModbusPyrometer._read_f32` → empty register response (pymodbus returned no data)
- `ExactusSerialPyrometer._read_one_packet` → serial timeout waiting for the `0x81` header (no bytes at all in 2 s)

The two failures together indicate the pyrometer is not transmitting on
COM4 — either powered off, cable disconnected, IFD-5 converter unplugged,
or in a non-transmitting state after TemperaSure was closed. Opening
TemperaSure briefly did not restore streaming.

Ancillary improvement shipped this PR: `ModbusPyrometer._read_f32` /
`_read_u16` / `_read_ascii` now distinguish empty-response failures and
surface the recovery hint (ported from `scripts/pyrometer_modbus_smoke.py`)
instead of a bare "Modbus read error" message. Future silent-pyrometer
sessions will get an actionable error immediately.

## Follow-ups (out of scope for this PR)

- Dedicated MISTRAL PSU-state UI panel showing all `ads_cell_count` cells
  by number, decoupled from `_cell_displays`. Would enable O-MBE ADS data
  visualization without requiring the material-mapping confirmation.
- Once Jiangang confirms Cell{N} → material mapping on O-MBE, flip
  `OXIDE_MBE.ads_display_confirmed=True` and align `cell_display` labels
  with ADS Cell numbering.
- Log `cell{i}_state` and shutter status to CSV (currently UI-only; needs
  a semantic column-name decision).
- Task #187 — enable Vimba multicast on the camera via Vimba X Viewer so
  AIQM can run in `AccessMode.Read` alongside kSA in `AccessMode.Full`.
- Task #189 — LakeShore 335 SCPI direct-read on COM4.
- Task #190 — kSA SQL query interface for RHEED metrics on Ch-MBE.
- Task #195 — diagnose MISTRAL screengrab failure on Ch-MBE.
- Task #196 — physical debugging of the Bulbasaur BASF Exactus pyrometer.
- Task #200 — discover MISTRAL JSON-RPC method names on Bulbasaur
  (deprioritized after Path B; still useful for architectural consistency
  with the vendor-intended path).
- Bulbasaur cleanup: uninstall the redundant older Vimba SDK to reduce
  camera-discovery latency from ~20 s to ~2 s.

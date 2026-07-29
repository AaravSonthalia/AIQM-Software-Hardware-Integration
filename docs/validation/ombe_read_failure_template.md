# O-MBE Fault, Disconnect, and Recovery Validation

## Run Identity

- Date, operator, observer, owner/approver:
- Campaign, scenario, and episode IDs:
- Recorder/software commits and dirty state: see `run_info.json`
- Windows version, workstation, DPI, vendor software, driver, and firmware:
- Exact target and physical/logical boundary:
- Authorization and SOP references:
- Raw run/job directories and verified SHA-256 manifest:

## Safety Boundary

The probe records observations and operator markers. It sends no
state-changing command, does not close vendor software, disable a NIC/COM
device, operate a relay/PDU, change a setpoint, or switch power. Direct
Exactus, Modbus, and JSON-RPC modes do issue normal read requests. The local
network watcher uses `netsh` link state and sends no probe packet.

A marker records an approved manual action; it never authorizes or performs
one. Every S1-S4 run requires named roles, an approved safe state, an SOP,
written abort criteria, `--confirm-safe-state`, and a `precheck-complete`
marker. Stopping a file writer or closing/minimizing vendor software can affect
control and is not assumed safe merely because it is classified S1.

Never induce S2/S3 during growth, heating/ramping, autonomous control, or when
the target participates in control, interlocks, vacuum, pumping, plasma,
source output, shutters, or safety monitoring. Never deliberately power-cycle
the RHEED gun/HV, effusion cells, heaters, pumps, vacuum/plasma/PLC
controllers, shared racks/switches, or an active Bulbasaur workstation.

## Safety Tiers

| Tier | Fault class | Allowed execution |
|---|---|---|
| S0 | `offline-injection` | Unit-test harness only; live `run` rejects this label |
| S1 | `window-source-loss`, `file-feed-stop` | Approved manual software/display/feed action |
| S2 | `transport-loss` | Approved manual interruption of one dedicated telemetry link |
| S3 | `instrument-power-loss` | Vendor-approved non-control-critical device/adapter only |
| S4 | `workstation-power-observation` | Tabletop or planned maintenance with independent evidence |

An ineligible scenario is recorded as **not tested**, not forced into the
matrix. S4 normally uses a second computer, UPS, or Windows event evidence; do
not intentionally hard-cut a workstation that controls equipment.

## Preflight

- [ ] No growth, ramp, PID/autonomous action, or sample at risk.
- [ ] Outputs are safe and shutters are closed where applicable.
- [ ] Pressure, vacuum, cooling, and interlocks are confirmed safe.
- [ ] The exact cable/device/window/feed boundary is identified.
- [ ] The owner confirms it is not a control or safety path.
- [ ] Operator, observer, approver, authorization, and SOP are recorded.
- [ ] Protocol, COM/baud/device ID, NIC index, and hardware identity are known.
- [ ] Abort criteria and vendor recovery procedure are immediately available.
- [ ] Growth Monitor is advisory/disarmed and cannot auto-resume.

The CLI fails closed when required metadata is missing. For serial/USB tests,
the `failure-start` marker also requires a recent watchdog sample with the
exact COM configuration and a device HWID/serial number. Network monitoring
records development evidence only; S2 Ethernet is disabled until the endpoint
and driver are bound. Every S1-S4 `failure-start` also requires the
recorder's kernel liveness lease to be active, the latest State and watchdog
samples to be fresh and on the current boot, and a healthy trailing baseline
for every relevant channel. A completed `summary.json` blocks a new failure
start.

## Durable Recorder

Use new raw and job directories. Growth Monitor remains interactive and is not
launched by the job manager.

```powershell
$manager = "$env:USERPROFILE\.codex\skills\managed-long-jobs\scripts\managed_job.cmd"
$python = "C:\Users\Yao_Yufan\.conda\envs\ai4mbe-gui\python.exe"
$checkout = "D:\path\to\failure-validation-checkout"
$run = "D:\O-MBE-validation\failure\serial_link_YYYYMMDD_HHMMSS"
$job = "D:\O-MBE-validation\jobs\serial_link_YYYYMMDD_HHMMSS"

& $manager launch --output-dir $job --cwd $checkout -- $python `
  scripts\ombe_failure_probe.py run `
  --source pyrometer --mode exactus --port COM4 --baudrate 115200 `
  --fault-class transport-loss --link-kind usb `
  --campaign-id OMBE-FAULT-001 --scenario-id exactus-link-loss-01 `
  --target "Exactus read-only pyrometer interface" `
  --boundary "dedicated PC-side USB/RS-422 adapter for COM4" `
  --operator "<name>" --observer "<name>" --approver "<owner>" `
  --authorization-ref "<maintenance record>" `
  --sop-ref "<disconnect/recovery SOP>" `
  --safe-state-note "<verified idle/advisory state>" `
  --abort-criteria "<alarms, interlock, output, wrong cable>" `
  --confirm-safe-state --duration-s 600 --baseline-duration-s 30 `
  --output-dir $run
```

This only starts observation. Exactus and Modbus share a per-port Windows
mutex. Close TemperaSure and do not run another serial reader. Non-default
Modbus COM/baud/device-ID arguments are rejected before a port is opened
because the b24ceb1 worker does not forward them. If a worker cannot stop, its
mutex remains held until the recorder process exits.

Poll the durable job at least every 30 seconds:

```powershell
& $manager status --output-dir $job
```

## Manual Marker Sequence

After starting the recorder, wait at least `--baseline-duration-s` and confirm
that the latest State and watchdog records are valid before any physical or
software fault action. The CLI rejects a baseline that is short, stale, on an
old boot, or unhealthy. Use one episode ID and one fault episode per run:

```powershell
& $python "$checkout\scripts\ombe_failure_probe.py" mark `
  --output-dir $run --kind precheck-complete `
  --episode-id exactus-link-01 --label precheck

& $python "$checkout\scripts\ombe_failure_probe.py" mark `
  --output-dir $run --kind failure-start `
  --episode-id exactus-link-01 --label link-removal-start

# The authorized operator performs the SOP action; the script does nothing.

& $python "$checkout\scripts\ombe_failure_probe.py" mark `
  --output-dir $run --kind failure-observed `
  --episode-id exactus-link-01 --label expected-failure-visible

& $python "$checkout\scripts\ombe_failure_probe.py" mark `
  --output-dir $run --kind recovery `
  --episode-id exactus-link-01 --label link-restored

& $python "$checkout\scripts\ombe_failure_probe.py" mark `
  --output-dir $run --kind vendor-ready `
  --episode-id exactus-link-01 --label vendor-ready

& $python "$checkout\scripts\ombe_failure_probe.py" mark `
  --output-dir $run --kind gui-reconnected `
  --episode-id exactus-link-01 --label gui-reconnected

& $python "$checkout\scripts\ombe_failure_probe.py" mark `
  --output-dir $run --kind rearmed `
  --episode-id exactus-link-01 --label explicit-rearm
```

The CLI enforces precheck, terminal-marker, and recovery-action order. If
recovery cannot be completed, terminate the episode with `abort`,
`persistent-failure`, or `incomplete`; these outcomes remain non-successful.
No fault episode, an unmatched start, or a missing re-ARM also returns nonzero.
Exactly one `failure-start` is allowed in each run directory. `rearmed` is an
operator/observer assertion: the probe does not inspect the Growth Monitor ARM
state, so verify it visually or with independent evidence.

## Network Boundary

List local IPv4 interface indices without contacting an instrument:

```powershell
netsh interface ipv4 show interfaces
```

The passive development watcher accepts `--network-interface-index <index>`
and optionally `--network-interface "<exact name>"`. It samples local adapter
state at `--network-probe-interval` (default 2 s) without opening a TCP
connection. The live command currently rejects S2 Ethernet/network loss
because no GUI driver is bound to a declared remote endpoint. Local NIC-down
is fault evidence only; it cannot prove that the selected instrument path
failed.

## Currently Enforceable Disconnect Matrix

| Scenario | Recorder support |
|---|---|
| Exactus/Modbus dedicated COM/USB link loss | S2, with active-driver COM settings and HWID/serial identity |
| Telemetry-only Ethernet adapter loss | **Not evaluable** until NIC, remote endpoint, and active driver are bound |
| Camera/MISTRAL/EvapControl USB cable loss | Not yet evaluable: add a source-bound PnP identity probe first |
| Window, UIA, WGC, or Elog source loss | S1 software/source-path test only |
| Workstation/site outage | S4 observation/tabletop with independent recorder |

Do not substitute the presence of an unrelated COM port or NIC for the target
interface. Unsupported cases are recorded as **not tested**.

## Source-Specific Interpretation

| Source | Candidate boundary | Recovery evidence and limitation |
|---|---|---|
| WGC/kSA | Vendor window; hardware link requires a future identity probe | WGC sequence proves capture delivery, not a new exposure; hardware recovery needs a vendor exposure counter or independent evidence |
| MISTRAL | Window only; Ethernet awaits endpoint/driver binding | JSON-RPC all-`None` is not evaluable; OCR confirms only the display path |
| EvapControl | Elog/display path; controller links are normally ineligible | Recovery requires advancing Elog source timestamps; mtime-only changes do not count |
| TemperaSure | Window/UIA path; COM loss requires a future identity probe | UI/OCR can retain stale display values and cannot prove hardware recovery |
| Exactus/Modbus | Dedicated USB/RS-422 link | Consecutive valid direct-read cycles confirm the read path, not instrument safe state |

The camera fingerprint is a sampled freeze candidate only: a static surface
may genuinely look unchanged, and a vendor window may repaint its last image.
For Elog, `watchdog.jsonl` records path, size, mtime, last-record source time,
and tail hash, but only source-time advancement confirms new records.

## Instrument Power Loss

Use `--fault-class instrument-power-loss` only for an exact, approved power
boundary. Never use a generic "instrument power" target. The authorization
must name the device/adapter, explain why it is not control- or safety-critical,
and cite the vendor shutdown/recovery procedure. The script performs no power
operation.

The current executable S3 path is deliberately limited to an approved
Exactus/Modbus pyrometer device or adapter whose COM endpoint is bound to the
active direct-read driver. Camera, MISTRAL, EvapControl, controller, rack, and
shared-supply power tests remain **not tested** until independent power and
device-identity evidence exists.

S3 also requires `--independent-observer-ref`, distinct operator and observer
identities, and an `independent-observer` `failure-start` marker. This records
an observer-asserted power event; it is not a power measurement. A result
describes read-path response and recovery only.

After restoration, verify vendor-ready state, protocol/mode, identity, and
three source-appropriate fresh samples. Mark `gui-reconnected` and `rearmed`;
never automatically resume RUNNING or control.

## Workstation or Process Interruption

The same-process watchdog runs in a dedicated Python thread, independent of
the worker and Qt event loop. It cannot observe a process/GIL/filesystem hang,
workstation outage, or site-power loss. S4 therefore requires second-machine,
UPS, or event-log evidence.

After reboot or abrupt process death, salvage complete fsynced JSONL records:

```powershell
& $python "$checkout\scripts\ombe_failure_probe.py" finalize `
  --output-dir $run
```

`finalize` records the Windows boot ID, reports truncated rows, preserves prior
lifecycle failures, recomputes marker/evidence errors, rejects cross-boot
ordering that could reuse old-boot rows, and rebuilds the SHA-256 manifest.
Post-run markers also refresh the summary automatically. If the watchdog
thread did not terminate, the live process writes no manifest because evidence
may still change. After that recorder process exits, run `finalize --force`
because `summary.json` already exists. For a process/PC interruption with no
summary, use `finalize` without `--force`; the run remains lifecycle-fatal and
cannot be promoted to a pass. Start a separate recovery run after a workstation
outage; do not claim same-process outage coverage.

## Evidence and Acceptance

`states.jsonl` contains State emissions, validity, data age, value changes, and
camera fingerprints. `watchdog.jsonl` contains emission age, worker/driver
identity, passive COM enumeration, local NIC state, and Elog source freshness.

A channel is evaluable only when its own healthy **trailing** baseline spans
the requested duration. The fault window must last at least
`max(2 x healthy p95 cadence, 3 s) + one relevant sample`; a shorter restore is
not evaluable. Link loss, Elog freeze, and recorder emission timeout are
external fault evidence, not proof that the production GUI failed closed.
Only an invalid/error/disconnected target State (or a future production
freshness guard) counts as detection, and it must occur by the stale threshold.
Any State still marked valid after that deadline is a validation failure even
if its displayed value changes. Record:

- invalid/error/disconnected/all-None, emission-timeout, local-link, and
  source-stale detection latencies;
- retained old values and whether logging/classification/images continued;
- first emission, valid value, value/frame/link change after restoration;
- source-appropriate three-sample recovery evidence and its limitation;
- device/interface identity and vendor/GUI/re-ARM actions;
- any automatic resumption of logging or control.

Exit code 0 means a complete passing episode, 2 means recorder/evidence fatal
error, 3 means incomplete or not evaluable, and 4 means a completed validation
that exposed no detection or retained stale-valid data.

## Results

| Tier/source/mode | Target/link | Baseline | Detection channel/latency | Stale retained | Recovery evidence/latency | Re-ARM | Result |
|---|---|---|---|---|---|---|---|
| S1 RHEED WGC | | | | | | | |
| S1 MISTRAL OCR | | | | | | | |
| S1 Evap OCR/Elog | | | | | | | |
| S1 TemperaSure UIA | | | | | | | |
| S2 approved serial/USB | | | | | | | |
| S2 Ethernet - not evaluable | | | | | | | |
| S3 approved device/adapter | | | | | | | |
| S4 observation/tabletop | | | | | | | |

## Findings

- Confirmed behavior:
- Not evaluable or ineligible scenarios:
- Safety/abort observations:
- Required production hardening:
- Additional approved tests:

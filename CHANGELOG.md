# Changelog

## [0.2.2] - 2026-09-07

### Fix: SignalK output silently freezing after enabling raw magnetic field

Found on real hardware: shortly after enabling the raw-magnetic-field
SignalK output, *every* SignalK output from this device (heading, rate
of turn, attitude, magnetic field) stopped updating — permanently, and
survived a reboot. N2K kept working throughout, since it's a
completely separate code path (CAN/TWAI, no WiFi dependency).

Root cause, found via the firmware's own `/api/log` endpoint: SensESP's
`SKWSClient` bundles all pending SignalK path updates into one
WebSocket message per flush cycle, and silently drops the *entire*
bundle (not a partial/truncated one) if it exceeds
`SENSESP_SK_WS_BUFFER_SIZE` — 1024 bytes by default. This firmware's
full path count, all enabled, needs 1213 bytes once magnetic field
output is on, so every flush was dropped, forever, with no visible
symptom beyond a rate-limited warning in the ESP-IDF log:

    W signalk_ws_client.cpp: Delta too large (1213 B > 1024 buffer);
    dropped to keep the connection alive -- raise SENSESP_SK_WS_BUFFER_SIZE

Fixed by raising `SENSESP_SK_WS_BUFFER_SIZE` to 4096 via a
`platformio.ini` build flag — no code change. See SPEC.md §11,
ARCHITECTURE.md §2.7.

## [0.2.1] - 2026-09-07

### Decoded values in the web UI serial log

The web UI's Serial Log panel now shows each frame's decoded
interpretation alongside its hex dump — e.g. `55 53 ... -- heading=
90.00 roll=0.00 pitch=-45.00 (deg)` for an Angle packet, `mag x=1234
y=-567 z=89` for a Magnetic packet, or `invalid frame (bad
header/checksum/type)` for one that doesn't parse. Implemented as
`DescribeHWT901BFrame()` (`hwt901b_parser.h/.cpp`), a pure, unit-tested
function reusing the same `ParseHWT901BFrame()` the rest of the
firmware relies on. Unlike the `HALSER_DEBUG_SERIAL`-gated console
logging added in v0.2.0-debug, this is always available — it's a cheap
render step on an existing panel, not continuous verbose logging — and
in particular makes it possible to check straight from the web UI
whether the module is actually streaming `0x54` Magnetic packets
continuously, which the N2K PGN outputs alone can't tell you (SPEC.md
§8.1).

## [0.2.0] - 2026-09-07

### Verbose serial debug logging, disabled by default

Confirmed working against real WT901B hardware during the
`v0.2.0-debug` pre-release bring-up. For this release the logging is
gated behind a new `HALSER_DEBUG_SERIAL` compile-time flag
(`platformio.ini`, commented out by default) rather than shipping
enabled — it's a hardware bring-up/debugging tool, not something a
normal install needs running continuously. Uncomment the flag to bring
it back for a future debugging session; nothing else changes.

The always-on per-frame hex dump (one line per received frame,
present since v0.1.0) is unaffected and still logs unconditionally.

See `[0.2.0-debug]` below for what the logging itself covers (setup
steps, every TX write, every RX frame decoded).

## [0.2.0-debug] - 2026-09-07

### Verbose serial debug logging (pre-release)

Not a functional/protocol change — adds debug visibility for hardware
bring-up against a real WT901B, ahead of ever having one to test
against (SPEC.md §11 lists everything about the protocol layer that
still needs real-hardware verification; this release is a tool for
doing that verification, not a claim that it's done).

- `HWT901BSerialIO::Begin()`, `DetectBaud()`, and `SetBaudRate()` now
  log each serial setup step at `ESP_LOGI` (opening the port, each
  autobaud candidate tried and its outcome, the settle delay/reopen
  during a runtime baud switch).
- Every byte this firmware writes to the WT901B (calibration CALSW/SAVE
  writes, BANDWIDTH/RRATE/BAUD register writes) is now logged as a hex
  dump plus a human-readable description before it's sent.
- Every complete 11-byte frame read from the WT901B is logged as a hex
  dump (as before), plus — new — its decoded interpretation once
  successfully parsed (heading/roll/pitch for `0x53`, `gyro_z` for
  `0x52`, raw magnetic counts for `0x54`), and a distinct log line when
  a frame is dropped for a bad header/checksum/type.
- `hwt901b_serial.cpp` now forces `LOG_LOCAL_LEVEL` to `ESP_LOG_DEBUG`
  before including `esp_log.h`, so this logging can't be silently
  compiled out by a future change to `CONFIG_LOG_MAXIMUM_LEVEL` — only
  `gateway.cpp`'s existing runtime `SetupLogging(ESP_LOG_DEBUG)` call
  was in place before, which doesn't protect against compile-time
  stripping.

## [0.1.0] - 2026-09-07

### Fork from halser-hwt3100-interface

Forked from `BoatHacks/halser-hwt3100-interface` (WitMotion HWT3100-TTL,
compass-only) and ported to the WitMotion WT901B, a full 9-axis IMU. This
is a real protocol and data-model port, not a rename — see SPEC.md §1.2
and §10 for the full rationale, and §11 for what remains unverified
without physical WT901B hardware in this environment.

Changed relative to the parent project:

- Sensor wire protocol: the HWT3100's line-based ASCII protocol
  (`Magx:...,Yaw:...\r\n`, `AT+`-prefixed text commands) replaced with
  the WT901B's binary protocol (11-byte `0x55`-framed packets with a
  checksum; 5-byte register writes).
- Data model: `HeadingReading` → `ImuReading`, adding `roll`, `pitch`,
  and `gyro_z` — fields the HWT3100 could never populate.
- Rate of turn is now sensed directly from the WT901B's gyroscope,
  replacing the parent project's sliding-window regression estimator
  (`RateOfTurnEstimator`, deleted along with its config and tests — no
  longer needed now that a real gyroscope exists).
- PGN 127257 (Attitude) and `navigation.attitude` newly implemented,
  using real roll/pitch — the parent project explicitly did not
  implement this PGN because the HWT3100 had no accelerometer.
- Calibration commands: two actions (start/stop magnetic calibration),
  not three — no confidently-sourced WT901B register value exists for a
  "clear calibration" action equivalent to the parent's `AT+CALI=2`.
- No calibration-reply status item — the WT901B's register protocol has
  no acknowledgment packet, unlike the HWT3100's plain-text `AT+CALI`
  replies.
- On-module output smoothing (`AT+FILT`), output rate (`AT+PRATE`), and
  UART baud (`AT+UART`) replaced by their WT901B register equivalents
  (BANDWIDTH, RRATE, BAUD), same persisted-config/snap-to-nearest design.
- Config paths renamed `/hwt3100/*` → `/hwt901b/*`; SignalK diagnostic
  paths renamed `sensors.hwt3100.*` → `sensors.hwt901b.*`.

Carried over unchanged: N2K/SignalK plumbing (`ExpiringValue`, per-PGN/
per-delta toggles, `meta.timeout` staleness), the B&G Precision-9 device
identity (now more fully honest, since PGN 127257 is real data), the MFD
calibration bridge (PGN 130850/130851), the magnetic variation listener,
the SensESP config-REST-API-based serial terminal, and the overall
HALSER board wiring/pin assignments.

See the parent project's own CHANGELOG.md for its pre-fork history,
which no longer applies to this codebase.

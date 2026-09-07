# Changelog

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

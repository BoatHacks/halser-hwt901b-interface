# Changelog

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

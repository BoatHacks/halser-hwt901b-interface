# HALSER-HWT901B-interface Specification

## 1. Introduction

### 1.1 Purpose

HALSER-HWT901B-interface is ESP32-C3 firmware that reads orientation
data from a WitMotion WT901B 9-axis IMU module over a serial (UART)
link, and makes that data available to a boat's instrument network in
two independent ways:

- as **NMEA 2000 (N2K)** messages on the vessel's CAN bus, for
  chartplotters and other N2K instruments, and
- as **SignalK deltas**, sent over WiFi to a SignalK server.

Either output can be enabled or disabled independently.

On the N2K side, the firmware presents itself as a **B&G Precision-9**
compass (device-identity fields, PGN set) rather than as a
generic/custom device — see §1.2, §5.1, §10.

This project is a fork of `BoatHacks/halser-hwt3100-interface`, adapted
for a different WitMotion module. The parent project targeted the
**HWT3100-TTL**, a compass-only module (heading + raw magnetic field,
line-based ASCII protocol). This project's target, the **WT901B**, is a
full 9-axis IMU (accelerometer + gyroscope + magnetometer, fused
roll/pitch/heading) speaking a binary register protocol. Most of this
firmware's architecture, N2K/SignalK plumbing, and safety philosophy
(§2, §8) is carried over unchanged; the sensor protocol layer and data
model are a real port, not a rename — see §1.2, §3, and the Design
Decisions in §10 for what changed and why.

### 1.2 Background

**I do not have physical WT901B hardware in this environment.** Every
protocol detail below (frame format, register addresses, scale factors)
comes from WitMotion's publicly documented register protocol — shared
across their JY901/WT901/HWT901B product family and mirrored in the
vendor's own `wit_c_sdk` and multiple open-source drivers — not from
testing a real unit. The parent HWT3100 project's own SPEC.md records an
incident where a manual-derived assumption about that module's line
format was wrong until checked against real hardware; treat every
protocol detail here with the same suspicion until someone verifies it
against a physical WT901B. Everything not independently confirmable is
listed in §11, not silently assumed.

**Wire format.** The module streams continuous 11-byte binary packets:

```
0x55 <type> <8 data bytes> <checksum>
```

`checksum` is the low byte of the sum of the first 10 bytes. The `type`
byte selects what the 8 data bytes mean; this firmware consumes four
types:

| Type   | Contents                              | Used for |
|--------|----------------------------------------|----------|
| `0x51` | Acceleration (ax, ay, az, temp)        | Recognized, not consumed (§3) |
| `0x52` | Angular velocity / gyro (wx, wy, wz, temp) | `gyro_z` → rate of turn |
| `0x53` | Angle (roll, pitch, yaw, version)      | `roll`, `pitch`, `heading` |
| `0x54` | Magnetic field (hx, hy, hz, temp)      | Raw diagnostic field |

Each value is a little-endian `int16`. Angle values scale by
`180/32768` (degrees); gyro values scale by `2000/32768` (degrees/sec);
magnetic field values are raw counts, same diagnostic-only treatment the
HWT3100 fork gave its raw magnetic field (no documented counts-to-µT
conversion).

**Command protocol.** The module accepts 5-byte register writes:
`0xFF 0xAA <register> <dataL> <dataH>`. Relevant registers:

- `0x01 CALSW` — calibration mode. `7` starts magnetic-field
  calibration, `0` stops it (per `wit_c_sdk`'s
  `WitStartMagCali()`/`WitStopMagCali()`). These are the *only* two
  values this firmware ever writes — see the domain rule in §2.
- `0x03 RRATE` — output data rate, an enumerated code table (0.2 Hz
  through 200 Hz). The table also documents a "no output" code, which
  would silence the continuous stream this firmware's read pipeline
  depends on — like `AT+MODE` in the parent project, this code is
  simply never a value this firmware's formatter can produce (§2, §9.3).
- `0x04 BAUD` — UART baud, one of 4800/9600/19200/38400/57600/115200/
  230400.
- `0x1F BANDWIDTH` — accel/gyro output low-pass filter bandwidth, one of
  256/184/94/44/21/10/5 Hz.
- `0x00 SAVE` — persists current settings. This firmware only ever
  writes the "save current settings" value; the register also
  documents a "restore factory defaults" value, which would silently
  wipe calibration/config — **never implemented, anywhere, for any
  reason**, the same permanent-exclusion treatment §1.2 of the parent
  project gave `AT+MODE` (see §2, §9.3).

There is no plain-text acknowledgment for any of these writes (unlike
the HWT3100's `AT+CALI` replies) — the module doesn't echo back a
confirmation string on this wire. See §8.2, §10.

> **⚠️ Model scope.** This firmware targets the WT901B's TTL/UART
> variant. It has not been checked against other WT901-family variants
> (I2C, Bluetooth/BLE, RS485) — those use different physical-layer
> signaling or an entirely different host interface that HALSER's UART
> terminal block doesn't support.

All N2K PGNs applicable to the data this firmware produces (heading,
rate of turn, attitude — see §3) are implemented and individually
selectable in the config UI — see §5.1 and §7.

**B&G Precision-9 emulation.** Carried over unchanged from the parent
project: this firmware identifies itself on the N2K bus as a B&G
Precision-9, cloning device-identity fields from the reference
implementation `htool/ESP32_Precision-9_compass_CMPS14` (see §1.2 of the
parent SPEC and §10 here for exactly which fields). Unlike the parent
project, this identity emulation is now more fully honest: the
Precision-9 reference sends PGN 127257 (Attitude) using real heel/trim
from its own accelerometer-equipped sensor, and the WT901B — unlike the
HWT3100 — actually has one, so this firmware implements that PGN for
real (§5.1, §9.3, §10).

### 1.3 Terminology

- **Heading** — magnetic heading angle, from the WT901B's angle packet
  (`0x53`)'s yaw field.
- **Roll / pitch** — vessel attitude angles, from the same angle packet.
  New in this fork; the HWT3100 this project is adapted from had no
  accelerometer and could not produce these.
- **Rate of turn** — the rate the vessel's heading is changing,
  radians/second, positive = starboard. Unlike the HWT3100 fork (which
  computed this via a sliding-window regression over heading history,
  since that module had no gyroscope), this firmware sources it directly
  from the WT901B's real gyroscope (`gyro_z`, the `0x52` packet's yaw-
  axis reading) — see §10 for why the estimator was removed rather than
  kept as a fallback.
- **Magnetic field reading** — the raw 3-axis magnetic field vector,
  diagnostic-only, same treatment as the parent project.
- **WT901B** — shorthand for the WitMotion WT901B TTL/UART module, this
  firmware's sole sensor input.
- **Calibration offset** — a user-configurable correction applied to raw
  WT901B heading readings, for mounting misalignment. Distinct from
  on-module magnetic calibration (below).
- **On-module (magnetic) calibration** — the WT901B's own
  magnetic-field calibration procedure, triggered via CALSW register
  writes (§8.2).
- **Stale** — sensor data that has aged past its expected update
  interval without a fresh reading.

## 2. Domain Rules

- **The firmware must never write the SAVE register's "restore factory
  defaults" value, and must never write any CALSW value other than `0`
  or `7`.** These are the WT901B-protocol equivalents of the parent
  project's `AT+MODE` exclusion (§1.2) — not an allowlist filter applied
  to an otherwise-general write path, but commands this codebase simply
  never implements anywhere.
- Heading is referenced to magnetic north as read by the WT901B (no
  true-heading correction on-module; magnetic variation correction, if
  needed, happens via the bus-sourced mechanism in §5.1a).
- Heading is relative to the module's mounted orientation, adjusted by
  the configured calibration offset (§1.3) before transmission. Roll and
  pitch are not offset-corrected (§9 — same scope decision the parent
  project made for heading-only calibration).
- The calibration offset is applied in firmware, once, before data is
  handed to either output path — both outputs must always see the same,
  already-corrected value (carried over from the parent project's §2).

## 3. Data Model

### ImuReading (MVP)

| Field     | Type  | Notes |
|-----------|-------|-------|
| heading   | float | Magnetic heading, degrees, 0–360, offset-corrected — internal representation throughout the pipeline, matching the WT901B's own angle-packet scale. N2K/SignalK both require radians; converted once, at each output boundary. |
| roll      | float | Degrees, -180..180. New in this fork (§1.3, §9.3). |
| pitch     | float | Degrees, -90..90. New in this fork. |
| gyro\_z   | float | Degrees/second, raw yaw-axis angular rate — feeds rate of turn (§1.3, §5.1). New in this fork; replaces the parent project's computed sliding-window estimate. |
| magX, magY, magZ | int | Raw magnetic field components, diagnostic use only (§5, §10) |
| timestamp | uint  | Millis of last packet contributing to this reading (for staleness) |

Acceleration (the `0x51` packet) is recognized by the parser as a valid
frame but not captured into `ImuReading` — nothing in this firmware's
output set consumes raw acceleration directly (roll/pitch already come
pre-fused from the module's own `0x53` angle packet).

### CalibrationOffset (config, persisted)

| Field         | Type  | Default |
|---------------|-------|---------|
| headingOffset | float | 0.0     |

Same shape as the parent project's — heading-only, no roll/pitch
offset (§9).

### Rate of turn

No longer a derived/computed field (contrast with the parent project's
§3, which built a `RateOfTurnEstimator` sliding-window regression
because the HWT3100 had no gyroscope). This firmware's rate of turn is
`ImuReading.gyro_z`, sign-corrected and unit-converted at the output
boundary (§5.1, §5.2, §10) — a direct sensor reading, not a derived
estimate.

## 4. Sources / Inputs

- **Single source**: the WT901B module, connected via UART to the
  ESP32-C3, GPIO2 TX / GPIO3 RX (HALSER's UART terminal block, RX-select
  jumper on "U" — unchanged from the parent project's confirmed wiring).
  Baud auto-detected at boot among the module's seven supported rates
  (§8.2c). **Power**: the module's GND and VCC leads connect to the N2K
  bus's own GND and 12V — carried over from the parent project's
  confirmed hardware wiring; **not independently re-verified for the
  WT901B specifically** (§11).
- There is no secondary/fallback sensor. If the WT901B stops sending
  data, the firmware marks the data stale (§6) rather than substituting
  another source.

## 5. API Specification

### 5.1 N2K Output

Presents as a B&G Precision-9 compass (§1.2). Three PGNs implemented:

- **PGN 127250 — Vessel Heading**: real data, radians (converted from
  the firmware's internal degrees at this boundary). Deviation is always
  "not available"; Variation is populated from the bus when available
  (§5.1a).
- **PGN 127251 — Rate of Turn**: real data, from the WT901B's
  gyroscope (§1.3, §3) — unlike the parent project, this is sensed, not
  computed.
- **PGN 127257 — Attitude**: real data, roll and pitch from the
  WT901B's accelerometer. **Newly implemented in this fork** — the
  parent project's SPEC explicitly did *not* implement this PGN because
  the HWT3100 had no accelerometer (§9.3 of the parent project); the
  WT901B does, so emulating the Precision-9 identity can now be honest
  about this PGN too (§1.2, §10). Yaw is always "not available" — PGN
  127250 already carries heading, and duplicating it into Attitude's Yaw
  field isn't verified against how a real Precision-9 behaves (§11).

All three PGNs are independently selectable in the config UI on top of
the master N2K enable/disable switch (§7), and all three are declared to
`ExtendTransmitMessages()` so PGN 126464 ("PGN List — Transmit") reports
them correctly (same requirement/rationale as the parent project's
ARCHITECTURE.md §2.4).

### 5.1a Magnetic Variation (Bus-Sourced) and True Heading

Unchanged from the parent project (§5.1a there): this firmware has no
GPS or geomagnetic model, so it listens for PGN 127258 (Magnetic
Variation) from another N2K device and uses it to fill PGN 127250's
Variation field and compute `navigation.headingTrue`. Deliberately
passive/read-only — never transmits PGN 127258 itself.

### 5.2 SignalK Output

- `navigation.headingMagnetic` (radians) — same as parent project.
- `navigation.rateOfTurn` (rad/s, positive = starboard) — real gyro
  data now, not computed (§3, §5.1).
- `navigation.headingTrue` (radians) — same mechanism as parent (§5.1a).
- `navigation.attitude` — **new in this fork**: `{roll, pitch, yaw}`
  object, radians, yaw always `0`. SensESP has no dedicated
  attitude-output class (unlike its numeric `SKOutputFloat`), so this is
  published via `SKOutputRawJson` — see ARCHITECTURE.md §2.7.
- Raw magnetic field: `sensors.hwt901b.magneticField.x/y/z`, same
  diagnostic-only treatment as the parent project's
  `sensors.hwt3100.magneticField.*` (raw counts, no unit, off by
  default).

Staleness surfaced via `meta.timeout`, same mechanism as the parent
project (§6).

## 6. Fault Handling / Persistence

Unchanged from the parent project: N2K sends "not available" values on
stale data (`ExpiringValue`); SignalK uses `meta.timeout` (5 seconds),
not an active notification. No dedicated fault LED — SensESP's own
connection-status LED keeps sole ownership of GPIO8 (parent project's
§10 Design Decisions, carried over unchanged — this is an ESP32-C3/
HALSER-board concern, not a sensor-specific one).

**Persistence**: heading calibration offset and WiFi/N2K/SignalK
enable-disable configuration survive a restart. Live sensor readings are
ephemeral.

## 7. Configuration

User-tunable, exposed via the SensESP web UI:

- Enable/disable N2K output (master), plus PGN 127250/127251/127257
  independently.
- Enable/disable SignalK output (master), plus
  `navigation.headingMagnetic`, `navigation.rateOfTurn`,
  `navigation.headingTrue`, `navigation.attitude`, and the raw magnetic
  field deltas independently.
- Calibration offset: heading.
- On-module output bandwidth (BANDWIDTH register, §8.2a) and output
  rate (RRATE register, §8.2b), and UART baud rate (BAUD register,
  §8.2c) — same persisted-config, apply-at-boot-and-on-change pattern as
  the parent project's `AT+FILT`/`AT+PRATE`/`AT+UART`.
- WiFi credentials / SignalK server connection (standard SensESP
  config).

Unlike the parent project, there is no rate-of-turn window/min-span
config — that concept doesn't exist here; rate of turn is a direct
sensor reading (§3, §10).

## 8. User Interface

### 8.1 Serial Terminal (Monitor)

Same mechanism as the parent project (SensESP's config REST API, §8.1
there) — the last 30 raw frames received from the WT901B. Unlike the
HWT3100's plain-ASCII lines, WT901B frames are binary, so each entry is
rendered as a hex byte string rather than raw text, followed by a
decoded interpretation of that frame's contents (e.g. `heading=90.00
roll=0.00 pitch=-45.00 (deg)` for an Angle packet, `gyro_z=12.50 deg/s`
for a Gyro packet, `mag x=1234 y=-567 z=89` for a Magnetic packet, or
`invalid frame (bad header/checksum/type)` for one that doesn't parse)
— so wiring/protocol issues are visible straight from the web UI,
without also needing a serial console or the `HALSER_DEBUG_SERIAL`
build flag (ARCHITECTURE.md §2.1).

### 8.2 In-Place Calibration Commands

Two named actions (not three, unlike the parent project — see §10):
"Start Magnetic Calibration" (writes CALSW=7) and "Stop Magnetic
Calibration" (writes CALSW=0, then SAVE=save-current). Same `UIButton`
mechanism as the parent project (still depends on `BoatHacks/SensESP`
pending the upstream fix — see `platformio.ini`).

**No calibration-reply status item exists** — unlike the parent
project's §8.2, which surfaced the HWT3100's plain-text `AT+CALI`
replies on a Status page item, the WT901B's register protocol has no
acknowledgment packet to correlate with a click. Dropped, not ported
(§10).

The full set of relevant register writes:

| Write | Effect |
|---|---|
| CALSW=7 | Start magnetic-field calibration (rotate module 2-3 full turns) |
| CALSW=0 | Stop calibration |
| SAVE=save-current | Persist the just-completed calibration |

(SAVE's "restore factory defaults" value is permanently excluded — §1.2,
§2, §9.3. Whether CALSW=0 alone already persists the calibration, making
the explicit SAVE write redundant, is unverified — §11.)

### 8.2a Output Bandwidth (BANDWIDTH register)

Per WitMotion's register protocol: one of 256/184/94/44/21/10/5 Hz,
snapped to the nearest requested value — the WT901B analog of the parent
project's `AT+FILT`. Smaller = more smoothing, more lag.

### 8.2b Output Rate (RRATE register)

Per WitMotion's register protocol: one of eleven supported rates from
0.2 Hz to 200 Hz, expressed in this firmware as centihertz so 0.2 Hz
stays representable as an integer, snapped to the nearest requested
value. **Recommended minimum: 1000 centihertz (10 Hz)**, same rationale
as the parent project's `AT+PRATE` guidance.

> **⚠️ The RRATE register's documented "no output" code silences the
> module's continuous stream this firmware's read pipeline depends on.**
> Unlike `AT+PRATE=0` in the parent project (which this firmware's
> formatter could still produce, with a warning), this fork's
> `FormatRateCommand()` **cannot produce that code at all** — the lowest
> reachable value is 0.2 Hz, not "off." A stricter version of the same
> safety posture, made possible because (unlike `AT+PRATE`) this
> firmware never needs to query the module's current rate back (no
> reply mechanism exists to read one from, §8.2), so there was no reason
> to keep the dangerous value reachable even as a documented-but-
> discouraged option.

### 8.2c Baud Rate: Auto-Detection and Runtime Switching (BAUD register)

Same auto-detect-at-boot-then-snap-to-nearest-on-change pattern as the
parent project's `AT+UART`, now over seven supported rates
(4800/9600/19200/38400/57600/115200/230400) instead of three. No
register write happens during detection — passive listening only.

### 8.3 MFD-Triggered Calibration (N2K)

Unchanged from the parent project (§8.3 there): PGN 130850/130851,
reverse-engineered proprietary Navico/Simnet protocol, unverified
against real hardware, dispatching through the same
`CalibrationCommandHandler` the web UI uses. No MFD-triggered "clear"
path (never existed even in the parent project's implementation).

## 9. MVP Scope

### 9.1 MVP Features

- Read heading, roll, pitch, rate of turn (real gyro), and raw magnetic
  field from the WT901B over serial.
- Apply a configurable heading calibration offset.
- Transmit heading (PGN 127250), rate of turn (PGN 127251), and attitude
  (PGN 127257) — all independently toggleable, plus a master N2K
  enable/disable switch.
- Present as a B&G Precision-9 on the N2K bus.
- Transmit heading, rate of turn, true heading, and attitude via
  SignalK deltas, independently toggleable.
- Detect stale sensor data and indicate it (N2K "not available" +
  SignalK `meta.timeout`).
- Live serial terminal (hex frames) in the web UI.
- In-place calibration commands: two named, allowlisted CALSW/SAVE
  register-write actions.
- MFD-triggered calibration start/stop over N2K, unverified against real
  hardware.
- On-module output bandwidth, output rate, and UART baud: persisted
  config, auto-detected/learned where applicable.
- Raw magnetic field as SignalK deltas, independently toggleable.
- Bus-sourced magnetic variation (listened for, never transmitted):
  fills PGN 127250's Variation field, enables `navigation.headingTrue`.
- OTA firmware upgrades (SensESP built-in).

### 9.2 Post-MVP / Deferred

None at this version — see §9.1.

### 9.3 Out of Scope (Not Deferred — Deliberate Exclusion)

- **Restoring factory defaults via the SAVE register, and any CALSW
  value other than `0`/`7`.** Permanent exclusion (§1.2, §2), the same
  posture the parent project applied to `AT+MODE`.
- **A "clear/reset calibration" action.** The parent project had one
  (`AT+CALI=2`); this fork doesn't, because I don't have a confidently-
  sourced WT901B register value for it (§11) — not guessed at.

## 10. Design Decisions

- **This is a real protocol/data-model port, not a rename** — the user
  explicitly asked for full adaptation rather than a find-and-replace
  fork, because the parent project's SPEC made hardware-capability
  claims ("no pitch/roll", "no gyroscope") that are true for the HWT3100
  but false for the WT901B. Carrying those claims forward unedited would
  have made the fork actively misleading about what the firmware could
  do.
- **PGN 127257 (Attitude) is now implemented, resolving the parent
  project's own stated tension the other way.** The parent SPEC's §10
  explains at length why it deliberately did *not* send Attitude despite
  emulating a Precision-9 identity that implies attitude sensing — the
  HWT3100 had no accelerometer, so sending it would have been dishonest.
  The WT901B does have one, so the same "identity emulation shouldn't
  imply capability the hardware lacks" principle now argues for sending
  it, not against.
- **`RateOfTurnEstimator` (the parent project's sliding-window
  regression) was deleted, not ported.** It existed specifically because
  the HWT3100 had no gyroscope (parent SPEC §1.3, §10). The WT901B has a
  real one; feeding a real gyro reading through a windowed-derivative
  estimator built to approximate a missing sensor would be strictly
  worse than using the real reading directly, and keeping the dead code
  "just in case" would misrepresent this firmware's actual data
  provenance. This is a real simplification, not a compatibility loss.
- **No calibration-reply status item.** The parent project's §8.2
  surfaced the HWT3100's plain-text `AT+CALI` reply strings
  ("Calibrating", "Calibration completed", ...) on a Status page item,
  since `UIButton` clicks are fire-and-forget with no return value. The
  WT901B's register protocol has no acknowledgment packet at all — there
  is nothing on the wire to correlate a click with, so this feature has
  no honest WT901B equivalent and was dropped rather than faked.
- **No "clear calibration" action.** The parent project had a third
  button mapped to `AT+CALI=2`. I don't have a confidently-sourced
  WT901B register value for a magnetic-calibration reset distinct from
  simply re-running start/stop (§11) — rather than guess a register
  value for a firmware feature that writes to real hardware, this
  fork ships with two calibration actions, not three.
- **Rate-of-turn sign convention is carried over from the HWT3100's
  confirmed-on-real-hardware finding, unverified for the WT901B.** The
  parent project confirmed (on real hardware) that its raw Yaw axis
  increases counterclockwise, opposite N2K/SignalK's convention, and
  negates accordingly. This fork applies the same negation to both
  heading and `gyro_z`, for internal consistency — but this is an
  assumption carried across products, not an independently-confirmed
  WT901B finding. See §11 and the parent project's own history (its
  CHANGELOG records a case where exactly this kind of manual-derived
  assumption was wrong until checked against a real device) as the
  reason this is flagged rather than asserted.
- **Baud/bandwidth/rate command formatters never expose the
  protocol's "dangerous" values** (factory-reset SAVE, "no output"
  RRATE code), the same posture the parent project took with `AT+MODE`
  — a command that doesn't exist in the code can't be sent by a bug,
  typo, or future well-meaning addition.
- Every other architectural decision from the parent project (both
  outputs first-class and independently toggleable; calibration offset
  applied once in firmware; reuse SensESP; B&G Precision-9 identity
  emulation with a MAC-derived unique number, not the reference's
  hardcoded one; `meta.timeout` over an active SignalK notification; no
  dedicated fault LED) carries over unchanged — see the parent project's
  SPEC.md §10 for the original rationale, not restated here.

## 11. Open Questions

Everything in this section reflects genuine gaps I could not close
without physical WT901B hardware:

- **Register values and scale factors** (frame format, CALSW=7/0,
  RRATE/BAUD/BANDWIDTH code tables, angle/gyro scale factors) are taken
  from WitMotion's publicly documented register protocol and mirrored
  open-source implementations, **not verified against a physical WT901B
  in this environment.** The parent HWT3100 project's own history (its
  line format was wrong until checked against a real device) is a
  direct warning that this class of detail has been wrong before.
- **Heading/rate-of-turn sign convention** — carried over from the
  HWT3100's confirmed finding, not independently verified for the
  WT901B's AHRS-fused yaw or its gyro axis polarity (§10).
- **Whether CALSW=0 alone persists the just-completed magnetic
  calibration**, or whether the explicit SAVE write this firmware sends
  afterward is actually required — unverified (§8.2).
- **No confidently-sourced register value exists for a "clear/reset
  magnetic calibration" action** distinct from a fresh start/stop cycle
  — this is why this fork has two calibration actions, not the parent
  project's three (§9.3, §10).
- **PGN 127257's Yaw field convention** — left "not available" here;
  unverified against how a real B&G Precision-9 populates it, or whether
  a consuming MFD expects a value there at all (§5.1).
- **Power wiring** (module GND/VCC to the N2K bus's own supply) is
  carried over from the parent project's confirmed-on-real-hardware
  finding for the HWT3100, not independently re-verified for the
  WT901B — the two modules may have different current draw or supply
  voltage tolerances (§4).
- **The entire PGN 130850/130851 MFD-calibration mechanism (§8.3)**
  remains unverified against real hardware, unchanged from the parent
  project's own §11.
- **Settle-delay timing for BAUD register switching** (§8.2c) is a
  reasonable guessed value, not derived from a WT901B timing spec, same
  caveat the parent project flagged for `AT+UART`.

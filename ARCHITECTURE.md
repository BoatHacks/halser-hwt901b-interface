# HALSER-HWT901B-interface Architecture

See SPEC.md for requirements and rationale; this document covers how the
firmware is built.

This project is forked from `BoatHacks/halser-hwt3100-interface`. Its
N2K/SignalK plumbing, config-item patterns, and overall component layout
are carried over largely unchanged; the sensor protocol layer (§2.1),
data model (§3), and calibration/rate/bandwidth commands (§2.2-2.2d) are
a real port for the WT901B's binary register protocol, not a rename of
the HWT3100's line-based ASCII one. Sections below call out what changed
and why; unchanged sections say so explicitly rather than restating the
parent project's rationale in full.

## 1. Overview

```
                     UART (GPIO2 TX / GPIO3 RX), auto-detected baud
                     HALSER "UART" terminal block,
                     RX-select jumper on "U"
WT901B       ◄────────────────────────────────────► Serial1
   ▲  (read: continuous 11-byte binary frames) (write: CALSW/RRATE/
   │                                             BANDWIDTH/BAUD register
   │                                             writes only)
   │                                                       │
   │                                                       ▼
   │                                      WT901B binary frame parser
   │                          "0x55 <type> <8 data bytes> <checksum>"
   │                                                       │
   │                                            ImuReading (heading,
   │                                     roll, pitch, gyro_z, magX/Y/Z,
   │                                                    timestamp)
   │                             ┌──────────────────┬──────┴────────────┐
   │                             ▼                  ▼                   ▼
   │                   Calibration offset    Serial log ring buffer  (raw
   │                   applied (once)        (30 frames, config REST  frames
   │                             │            API, hex-dump, §2.5)    also
   │                             ├──────────────────┐                 feed
   │                             ▼                  ▼                 here)
   │              N2K senders: PGN 127250    SignalK delta sender
   │              (heading), 127251 (rate    (navigation.headingMagnetic,
   │              of turn, real gyro),       .rateOfTurn (real gyro),
   │              127257 (attitude, real     .headingTrue, .attitude,
   │              roll/pitch) — all          SensESP, WiFi)
   │              ExpiringValue pattern                  │
   │                             │                       ▼
   │                             ▼                SignalK server (WiFi)
   │        tNMEA2000_esp32 (TWAI, GPIO4/5) — identifies as
   │        B&G Precision-9 (§5, §10)
   │
   └──────────────── Calibration command handler ◄──── Web UI (config
                      (CALSW=7/0 only, §2.2)             REST toggles,
                                                          not free text)
```

Single ESP32-C3 firmware, no boot-mode routing — same as the parent
project (no test-jig requirement). One FreeRTOS task reads the WT901B's
binary frame stream; the main SensESP event loop drives calibration, N2K
sending, SignalK delta publishing, and (on user action) calibration
command writes.

The WT901B is a **full 9-axis IMU**, unlike the HWT3100 this project is
adapted from (compass-only): it reports magnetic heading, roll, pitch,
raw gyroscope, and raw magnetic field. Its wire protocol is a binary
frame stream for reads and 5-byte register writes for commands — neither
resembles the HWT3100's line-based ASCII `AT+` protocol; see §2.1, §4.

On the N2K bus, this firmware presents as a **B&G Precision-9**, same
identity-cloning approach as the parent project (§1, §4 there). Unlike
the parent project, PGN 127257 (Attitude) is now implemented with real
data — the WT901B's accelerometer makes this an honest addition (SPEC.md
§5.1, §9.3, §10).

## 2. System Components

### 2.1 WT901B Serial I/O (`hwt901b_serial.h/.cpp`, plus `hwt901b_parser.h/.cpp`)

Owns the UART1 (`Serial1`) handle exclusively — no other component ever
touches `Serial1` directly. Two directions, both gated:

Verbose serial debug logging (every setup step, every TX write, and
every RX frame — hex plus decoded values) is available behind the
`HALSER_DEBUG_SERIAL` compile-time flag (`platformio.ini`), off by
default; it was used for the initial hardware bring-up build
(`v0.2.0-debug`) and left in place, gated off, for the next time this
needs debugging rather than deleted. The unconditional per-frame hex
dump (one `ESP_LOGD` line per received frame) predates that flag and
stays always-on regardless, same as the HWT3100 fork's own per-line
logging.

- **Read** (continuous, dedicated FreeRTOS task, same pattern as the
  parent project's read task): accumulates bytes, syncing on `0x55`
  header bytes, into 11-byte frames, then hands each frame to
  `ParseHWT901BFrame()` — a pure function, in its own file
  (`hwt901b_parser.h/.cpp`) with no Arduino dependency, unit-tested on
  the host (`pio test -e native`). Unlike the parent project's line
  parser (which produced one complete `HeadingReading` per line), this
  parser is called once per *packet* and merges each packet's fields
  into a caller-held `ImuReading` — the WT901B streams acceleration,
  gyro, angle, and magnetic data as separate frames, not one combined
  record. `hwt901b_serial.h/.cpp` itself is the thin hardware wrapper:
  byte-level frame sync, calling the parser, setting `timestamp` from
  `millis()` after each successfully-parsed frame, and marshaling to the
  main loop via two `TaskQueueProducer`s (one for the accumulated
  `ImuReading`, one for the raw-frame tap, `HWT901BRawFrame` — a
  fixed-size POD, not `Arduino::String`, same allocation-free rationale
  as the parent project). Both producers are constructed by, and owned
  by, `gateway.cpp` (2.6) and passed into `HWT901BSerialIO` by pointer.
- **Write**: `SendCommand(HWT901BCommand cmd)`, where `HWT901BCommand`
  is a closed enum with exactly two values —
  `kStartMagCalibration`, `kStopMagCalibration` — mapping to CALSW
  register writes of `7` and `0` respectively (the stop path also writes
  SAVE=save-current afterward, SPEC.md §8.2). Plus `SetBandwidth()`
  (2.2b), `SetRate()` (2.2c), and `SetBaudRate()` (2.2d) — each a single
  bounded, clamped numeric write via a 5-byte register-write frame, not
  a raw-byte passthrough. There is no method on this class that accepts
  arbitrary register/data values, and **no enum value for a
  factory-reset SAVE write exists at all** — same "never added to the
  enum's definition" posture the parent project applied to `AT+MODE`
  (SPEC.md §1.2, §2).
- **Baud detection** (`DetectBaud()`, 2.2d): the one method here that
  writes nothing at all — pure passive listening at each candidate rate,
  used only before `Begin()` starts the background read task.

### 2.2 Calibration Command Handler (`hwt901b_calibration_commands.h`)

Same shape as the parent project's — receives named calibration actions
from two trigger sources (web UI, N2K via `MfdCalibrationBridge`, 2.2a)
and maps each to an `HWT901BCommand` enum value, then calls
`HWT901BSerialIO::SendCommand` (2.1). The *only* component with access
to `SendCommand`. Now two methods instead of three
(`StartCalibration()`/`EndCalibration()`) — see SPEC.md §9.3, §10 for why
there is no third "clear" action in this fork.

### 2.2a MFD Calibration Bridge (`mfd_calibration_bridge.h/.cpp`)

Unchanged from the parent project (component and file both carried over
without modification — this is pure N2K plumbing, not sensor-protocol-
specific). PGN 130850/130851, reverse-engineered proprietary Navico/
Simnet message, unverified against real hardware (SPEC.md §8.3, §11).
Dispatches through `CalibrationCommandHandler::StartCalibration()`/
`EndCalibration()`, same two methods 2.2 now exposes.

### 2.2b Output Bandwidth (`hwt901b_bandwidth_command.h/.cpp`)

`FormatBandwidthCommand()` (SPEC.md §8.2a) — pure, unit-tested function
that snaps a requested Hz value to the nearest of the BANDWIDTH
register's seven supported codes (256/184/94/44/21/10/5 Hz) and formats
the 5-byte register-write frame, used by
`HWT901BSerialIO::SetBandwidth()` (2.1). This is the WT901B analog of
the parent project's `AT+FILT` config wiring in `gateway.cpp` (2.6): a
single persisted `PersistingObservableValue<int>` sent to the module
once at boot and again on every config change, since the module can't
report its current bandwidth back.

### 2.2c Output Rate (`hwt901b_rate_command.h/.cpp`)

`FormatRateCommand()` (SPEC.md §8.2b) — pure, unit-tested function that
snaps a requested centihertz value to the nearest of the RRATE
register's eleven supported codes and formats the register-write frame,
used by `HWT901BSerialIO::SetRate()` (2.1). Unlike the parent project's
`AT+PRATE` (which had a query/reply mechanism and an "unknown" sentinel
default learned from the module at boot), the WT901B's register protocol
has no reply channel to query a current rate from — so this config item
uses a plain, immediately-applied default (SPEC.md §7) rather than the
parent's discover-then-persist pattern. The RRATE register's "no output"
code is never reachable through this formatter at all (SPEC.md §8.2b) —
a stricter version of the parent project's `AT+PRATE=0` warning, made
possible by not needing to keep that value reachable for a query
round-trip that doesn't exist here.

### 2.2d UART Baud Rate (`hwt901b_baud_command.h/.cpp`)

`FormatBaudCommand()` (SPEC.md §8.2c) — pure, unit-tested function that
snaps a requested baud to the nearest of the BAUD register's seven
supported rates (4800/9600/19200/38400/57600/115200/230400) and formats
the register-write frame, used by `HWT901BSerialIO::SetBaudRate()`.
Same unknown-sentinel-then-auto-detect pattern as the parent project's
`AT+UART` (`halser::kBaudUnknown = -1`), `DetectBaud()` (2.1) trying
candidates in order (115200 recommended first) before `Begin()` starts
the background read task. Same known race-condition limitation as the
parent project: `SetBaudRate()` reconfigures `Serial1` from the
main-loop thread while the background read task may concurrently be
accessing it — narrow, rare (only during an explicit user-triggered baud
change), accepted rather than solved with extra synchronization.

### 2.3 Calibration Offset

Unchanged in shape from the parent project (`calibration_offset.h`):
applies the configured heading offset to each `ImuReading` before either
output path, negating the raw yaw axis first (SPEC.md §2, §10 — the sign
convention is carried over from the HWT3100's confirmed finding, flagged
unverified for the WT901B in SPEC.md §11). Roll, pitch, gyro, and mag
fields pass through unchanged — only heading is offset-corrected
(SPEC.md §9).

### 2.4 N2K Senders (`n2k_senders.h`)

- `N2kHeadingSender` — PGN 127250, unchanged from the parent project.
- `N2kRateOfTurnSender` — PGN 127251, same `ExpiringValue<T>`/sender
  shape as the parent project, but now fed directly by
  `ImuReading.gyro_z` (sign-corrected, converted to rad/s) in
  `gateway.cpp`'s consumer lambda (2.6) — not by a `RateOfTurnEstimator`
  (deleted in this fork; see SPEC.md §10 for why).
- `N2kAttitudeSender` — PGN 127257, **new in this fork**. Same
  `ExpiringValue<T>` pattern as the other two senders, fed by real
  roll/pitch. Yaw is always `N2kDoubleNA` (SPEC.md §5.1, §11).

All three PGNs declared to `tNMEA2000::ExtendTransmitMessages()` (SPEC.md
§5.1, same requirement/rationale as the parent project's ARCHITECTURE.md
§2.4).

### 2.5 Serial Terminal (`serial_terminal.h/.cpp`, class `SerialTerminal`)

Same mechanism as the parent project — a fixed-size ring buffer (last 30
entries) exposed through SensESP's config REST API (no public WebSocket
extension point exists in SensESP 3.2.0, per the parent project's own
investigation, `docs/plans/gateway-wiring.md`, carried over unchanged
here since it's a SensESP-version finding, not a sensor-protocol one).
Unlike the parent's `AddLine(HWT3100RawLine)` (plain text), this fork's
`AddFrame(HWT901BRawFrame)` stores the raw 11-byte binary frame and
`to_json()` renders each as a hex byte string (`"55 53 ..."`) — a hex
dump is the only sensible display for a fixed-length binary packet.

### 2.6 Configuration / Web UI Wiring (`gateway.cpp`)

Same `ConfigItem` + `PersistingObservableValue` pattern as the parent
project for every SPEC.md §7 config value. Notable differences from the
parent's `gateway.cpp`:

- Config paths renamed `/hwt901b/*` (was `/hwt3100/*`): `output_bandwidth`,
  `output_rate_chz`, `baud`.
- No rate-of-turn window/min-span config items — that concept doesn't
  exist in this fork (SPEC.md §7, §10).
- Two calibration `UIButton`s, not three (`hwt901b_calibration_1_start`,
  `hwt901b_calibration_2_stop`) — same alphabetical-by-name
  Control-tab-ordering mechanism as the parent project.
- No calibration-reply `StatusPageItem` — the WT901B's register protocol
  has no acknowledgment packet to correlate with a click (SPEC.md §8.2,
  §10), so this piece of the parent's wiring has no equivalent here.
- New: `N2kAttitudeSender` wiring and the `navigation.attitude`
  `SKOutputRawJson` consumer (2.7).

`MfdCalibrationBridge` (2.2a) and `MagneticVariationListener` (2.4b, see
below) are constructed the same way as the parent project — self-
attaching `tNMEA2000::tMsgHandler` subclasses, unmodified.

### 2.4b Magnetic Variation Listener (`magnetic_variation_listener.h/.cpp`)

Unchanged from the parent project — file carried over without
modification, pure N2K plumbing (PGN 127258 listener, read-only).

### 2.7 SignalK Delta Sender

Publishes `navigation.headingMagnetic`, `navigation.rateOfTurn`,
`navigation.headingTrue`, `navigation.attitude`, and
`sensors.hwt901b.magneticField.x/y/z` via SensESP's existing SignalK/
WiFi transport. Each has its own enable flag, same master-plus-per-
output pattern as the parent project's ARCHITECTURE.md §2.7.

`navigation.attitude` is new in this fork and is the one output that
doesn't fit SensESP's `SKOutputNumeric<T>`/`SKOutputFloat` family
(checked against SensESP's actual `signalk_output.h` — there is no
dedicated attitude/object-output class): it's published via
`SKOutputRawJson`, hand-building the `{"roll":...,"pitch":...,"yaw":0}`
JSON string in `gateway.cpp`'s `ImuReading` consumer lambda before
calling `set()`. `navigation.rateOfTurn` is now sourced from
`ImuReading.gyro_z` directly (sign-corrected, degrees/s → rad/s) instead
of a `RateOfTurnEstimator` output (SPEC.md §10).

### 2.9 No Dedicated Fault LED

Unchanged from the parent project — SensESP's own `RGBSystemStatusLed`
(GPIO8, `PIN_RGB_LED` build flag) has sole ownership, no exposed
pause/share hook. Not a sensor-specific concern; not revisited in this
fork.

## 3. Data Models

See SPEC.md §3 for the conceptual model. In code:

```cpp
struct ImuReading {
  float heading = 0.0f;  // degrees, 0-360, magnetic, offset-corrected
  float roll = 0.0f;     // degrees, -180..180
  float pitch = 0.0f;    // degrees, -90..90
  float gyro_z = 0.0f;   // degrees/second, raw yaw-axis rate
  int32_t mag_x = 0;     // raw magnetic field X, diagnostic use only
  int32_t mag_y = 0;     // raw magnetic field Y, diagnostic use only
  int32_t mag_z = 0;     // raw magnetic field Z, diagnostic use only
  unsigned long timestamp = 0;  // millis() of last packet contributing here
};

enum class HWT901BCommand {
  kStartMagCalibration,  // CALSW=7
  kStopMagCalibration,   // CALSW=0 (+ SAVE=save-current)
};

struct HWT901BRawFrame {  // raw 11-byte frame for the terminal (§2.5)
  static constexpr size_t kLength = 11;
  uint8_t bytes[kLength] = {0};
};
```

No separate `CalibrationOffset` struct exists, same as the parent
project — a single `float`, held by a `PersistingObservableValue<float>`
in `gateway.cpp`.

## 4. Technology Stack

Same as the parent project, minus the parent's custom line-based ASCII
parser (replaced by a binary frame parser) and rate-of-turn estimator
(deleted, §2.4/SPEC.md §10):

| Layer | Choice | Why |
|---|---|---|
| Framework | Arduino (ESP32-C3), SensESP 3.2.0 | Same as parent project. |
| N2K | ttlappalainen/NMEA2000-library + NMEA2000_twai | Same as parent; adds `SetN2kPGN127257` (Attitude) alongside the parent's 127250/127251. |
| N2K device identity | Cloned from `htool/ESP32_Precision-9_compass_CMPS14` | Unchanged from parent project — see SPEC.md §1.2, §10. |
| Sensor frame parsing | Custom (this project) | Binary 11-byte frame parsing with checksum validation — a real port from the parent's line-based ASCII parser, not a rename (SPEC.md §1.2). |
| Rate of turn | WT901B's own gyroscope | No estimator needed, unlike the parent project — a direct sensor reading (SPEC.md §1.3, §10). |
| Serial log transport | SensESP's existing config REST API (no new dependency) | Same finding as the parent project (`docs/plans/gateway-wiring.md`) — unrelated to sensor choice. |
| RGB LED | Not used by this firmware's own code | Unchanged from parent project. |

## 5. Integration Points

- **WT901B module** — UART1, `Serial1`, GPIO2 TX / GPIO3 RX, baud
  auto-detected among 4800/9600/19200/38400/57600/115200/230400
  (`kHWT901BDefaultBaud = 115200` is the fallback/first-tried candidate,
  `halser_const.h`). Wired to HALSER's UART terminal block with the
  RX-select jumper on "U" — same physical wiring as the parent project
  (unchanged, board-specific, not sensor-specific). Protocol: binary
  frame stream for data, 5-byte register writes for commands (SPEC.md
  §1.2) — **not independently verified against a physical WT901B in
  this environment** (SPEC.md §11).

  **Power**: carried over from the parent project's confirmed wiring
  (module GND/VCC to the N2K bus's own supply) — not independently
  re-verified for the WT901B specifically (SPEC.md §4, §11).
- **NMEA 2000 bus** — via `tNMEA2000_esp32`, GPIO4 TX / GPIO5 RX (TWAI),
  unchanged from the parent project.
- **SignalK server** — via SensESP's WiFi + mDNS + WebSocket delta
  client, unchanged.
- **Browser (web UI)** — SensESP's own config UI only, unchanged.

## 6. Security Considerations

Same trust model as the parent project: a device on a private boat LAN,
no additional auth/encryption scoped.

The hardware-specific rule that *is* a hard requirement: **the firmware
must never write the SAVE register's "restore factory defaults" value,
and must never write any CALSW value other than `0` or `7`** (SPEC.md
§1.2, §2, §9.3) — the WT901B-protocol equivalent of the parent project's
`AT+MODE` exclusion.

- `Serial1` is owned exclusively by `HWT901BSerialIO` (2.1); no other
  component ever gets a reference to it.
- `HWT901BSerialIO` exposes exactly four write methods, and they are the
  *only* code in this firmware that writes to `Serial1`:
  - `SendCommand()` takes a closed `HWT901BCommand` enum with exactly
    two values (2.1, §3). No overload, no debug backdoor, and **no enum
    value that maps to a factory-reset SAVE write** — not filtered out,
    simply never defined.
  - `SetBandwidth(int)` (§2.2b), `SetRate(int)` (§2.2c), and
    `SetBaudRate(int, ...)` (§2.2d) each construct a bounded 5-byte
    register-write frame via a pure formatter that snaps the input to
    one of a small closed set of supported values before anything
    reaches the wire.
  All four are bounded, closed-domain writes, not a raw-register
  backdoor.
- The calibration command handler (2.2) and the bandwidth/rate/baud
  config wiring (2.2b-2.2d, `gateway.cpp`) are the only components
  holding references to these write methods; the serial terminal (2.5)
  and everything else remain read-only.
- Because the enum and all three numeric formatters have small, closed,
  clamped domains, a code reviewer can verify the entire write surface
  by reading `HWT901BCommand`'s definition, `SendCommand()`'s lookup
  table, and the three `Format*Command()` functions — all fit on one
  screen combined, same auditability property the parent project's §6
  established.

## 7. File Structure

```
src/
  main.cpp                             — entry point, run_hwt901b_gateway()
  halser_const.h                        — pin assignments (unchanged from
                                         parent) + N2K device identity
                                         constants (unchanged, §1, §4)
  hwt901b_types.h                        — ImuReading, HWT901BCommand,
                                         HWT901BRawFrame (§3). No Arduino
                                         dependency.
  hwt901b_parser.h/.cpp                  — ParseHWT901BFrame(): pure
                                         binary frame -> ImuReading
                                         (merging) parsing, no Arduino
                                         dependency, unit tested via
                                         `pio test -e native`
                                         (docs/plans/hwt901b-serial-parser.md)
  hwt901b_serial.h/.cpp                  — Serial1 owner (2.1): read task
                                         (frame sync, calls
                                         ParseHWT901BFrame()) +
                                         SendCommand()/SetBandwidth()/
                                         SetRate()/SetBaudRate() write
                                         paths
  hwt901b_calibration_commands.h          — CalibrationCommandHandler
                                         (2.2): two named actions ->
                                         SendCommand()
  mfd_calibration_bridge.h/.cpp           — MfdCalibrationBridge (2.2a),
                                         unchanged from parent project
  hwt901b_bandwidth_command.h/.cpp        — BANDWIDTH register formatter
                                         (2.2b), unit tested
  hwt901b_rate_command.h/.cpp             — RRATE register formatter
                                         (2.2c), unit tested
  hwt901b_baud_command.h/.cpp             — BAUD register formatter
                                         (2.2d), unit tested
  calibration_offset.h                    — heading offset application
                                         (2.3), a pure function
  n2k_senders.h                           — ExpiringValue<T> +
                                         N2kHeadingSender (PGN 127250) +
                                         N2kRateOfTurnSender (PGN 127251)
                                         + N2kAttitudeSender (PGN 127257,
                                         new in this fork) (2.4)
  magnetic_variation_listener.h/.cpp      — unchanged from parent project
  serial_terminal.h/.cpp                  — SerialTerminal (2.5): 30-frame
                                         ring buffer, hex-dump rendering
  gateway.h/.cpp                          — SensESP app wiring (2.6)
test/
  test_hwt901b_parser/                    — Unity tests for
                                         ParseHWT901BFrame(), native env
  test_hwt901b_bandwidth_command/         — Unity tests, native env
  test_hwt901b_rate_command/              — Unity tests, native env
  test_hwt901b_baud_command/              — Unity tests, native env
  test_calibration_offset/                — Unity tests, native env
docs/
  plans/                                 — per-feature implementation
                                         plans (see
                                         IMPLEMENTATION_CHECKLIST.md);
                                         several carried over from the
                                         parent project with an
                                         "adapted-fork note" banner where
                                         WT901B specifics diverge
SPEC.md
ARCHITECTURE.md
platformio.ini
```

Deleted relative to the parent project (SPEC.md §10): `rate_of_turn.h/
.cpp` and its test (no WT901B equivalent needed — real gyro replaces
the estimator); `hwt3100_calibration_reply.h/.cpp` and its test (no
WT901B equivalent exists — the register protocol has no acknowledgment
packet).

## 8. Deployment

Unchanged from the parent project: `pio run -t upload` over USB for
initial flash, SensESP's built-in OTA for subsequent updates.

## 9. Future Considerations

- Further WT901B register writes exposed as config (e.g. accelerometer/
  gyro bias calibration, if a confidently-sourced register value is ever
  found) would follow the same pattern established for BANDWIDTH/RRATE/
  BAUD (2.2b-2.2d): a small pure format function plus one
  `HWT901BSerialIO` method.
- **A factory-reset SAVE write and any CALSW value beyond `0`/`7` are
  not future considerations** — SPEC.md §9.3 treats this as a permanent,
  deliberate exclusion, same posture as the parent project's `AT+MODE`.
- If real WT901B hardware becomes available, the highest-value
  verification pass is SPEC.md §11 in order: frame/checksum format
  first (nothing else works if this is wrong, same lesson the parent
  project's own history teaches), then the heading/gyro sign convention,
  then the calibration register values.

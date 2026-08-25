# AGENTS.md

## Project Overview

HALSER-HWT901B-interface — ESP32-C3 firmware that reads orientation data
from a WitMotion WT901B 9-axis IMU over serial, and republishes it as
NMEA 2000 messages and SignalK deltas. Read [SPEC.md](SPEC.md) and
[ARCHITECTURE.md](ARCHITECTURE.md) before making changes — this document
is a quick-reference summary of those, not a replacement for them.

This project is forked from `BoatHacks/halser-hwt3100-interface`
(compass-only HWT3100-TTL). The WT901B is a full IMU: it *does* have an
accelerometer and gyroscope, and *does* produce roll/pitch and a real
rate of turn — unlike the parent project's hardware. Don't carry over
the parent project's "the hardware can't do this" reasoning without
checking SPEC.md §1.2/§3/§9.3 first; several things that were permanent
exclusions there are implemented features here.

**Hard safety constraint**: never write the SAVE register's "restore
factory defaults" value, and never write any CALSW value other than `0`
or `7`. See SPEC.md §1.2/§2/§9.3 and ARCHITECTURE.md §6 — the WT901B
equivalent of the parent project's `AT+MODE` exclusion.

**Unverified-hardware caveat**: this project's protocol layer (frame
format, register addresses, scale factors) was written against
WitMotion's publicly documented register protocol, without access to a
physical WT901B module. Treat anything flagged in SPEC.md §11 as
genuinely uncertain, not as a settled fact — the parent project's own
history (a wrong line-format assumption that went unnoticed until
checked against real hardware) is the reason to take this seriously.

## Build Commands

```bash
# Build firmware
pio run

# Upload to connected board
pio run -t upload

# Monitor serial output
pio device monitor

# Host-only unit tests (parser, command formatters, calibration offset)
pio test -e native
```

## Architecture

See ARCHITECTURE.md for the full component breakdown. Summary:

### Data Flow

```
UART1 (baud auto-detected, GPIO 3 RX)
  → WT901B binary frame parser ("0x55 <type> <8 data> <checksum>")
  → ImuReading (heading, roll, pitch, gyro_z, magX/Y/Z, timestamp)
  → Calibration offset applied (heading only)
  → N2K senders (PGN 127250 heading, 127251 rate of turn [real gyro],
    127257 attitude [real roll/pitch]) + SignalK delta sender,
    independently toggleable
```

Calibration commands (CALSW=7/0 register writes) are sent through a
separate, allowlisted write path — see ARCHITECTURE.md §2.1, §2.2, §6.

### Source Layout

- `src/main.cpp` — entry point
- `src/halser_const.h` — pin assignments and constants
- `src/gateway.h/.cpp` — SensESP application wiring
- `src/hwt901b_types.h`, `hwt901b_parser.h/.cpp`, `hwt901b_serial.h/.cpp`
  — sensor protocol layer
- `src/hwt901b_bandwidth_command.h/.cpp`,
  `hwt901b_rate_command.h/.cpp`, `hwt901b_baud_command.h/.cpp` — register
  write formatters
- `src/hwt901b_calibration_commands.h`, `calibration_offset.h`,
  `n2k_senders.h`, `serial_terminal.h/.cpp`,
  `mfd_calibration_bridge.h/.cpp`, `magnetic_variation_listener.h/.cpp`
  — see ARCHITECTURE.md §2 for the full component breakdown

### System Health Reporting

`gateway.cpp` calls `SensESPAppBuilder::enable_system_info_sensors()`,
which publishes SensESP's built-in system-health sensors to SignalK
under `sensors.halser-hwt901b.*`: `systemHz`, `freeMemory`, `uptime`,
`ipAddress`, `wifiSignalLevel`.

### Hardware Pin Assignments

| Pin | Function |
|-----|----------|
| GPIO 2 | UART1 TX (to WT901B, via HALSER's UART terminal block, jumper on "U") |
| GPIO 3 | UART1 RX (from WT901B) |
| GPIO 4 | CAN TX |
| GPIO 5 | CAN RX |
| GPIO 8 | RGB LED (SK6805) |
| GPIO 9 | Button |

## Dependencies

- SensESP 3.2.0 — IoT framework (WiFi, web UI, Signal K)
- NMEA2000-library — NMEA 2000 message handling
- NMEA2000_twai — ESP32 TWAI (CAN) driver
- Adafruit NeoPixel — RGB LED control

# HALSER-HWT901B-interface

WitMotion WT901B 9-axis IMU to NMEA 2000 / SignalK bridge firmware for
the [HALSER](https://shop.hatlabs.fi/products/halser) ESP32-C3 serial
interface board.

Forked from [`BoatHacks/halser-hwt3100-interface`](https://github.com/BoatHacks/halser-hwt3100-interface),
which targets a different, compass-only WitMotion module. This project
is a real protocol/data-model port, not a rename — see SPEC.md §1.2 and
§10 for what changed and why, and §11 for what remains unverified
without physical WT901B hardware.

Reads heading, roll, pitch, rate of turn, and raw magnetic field from a
WT901B over UART, and republishes it as both NMEA 2000 messages and
SignalK deltas — either output independently enable/disable-able.

See [SPEC.md](SPEC.md) for requirements and rationale, and
[ARCHITECTURE.md](ARCHITECTURE.md) for how the firmware is built.

## Features (see IMPLEMENTATION_CHECKLIST.md)

- Reads the WT901B's binary frame stream (heading, roll, pitch, rate of
  turn from a real gyroscope, raw magnetic field)
- Transmits NMEA 2000 PGN 127250 (Vessel Heading), PGN 127251 (Rate of
  Turn, real gyro data), and PGN 127257 (Attitude, real roll/pitch) —
  the last two both newly implementable relative to the compass-only
  parent project
- Publishes `navigation.headingMagnetic`, `navigation.rateOfTurn`,
  `navigation.headingTrue`, and `navigation.attitude` SignalK deltas
- Configurable heading calibration offset (mounting misalignment)
- In-place on-module magnetic-field calibration, from a fixed allowlist
  of two register writes (start/stop) — see SPEC.md §8.2
- Live serial terminal (hex frame dump) in the web UI for
  wiring/troubleshooting
- SensESP-based: WiFi AP/client, web UI configuration, OTA updates

**Not supported, deliberately** (see SPEC.md §9.3): writing the SAVE
register's factory-reset value or any CALSW value other than `0`/`7` —
the WT901B-protocol equivalent of the parent project's `AT+MODE`
exclusion — and a "clear/reset calibration" action, dropped because no
confidently-sourced register value exists for it (SPEC.md §11).

## Building

Requires [PlatformIO](https://platformio.org/).

```bash
pio run
```

Host-only unit tests (parser, register-write command formatters,
calibration offset — no board required):

```bash
pio test -e native
```

## License

MIT

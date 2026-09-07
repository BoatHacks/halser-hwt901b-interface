# HALSER-HWT901B-interface

WitMotion WT901B 9-axis IMU to NMEA 2000 / SignalK bridge firmware for
the [HALSER](https://shop.hatlabs.fi/products/halser) ESP32-C3 serial
interface board.

Reads heading, roll, pitch, rate of turn, and raw magnetic field from a
WT901B over UART, and republishes it as NMEA 2000 messages and SignalK
deltas — each output independently enable/disable-able.

See [SPEC.md](SPEC.md) for requirements and [ARCHITECTURE.md](ARCHITECTURE.md)
for how the firmware is built.

## Features

- Reads the WT901B's binary frame stream: heading, roll, pitch, rate of
  turn (gyroscope), raw magnetic field
- Transmits NMEA 2000 PGN 127250 (Vessel Heading), PGN 127251 (Rate of
  Turn), PGN 127257 (Attitude)
- Publishes `navigation.headingMagnetic`, `navigation.rateOfTurn`,
  `navigation.headingTrue`, `navigation.attitude` SignalK deltas
- Configurable heading calibration offset
- On-module magnetic-field calibration (start/stop) from the web UI or
  over N2K
- Live serial log in the web UI: hex dump plus decoded values per frame
- SensESP-based: WiFi AP/client, web UI configuration, OTA updates

Not supported: writing the SAVE register's factory-reset value, any
CALSW value other than `0`/`7`, or a "clear/reset calibration" action
— see SPEC.md §9.3.

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

# Implementation Plan: WT901B hardware serial I/O

## Overview

The hardware-facing half of ARCHITECTURE.md §2.1: a dedicated FreeRTOS
task that owns `Serial1`, reads bytes into 11-byte binary frames, calls
the already-tested `ParseHWT901BFrame()`
(docs/plans/hwt901b-serial-parser.md), and marshals results to the main
SensESP loop. Also implements the *only* write paths to the WT901B —
`SendCommand(HWT901BCommand)`, `SetBandwidth()`, `SetRate()`,
`SetBaudRate()` — with no code path that accepts an arbitrary register
write.

Adapted from `BoatHacks/halser-hwt3100-interface`'s equivalent plan: this
project's fork target (the WitMotion WT901B) is a full 9-axis IMU
speaking a binary register protocol, not the HWT3100's line-based ASCII
— the framing and command formats below are new, but the task/queue
architecture is carried over unchanged.

## Relevant SPEC/ARCHITECTURE Sections

- SPEC.md §1.2 (wire format, the permanently-excluded factory-reset SAVE
  write), §2 (domain rule), §8.2 (calibration command allowlist)
- ARCHITECTURE.md §2.1 (component split), §6 (security: single write
  chokepoint), §7 (file structure)

## Approach

- `HWT901BSerialIO` owns `Serial1` exclusively. Constructed with
  references to two `sensesp::TaskQueueProducer` instances (owned by the
  caller, `gateway.cpp`) — one for parsed `ImuReading`s, one for raw
  frames (for the serial terminal, §8.1).
- Raw frames are marshaled as a small fixed-size POD (`HWT901BRawFrame`,
  11 bytes), not `Arduino::String`, to avoid heap allocation across the
  task boundary.
- Frame sync: the read task only starts accumulating a frame on a `0x55`
  header byte. A stray `0x55` inside an in-progress frame's data bytes
  will fail the checksum check and drop that frame — no mid-frame resync
  is implemented, a known simplification (SPEC.md §11).
- `timestamp` is stamped with `millis()` in the read task, after each
  successfully-parsed frame — `ParseHWT901BFrame()` deliberately doesn't
  set it itself, same rationale as the parser it replaces.
- `SendCommand()` switches on the closed `HWT901BCommand` enum with no
  `default:` case, so adding an enum value without updating the command
  table is a compiler warning, not a silent gap.

## Test Strategy

The parsing logic and the three register-write formatters already have
host-side unit tests. This component is Arduino/FreeRTOS-dependent (owns
a real `HardwareSerial`, spawns a real task) and can't run in the
`native` test environment — verification here is a successful
`pio run -e halser` build (compiles against the real ESP32 framework)
plus manual hardware-in-the-loop testing once wired into `gateway.cpp`
and connected to a real WT901B module.

## Implementation Steps

- [x] Add `HWT901BRawFrame` to `hwt901b_types.h`
- [x] Add `src/hwt901b_serial.h/.cpp`
- [x] Verify `pio test -e native` and `pio run -e halser` both build
- [x] Update ARCHITECTURE.md if the implementation diverged from the plan

## Files to Create/Modify

- `src/hwt901b_types.h` (add `HWT901BRawFrame`)
- `src/hwt901b_serial.h` (new)
- `src/hwt901b_serial.cpp` (new)

# Implementation Plan: WT901B binary frame parser

## Overview

Parse the WT901B's continuous 11-byte binary packet stream
(`0x55 <type> <8 data bytes> <checksum>`) into an `ImuReading`. This is
the first piece of ARCHITECTURE.md §2.1 (WT901B Serial I/O) — the
read/parse half, kept separate from the hardware serial I/O plumbing so
the parsing logic itself is unit-testable without a real board.

Adapted from `BoatHacks/halser-hwt3100-interface`'s equivalent plan for
its line-based ASCII parser — the WT901B speaks a different, binary
protocol, so the parser itself is a new implementation, but the
file-split rationale (Arduino-free pure logic vs. hardware plumbing)
carries over unchanged.

## Relevant SPEC/ARCHITECTURE Sections

- SPEC.md §1.2 (wire format — per WitMotion's published register
  protocol, **not verified against a physical WT901B in this
  environment**), §3 (data model), §11 (open questions: register values
  and scale factors this project couldn't independently confirm)
- ARCHITECTURE.md §2.1 (WT901B Serial I/O), §3 (data model in code)

## Approach

Split what ARCHITECTURE.md calls `hwt901b_serial.h/.cpp` into two
pieces, same split as the HWT3100 fork:

- `hwt901b_types.h` — `ImuReading`, `HWT901BCommand`, `HWT901BRawFrame`,
  no Arduino dependency.
- `hwt901b_parser.h/.cpp` — a pure function, `ParseHWT901BFrame()`, that
  takes one 11-byte frame, validates its header/checksum, and merges the
  fields that frame's type carries into a caller-held `ImuReading`
  (angle, gyro, magnetic, and acceleration arrive as *separate* packets
  — this parser is called once per packet, not once per "full update").
  No Arduino dependency, so it can be unit tested on the host.
- The hardware-facing half (owning `Serial1`, byte-level frame sync,
  calling the parser, and the write paths) is a separate piece
  (docs/plans/hwt901b-serial-io.md).

## Test Strategy

Unit tests (PlatformIO's `native` test environment + Unity, no board
required) covering:
- Each of the four handled packet types (angle, gyro, magnetic,
  acceleration) individually
- Fields from one packet type surviving a subsequent call with a
  different type (the "accumulate across packets" behavior)
- A corrupted checksum, a non-`0x55` header, an unrecognized type byte,
  and a wrong-length buffer, all rejected
- Null-argument handling

Real-hardware verification (does a physical WT901B actually emit exactly
this framing, these scale factors, this packet-type byte assignment) is
an open question per SPEC.md §11 — these tests validate the parser
against WitMotion's publicly documented register protocol, not against a
live module.

## Implementation Steps

- [x] Add `hwt901b_types.h`
- [x] Add `hwt901b_parser.h/.cpp`
- [x] Add `native` PlatformIO test environment + Unity tests
- [x] Update ARCHITECTURE.md for the WT901B data model
- [x] Verify: `pio test -e native`, `pio run -e halser` (main firmware
      env still builds)

## Files to Create/Modify

- `src/hwt901b_types.h` (new)
- `src/hwt901b_parser.h` (new)
- `src/hwt901b_parser.cpp` (new)
- `test/test_hwt901b_parser/test_hwt901b_parser.cpp` (new)
- `platformio.ini` (`[env:native]` build_src_filter)
- `ARCHITECTURE.md` (§7 file structure, §3 data model)

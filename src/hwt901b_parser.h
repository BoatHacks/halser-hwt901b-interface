#ifndef HALSER_SRC_HWT901B_PARSER_H_
#define HALSER_SRC_HWT901B_PARSER_H_

#include <cstddef>
#include <cstdint>

#include "hwt901b_types.h"

// WT901B packet type byte (second byte of every 11-byte frame). Only the
// five types this firmware actually consumes are named here — the module
// can be configured (RSW register) to also emit others (time, GPS,
// quaternion, port status, ...) that this firmware never asks for and
// this parser treats as "unknown type" (ParseHWT901BFrame returns
// false). Confirmed against real hardware (SPEC.md §11): 0x51/0x52/0x53/
// 0x54/0x56 all stream continuously on this board; 0x50/0x55/0x57/0x58/
// 0x59/0x5A were never observed, either not enabled in the module's
// output-content register or genuinely unsupported by this SKU (no GPS
// receiver on a plain WT901B).
//
// Per WitMotion's publicly documented register protocol (shared across
// the JY901/WT901/HWT901B family, mirrored in the vendor's wit_c_sdk and
// multiple open-source drivers) — frame format and field layout NOT
// independently verified against a physical WT901B in this environment,
// though 0x56's pressure field decoded to a plausible atmospheric value
// (~101 kPa) from a real captured frame. See SPEC.md §11.
enum class HWT901BPacketType : uint8_t {
  kAcceleration = 0x51,
  kAngularVelocity = 0x52,
  kAngle = 0x53,
  kMagnetic = 0x54,
  kPressure = 0x56,
};

// Parses one 11-byte WT901B frame: `0x55 <type> <8 data bytes>
// <checksum>`, checksum = low byte of the sum of the first 10 bytes.
//
// Unlike the line-based HWT3100 protocol this project's parser used to
// handle, the WT901B streams several *different* packet types in
// sequence (acceleration, then gyro, then angle, then magnetic, ...) —
// each call to this function decodes exactly one frame and merges the
// fields that frame's type carries into *out; it never resets or
// clears fields a different packet type owns. The caller is expected to
// hold one ImuReading across a run of frames (see hwt901b_serial.cpp) so
// a full update accumulates across the module's own packet cadence.
//
// Returns false (leaving *out unmodified) if `frame` isn't exactly
// kLength bytes, doesn't start with 0x55, fails the checksum, or its
// type byte isn't one of HWT901BPacketType's five handled values —
// doesn't set out->timestamp either way (a hardware clock read has no
// business in a function meant to be testable without a board, same
// rationale as the HWT3100 parser this replaces).
//
// The 0x56 (Pressure) packet also carries a "height" field (pressure
// converted to an altitude estimate against a fixed sea-level
// reference) — deliberately not parsed or exposed anywhere in this
// firmware. A vessel's elevation isn't a meaningful vessel instrument
// reading (unlike pressure itself), so there's no honest place to put
// it (same "don't fabricate/misrepresent data" reasoning as PGN 127257's
// Yaw field, SPEC.md §5.1).
bool ParseHWT901BFrame(const uint8_t* frame, size_t len, ImuReading* out);

// Renders a short, human-readable decoded description of one 11-byte
// WT901B frame into `out` (NUL-terminated, truncated to fit `out_len`
// if necessary) — e.g. "heading=90.00 roll=0.00 pitch=-45.00 (deg)"
// for an Angle (0x53) packet, "gyro x=1.00 y=2.00 z=12.50 deg/s" for an
// Angular Velocity (0x52) packet, "accel x=0.01 y=-0.02 z=1.00 g" for
// an Acceleration (0x51) packet, "mag x=1234 y=-567 z=89" for a
// Magnetic (0x54) packet, "pressure=101046 Pa" for a Pressure (0x56)
// packet, or "invalid frame (bad header/checksum/type)" if the frame
// doesn't parse via ParseHWT901BFrame() at all.
//
// Pure, no Arduino dependency, unit tested directly — used by the web
// UI's serial log (serial_terminal.cpp) to show what each raw frame
// actually means alongside its hex dump (SPEC.md §8.1), not just by
// the HALSER_DEBUG_SERIAL-gated console logging in hwt901b_serial.cpp.
void DescribeHWT901BFrame(const uint8_t* frame, size_t len, char* out, size_t out_len);

#endif  // HALSER_SRC_HWT901B_PARSER_H_

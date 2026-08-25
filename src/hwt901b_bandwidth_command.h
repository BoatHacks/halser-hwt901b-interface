#ifndef HALSER_SRC_HWT901B_BANDWIDTH_COMMAND_H_
#define HALSER_SRC_HWT901B_BANDWIDTH_COMMAND_H_

#include <cstddef>
#include <cstdint>

namespace halser {

// The BANDWIDTH register (0x1F) low-pass filter bandwidth codes, per
// WitMotion's register protocol documentation (§11: unverified against
// real hardware in this environment) — the WT901B analog of the HWT3100
// fork's AT+FILT output smoothing filter. Smaller bandwidth = more
// smoothing/more lag, same trade-off AT+FILT's clamp comment described.
constexpr int kBandwidthHz[7] = {256, 184, 94, 44, 21, 10, 5};

// Formats a BANDWIDTH register write (0xFF 0xAA 0x1F <code> 0x00) for
// the WT901B's accel/gyro output low-pass filter. `requested_hz` is
// snapped to the nearest of kBandwidthHz's seven supported values (ties
// broken toward the higher/less-filtered value, matching kBandwidthHz's
// descending order) rather than rejected — same "clamp to nearest, don't
// reject" numeric-command philosophy as the HWT3100 fork's AT+FILT/
// AT+UART formatters. Writes exactly 5 bytes into buf (buf_len must be
// >= 5) and returns the bandwidth (Hz) actually selected.
int FormatBandwidthCommand(int requested_hz, uint8_t* buf, size_t buf_len);

}  // namespace halser

#endif  // HALSER_SRC_HWT901B_BANDWIDTH_COMMAND_H_

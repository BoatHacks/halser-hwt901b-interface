#ifndef HALSER_SRC_HWT901B_RATE_COMMAND_H_
#define HALSER_SRC_HWT901B_RATE_COMMAND_H_

#include <cstddef>
#include <cstdint>

namespace halser {

// Sentinel meaning "not yet known" for the persisted output-rate config
// value (gateway.cpp) — not a value ever sent to the module. Outside the
// real domain (positive centihertz), so it can't collide with a real
// setting. Mirrors kPrateUnknown from the HWT3100 fork's AT+PRATE.
constexpr int kRateUnknown = -1;

// The RRATE register (0x03) output-rate codes, expressed in centihertz
// (Hz x10, so 0.2 Hz is representable as an int) — per WitMotion's
// register protocol documentation (§11: unverified against real hardware
// in this environment). Deliberately excludes the documented "single
// return"/"no output" codes: RRATE's "no output" value is this
// firmware's equivalent of the HWT3100 fork's dangerous AT+PRATE=0 (it
// would silence the continuous stream this firmware's entire read
// pipeline depends on), so — same as that fork's `AT+MODE` precedent —
// it is simply never a value this formatter can produce, not filtered
// out from a larger allowed set.
constexpr int kRateCentihertz[11] = {2, 5, 10, 20, 50, 100, 200, 500, 1000, 1250, 2000};

// Formats an RRATE register write (0xFF 0xAA 0x03 <code> 0x00).
// `requested_centihertz` is snapped to the nearest of
// kRateCentihertz's eleven supported values (ties broken toward the
// lower rate) rather than rejected, same clamp-to-nearest philosophy as
// hwt901b_bandwidth_command.h. Writes exactly 5 bytes into buf (buf_len
// must be >= 5) and returns the rate (centihertz) actually selected.
//
// Recommended minimum: 1000 (10 Hz) — below that, PGN 127250/127251's
// helm/autopilot-facing refresh rate loses meaningful resolution, same
// guidance the HWT3100 fork gave for AT+PRATE. Not enforced as a hard
// floor; the lowest value in kRateCentihertz remains reachable.
int FormatRateCommand(int requested_centihertz, uint8_t* buf, size_t buf_len);

}  // namespace halser

#endif  // HALSER_SRC_HWT901B_RATE_COMMAND_H_

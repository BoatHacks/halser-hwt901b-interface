#ifndef HALSER_SRC_HWT901B_BAUD_COMMAND_H_
#define HALSER_SRC_HWT901B_BAUD_COMMAND_H_

#include <cstddef>
#include <cstdint>

namespace halser {

// Sentinel meaning "not yet known" for the persisted WT901B UART baud
// rate (gateway.cpp) — never sent to the module or passed to
// Serial1.begin(). Mirrors kBaudUnknown from the HWT3100 fork.
constexpr int kBaudUnknown = -1;

// The BAUD register (0x04) supported rates, per WitMotion's register
// protocol documentation (§11: unverified against real hardware in this
// environment). Index into this array is the register code (0-6).
constexpr int kBaudRates[7] = {4800, 9600, 19200, 38400, 57600, 115200, 230400};

// Formats a BAUD register write (0xFF 0xAA 0x04 <code> 0x00) selecting
// one of kBaudRates's seven supported baud rates. `requested_baud` is
// snapped to the nearest supported rate (ties broken toward the lower
// rate) rather than rejected — same clamp-to-nearest philosophy as the
// HWT3100 fork's AT+UART formatter. Writes exactly 5 bytes into buf
// (buf_len must be >= 5) and returns the baud rate actually selected, so
// the caller knows what to reconfigure Serial1 to.
int FormatBaudCommand(int requested_baud, uint8_t* buf, size_t buf_len);

}  // namespace halser

#endif  // HALSER_SRC_HWT901B_BAUD_COMMAND_H_

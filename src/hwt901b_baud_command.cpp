#include "hwt901b_baud_command.h"

namespace halser {

int FormatBaudCommand(int requested_baud, uint8_t* buf, size_t buf_len) {
  int best_index = 0;
  long best_distance = -1;
  for (int i = 0; i < 7; i++) {
    long distance = requested_baud > kBaudRates[i]
                         ? static_cast<long>(requested_baud) - kBaudRates[i]
                         : static_cast<long>(kBaudRates[i]) - requested_baud;
    if (best_distance < 0 || distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }

  if (buf != nullptr && buf_len >= 5) {
    buf[0] = 0xFF;
    buf[1] = 0xAA;
    buf[2] = 0x04;  // BAUD register
    buf[3] = static_cast<uint8_t>(best_index);
    buf[4] = 0x00;
  }

  return kBaudRates[best_index];
}

}  // namespace halser

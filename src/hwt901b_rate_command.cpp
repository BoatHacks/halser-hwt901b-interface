#include "hwt901b_rate_command.h"

namespace halser {

int FormatRateCommand(int requested_centihertz, uint8_t* buf, size_t buf_len) {
  int best_index = 0;
  long best_distance = -1;
  for (int i = 0; i < 11; i++) {
    long distance = requested_centihertz > kRateCentihertz[i]
                         ? static_cast<long>(requested_centihertz) - kRateCentihertz[i]
                         : static_cast<long>(kRateCentihertz[i]) - requested_centihertz;
    if (best_distance < 0 || distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }

  // RRATE codes 0x01..0x0B map 1:1, in order, to kRateCentihertz's
  // eleven ascending values (per WitMotion's register table, §11).
  uint8_t code = static_cast<uint8_t>(best_index + 1);

  if (buf != nullptr && buf_len >= 5) {
    buf[0] = 0xFF;
    buf[1] = 0xAA;
    buf[2] = 0x03;  // RRATE register
    buf[3] = code;
    buf[4] = 0x00;
  }

  return kRateCentihertz[best_index];
}

}  // namespace halser

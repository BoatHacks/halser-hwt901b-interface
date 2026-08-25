#include "hwt901b_bandwidth_command.h"

namespace halser {

int FormatBandwidthCommand(int requested_hz, uint8_t* buf, size_t buf_len) {
  int best_index = 0;
  long best_distance = -1;
  for (int i = 0; i < 7; i++) {
    long distance = requested_hz > kBandwidthHz[i]
                         ? static_cast<long>(requested_hz) - kBandwidthHz[i]
                         : static_cast<long>(kBandwidthHz[i]) - requested_hz;
    if (best_distance < 0 || distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }

  if (buf != nullptr && buf_len >= 5) {
    buf[0] = 0xFF;
    buf[1] = 0xAA;
    buf[2] = 0x1F;  // BANDWIDTH register
    buf[3] = static_cast<uint8_t>(best_index);
    buf[4] = 0x00;
  }

  return kBandwidthHz[best_index];
}

}  // namespace halser

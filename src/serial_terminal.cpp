#include "serial_terminal.h"

#include <cstdio>

#include "hwt901b_parser.h"

namespace halser {

void SerialTerminal::AddFrame(const HWT901BRawFrame& frame) {
  frames_[next_] = frame;
  next_ = (next_ + 1) % kBufferSize;
  if (count_ < kBufferSize) count_++;
}

bool SerialTerminal::to_json(JsonObject& config) {
  JsonArray lines = config["lines"].to<JsonArray>();
  // Oldest-first: once the buffer has wrapped, the oldest entry is at
  // next_ (the slot about to be overwritten); before that, it's index 0.
  size_t start_index = (count_ < kBufferSize) ? 0 : next_;
  for (size_t i = 0; i < count_; i++) {
    size_t idx = (start_index + i) % kBufferSize;
    const HWT901BRawFrame& f = frames_[idx];
    char hex[HWT901BRawFrame::kLength * 3 + 1];
    for (size_t b = 0; b < HWT901BRawFrame::kLength; b++) {
      snprintf(hex + b * 3, 4, "%02X ", f.bytes[b]);
    }
    hex[HWT901BRawFrame::kLength * 3 - 1] = '\0';

    // Decoded values (SPEC.md §8.1) alongside the hex dump -- makes it
    // possible to sanity-check what the WT901B is actually reporting
    // straight from the web UI, without also needing
    // HALSER_DEBUG_SERIAL/a serial console attached.
    char decoded[64];
    DescribeHWT901BFrame(f.bytes, HWT901BRawFrame::kLength, decoded, sizeof(decoded));

    char line[sizeof(hex) + 4 + sizeof(decoded)];
    snprintf(line, sizeof(line), "%s -- %s", hex, decoded);
    lines.add(String(line));
  }
  return true;
}

}  // namespace halser

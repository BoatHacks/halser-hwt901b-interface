#include "hwt901b_parser.h"

namespace {

int16_t ReadInt16LE(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}

// Scale factors per WitMotion's register protocol documentation.
constexpr float kAngleScale = 180.0f / 32768.0f;       // §11: unverified
constexpr float kGyroScale = 2000.0f / 32768.0f;       // §11: unverified

}  // namespace

bool ParseHWT901BFrame(const uint8_t* frame, size_t len, ImuReading* out) {
  if (frame == nullptr || out == nullptr) return false;
  if (len != HWT901BRawFrame::kLength) return false;
  if (frame[0] != 0x55) return false;

  uint8_t sum = 0;
  for (size_t i = 0; i < 10; i++) sum = static_cast<uint8_t>(sum + frame[i]);
  if (sum != frame[10]) return false;

  const uint8_t type = frame[1];
  const uint8_t* data = frame + 2;  // 8 data bytes

  switch (static_cast<HWT901BPacketType>(type)) {
    case HWT901BPacketType::kAcceleration:
      // Acceleration isn't part of ImuReading (SPEC.md §3 — not consumed
      // by anything this firmware outputs); still a recognized/valid
      // frame type, so this returns true with *out unchanged.
      return true;

    case HWT901BPacketType::kAngularVelocity: {
      int16_t wz = ReadInt16LE(data + 4);
      out->gyro_z = static_cast<float>(wz) * kGyroScale;
      return true;
    }

    case HWT901BPacketType::kAngle: {
      int16_t roll = ReadInt16LE(data + 0);
      int16_t pitch = ReadInt16LE(data + 2);
      int16_t yaw = ReadInt16LE(data + 4);
      out->roll = static_cast<float>(roll) * kAngleScale;
      out->pitch = static_cast<float>(pitch) * kAngleScale;
      out->heading = static_cast<float>(yaw) * kAngleScale;
      return true;
    }

    case HWT901BPacketType::kMagnetic: {
      int16_t hx = ReadInt16LE(data + 0);
      int16_t hy = ReadInt16LE(data + 2);
      int16_t hz = ReadInt16LE(data + 4);
      out->mag_x = hx;
      out->mag_y = hy;
      out->mag_z = hz;
      return true;
    }

    default:
      return false;
  }
}

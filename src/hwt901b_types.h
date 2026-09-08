#ifndef HALSER_SRC_HWT901B_TYPES_H_
#define HALSER_SRC_HWT901B_TYPES_H_

#include <cstddef>
#include <cstdint>

// See ARCHITECTURE.md §3. Unlike the HWT3100-TTL/232 (compass-only), the
// WT901B is a full 9-axis IMU: roll/pitch/heading (from its internal
// AHRS fusion) plus a real gyroscope, so rate of turn no longer needs to
// be derived (SPEC.md §1.3).
struct ImuReading {
  float heading = 0.0f;  // degrees, 0-360, magnetic, offset-corrected
  float roll = 0.0f;     // degrees, -180..180, positive = starboard down
  float pitch = 0.0f;    // degrees, -90..90, positive = bow up
  float gyro_x = 0.0f;   // degrees/second, raw roll-axis angular rate, diagnostic use only
  float gyro_y = 0.0f;   // degrees/second, raw pitch-axis angular rate, diagnostic use only
  float gyro_z = 0.0f;   // degrees/second, raw yaw-axis angular rate (§1.3) -- feeds rate of turn
  float accel_x = 0.0f;  // g, raw accelerometer X, diagnostic use only
  float accel_y = 0.0f;  // g, raw accelerometer Y, diagnostic use only
  float accel_z = 0.0f;  // g, raw accelerometer Z, diagnostic use only
  int32_t mag_x = 0;     // raw magnetic field X, diagnostic use only
  int32_t mag_y = 0;     // raw magnetic field Y, diagnostic use only
  int32_t mag_z = 0;     // raw magnetic field Z, diagnostic use only
  float pressure_pa = 0.0f;  // Pascals, atmospheric pressure from the 0x56 packet
  unsigned long timestamp = 0;  // millis() of last packet contributing here
};

// The complete, deliberately small set of commands this firmware will
// ever send to the WT901B. There is no value here for a factory-reset
// SAVE-register write — see SPEC.md §1.2/§2 and ARCHITECTURE.md §6 for
// why that's permanent, the same way AT+MODE was permanently excluded
// from the HWT3100 fork this project is adapted from.
enum class HWT901BCommand {
  kStartMagCalibration,  // CALSW=7
  kStopMagCalibration,   // CALSW=0 (also persists via SAVE=save-current)
};

// A raw 11-byte packet tapped from the WT901B's serial output, for the
// live serial terminal (SPEC.md §8.1). Fixed-size POD rather than
// Arduino::String so it can cross the FreeRTOS task boundary via
// TaskQueueProducer without heap allocation.
struct HWT901BRawFrame {
  static constexpr size_t kLength = 11;
  uint8_t bytes[kLength] = {0};
};

#endif  // HALSER_SRC_HWT901B_TYPES_H_

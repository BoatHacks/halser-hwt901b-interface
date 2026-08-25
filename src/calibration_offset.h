#ifndef HALSER_SRC_CALIBRATION_OFFSET_H_
#define HALSER_SRC_CALIBRATION_OFFSET_H_

#include <cmath>

#include "hwt901b_types.h"

namespace halser {

// Applies the configured heading calibration offset (SPEC.md §2, §9) to
// an ImuReading, wrapping the result into [0, 360). Pure function — mag
// field/roll/pitch/gyro pass through unchanged; only heading gets the
// mounting-misalignment correction (SPEC.md §5.2, §9).
//
// The sign convention below (negate the raw yaw before applying the
// offset) carries over the HWT3100 fork's confirmed-on-real-hardware
// finding for *that* module (raw Yaw increases counterclockwise, the
// opposite of N2K/SignalK's clockwise-positive convention). Whether the
// WT901B's AHRS-fused yaw output uses the same convention is
// **unverified** in this environment — SPEC.md §11 flags this
// explicitly, since the HWT3100 fork's own history is a direct warning
// that this kind of manual/assumption-derived detail has been wrong
// before until checked against a real device.
inline ImuReading ApplyCalibrationOffset(const ImuReading& reading,
                                          float heading_offset_degrees) {
  ImuReading corrected = reading;
  float heading = fmodf(-reading.heading + heading_offset_degrees, 360.0f);
  if (heading < 0.0f) heading += 360.0f;
  corrected.heading = heading;
  return corrected;
}

}  // namespace halser

#endif  // HALSER_SRC_CALIBRATION_OFFSET_H_

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
// opposite of N2K/SignalK's clockwise-positive convention). Confirmed
// on a physical WT901B (SPEC.md §10, §11): rotating the module
// clockwise while watching the raw parsed heading showed it decreasing
// (counterclockwise-positive), the same convention as the HWT3100 —
// so the negation applies here too, not just carried over unverified.
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

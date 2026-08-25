#ifndef HALSER_SRC_HWT901B_CALIBRATION_COMMANDS_H_
#define HALSER_SRC_HWT901B_CALIBRATION_COMMANDS_H_

#include "hwt901b_serial.h"
#include "hwt901b_types.h"

namespace halser {

// The only component that calls HWT901BSerialIO::SendCommand() to
// actually decide when a calibration command fires (SPEC.md §8.2,
// ARCHITECTURE.md §2.2, §6). Named actions only, one method per
// HWT901BCommand value — there is no method here, or anywhere in this
// firmware, that forwards arbitrary/user-supplied register writes to the
// module.
//
// Small enough to stay header-only rather than split into a .cpp: each
// method is a single call straight through to SendCommand().
class CalibrationCommandHandler {
 public:
  explicit CalibrationCommandHandler(HWT901BSerialIO* serial_io)
      : serial_io_(serial_io) {}

  void StartCalibration() {
    serial_io_->SendCommand(HWT901BCommand::kStartMagCalibration);
  }

  void EndCalibration() {
    serial_io_->SendCommand(HWT901BCommand::kStopMagCalibration);
  }

 private:
  HWT901BSerialIO* serial_io_;
};

}  // namespace halser

#endif  // HALSER_SRC_HWT901B_CALIBRATION_COMMANDS_H_

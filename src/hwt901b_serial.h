#ifndef HALSER_SRC_HWT901B_SERIAL_H_
#define HALSER_SRC_HWT901B_SERIAL_H_

#include <HardwareSerial.h>

#include "hwt901b_types.h"
#include "sensesp/system/task_queue_producer.h"

namespace halser {

// Owns the WT901B's UART link exclusively (SPEC.md §2, ARCHITECTURE.md
// §2.1, §6). No other component in this firmware should hold a
// reference to the underlying HardwareSerial.
//
// Runs a dedicated FreeRTOS task that reads the module's continuous
// 11-byte binary packet stream, parses it via ParseHWT901BFrame()
// (hwt901b_parser.h), accumulates it into one running ImuReading (the
// module emits acceleration/gyro/angle/magnetic as separate packets, not
// one combined record — see hwt901b_parser.h), and marshals results to
// the main SensESP loop. Also exposes the *only* write paths to the
// module: SendCommand() (two closed HWT901BCommand values),
// SetBandwidth() (BANDWIDTH register, a bounds-clamped value),
// SetRate() (RRATE register, same clamping approach), and SetBaudRate()
// (BAUD register, snapped to the nearest of seven supported rates —
// SPEC.md §8.2a/§8.2b/§8.2c, §9). There is deliberately no method here
// that accepts a raw/arbitrary register write. DetectBaud() is the one
// method here that writes nothing at all — pure passive listening, used
// only before Begin() starts the background task.
class HWT901BSerialIO {
 public:
  // imu_producer/raw_frame_producer must outlive this object and the
  // FreeRTOS task started by Begin(). They're owned by the caller
  // (gateway.cpp) because constructing a TaskQueueProducer requires the
  // main event loop, which this class has no business knowing about.
  HWT901BSerialIO(HardwareSerial& serial,
                   sensesp::TaskQueueProducer<ImuReading>* imu_producer,
                   sensesp::TaskQueueProducer<HWT901BRawFrame>* raw_frame_producer);

  // Starts the serial port and the dedicated read task. Call once, from
  // the main setup path.
  void Begin(unsigned long baud, int rx_pin, int tx_pin);

  // Transmits one of HWT901BCommand's two known-safe values — no
  // overload, no debug backdoor (ARCHITECTURE.md §6). kStopMagCalibration
  // also writes SAVE=save-current immediately after CALSW=0, so the
  // completed calibration actually persists (SPEC.md §11 flags whether
  // this is strictly necessary as unverified).
  void SendCommand(HWT901BCommand cmd);

  // Writes the BANDWIDTH register, snapped to the nearest of
  // hwt901b_bandwidth_command.h's seven supported values — the WT901B
  // analog of the HWT3100 fork's AT+FILT (SPEC.md §8.2a).
  void SetBandwidth(int hz);

  // Writes the RRATE register, snapped to the nearest of
  // hwt901b_rate_command.h's eleven supported values (SPEC.md §8.2b).
  void SetRate(int centihertz);

  // Synchronous, blocking baud-rate discovery (SPEC.md §8.2c). Tries
  // each of candidate_bauds[0..num_candidates) in order: opens the port
  // at that rate and listens up to per_baud_timeout_ms for at least one
  // frame that parses as valid WT901B output. Returns true and fills
  // *detected_baud on the first candidate that works, false if none do.
  // Sends nothing — passive listening, not reconfiguration. Must be
  // called before Begin() starts the background read task.
  bool DetectBaud(const int* candidate_bauds, size_t num_candidates,
                   unsigned long per_baud_timeout_ms, int rx_pin, int tx_pin,
                   int* detected_baud);

  // Writes the BAUD register (via FormatBaudCommand(),
  // hwt901b_baud_command.h, which snaps requested_baud to the nearest of
  // the seven supported rates) at whatever rate the port is currently
  // open at, waits a fixed settle delay, then reconfigures the port to
  // the new rate. Returns the baud rate actually selected/switched to.
  // See SPEC.md §8.2c and §11 for the unverified timing assumptions this
  // makes.
  int SetBaudRate(int requested_baud, int rx_pin, int tx_pin);

 private:
  static void ReadTaskTrampoline(void* arg);
  void ReadTaskLoop();

  HardwareSerial& serial_;
  sensesp::TaskQueueProducer<ImuReading>* imu_producer_;
  sensesp::TaskQueueProducer<HWT901BRawFrame>* raw_frame_producer_;

  uint8_t frame_buffer_[HWT901BRawFrame::kLength];
  size_t frame_length_ = 0;
  ImuReading accumulated_reading_;
};

}  // namespace halser

#endif  // HALSER_SRC_HWT901B_SERIAL_H_

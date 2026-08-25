#include "hwt901b_serial.h"

#include <Arduino.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>

#include "hwt901b_bandwidth_command.h"
#include "hwt901b_baud_command.h"
#include "hwt901b_parser.h"
#include "hwt901b_rate_command.h"

namespace halser {

namespace {

// ESP_LOGD output is both written to the USB serial console and
// captured into SensESP's in-memory LogBuffer -- makes every packet
// this firmware sees visible for diagnosing a misbehaving/miswired
// WT901B, same rationale as the HWT3100 fork's per-line logging.
constexpr const char* kSerialLogTag = "hwt901b_serial";

// CALSW register write for each HWT901BCommand value (SPEC.md §8.2).
// This table is the entire calibration-trigger write surface of this
// firmware to the WT901B -- there is deliberately no entry, and no
// possible path to construct one, for a factory-reset SAVE-register
// write.
void CalibrationCommandBytes(HWT901BCommand cmd, uint8_t out[5]) {
  out[0] = 0xFF;
  out[1] = 0xAA;
  out[2] = 0x01;  // CALSW register
  switch (cmd) {
    case HWT901BCommand::kStartMagCalibration:
      out[3] = 0x07;
      break;
    case HWT901BCommand::kStopMagCalibration:
      out[3] = 0x00;
      break;
  }
  out[4] = 0x00;
}

constexpr int kReadTaskStackSize = 4096;
constexpr UBaseType_t kReadTaskPriority = 1;
constexpr TickType_t kReadTaskPollDelay = pdMS_TO_TICKS(5);

}  // namespace

HWT901BSerialIO::HWT901BSerialIO(
    HardwareSerial& serial,
    sensesp::TaskQueueProducer<ImuReading>* imu_producer,
    sensesp::TaskQueueProducer<HWT901BRawFrame>* raw_frame_producer)
    : serial_(serial),
      imu_producer_(imu_producer),
      raw_frame_producer_(raw_frame_producer) {}

void HWT901BSerialIO::Begin(unsigned long baud, int rx_pin, int tx_pin) {
  serial_.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
  xTaskCreate(&HWT901BSerialIO::ReadTaskTrampoline, "hwt901b_read",
              kReadTaskStackSize, this, kReadTaskPriority, nullptr);
}

void HWT901BSerialIO::SendCommand(HWT901BCommand cmd) {
  uint8_t bytes[5];
  CalibrationCommandBytes(cmd, bytes);
  serial_.write(bytes, sizeof(bytes));

  if (cmd == HWT901BCommand::kStopMagCalibration) {
    // Persist the just-completed calibration (SPEC.md §11: whether this
    // is strictly required, vs. automatic on CALSW=0, isn't confirmed
    // against a real unit).
    const uint8_t save[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    serial_.write(save, sizeof(save));
  }
}

void HWT901BSerialIO::SetBandwidth(int hz) {
  uint8_t buf[5];
  FormatBandwidthCommand(hz, buf, sizeof(buf));
  serial_.write(buf, sizeof(buf));
}

void HWT901BSerialIO::SetRate(int centihertz) {
  uint8_t buf[5];
  FormatRateCommand(centihertz, buf, sizeof(buf));
  serial_.write(buf, sizeof(buf));
}

bool HWT901BSerialIO::DetectBaud(const int* candidate_bauds,
                                  size_t num_candidates,
                                  unsigned long per_baud_timeout_ms, int rx_pin,
                                  int tx_pin, int* detected_baud) {
  for (size_t i = 0; i < num_candidates; i++) {
    int baud = candidate_bauds[i];
    serial_.begin(baud, SERIAL_8N1, rx_pin, tx_pin);

    uint8_t buf[HWT901BRawFrame::kLength];
    size_t len = 0;
    bool found = false;
    unsigned long deadline = millis() + per_baud_timeout_ms;

    while (!found && millis() < deadline) {
      while (serial_.available()) {
        uint8_t b = static_cast<uint8_t>(serial_.read());

        if (len == 0 && b != 0x55) continue;

        buf[len++] = b;

        if (len == HWT901BRawFrame::kLength) {
          ImuReading reading;
          if (ParseHWT901BFrame(buf, len, &reading)) {
            found = true;
            break;
          }
          len = 0;
        }
      }
      if (!found) {
        vTaskDelay(pdMS_TO_TICKS(5));
      }
    }

    serial_.end();

    if (found) {
      *detected_baud = baud;
      return true;
    }
  }
  return false;
}

int HWT901BSerialIO::SetBaudRate(int requested_baud, int rx_pin, int tx_pin) {
  // Settle delay between sending the BAUD register write and
  // reconfiguring our own side, chosen as a reasonable fixed value
  // (SPEC.md §11 — the register protocol docs don't specify a timing
  // spec for this transition).
  constexpr unsigned long kBaudSwitchSettleMs = 200;

  uint8_t buf[5];
  int actual_baud = FormatBaudCommand(requested_baud, buf, sizeof(buf));
  serial_.write(buf, sizeof(buf));
  delay(kBaudSwitchSettleMs);
  serial_.begin(actual_baud, SERIAL_8N1, rx_pin, tx_pin);
  return actual_baud;
}

void HWT901BSerialIO::ReadTaskTrampoline(void* arg) {
  static_cast<HWT901BSerialIO*>(arg)->ReadTaskLoop();
}

void HWT901BSerialIO::ReadTaskLoop() {
  for (;;) {
    while (serial_.available()) {
      uint8_t b = static_cast<uint8_t>(serial_.read());

      // Frame sync: only start accumulating on a header byte. A stray
      // 0x55 inside a still-incomplete frame's data bytes will fail the
      // checksum below and simply drop that frame — a known
      // simplification (no mid-frame resync), same class of trade-off
      // the HWT3100 fork's line parser made for oversized lines.
      if (frame_length_ == 0 && b != 0x55) continue;

      frame_buffer_[frame_length_++] = b;

      if (frame_length_ == HWT901BRawFrame::kLength) {
        ESP_LOGD(kSerialLogTag, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                  frame_buffer_[0], frame_buffer_[1], frame_buffer_[2],
                  frame_buffer_[3], frame_buffer_[4], frame_buffer_[5],
                  frame_buffer_[6], frame_buffer_[7], frame_buffer_[8],
                  frame_buffer_[9], frame_buffer_[10]);

        if (ParseHWT901BFrame(frame_buffer_, frame_length_, &accumulated_reading_)) {
          accumulated_reading_.timestamp = millis();
          if (imu_producer_ != nullptr) {
            imu_producer_->set(accumulated_reading_);
          }
        }

        if (raw_frame_producer_ != nullptr) {
          HWT901BRawFrame raw;
          memcpy(raw.bytes, frame_buffer_, HWT901BRawFrame::kLength);
          raw_frame_producer_->set(raw);
        }

        frame_length_ = 0;
      }
    }
    vTaskDelay(kReadTaskPollDelay);
  }
}

}  // namespace halser

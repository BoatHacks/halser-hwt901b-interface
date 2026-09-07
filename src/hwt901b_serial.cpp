#include "hwt901b_serial.h"

// Forces ESP_LOGD/ESP_LOGI in this file to actually compile in and be
// eligible to print, regardless of whatever CONFIG_LOG_MAXIMUM_LEVEL
// sdkconfig.defaults ends up with — this file's entire debug-logging
// purpose (SPEC.md §8.1, ARCHITECTURE.md §2.1) would silently vanish at
// compile time otherwise. Must be defined before the esp_log.h include
// below.
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <Arduino.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>
#include <cstring>

#include "hwt901b_bandwidth_command.h"
#include "hwt901b_baud_command.h"
#include "hwt901b_parser.h"
#include "hwt901b_rate_command.h"

namespace halser {

namespace {

// ESP_LOGD/ESP_LOGI output is both written to the USB serial console
// and captured into SensESP's in-memory LogBuffer -- makes every step
// of serial setup (baud autodetection, register writes) and every byte
// this firmware sends/receives visible for diagnosing a misbehaving/
// miswired WT901B, same rationale as the HWT3100 fork's per-line
// logging (SPEC.md §8.1).
constexpr const char* kSerialLogTag = "hwt901b_serial";

// Renders `len` bytes as a space-separated uppercase hex string into
// `out` ("55 53 00 ..."). `out` must be at least 3*len bytes (2 hex
// digits + separator per byte, no trailing separator needed since the
// last iteration's space is overwritten by the terminating NUL).
void FormatHex(const uint8_t* bytes, size_t len, char* out, size_t out_len) {
  size_t pos = 0;
  for (size_t i = 0; i < len && pos + 3 <= out_len; i++) {
    pos += snprintf(out + pos, out_len - pos, "%02X ", bytes[i]);
  }
  if (pos > 0 && out[pos - 1] == ' ') out[pos - 1] = '\0';
  if (pos == 0 && out_len > 0) out[0] = '\0';
}

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

const char* CommandDescription(HWT901BCommand cmd) {
  switch (cmd) {
    case HWT901BCommand::kStartMagCalibration:
      return "CALSW=7 (start magnetic calibration)";
    case HWT901BCommand::kStopMagCalibration:
      return "CALSW=0 (stop magnetic calibration)";
  }
  return "unknown";
}

// Logs one outgoing register-write frame at DEBUG, hex + description,
// then actually writes it. Every write this firmware ever makes to the
// WT901B goes through this, so every one is logged the same way.
void WriteAndLog(HardwareSerial& serial, const uint8_t* bytes, size_t len,
                  const char* description) {
  char hex[32];
  FormatHex(bytes, len, hex, sizeof(hex));
  ESP_LOGD(kSerialLogTag, "TX: %s -- %s", hex, description);
  serial.write(bytes, len);
}

// Logs the decoded interpretation of one just-parsed frame, based on
// its packet type byte (frame[1]) and the fields ParseHWT901BFrame()
// filled in *reading for that type (hwt901b_parser.h).
void LogDecoded(uint8_t type, const ImuReading& reading) {
  switch (type) {
    case 0x51:  // Acceleration -- recognized, not captured (hwt901b_parser.h)
      ESP_LOGD(kSerialLogTag, "RX decoded: 0x51 Acceleration (not captured)");
      break;
    case 0x52:  // Angular velocity / gyro
      ESP_LOGD(kSerialLogTag, "RX decoded: 0x52 Gyro: gyro_z=%.2f deg/s",
                reading.gyro_z);
      break;
    case 0x53:  // Angle
      ESP_LOGD(kSerialLogTag,
                "RX decoded: 0x53 Angle: heading=%.2f roll=%.2f pitch=%.2f (deg)",
                reading.heading, reading.roll, reading.pitch);
      break;
    case 0x54:  // Magnetic field
      ESP_LOGD(kSerialLogTag, "RX decoded: 0x54 Magnetic: x=%ld y=%ld z=%ld (raw counts)",
                static_cast<long>(reading.mag_x), static_cast<long>(reading.mag_y),
                static_cast<long>(reading.mag_z));
      break;
    default:
      ESP_LOGD(kSerialLogTag, "RX decoded: unrecognized type 0x%02X", type);
      break;
  }
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
  esp_log_level_set(kSerialLogTag, ESP_LOG_DEBUG);
  ESP_LOGI(kSerialLogTag, "Serial setup: opening Serial1 at %lu 8N1 (rx=%d tx=%d)",
            baud, rx_pin, tx_pin);
  serial_.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
  xTaskCreate(&HWT901BSerialIO::ReadTaskTrampoline, "hwt901b_read",
              kReadTaskStackSize, this, kReadTaskPriority, nullptr);
  ESP_LOGI(kSerialLogTag, "Serial setup: read task started");
}

void HWT901BSerialIO::SendCommand(HWT901BCommand cmd) {
  uint8_t bytes[5];
  CalibrationCommandBytes(cmd, bytes);
  WriteAndLog(serial_, bytes, sizeof(bytes), CommandDescription(cmd));

  if (cmd == HWT901BCommand::kStopMagCalibration) {
    // Persist the just-completed calibration (SPEC.md §11: whether this
    // is strictly required, vs. automatic on CALSW=0, isn't confirmed
    // against a real unit).
    const uint8_t save[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    WriteAndLog(serial_, save, sizeof(save), "SAVE=save-current");
  }
}

void HWT901BSerialIO::SetBandwidth(int hz) {
  uint8_t buf[5];
  int actual = FormatBandwidthCommand(hz, buf, sizeof(buf));
  char desc[48];
  snprintf(desc, sizeof(desc), "BANDWIDTH=%d Hz (requested %d)", actual, hz);
  WriteAndLog(serial_, buf, sizeof(buf), desc);
}

void HWT901BSerialIO::SetRate(int centihertz) {
  uint8_t buf[5];
  int actual = FormatRateCommand(centihertz, buf, sizeof(buf));
  char desc[64];
  snprintf(desc, sizeof(desc), "RRATE=%d centihertz (requested %d)", actual,
            centihertz);
  WriteAndLog(serial_, buf, sizeof(buf), desc);
}

bool HWT901BSerialIO::DetectBaud(const int* candidate_bauds,
                                  size_t num_candidates,
                                  unsigned long per_baud_timeout_ms, int rx_pin,
                                  int tx_pin, int* detected_baud) {
  ESP_LOGI(kSerialLogTag, "Autobaud: starting, %u candidate(s), %lums each",
            static_cast<unsigned>(num_candidates), per_baud_timeout_ms);

  for (size_t i = 0; i < num_candidates; i++) {
    int baud = candidate_bauds[i];
    ESP_LOGI(kSerialLogTag, "Autobaud: trying %d baud (candidate %u/%u)", baud,
              static_cast<unsigned>(i + 1), static_cast<unsigned>(num_candidates));
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
          char hex[32];
          FormatHex(buf, len, hex, sizeof(hex));

          ImuReading reading;
          if (ParseHWT901BFrame(buf, len, &reading)) {
            ESP_LOGI(kSerialLogTag,
                      "Autobaud: valid frame at %d baud (checksum OK): %s", baud,
                      hex);
            found = true;
            break;
          }
          ESP_LOGD(kSerialLogTag,
                    "Autobaud: frame at %d baud failed checksum, discarding: %s",
                    baud, hex);
          len = 0;
        }
      }
      if (!found) {
        vTaskDelay(pdMS_TO_TICKS(5));
      }
    }

    serial_.end();

    if (found) {
      ESP_LOGI(kSerialLogTag, "Autobaud: locked at %d baud", baud);
      *detected_baud = baud;
      return true;
    }
    ESP_LOGI(kSerialLogTag, "Autobaud: no valid frame seen at %d baud within %lums",
              baud, per_baud_timeout_ms);
  }
  ESP_LOGI(kSerialLogTag, "Autobaud: exhausted all candidates, none succeeded");
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
  char desc[64];
  snprintf(desc, sizeof(desc), "BAUD=%d (requested %d)", actual_baud, requested_baud);
  WriteAndLog(serial_, buf, sizeof(buf), desc);

  ESP_LOGI(kSerialLogTag, "Baud switch: waiting %lums settle delay, then reopening at %d",
            kBaudSwitchSettleMs, actual_baud);
  delay(kBaudSwitchSettleMs);
  serial_.begin(actual_baud, SERIAL_8N1, rx_pin, tx_pin);
  ESP_LOGI(kSerialLogTag, "Baud switch: Serial1 reopened at %d 8N1", actual_baud);
  return actual_baud;
}

void HWT901BSerialIO::ReadTaskTrampoline(void* arg) {
  static_cast<HWT901BSerialIO*>(arg)->ReadTaskLoop();
}

void HWT901BSerialIO::ReadTaskLoop() {
  ESP_LOGI(kSerialLogTag, "Read task running");
  for (;;) {
    while (serial_.available()) {
      uint8_t b = static_cast<uint8_t>(serial_.read());

      // Frame sync: only start accumulating on a header byte. A stray
      // 0x55 inside a still-incomplete frame's data bytes will fail the
      // checksum below and simply drop that frame — a known
      // simplification (no mid-frame resync), same class of trade-off
      // the HWT3100 fork's line parser made for oversized lines.
      if (frame_length_ == 0 && b != 0x55) {
        ESP_LOGD(kSerialLogTag, "RX: dropped stray byte %02X (not a frame header)", b);
        continue;
      }

      frame_buffer_[frame_length_++] = b;

      if (frame_length_ == HWT901BRawFrame::kLength) {
        char hex[36];
        FormatHex(frame_buffer_, HWT901BRawFrame::kLength, hex, sizeof(hex));
        ESP_LOGD(kSerialLogTag, "RX: %s", hex);

        uint8_t type = frame_buffer_[1];
        if (ParseHWT901BFrame(frame_buffer_, frame_length_, &accumulated_reading_)) {
          accumulated_reading_.timestamp = millis();
          LogDecoded(type, accumulated_reading_);
          if (imu_producer_ != nullptr) {
            imu_producer_->set(accumulated_reading_);
          }
        } else {
          ESP_LOGD(kSerialLogTag,
                    "RX: frame rejected (bad header/checksum/type): %s", hex);
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

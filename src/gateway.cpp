#include "gateway.h"

#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <esp_log.h>
#include <esp_mac.h>

#include <cstdio>

#include "calibration_offset.h"
#include "halser_const.h"
#include "hwt901b_bandwidth_command.h"
#include "hwt901b_baud_command.h"
#include "hwt901b_calibration_commands.h"
#include "hwt901b_rate_command.h"
#include "hwt901b_serial.h"
#include "hwt901b_types.h"
#include "magnetic_variation_listener.h"
#include "mfd_calibration_bridge.h"
#include "n2k_senders.h"
#include "sensesp/signalk/signalk_metadata.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/system/task_queue_producer.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/ui_button.h"
#include "sensesp_app_builder.h"
#include "serial_terminal.h"

using namespace sensesp;

namespace {

tNMEA2000* nmea2000 = nullptr;

// PGNs this firmware actually transmits, beyond the NMEA2000-library's
// own boilerplate (address claim, heartbeat, product/config info, which
// it reports automatically). Passed to ExtendTransmitMessages() so PGN
// 126464 ("PGN List - Transmit") queries — and any MFD/tool that uses
// that list to decide what data sources a device offers — see all four,
// not just the boilerplate set. Unlike the HWT3100 fork this project is
// adapted from, PGN 127257 (Attitude) is included: the WT901B's real
// accelerometer makes this an honest addition (SPEC.md §5.1, §9.3).
// PGN 130314 (Actual Pressure) is sourced from the WT901B's barometer
// (SPEC.md §3, §5.1). 0-terminated per the library's own convention;
// must outlive the call (the library stores the pointer, not a copy),
// hence file-scope rather than local.
const unsigned long kTransmitMessages[] PROGMEM = {127250L, 127251L, 127257L, 130314L, 0};

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegreesToRadians = kPi / 180.0f;

// Converts the WT901B's raw accelerometer readings (g) to SI units for
// the diagnostic sensors.hwt901b.acceleration.* SignalK outputs
// (SPEC.md §5.2) -- unlike raw magnetic field, the accel scale factor
// is known, so there's no reason to publish non-SI units here.
constexpr float kGToMetersPerSecondSquared = 9.80665f;

// Matches N2kHeadingSender's ExpiringValue expiry (SPEC.md §6, §10): the
// SignalK meta.timeout advisory should agree with when N2K actually
// starts sending "not available."
constexpr float kHeadingTimeoutSeconds = 5.0f;

// Matches N2kHeadingSender's variation_ expiry default (SPEC.md §5.1a)
// — much longer than heading's, since bus-sourced magnetic variation
// changes on a geographic timescale and typically isn't rebroadcast
// every second the way heading is.
constexpr float kVariationTimeoutSeconds = 300.0f;
constexpr float kTwoPi = 2.0f * kPi;

// Baud auto-detection order and per-candidate timeout (SPEC.md §8.2c):
// recommended rate first (fast path for an already-configured module),
// then the module's likely factory default, then the rest of the
// supported set. 1s/candidate is a reasonable guess at how long it
// takes to see at least one full frame at the module's documented
// output rates — not derived from real-hardware timing (SPEC.md §11).
constexpr int kBaudCandidates[] = {115200, 9600, 230400, 57600, 38400, 19200, 4800};
constexpr size_t kNumBaudCandidates =
    sizeof(kBaudCandidates) / sizeof(kBaudCandidates[0]);
constexpr unsigned long kBaudDetectTimeoutMs = 1000;

// Last-resort recovery for a real bug in BoatHacks/SensESP's SKWSClient
// (confirmed on real hardware, 2026-09-08): a send-side failure in
// send_delta() -- by design (see SignalK/SensESP#1033) -- never routes
// through on_disconnected()/on_error(), the only two places that set
// connection_state_ back to kSKWSDisconnected. If the underlying
// esp_websocket_client transport dies in a way that's only ever
// observed via that send-side failure (not via its own async
// disconnect/error event), connection_state_ stays stuck at whatever
// it last was, SKWSClient::connect()'s "only run if Disconnected"
// guard silently no-ops forever, and the device never reconnects on
// its own -- confirmed to sit in this state indefinitely (>15 minutes
// observed) with zero reconnect attempts, recovering only via a manual
// power cycle. This is a genuine upstream state-desync bug, not
// something this firmware's own code can correct by calling into
// SKWSClient differently -- there's no public API to force it back to
// Disconnected. A full device reboot is the only recovery available
// from outside that class.
constexpr unsigned long kWsWatchdogCheckIntervalMs = 30000;
constexpr unsigned long kWsWatchdogRebootThresholdMs = 5 * 60 * 1000;
constexpr const char* kWsWatchdogLogTag = "ws_watchdog";
unsigned long last_ws_connected_ms = 0;

/// Used for SetDeviceInformation()'s "unique number" — deliberately NOT
/// the Precision-9 reference's hardcoded value (SPEC.md §10), so that
/// two devices running this firmware don't collide on the same N2K bus.
static uint32_t GetBoardSerialNumber() {
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  return (mac[3] << 16) | (mac[4] << 8) | mac[5];
}

}  // namespace

void run_hwt901b_gateway() {
  Serial.setTxTimeoutMs(0);
  SetupLogging(ESP_LOG_DEBUG);

  // SensESP application
  SensESPAppBuilder builder;
  auto sensesp_app = (&builder)
                          ->set_hostname("halser-hwt901b")
                          ->set_button_pin(kButtonPin)
                          ->enable_ota("halser-hwt901b")
                          ->enable_system_info_sensors()
                          ->get_app();

  // No separate RGB LED use here — SensESP's own RGBSystemStatusLed
  // (auto-instantiated from the PIN_RGB_LED build flag) already owns
  // GPIO8 to show WiFi/WebSocket connection status, with no public hook
  // to share or override it. Fault indication (SPEC.md §6) is
  // SignalK-notification-only; see docs/plans/fault-indication.md.

  // --- SignalK WebSocket watchdog (see kWsWatchdogRebootThresholdMs
  // above for why this exists) ---
  auto ws_client = sensesp_app->get_ws_client();
  last_ws_connected_ms = millis();
  event_loop()->onRepeat(kWsWatchdogCheckIntervalMs, [ws_client]() {
    if (ws_client->get_connection_status() == "Connected") {
      last_ws_connected_ms = millis();
      return;
    }
    unsigned long down_for_ms = millis() - last_ws_connected_ms;
    if (down_for_ms > kWsWatchdogRebootThresholdMs) {
      ESP_LOGE(kWsWatchdogLogTag,
                "SignalK WebSocket has been disconnected for %lums (over "
                "the %lums threshold) -- rebooting to recover",
                down_for_ms, kWsWatchdogRebootThresholdMs);
      delay(200);  // let the log line actually flush before reset
      ESP.restart();
    }
  });

  // --- Configuration (SPEC.md §7) ---

  auto n2k_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/n2k/enabled");
  ConfigItem(n2k_enabled)
      ->set_title("Enable NMEA 2000 Output")
      ->set_sort_order(100)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto n2k_heading_pgn_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/n2k/heading_pgn_enabled");
  ConfigItem(n2k_heading_pgn_enabled)
      ->set_title("Enable PGN 127250 (Vessel Heading)")
      ->set_sort_order(110)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto n2k_rate_of_turn_pgn_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/n2k/rate_of_turn_pgn_enabled");
  ConfigItem(n2k_rate_of_turn_pgn_enabled)
      ->set_title("Enable PGN 127251 (Rate of Turn)")
      ->set_description(
          "Sourced from the WT901B's real gyroscope (SPEC.md §5.1) — unlike the HWT3100 fork this project is adapted from, this is sensed, not computed.")
      ->set_sort_order(111)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto n2k_attitude_pgn_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/n2k/attitude_pgn_enabled");
  ConfigItem(n2k_attitude_pgn_enabled)
      ->set_title("Enable PGN 127257 (Attitude)")
      ->set_description(
          "Roll/pitch from the WT901B's accelerometer (SPEC.md §5.1). Yaw is always \"not available\" — heading is already carried by PGN 127250.")
      ->set_sort_order(112)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto n2k_pressure_pgn_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/n2k/pressure_pgn_enabled");
  ConfigItem(n2k_pressure_pgn_enabled)
      ->set_title("Enable PGN 130314 (Actual Pressure)")
      ->set_description(
          "Atmospheric pressure from the WT901B's barometer (SPEC.md §5.1).")
      ->set_sort_order(113)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto signalk_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/signalk/enabled");
  ConfigItem(signalk_enabled)
      ->set_title("Enable SignalK Output")
      ->set_sort_order(120)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  // Per-delta toggles (SPEC.md §5.2, §7), same pattern as the N2K
  // per-PGN toggles above — each independently gated on top of the
  // signalk_enabled master switch, not a replacement for it.
  auto sk_heading_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/signalk/heading_enabled");
  ConfigItem(sk_heading_enabled)
      ->set_title("Enable navigation.headingMagnetic")
      ->set_sort_order(121)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto sk_rate_of_turn_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/signalk/rate_of_turn_enabled");
  ConfigItem(sk_rate_of_turn_enabled)
      ->set_title("Enable navigation.rateOfTurn")
      ->set_description(
          "Sourced from the WT901B's real gyroscope (SPEC.md §5.1).")
      ->set_sort_order(122)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto sk_heading_true_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/signalk/heading_true_enabled");
  ConfigItem(sk_heading_true_enabled)
      ->set_title("Enable navigation.headingTrue")
      ->set_description(
          "Computed as magnetic heading + variation (SPEC.md §5.1a). Only "
          "published when a recent magnetic variation has been seen from "
          "another device on the N2K bus (PGN 127258) — this firmware has "
          "no GPS or geomagnetic model of its own.")
      ->set_sort_order(123)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto sk_attitude_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/signalk/attitude_enabled");
  ConfigItem(sk_attitude_enabled)
      ->set_title("Enable navigation.attitude")
      ->set_description(
          "Roll/pitch/yaw object (SPEC.md §5.2). Yaw is always 0 — "
          "headingMagnetic is the authoritative heading value.")
      ->set_sort_order(124)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto sk_pressure_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/signalk/pressure_enabled");
  ConfigItem(sk_pressure_enabled)
      ->set_title("Enable environment.outside.pressure")
      ->set_description(
          "Atmospheric pressure from the WT901B's barometer (SPEC.md §5.1).")
      ->set_sort_order(125)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  // Raw magnetic field as SignalK deltas (SPEC.md §5.2) — diagnostic
  // data with no established SignalK path, gated separately from
  // signalk_enabled (both must be true to publish). On by default: the
  // signalk-hwt901b-calibration plugin needs this to visualize magnetic
  // field calibration quality, which is a routine part of installing
  // this firmware, not an edge case -- unlike raw gyro/accel below,
  // which stay off by default since nothing depends on them yet.
  auto raw_mag_field_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/signalk/raw_mag_field_enabled");
  ConfigItem(raw_mag_field_enabled)
      ->set_title("Enable Raw Magnetic Field SignalK Output")
      ->set_description(
          "Publishes sensors.hwt901b.magneticField.x/y/z -- raw, "
          "uncalibrated sensor counts from the WT901B (SPEC.md §5.2). "
          "Diagnostic-only; on by default so the calibration visualizer "
          "webapp works out of the box.")
      ->set_sort_order(126)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  // Raw gyro/accelerometer as SignalK deltas -- same diagnostic-only
  // treatment as raw magnetic field above: no established SignalK path
  // for raw angular rate or acceleration on a vessel, so these are
  // custom sensors.* paths, off by default (SPEC.md §5.2).
  auto raw_gyro_enabled = std::make_shared<PersistingObservableValue<bool>>(
      false, "/signalk/raw_gyro_enabled");
  ConfigItem(raw_gyro_enabled)
      ->set_title("Enable Raw Angular Rate SignalK Output")
      ->set_description(
          "Publishes sensors.hwt901b.angularRate.x/y/z -- raw gyroscope "
          "readings from the WT901B, rad/s (SPEC.md §5.2). Diagnostic-only; "
          "z duplicates navigation.rateOfTurn. Off by default.")
      ->set_sort_order(127)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto raw_accel_enabled = std::make_shared<PersistingObservableValue<bool>>(
      false, "/signalk/raw_accel_enabled");
  ConfigItem(raw_accel_enabled)
      ->set_title("Enable Raw Acceleration SignalK Output")
      ->set_description(
          "Publishes sensors.hwt901b.acceleration.x/y/z -- raw "
          "accelerometer readings from the WT901B, m/s^2 (SPEC.md §5.2). "
          "Diagnostic-only; off by default.")
      ->set_sort_order(128)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto heading_offset = std::make_shared<PersistingObservableValue<float>>(
      0.0f, "/calibration/heading_offset");
  ConfigItem(heading_offset)
      ->set_title("Heading Calibration Offset")
      ->set_description(
          "Degrees added to the raw WT901B heading to correct for mounting misalignment")
      ->set_sort_order(50)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Offset (degrees)","type":"number"}}})schema");

  // BANDWIDTH register (SPEC.md §8.2a): on-module accel/gyro output
  // low-pass filter — the WT901B analog of the HWT3100 fork's AT+FILT.
  // Persisted so a reboot re-applies it (the module has no way to
  // report its current setting back to us).
  auto output_bandwidth = std::make_shared<PersistingObservableValue<int>>(
      256, "/hwt901b/output_bandwidth");
  ConfigItem(output_bandwidth)
      ->set_title("WT901B Output Bandwidth (Hz)")
      ->set_description(
          "On-module accel/gyro low-pass filter bandwidth. Snapped to the "
          "nearest of 256/184/94/44/21/10/5 Hz — smaller values smooth "
          "more (and lag more).")
      ->set_sort_order(55)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Bandwidth (Hz)","type":"integer","minimum":5,"maximum":256}}})schema");

  // RRATE register (SPEC.md §8.2b): on-module output data rate,
  // expressed here in centihertz (Hz x10) so 0.2 Hz stays an integer.
  // Persisted so a reboot re-applies it, same pattern as
  // output_bandwidth above.
  auto output_rate_chz = std::make_shared<PersistingObservableValue<int>>(
      1000, "/hwt901b/output_rate_chz");
  ConfigItem(output_rate_chz)
      ->set_title("WT901B Output Rate (centihertz)")
      ->set_description(
          "Module's own output rate, in tenths of a Hz (1000 = 100 Hz). "
          "Snapped to the nearest supported RRATE value. 1000 (10 Hz) is "
          "the recommended minimum for usable heading/rate-of-turn "
          "resolution.")
      ->set_sort_order(56)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Rate (centihertz)","type":"integer","minimum":2,"maximum":2000}}})schema");

  // BAUD register (SPEC.md §8.2c): the UART baud rate itself, not just a
  // module setting -- unlike output_bandwidth/output_rate_chz above,
  // changing this also has to reconfigure this firmware's own Serial1,
  // not just send a command. Starts unknown (halser::kBaudUnknown = -1);
  // startup wiring below auto-detects it by trying candidate rates in
  // turn rather than assuming, since a wrong assumption here means no
  // data ever arrives at all (not just a suboptimal setting).
  auto hwt901b_baud = std::make_shared<PersistingObservableValue<int>>(
      halser::kBaudUnknown, "/hwt901b/baud");
  ConfigItem(hwt901b_baud)
      ->set_title("WT901B UART Baud Rate")
      ->set_description(
          "-1 = not yet known; auto-detected at boot by trying 115200 "
          "(recommended), 9600, then the module's other supported rates "
          "in turn. Set to one of 4800/9600/19200/38400/57600/115200/"
          "230400 to write the BAUD register and switch the module's "
          "baud live -- the firmware reconfigures its own UART to match "
          "immediately after (values are snapped to the nearest of "
          "these seven).")
      ->set_sort_order(58)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Baud","type":"integer"}}})schema");

  // --- NMEA 2000 (CAN bus via TWAI) ---

  // Identifies as a B&G Precision-9 (SPEC.md §1.2, §5.1, §10) — product
  // info and device function/class/manufacturer code are cloned from
  // htool/ESP32_Precision-9_compass_CMPS14, carried over unchanged from
  // the HWT3100 fork this project is adapted from. The "unique number"
  // below is deliberately NOT cloned; it's derived from this board's
  // own MAC (GetBoardSerialNumber()), per the design decision in
  // SPEC.md §10.
  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);
  nmea2000->SetN2kCANSendFrameBufSize(150);
  nmea2000->SetN2kCANReceiveFrameBufSize(150);
  nmea2000->SetProductInformation(
      kProductModelSerialCode,
      kProductCode,
      kProductModelId,
      kProductSoftwareVersion,
      kProductModelVersion
  );
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // unique number — MAC-derived, not cloned
      kDeviceFunction,
      kDeviceClass,
      kManufacturerCode
  );
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 74);
  nmea2000->EnableForward(false);
  nmea2000->ExtendTransmitMessages(kTransmitMessages);
  nmea2000->Open();

  // Process N2K messages (address claim, heartbeat, etc.)
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  auto heading_sender = new halser::N2kHeadingSender(nmea2000);
  auto rate_of_turn_sender = new halser::N2kRateOfTurnSender(nmea2000);
  auto attitude_sender = new halser::N2kAttitudeSender(nmea2000);
  auto pressure_sender = new halser::N2kPressureSender(nmea2000);

  event_loop()->onRepeat(100, [heading_sender, rate_of_turn_sender, attitude_sender,
                                pressure_sender, n2k_enabled, n2k_heading_pgn_enabled,
                                n2k_rate_of_turn_pgn_enabled,
                                n2k_attitude_pgn_enabled, n2k_pressure_pgn_enabled]() {
    if (n2k_enabled->get() && n2k_heading_pgn_enabled->get()) {
      heading_sender->send();
    }
    if (n2k_enabled->get() && n2k_rate_of_turn_pgn_enabled->get()) {
      rate_of_turn_sender->send();
    }
    if (n2k_enabled->get() && n2k_attitude_pgn_enabled->get()) {
      attitude_sender->send();
    }
    if (n2k_enabled->get() && n2k_pressure_pgn_enabled->get()) {
      pressure_sender->send();
    }
  });

  // --- SignalK output ---

  // Fault indication (SPEC.md §6) is meta.timeout, not an active
  // notification — see docs/plans/fault-indication.md for why. Any
  // SignalK-spec-aware consumer computes staleness itself from this
  // advisory value and the delta's own timestamp; this firmware doesn't
  // have to declare an alarm state at all.
  auto sk_heading_output = new SKOutputFloat(
      "navigation.headingMagnetic", "/signalk/heading_path",
      new SKMetadata("rad", "", "", "", kHeadingTimeoutSeconds));

  // navigation.rateOfTurn (rad/s, +ve = starboard) — sourced from the
  // WT901B's real gyroscope (SPEC.md §5.1), unlike the sliding-window
  // estimate the HWT3100 fork this project is adapted from computed.
  auto sk_rate_of_turn_output = new SKOutputFloat(
      "navigation.rateOfTurn", "/signalk/rate_of_turn_path",
      new SKMetadata("rad/s", "", "", "", kHeadingTimeoutSeconds));

  // navigation.headingTrue (SPEC.md §5.1a): magnetic heading + bus-
  // sourced variation, only set when a recent variation is available
  // (see the imu_producer consumer lambda below).
  auto sk_heading_true_output = new SKOutputFloat(
      "navigation.headingTrue", "/signalk/heading_true_path",
      new SKMetadata("rad", "", "", "", kVariationTimeoutSeconds));

  // navigation.attitude (SPEC.md §5.2): newly implementable in this
  // fork — the WT901B's accelerometer gives real roll/pitch, unlike the
  // HWT3100 this project is adapted from (SPEC.md §9.3 of the source
  // project). Yaw is always 0 (headingMagnetic is the authoritative
  // heading). SensESP has no dedicated attitude-object output class
  // (unlike SKOutputFloat for a single numeric path), so this uses
  // SKOutputRawJson to publish the {roll, pitch, yaw} object
  // navigation.attitude's schema requires — see the imu_producer
  // consumer lambda below for how the JSON is built.
  auto sk_attitude_output = new SKOutputRawJson(
      "navigation.attitude", "/signalk/attitude_path",
      new SKMetadata("rad", "", "", "", kHeadingTimeoutSeconds));

  // environment.outside.pressure (SPEC.md §5.1): real data, from the
  // WT901B's barometer (0x56 packet) -- unlike the raw magnetic field
  // below, Pascals is a well-defined physical unit, so this is a
  // standard SignalK path, not a diagnostic sensors.* one.
  auto sk_pressure_output = new SKOutputFloat(
      "environment.outside.pressure", "/signalk/pressure_path",
      new SKMetadata("Pa", "", "", "", kHeadingTimeoutSeconds));

  // Raw gyro/accelerometer (SPEC.md §5.2): same diagnostic-only,
  // custom sensors.* treatment as the raw magnetic field below -- no
  // established SignalK path for raw angular rate or acceleration on a
  // vessel. Converted to SI units (rad/s, m/s^2) at this boundary,
  // unlike magnetic field's raw counts, since the WT901B's own gyro/
  // accel scale factors are known (unlike its mag-to-µT factor).
  auto sk_gyro_x_output = new SKOutputFloat(
      "sensors.hwt901b.angularRate.x", "/signalk/gyro_x_path",
      new SKMetadata(
          "rad/s", "Gyro X",
          "Raw roll-axis angular rate from the WT901B's gyroscope. Diagnostic-only.",
          "GyroX", kHeadingTimeoutSeconds));
  auto sk_gyro_y_output = new SKOutputFloat(
      "sensors.hwt901b.angularRate.y", "/signalk/gyro_y_path",
      new SKMetadata(
          "rad/s", "Gyro Y",
          "Raw pitch-axis angular rate from the WT901B's gyroscope. Diagnostic-only.",
          "GyroY", kHeadingTimeoutSeconds));
  auto sk_gyro_z_output = new SKOutputFloat(
      "sensors.hwt901b.angularRate.z", "/signalk/gyro_z_path",
      new SKMetadata(
          "rad/s", "Gyro Z",
          "Raw yaw-axis angular rate from the WT901B's gyroscope -- duplicates "
          "navigation.rateOfTurn (same source), kept for diagnostic parity with "
          "the X/Y axes. Diagnostic-only.",
          "GyroZ", kHeadingTimeoutSeconds));
  auto sk_accel_x_output = new SKOutputFloat(
      "sensors.hwt901b.acceleration.x", "/signalk/accel_x_path",
      new SKMetadata(
          "m/s2", "Accel X",
          "Raw X-axis reading from the WT901B's accelerometer. Diagnostic-only.",
          "AccelX", kHeadingTimeoutSeconds));
  auto sk_accel_y_output = new SKOutputFloat(
      "sensors.hwt901b.acceleration.y", "/signalk/accel_y_path",
      new SKMetadata(
          "m/s2", "Accel Y",
          "Raw Y-axis reading from the WT901B's accelerometer. Diagnostic-only.",
          "AccelY", kHeadingTimeoutSeconds));
  auto sk_accel_z_output = new SKOutputFloat(
      "sensors.hwt901b.acceleration.z", "/signalk/accel_z_path",
      new SKMetadata(
          "m/s2", "Accel Z",
          "Raw Z-axis reading from the WT901B's accelerometer. Diagnostic-only.",
          "AccelZ", kHeadingTimeoutSeconds));

  // Raw magnetic field (SPEC.md §5.2): custom sensors.* paths, no
  // established standard, no unit (the register protocol docs don't
  // document a counts-to-µT conversion factor) — raw sensor counts
  // as-is, same values already visible via the serial terminal (§8.1).
  // description_ is filled in on all three since SignalK requires it
  // for any non-standard path (SKMetadata's own doc comment);
  // display_name_/short_name_ likewise, since consumers have nothing
  // else to show.
  auto sk_mag_x_output = new SKOutputFloat(
      "sensors.hwt901b.magneticField.x", "/signalk/mag_x_path",
      new SKMetadata(
          "", "Mag X",
          "Raw, uncalibrated magnetic field X-axis reading from the "
          "WT901B, in sensor counts -- the register protocol docs don't "
          "document a counts-to-µT conversion factor, so no unit is "
          "given. Diagnostic-only.",
          "MagX", kHeadingTimeoutSeconds));
  auto sk_mag_y_output = new SKOutputFloat(
      "sensors.hwt901b.magneticField.y", "/signalk/mag_y_path",
      new SKMetadata(
          "", "Mag Y",
          "Raw, uncalibrated magnetic field Y-axis reading from the "
          "WT901B, in sensor counts -- the register protocol docs don't "
          "document a counts-to-µT conversion factor, so no unit is "
          "given. Diagnostic-only.",
          "MagY", kHeadingTimeoutSeconds));
  auto sk_mag_z_output = new SKOutputFloat(
      "sensors.hwt901b.magneticField.z", "/signalk/mag_z_path",
      new SKMetadata(
          "", "Mag Z",
          "Raw, uncalibrated magnetic field Z-axis reading from the "
          "WT901B, in sensor counts -- the register protocol docs don't "
          "document a counts-to-µT conversion factor, so no unit is "
          "given. Diagnostic-only.",
          "MagZ", kHeadingTimeoutSeconds));

  // --- WT901B serial I/O, calibration offset, and dispatch to outputs ---

  auto imu_producer = new TaskQueueProducer<ImuReading>(ImuReading{});
  auto raw_frame_producer =
      new TaskQueueProducer<HWT901BRawFrame>(HWT901BRawFrame{});

  auto serial_terminal = std::make_shared<halser::SerialTerminal>("/hwt901b/serial_log");
  ConfigItem(serial_terminal)
      ->set_title("WT901B Serial Log")
      ->set_description(
          "Read-only: the most recent raw frames received from the WT901B, as hex bytes (SPEC.md §8.1)")
      ->set_sort_order(10)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"lines":{"title":"Lines","type":"array","items":{"type":"string"}}}})schema");

  raw_frame_producer->connect_to(new LambdaConsumer<HWT901BRawFrame>(
      [serial_terminal](HWT901BRawFrame frame) { serial_terminal->AddFrame(frame); }));

  // Unlike the HWT3100's plain-text AT+CALI replies, the WT901B's
  // register protocol has no acknowledgment packet — there is
  // deliberately no calibration-reply status item here (SPEC.md §8.2,
  // §10): there is nothing on the wire to correlate a click with.

  imu_producer->connect_to(new LambdaConsumer<ImuReading>(
      [=](ImuReading reading) {
        ImuReading corrected =
            halser::ApplyCalibrationOffset(reading, heading_offset->get());
        // ImuReading.heading is degrees throughout this firmware's
        // internal pipeline (matches the WT901B's own angle-packet
        // scale) — both N2K's SetN2kPGN127250 and SignalK's
        // navigation.headingMagnetic require radians, so the conversion
        // happens right at each output boundary, not upstream.
        float heading_rad = corrected.heading * kDegreesToRadians;
        heading_sender->heading_.update(heading_rad);
        if (signalk_enabled->get() && sk_heading_enabled->get()) {
          sk_heading_output->set(heading_rad);
        }

        // navigation.headingTrue (SPEC.md §5.1a): only computed/published
        // when a recent bus-sourced variation exists — omitted, not sent
        // as a placeholder, when it doesn't.
        if (heading_sender->variation_.is_valid()) {
          float heading_true_rad = fmodf(
              heading_rad + heading_sender->variation_.value(), kTwoPi);
          if (heading_true_rad < 0.0f) heading_true_rad += kTwoPi;
          if (signalk_enabled->get() && sk_heading_true_enabled->get()) {
            sk_heading_true_output->set(heading_true_rad);
          }
        }

        // Rate of turn: real gyro_z, sign-corrected the same direction
        // as heading (SPEC.md §1.3, §11 — the WT901B's raw yaw-axis
        // gyro sign convention relative to N2K/SignalK's "+ve =
        // starboard" is unverified in this environment; carried over
        // from the same assumption ApplyCalibrationOffset makes for
        // heading, for consistency, not because it's independently
        // confirmed).
        float rate_of_turn_rad_s = -corrected.gyro_z * kDegreesToRadians;
        rate_of_turn_sender->rate_of_turn_.update(rate_of_turn_rad_s);
        if (signalk_enabled->get() && sk_rate_of_turn_enabled->get()) {
          sk_rate_of_turn_output->set(rate_of_turn_rad_s);
        }

        // Attitude: real roll/pitch from the WT901B's accelerometer
        // (SPEC.md §5.1, §5.2) — the capability the HWT3100 fork this
        // project is adapted from never had.
        float roll_rad = corrected.roll * kDegreesToRadians;
        float pitch_rad = corrected.pitch * kDegreesToRadians;
        attitude_sender->roll_.update(roll_rad);
        attitude_sender->pitch_.update(pitch_rad);
        if (signalk_enabled->get() && sk_attitude_enabled->get()) {
          char attitude_json[96];
          snprintf(attitude_json, sizeof(attitude_json),
                    "{\"roll\":%.6f,\"pitch\":%.6f,\"yaw\":0}", roll_rad,
                    pitch_rad);
          sk_attitude_output->set(String(attitude_json));
        }

        if (signalk_enabled->get() && raw_mag_field_enabled->get()) {
          sk_mag_x_output->set(static_cast<float>(corrected.mag_x));
          sk_mag_y_output->set(static_cast<float>(corrected.mag_y));
          sk_mag_z_output->set(static_cast<float>(corrected.mag_z));
        }

        // Pressure: real data (SPEC.md §5.1), already in Pascals
        // internally -- no conversion needed at this boundary, unlike
        // heading/gyro/accel.
        pressure_sender->pressure_.update(corrected.pressure_pa);
        if (signalk_enabled->get() && sk_pressure_enabled->get()) {
          sk_pressure_output->set(corrected.pressure_pa);
        }

        // Raw gyro/accel (SPEC.md §5.2): diagnostic-only, converted to
        // SI units at this boundary same as every other output.
        if (signalk_enabled->get() && raw_gyro_enabled->get()) {
          sk_gyro_x_output->set(corrected.gyro_x * kDegreesToRadians);
          sk_gyro_y_output->set(corrected.gyro_y * kDegreesToRadians);
          sk_gyro_z_output->set(corrected.gyro_z * kDegreesToRadians);
        }
        if (signalk_enabled->get() && raw_accel_enabled->get()) {
          sk_accel_x_output->set(corrected.accel_x * kGToMetersPerSecondSquared);
          sk_accel_y_output->set(corrected.accel_y * kGToMetersPerSecondSquared);
          sk_accel_z_output->set(corrected.accel_z * kGToMetersPerSecondSquared);
        }
      }));

  auto hwt901b_serial =
      new halser::HWT901BSerialIO(Serial1, imu_producer, raw_frame_producer);

  // Baud auto-detection (SPEC.md §8.2c): if the persisted baud is still
  // unknown, try each candidate in turn (passive listening only, no
  // register write sent) and persist whichever one produces valid data.
  // Must happen before Begin() starts the background read task, since
  // DetectBaud() isn't safe to call concurrently with it. If nothing is
  // found in this pass, don't persist a guess -- read at the
  // recommended default for this boot and let the next boot retry
  // detection fresh.
  int startup_baud = kHWT901BDefaultBaud;
  if (hwt901b_baud->get() == halser::kBaudUnknown) {
    int detected = 0;
    if (hwt901b_serial->DetectBaud(kBaudCandidates, kNumBaudCandidates,
                                    kBaudDetectTimeoutMs, kUART1RxPin,
                                    kUART1TxPin, &detected)) {
      hwt901b_baud->set(detected);
      startup_baud = detected;
    }
  } else {
    startup_baud = hwt901b_baud->get();
  }
  hwt901b_serial->Begin(startup_baud, kUART1RxPin, kUART1TxPin);

  // Runtime baud switching: only fires on an explicit config change
  // *after* the wiring above, since it's attached after any startup
  // hwt901b_baud->set() from auto-detection -- so discovering the
  // module's existing rate never itself triggers an unwanted register
  // write.
  hwt901b_baud->connect_to(new LambdaConsumer<int>([hwt901b_serial](int value) {
    if (value != halser::kBaudUnknown) {
      hwt901b_serial->SetBaudRate(value, kUART1RxPin, kUART1TxPin);
    }
  }));

  // Re-apply the persisted BANDWIDTH/RRATE settings on every boot (the
  // module can't report either back to us) and again whenever the
  // config value changes.
  hwt901b_serial->SetBandwidth(output_bandwidth->get());
  output_bandwidth->connect_to(new LambdaConsumer<int>(
      [hwt901b_serial](int value) { hwt901b_serial->SetBandwidth(value); }));

  hwt901b_serial->SetRate(output_rate_chz->get());
  output_rate_chz->connect_to(new LambdaConsumer<int>(
      [hwt901b_serial](int value) { hwt901b_serial->SetRate(value); }));

  // --- Calibration commands (SPEC.md §8.2) ---

  auto calibration_commands =
      new halser::CalibrationCommandHandler(hwt901b_serial);

  // Lets a compatible MFD start/stop calibration over the N2K bus, the
  // same way htool/ESP32_Precision-9_compass_CMPS14 does (SPEC.md §8.2,
  // §10) — reverse-engineered proprietary protocol, unverified against
  // real hardware; see docs/plans/mfd-calibration.md. Self-attaches to
  // nmea2000 via its tMsgHandler base constructor.
  new halser::MfdCalibrationBridge(nmea2000, calibration_commands);

  // Listens for PGN 127258 (Magnetic Variation) from another N2K
  // device (SPEC.md §5.1a) — feeds heading_sender's variation_ so PGN
  // 127250's own Variation field and navigation.headingTrue (below)
  // both get real data when a source exists on the bus. Read-only:
  // never transmits PGN 127258 itself. Self-attaches to nmea2000 via
  // its tMsgHandler base constructor, same as MfdCalibrationBridge.
  new halser::MagneticVariationListener(nmea2000, &heading_sender->variation_);

  // Two adjacent, consistently-worded actions (SPEC.md §8.2), real
  // UIButtons on the web UI's Control tab (BoatHacks/SensESP; see the
  // platformio.ini comment and docs/plans/calibration-control-tab.md —
  // temporary until the upstream PR lands). must_confirm is left at its
  // default (true) for both: each one changes the module's on-module
  // magnetic calibration state, which is annoying to redo if clicked by
  // accident. Fire-and-forget by design — UIButton has no return-value
  // mechanism, and unlike the HWT3100 fork this project is adapted
  // from, there is no reply packet to show a status item for (SPEC.md
  // §8.2, §10).
  //
  // The Control tab renders buttons in UIButton::get_ui_buttons()'s
  // std::map key order, i.e. sorted by the `name` argument below, not
  // registration order — the "1_"/"2_" prefixes are what actually pin
  // the displayed order to Start, Stop.
  sensesp::UIButton::add(
      "hwt901b_calibration_1_start",
      "Start Magnetic Calibration (rotate 360° at least 3x, then press Stop)")
      ->attach([calibration_commands]() { calibration_commands->StartCalibration(); });

  sensesp::UIButton::add("hwt901b_calibration_2_stop", "Stop Magnetic Calibration")
      ->attach([calibration_commands]() { calibration_commands->EndCalibration(); });

  while (true) {
    event_loop()->tick();
  }
}

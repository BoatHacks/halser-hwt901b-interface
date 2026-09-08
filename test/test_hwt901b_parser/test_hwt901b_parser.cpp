#include <unity.h>

#include <cstdint>
#include <cstring>

#include "hwt901b_parser.h"
#include "hwt901b_types.h"

void setUp(void) {}
void tearDown(void) {}

// Builds an 11-byte frame from a type byte and 8 data bytes, computing
// the checksum the way the WT901B protocol requires (low byte of the
// sum of the first 10 bytes).
static void BuildFrame(uint8_t type, const uint8_t data[8], uint8_t out[11]) {
  out[0] = 0x55;
  out[1] = type;
  for (int i = 0; i < 8; i++) out[2 + i] = data[i];
  uint8_t sum = 0;
  for (int i = 0; i < 10; i++) sum = static_cast<uint8_t>(sum + out[i]);
  out[10] = sum;
}

static void Int16ToLE(int16_t v, uint8_t* out) {
  out[0] = static_cast<uint8_t>(v & 0xFF);
  out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

static void test_angle_packet_fills_heading_roll_pitch(void) {
  // roll=90deg -> 16384, pitch=-45deg -> -8192, yaw=180deg -> 32767(ish)
  uint8_t data[8] = {0};
  Int16ToLE(16384, data + 0);   // roll
  Int16ToLE(-8192, data + 2);   // pitch
  Int16ToLE(16384, data + 4);   // yaw (90 deg)
  uint8_t frame[11];
  BuildFrame(0x53, data, frame);

  ImuReading r;
  TEST_ASSERT_TRUE(ParseHWT901BFrame(frame, sizeof(frame), &r));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 90.0f, r.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, -45.0f, r.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 90.0f, r.heading);
}

static void test_magnetic_packet_fills_mag_xyz(void) {
  uint8_t data[8] = {0};
  Int16ToLE(1234, data + 0);
  Int16ToLE(-567, data + 2);
  Int16ToLE(89, data + 4);
  uint8_t frame[11];
  BuildFrame(0x54, data, frame);

  ImuReading r;
  TEST_ASSERT_TRUE(ParseHWT901BFrame(frame, sizeof(frame), &r));
  TEST_ASSERT_EQUAL_INT32(1234, r.mag_x);
  TEST_ASSERT_EQUAL_INT32(-567, r.mag_y);
  TEST_ASSERT_EQUAL_INT32(89, r.mag_z);
}

static void test_gyro_packet_fills_gyro_xyz(void) {
  uint8_t data[8] = {0};
  Int16ToLE(8192, data + 0);   // 500 deg/s
  Int16ToLE(-8192, data + 2);  // -500 deg/s
  Int16ToLE(16384, data + 4);  // 1000 deg/s
  uint8_t frame[11];
  BuildFrame(0x52, data, frame);

  ImuReading r;
  TEST_ASSERT_TRUE(ParseHWT901BFrame(frame, sizeof(frame), &r));
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 500.0f, r.gyro_x);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -500.0f, r.gyro_y);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, r.gyro_z);
}

static void test_acceleration_packet_fills_accel_xyz(void) {
  uint8_t data[8] = {0};
  Int16ToLE(16384, data + 0);  // 8g
  Int16ToLE(-16384, data + 2); // -8g
  Int16ToLE(32767, data + 4);  // ~16g
  uint8_t frame[11];
  BuildFrame(0x51, data, frame);

  ImuReading r;
  TEST_ASSERT_TRUE(ParseHWT901BFrame(frame, sizeof(frame), &r));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 8.0f, r.accel_x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -8.0f, r.accel_y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 16.0f, r.accel_z);
}

static void test_pressure_packet_fills_pressure_pa(void) {
  uint8_t data[8] = {0};
  // 101046 Pa, little-endian int32 -- matches a real captured frame
  // (SPEC.md §11).
  int32_t pressure = 101046;
  data[0] = static_cast<uint8_t>(pressure & 0xFF);
  data[1] = static_cast<uint8_t>((pressure >> 8) & 0xFF);
  data[2] = static_cast<uint8_t>((pressure >> 16) & 0xFF);
  data[3] = static_cast<uint8_t>((pressure >> 24) & 0xFF);
  uint8_t frame[11];
  BuildFrame(0x56, data, frame);

  ImuReading r;
  TEST_ASSERT_TRUE(ParseHWT901BFrame(frame, sizeof(frame), &r));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 101046.0f, r.pressure_pa);
}

static void test_fields_accumulate_across_separate_packets(void) {
  ImuReading r;

  uint8_t mag_data[8] = {0};
  Int16ToLE(1, mag_data + 0);
  Int16ToLE(2, mag_data + 2);
  Int16ToLE(3, mag_data + 4);
  uint8_t mag_frame[11];
  BuildFrame(0x54, mag_data, mag_frame);
  TEST_ASSERT_TRUE(ParseHWT901BFrame(mag_frame, sizeof(mag_frame), &r));

  uint8_t angle_data[8] = {0};
  Int16ToLE(0, angle_data + 0);
  Int16ToLE(0, angle_data + 2);
  Int16ToLE(0, angle_data + 4);
  uint8_t angle_frame[11];
  BuildFrame(0x53, angle_data, angle_frame);
  TEST_ASSERT_TRUE(ParseHWT901BFrame(angle_frame, sizeof(angle_frame), &r));

  // The magnetic-packet fields from the first call must still be
  // present after a second call with a different packet type.
  TEST_ASSERT_EQUAL_INT32(1, r.mag_x);
  TEST_ASSERT_EQUAL_INT32(2, r.mag_y);
  TEST_ASSERT_EQUAL_INT32(3, r.mag_z);
}

static void test_bad_checksum_rejected(void) {
  uint8_t data[8] = {0};
  uint8_t frame[11];
  BuildFrame(0x53, data, frame);
  frame[10] ^= 0xFF;  // corrupt checksum

  ImuReading r;
  TEST_ASSERT_FALSE(ParseHWT901BFrame(frame, sizeof(frame), &r));
}

static void test_wrong_header_byte_rejected(void) {
  uint8_t data[8] = {0};
  uint8_t frame[11];
  BuildFrame(0x53, data, frame);
  frame[0] = 0x00;

  ImuReading r;
  TEST_ASSERT_FALSE(ParseHWT901BFrame(frame, sizeof(frame), &r));
}

static void test_unknown_type_rejected(void) {
  uint8_t data[8] = {0};
  uint8_t frame[11];
  BuildFrame(0x99, data, frame);

  ImuReading r;
  TEST_ASSERT_FALSE(ParseHWT901BFrame(frame, sizeof(frame), &r));
}

static void test_wrong_length_rejected(void) {
  uint8_t data[8] = {0};
  uint8_t frame[11];
  BuildFrame(0x53, data, frame);

  ImuReading r;
  TEST_ASSERT_FALSE(ParseHWT901BFrame(frame, 10, &r));
}

static void test_null_arguments_rejected(void) {
  uint8_t data[8] = {0};
  uint8_t frame[11];
  BuildFrame(0x53, data, frame);

  ImuReading r;
  TEST_ASSERT_FALSE(ParseHWT901BFrame(nullptr, sizeof(frame), &r));
  TEST_ASSERT_FALSE(ParseHWT901BFrame(frame, sizeof(frame), nullptr));
}

static void test_describe_angle_frame(void) {
  uint8_t data[8] = {0};
  Int16ToLE(16384, data + 0);  // roll = 90
  Int16ToLE(-8192, data + 2);  // pitch = -45
  Int16ToLE(16384, data + 4);  // heading = 90
  uint8_t frame[11];
  BuildFrame(0x53, data, frame);

  char out[64];
  DescribeHWT901BFrame(frame, sizeof(frame), out, sizeof(out));
  TEST_ASSERT_TRUE(strstr(out, "heading=90.00") != nullptr);
  TEST_ASSERT_TRUE(strstr(out, "roll=90.00") != nullptr);
  TEST_ASSERT_TRUE(strstr(out, "pitch=-45.00") != nullptr);
}

static void test_describe_gyro_frame(void) {
  uint8_t data[8] = {0};
  Int16ToLE(16384, data + 4);  // gyro_z = 1000 deg/s
  uint8_t frame[11];
  BuildFrame(0x52, data, frame);

  char out[64];
  DescribeHWT901BFrame(frame, sizeof(frame), out, sizeof(out));
  TEST_ASSERT_TRUE(strstr(out, "z=1000.00") != nullptr);
}

static void test_describe_magnetic_frame(void) {
  uint8_t data[8] = {0};
  Int16ToLE(1234, data + 0);
  Int16ToLE(-567, data + 2);
  Int16ToLE(89, data + 4);
  uint8_t frame[11];
  BuildFrame(0x54, data, frame);

  char out[64];
  DescribeHWT901BFrame(frame, sizeof(frame), out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("mag x=1234 y=-567 z=89", out);
}

static void test_describe_acceleration_frame(void) {
  uint8_t data[8] = {0};
  uint8_t frame[11];
  BuildFrame(0x51, data, frame);

  char out[64];
  DescribeHWT901BFrame(frame, sizeof(frame), out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("accel x=0.00 y=0.00 z=0.00 g", out);
}

static void test_describe_pressure_frame(void) {
  uint8_t data[8] = {0};
  int32_t pressure = 101046;
  data[0] = static_cast<uint8_t>(pressure & 0xFF);
  data[1] = static_cast<uint8_t>((pressure >> 8) & 0xFF);
  data[2] = static_cast<uint8_t>((pressure >> 16) & 0xFF);
  data[3] = static_cast<uint8_t>((pressure >> 24) & 0xFF);
  uint8_t frame[11];
  BuildFrame(0x56, data, frame);

  char out[64];
  DescribeHWT901BFrame(frame, sizeof(frame), out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("pressure=101046 Pa", out);
}

static void test_describe_invalid_frame(void) {
  uint8_t data[8] = {0};
  uint8_t frame[11];
  BuildFrame(0x53, data, frame);
  frame[10] ^= 0xFF;  // corrupt checksum

  char out[64];
  DescribeHWT901BFrame(frame, sizeof(frame), out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("invalid frame (bad header/checksum/type)", out);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_angle_packet_fills_heading_roll_pitch);
  RUN_TEST(test_magnetic_packet_fills_mag_xyz);
  RUN_TEST(test_gyro_packet_fills_gyro_xyz);
  RUN_TEST(test_acceleration_packet_fills_accel_xyz);
  RUN_TEST(test_pressure_packet_fills_pressure_pa);
  RUN_TEST(test_fields_accumulate_across_separate_packets);
  RUN_TEST(test_bad_checksum_rejected);
  RUN_TEST(test_wrong_header_byte_rejected);
  RUN_TEST(test_unknown_type_rejected);
  RUN_TEST(test_wrong_length_rejected);
  RUN_TEST(test_null_arguments_rejected);
  RUN_TEST(test_describe_angle_frame);
  RUN_TEST(test_describe_gyro_frame);
  RUN_TEST(test_describe_magnetic_frame);
  RUN_TEST(test_describe_acceleration_frame);
  RUN_TEST(test_describe_pressure_frame);
  RUN_TEST(test_describe_invalid_frame);
  return UNITY_END();
}

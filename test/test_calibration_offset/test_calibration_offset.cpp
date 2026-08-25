#include <unity.h>

#include "calibration_offset.h"
#include "hwt901b_types.h"

using halser::ApplyCalibrationOffset;

void setUp(void) {}
void tearDown(void) {}

static void test_zero_raw_yaw_stays_zero_with_no_offset(void) {
  ImuReading reading;
  reading.heading = 0.0f;
  ImuReading corrected = ApplyCalibrationOffset(reading, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, corrected.heading);
}

static void test_turning_toward_west_decreases_compass_heading(void) {
  // Carried over from the HWT3100 fork's confirmed-on-real-hardware
  // sign convention (SPEC.md §11 flags whether the WT901B's fused yaw
  // shares it as unverified).
  ImuReading at_north;
  at_north.heading = 0.0f;
  ImuReading turned_toward_west;
  turned_toward_west.heading = 30.0f;

  float north_heading = ApplyCalibrationOffset(at_north, 0.0f).heading;
  float west_turn_heading = ApplyCalibrationOffset(turned_toward_west, 0.0f).heading;

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 330.0f, west_turn_heading);
  TEST_ASSERT_TRUE(west_turn_heading != 30.0f);
  (void)north_heading;
}

static void test_raw_yaw_at_due_west_reads_270(void) {
  ImuReading reading;
  reading.heading = 90.0f;
  ImuReading corrected = ApplyCalibrationOffset(reading, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 270.0f, corrected.heading);
}

static void test_offset_still_applies_after_axis_conversion(void) {
  ImuReading reading;
  reading.heading = 90.0f;
  ImuReading corrected = ApplyCalibrationOffset(reading, 10.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 280.0f, corrected.heading);
}

static void test_result_always_wraps_into_zero_to_360(void) {
  ImuReading reading;
  reading.heading = -10.0f;
  ImuReading corrected = ApplyCalibrationOffset(reading, 0.0f);
  TEST_ASSERT_TRUE(corrected.heading >= 0.0f);
  TEST_ASSERT_TRUE(corrected.heading < 360.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, corrected.heading);
}

static void test_roll_pitch_mag_and_gyro_pass_through_unchanged(void) {
  ImuReading reading;
  reading.heading = 45.0f;
  reading.roll = 12.5f;
  reading.pitch = -3.25f;
  reading.gyro_z = 7.0f;
  reading.mag_x = 111;
  reading.mag_y = -222;
  reading.mag_z = 333;
  ImuReading corrected = ApplyCalibrationOffset(reading, 5.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.5f, corrected.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -3.25f, corrected.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.0f, corrected.gyro_z);
  TEST_ASSERT_EQUAL_INT32(111, corrected.mag_x);
  TEST_ASSERT_EQUAL_INT32(-222, corrected.mag_y);
  TEST_ASSERT_EQUAL_INT32(333, corrected.mag_z);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_zero_raw_yaw_stays_zero_with_no_offset);
  RUN_TEST(test_turning_toward_west_decreases_compass_heading);
  RUN_TEST(test_raw_yaw_at_due_west_reads_270);
  RUN_TEST(test_offset_still_applies_after_axis_conversion);
  RUN_TEST(test_result_always_wraps_into_zero_to_360);
  RUN_TEST(test_roll_pitch_mag_and_gyro_pass_through_unchanged);
  return UNITY_END();
}

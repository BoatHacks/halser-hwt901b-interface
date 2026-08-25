#include <cstring>

#include <unity.h>

#include "hwt901b_bandwidth_command.h"

using halser::FormatBandwidthCommand;

void setUp(void) {}
void tearDown(void) {}

static void AssertFrame(int hz, int expected_hz, uint8_t expected_code) {
  uint8_t buf[5];
  int actual = FormatBandwidthCommand(hz, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(expected_hz, actual);
  const uint8_t expected[5] = {0xFF, 0xAA, 0x1F, expected_code, 0x00};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buf, 5);
}

static void test_exact_256(void) { AssertFrame(256, 256, 0); }
static void test_exact_5(void) { AssertFrame(5, 5, 6); }

static void test_snaps_to_nearest(void) {
  // 100 is closer to 94 (delta 6) than to 44 (delta 56) or 184 (delta 84).
  AssertFrame(100, 94, 2);
}

static void test_negative_snaps_to_lowest(void) { AssertFrame(-10, 5, 6); }

static void test_very_large_snaps_to_highest(void) { AssertFrame(10000, 256, 0); }

static void test_buffer_too_small_still_returns_value(void) {
  uint8_t buf[4];
  int actual = FormatBandwidthCommand(256, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(256, actual);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_exact_256);
  RUN_TEST(test_exact_5);
  RUN_TEST(test_snaps_to_nearest);
  RUN_TEST(test_negative_snaps_to_lowest);
  RUN_TEST(test_very_large_snaps_to_highest);
  RUN_TEST(test_buffer_too_small_still_returns_value);
  return UNITY_END();
}

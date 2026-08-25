#include <cstring>

#include <unity.h>

#include "hwt901b_rate_command.h"

using halser::FormatRateCommand;

void setUp(void) {}
void tearDown(void) {}

static void AssertFrame(int chz, int expected_chz, uint8_t expected_code) {
  uint8_t buf[5];
  int actual = FormatRateCommand(chz, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(expected_chz, actual);
  const uint8_t expected[5] = {0xFF, 0xAA, 0x03, expected_code, 0x00};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buf, 5);
}

static void test_exact_10hz(void) { AssertFrame(1000, 1000, 0x09); }
static void test_exact_0_2hz(void) { AssertFrame(2, 2, 0x01); }
static void test_exact_200hz(void) { AssertFrame(2000, 2000, 0x0B); }

static void test_snaps_to_nearest(void) {
  // 1100 (11 Hz) is closer to 1000 (10 Hz, delta 100) than to 1250 (12.5 Hz, delta 150).
  AssertFrame(1100, 1000, 0x09);
}

static void test_negative_snaps_to_lowest_never_off(void) {
  // The RRATE "no output" code is never reachable through this
  // formatter (SPEC.md §2) — a caller passing 0 or a negative value
  // still gets the lowest real rate, not the dangerous "silence the
  // stream" code.
  AssertFrame(-5, 2, 0x01);
  AssertFrame(0, 2, 0x01);
}

static void test_very_large_snaps_to_highest(void) { AssertFrame(100000, 2000, 0x0B); }

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_exact_10hz);
  RUN_TEST(test_exact_0_2hz);
  RUN_TEST(test_exact_200hz);
  RUN_TEST(test_snaps_to_nearest);
  RUN_TEST(test_negative_snaps_to_lowest_never_off);
  RUN_TEST(test_very_large_snaps_to_highest);
  return UNITY_END();
}

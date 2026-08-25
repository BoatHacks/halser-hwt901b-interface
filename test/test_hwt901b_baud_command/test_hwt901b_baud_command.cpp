#include <cstring>

#include <unity.h>

#include "hwt901b_baud_command.h"

using halser::FormatBaudCommand;

void setUp(void) {}
void tearDown(void) {}

static void AssertFrame(int requested, int expected_baud, uint8_t expected_code) {
  uint8_t buf[5];
  int actual = FormatBaudCommand(requested, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(expected_baud, actual);
  const uint8_t expected[5] = {0xFF, 0xAA, 0x04, expected_code, 0x00};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buf, 5);
}

static void test_exact_4800(void) { AssertFrame(4800, 4800, 0); }
static void test_exact_9600(void) { AssertFrame(9600, 9600, 1); }
static void test_exact_115200(void) { AssertFrame(115200, 115200, 5); }
static void test_exact_230400(void) { AssertFrame(230400, 230400, 6); }

static void test_snaps_to_nearest_low(void) {
  // 50000 is closer to 57600 (delta 7600) than to 38400 (delta 11600).
  AssertFrame(50000, 57600, 4);
}

static void test_snaps_to_nearest_mid(void) {
  // 200000 is closer to 230400 (delta 30400) than to 115200 (delta 84800).
  AssertFrame(200000, 230400, 6);
}

static void test_zero_snaps_to_4800(void) { AssertFrame(0, 4800, 0); }
static void test_very_large_snaps_to_230400(void) { AssertFrame(2000000, 230400, 6); }

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_exact_4800);
  RUN_TEST(test_exact_9600);
  RUN_TEST(test_exact_115200);
  RUN_TEST(test_exact_230400);
  RUN_TEST(test_snaps_to_nearest_low);
  RUN_TEST(test_snaps_to_nearest_mid);
  RUN_TEST(test_zero_snaps_to_4800);
  RUN_TEST(test_very_large_snaps_to_230400);
  return UNITY_END();
}

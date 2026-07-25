#include <assert.h>

#include "../board/chrysler_long_guard.h"


static bool valid_brake_command(void) {
  return chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, true, false,
    CHRYSLER_LONG_TORQUE_ZERO_RAW, 100, false, false, false);
}


int main(void) {
  assert(CHRYSLER_LONG_ACTUATION == 0U);

  assert(chrysler_long_is_fresh(1000000U, 950000U, true, 100000U));
  assert(!chrysler_long_is_fresh(1000000U, 899999U, true, 100000U));
  assert(!chrysler_long_is_fresh(1000000U, 950000U, false, 100000U));
  assert(chrysler_long_is_fresh(50U, 0xFFFFFFF0U, true, 100U));

  assert(valid_brake_command());
  assert(chrysler_long_commands_valid(
    true, true, true, false, false, 4094, 0, false, true,
    7845, 5000, false, false, false));

  assert(!chrysler_long_commands_valid(
    false, true, true, false, false, 2866, 1, true, false,
    7767, 100, false, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2500, 1, true, false,
    7767, 100, false, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, false, false,
    7767, 100, false, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 4094, 0, false, true,
    8000, 5000, false, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, true, false,
    7767, 100, true, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, true, false,
    7767, 100, false, true, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, true, false,
    7767, 100, false, false, true));
  assert(!chrysler_long_commands_valid(
    true, true, true, true, false, 2866, 1, true, false,
    7767, 300, false, false, false));

  return 0;
}

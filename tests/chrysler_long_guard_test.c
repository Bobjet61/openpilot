#include <assert.h>

#include "../board/chrysler_long_guard.h"


static bool valid_brake_command(void) {
  return chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, false, false,
    CHRYSLER_LONG_TORQUE_ZERO_RAW, 100, false, false, false);
}


int main(void) {
  assert(CHRYSLER_LONG_ACTUATION == 1U);

  assert(chrysler_long_is_fresh(1000000U, 950000U, true, 100000U));
  assert(!chrysler_long_is_fresh(1000000U, 899999U, true, 100000U));
  assert(!chrysler_long_is_fresh(1000000U, 950000U, false, 100000U));
  assert(chrysler_long_is_fresh(50U, 0xFFFFFFF0U, true, 100U));

  bool counter_seen = false;
  int counter_last = 0;
  assert(chrysler_long_counter_step_valid(&counter_seen, &counter_last, 14));
  assert(chrysler_long_counter_step_valid(&counter_seen, &counter_last, 15));
  assert(chrysler_long_counter_step_valid(&counter_seen, &counter_last, 0));
  assert(!chrysler_long_counter_step_valid(&counter_seen, &counter_last, 0));
  assert(!chrysler_long_counter_step_valid(&counter_seen, &counter_last, 2));
  assert(chrysler_long_counter_step_valid(&counter_seen, &counter_last, 3));
  assert(chrysler_long_counters_aligned(true, 3, true, 3, true, 3));
  assert(!chrysler_long_counters_aligned(true, 3, true, 2, true, 3));
  assert(!chrysler_long_counters_aligned(true, 3, false, 3, true, 3));

  // A new private cycle is committed only after all three valid frames carry
  // the same counter. The first two arrivals must leave the prior command in
  // force rather than briefly passing a factory command through.
  assert(!chrysler_long_staged_cycle_ready(
    true, 4, true, 3, true, 3));
  assert(!chrysler_long_staged_cycle_ready(
    true, 4, true, 4, true, 3));
  assert(chrysler_long_staged_cycle_ready(
    true, 4, true, 4, true, 4));
  assert(!chrysler_long_staged_cycle_ready(
    true, 4, true, 4, false, 4));

  assert(valid_brake_command());
  assert(chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_MIN_RAW,
    1, false, false, CHRYSLER_LONG_TORQUE_ZERO_RAW,
    CHRYSLER_LONG_MOVING_SPEED_MIN_RAW, false, false, false));
  assert(chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, true,
    2310, 100, false, false, false));

  assert(!chrysler_long_commands_valid(
    false, true, true, false, false, 2866, 1, false, false,
    2000, 100, false, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_MIN_RAW - 1,
    1, false, false,
    2000, 100, false, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, true, false,
    2000, 100, false, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, true,
    2500, 100, false, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, false, false,
    2000, 100, true, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, false, false,
    2000, 100, false, true, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, false, false,
    2000, 100, false, false, true));
  assert(!chrysler_long_commands_valid(
    true, true, true, true, false, 2866, 1, false, false,
    2000, 100, false, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, false, false,
    2000, CHRYSLER_LONG_MOVING_SPEED_MIN_RAW - 1,
    false, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, false, true,
    2100, 100, false, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 3200, 0, false, false,
    2000, 100, false, false, false));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, false,
    2100, 100, false, false, false));

  assert(chrysler_long_diagnostic_mask(
    true, true, true, true, true, true, true, true, true, true, true,
    true, false, false, false, true) == 0U);

  const uint16_t all_failures = chrysler_long_diagnostic_mask(
    false, false, false, false, false, false, false, false, false,
    false, false, false, true, true, true, false);
  assert(all_failures == 0xFFFFU);

  assert(chrysler_long_diagnostic_mask(
    true, true, true, true, true, true, true, true, true, true, true,
    false, false, false, false, false) ==
    (CHRYSLER_LONG_DIAG_SPEED_TOO_LOW |
     CHRYSLER_LONG_DIAG_COMMAND_ENVELOPE));
  assert(chrysler_long_diagnostic_mask(
    true, true, true, true, true, true, true, true, true, true, true,
    true, true, false, false, false) ==
    (CHRYSLER_LONG_DIAG_DRIVER_BRAKE |
     CHRYSLER_LONG_DIAG_COMMAND_ENVELOPE));

  return 0;
}

#include <assert.h>

#include "../board/chrysler_long_guard.h"


static bool valid_brake_command(void) {
  return chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, false, false,
    CHRYSLER_LONG_TORQUE_ZERO_RAW, 100, false, false, false,
    CHRYSLER_LONG_LOW_DRIVE);
}


int main(void) {
  assert(CHRYSLER_LONG_ACTUATION == 1U);

  // Diagnostic packing preserves the two raw factory DAS_3 words and the
  // exact output engine word in their original byte order.
  assert(chrysler_long_status_diagnostic_word(0xB7U, 0xA55AU, 0xC3U) ==
         0xC3A55AB7U);
  assert(chrysler_long_command_diagnostic_low(0xA583U) == 0xA58301C1U);
  assert(chrysler_long_command_diagnostic_high(0x8960U, 0x7C4DU) ==
         0x7C4D8960U);
  assert(chrysler_long_owner_diagnostic_word(
    CHRYSLER_LONG_OWNER_OPENPILOT, true, true, false, 0, false, true) ==
    0x000019D2U);
  assert(chrysler_long_owner_diagnostic_word(
    CHRYSLER_LONG_OWNER_OFF, false, false, true, 2, true, false) ==
    0x000206D0U);

  assert(chrysler_long_is_fresh(1000000U, 950000U, true, 100000U));
  assert(!chrysler_long_is_fresh(1000000U, 899999U, true, 100000U));
  assert(!chrysler_long_is_fresh(1000000U, 950000U, false, 100000U));
  assert(chrysler_long_is_fresh(50U, 0xFFFFFFF0U, true, 100U));

  // The first valid host request starts a Cancel handshake but does not yet
  // substitute a factory command.
  uint8_t owner = chrysler_long_next_owner_state(
    CHRYSLER_LONG_OWNER_OFF, true, true, true, true, 0, false);
  assert(owner == CHRYSLER_LONG_OWNER_CANCELING);
  assert(!chrysler_long_should_substitute_das3(
    owner, true, true, 0, false, true, true));
  // Full ownership begins only after a later factory frame confirms inactive
  // ACC while main availability remains set.
  owner = chrysler_long_next_owner_state(
    owner, true, true, true, false, 0, false);
  assert(owner == CHRYSLER_LONG_OWNER_OPENPILOT);
  assert(chrysler_long_should_substitute_das3(
    owner, true, true, 0, false, true, false));
  // A factory re-engagement suspends substitution and restarts cancellation.
  owner = chrysler_long_next_owner_state(
    owner, true, true, true, true, 0, false);
  assert(owner == CHRYSLER_LONG_OWNER_CANCELING);
  assert(!chrysler_long_should_substitute_das3(
    owner, true, true, 0, false, true, true));
  // Every authority or stock-safety loss drops ownership immediately.
  assert(chrysler_long_next_owner_state(
    CHRYSLER_LONG_OWNER_OPENPILOT, false, true, true, false, 0, false) ==
    CHRYSLER_LONG_OWNER_OFF);
  assert(chrysler_long_next_owner_state(
    CHRYSLER_LONG_OWNER_OPENPILOT, true, true, false, false, 0, false) ==
    CHRYSLER_LONG_OWNER_OFF);
  assert(chrysler_long_next_owner_state(
    CHRYSLER_LONG_OWNER_OPENPILOT, true, true, true, false, 2, false) ==
    CHRYSLER_LONG_OWNER_OFF);
  assert(chrysler_long_next_owner_state(
    CHRYSLER_LONG_OWNER_OPENPILOT, true, true, true, false, 0, true) ==
    CHRYSLER_LONG_OWNER_OFF);
  assert(chrysler_long_next_owner_state(
    CHRYSLER_LONG_OWNER_FAILED, true, true, true, true, 0, false) ==
    CHRYSLER_LONG_OWNER_FAILED);
  assert(chrysler_long_next_owner_state(
    CHRYSLER_LONG_OWNER_FAILED, false, true, true, true, 0, false) ==
    CHRYSLER_LONG_OWNER_OFF);
  assert(!chrysler_long_should_substitute_das3(
    CHRYSLER_LONG_OWNER_OPENPILOT, true, true, 2, false, true, false));
  assert(!chrysler_long_should_substitute_das3(
    CHRYSLER_LONG_OWNER_OPENPILOT, true, true, 0, true, true, false));
  assert(!chrysler_long_should_substitute_das3(
    CHRYSLER_LONG_OWNER_OPENPILOT, true, false, 0, false, true, false));

  assert(chrysler_long_current_stock_frame_valid(
    true, 14, 15, 8, true));
  assert(chrysler_long_current_stock_frame_valid(
    true, 15, 0, 8, true));
  assert(!chrysler_long_current_stock_frame_valid(
    false, 14, 15, 8, true));
  assert(!chrysler_long_current_stock_frame_valid(
    true, 14, 0, 8, true));
  assert(!chrysler_long_current_stock_frame_valid(
    true, 14, 15, 7, true));
  assert(!chrysler_long_current_stock_frame_valid(
    true, 14, 15, 8, false));
  assert(!chrysler_long_cancel_timed_out(
    CHRYSLER_LONG_OWNER_CANCELING, 800000U, 0U));
  assert(chrysler_long_cancel_timed_out(
    CHRYSLER_LONG_OWNER_CANCELING, 800001U, 0U));
  assert(!chrysler_long_cancel_timed_out(
    CHRYSLER_LONG_OWNER_OPENPILOT, 800001U, 0U));
  assert(chrysler_long_cancel_timed_out(
    CHRYSLER_LONG_OWNER_CANCELING, 50U, 0xFFF3CB00U));

  // Canceling sends Cancel and blocks speed controls toward factory ACC.
  assert(chrysler_long_filter_button_byte(
    0x9EU, CHRYSLER_LONG_OWNER_CANCELING) == 0x83U);
  // Once owned, physical Cancel, main, and distance buttons pass; Set/Resume
  // and speed adjustment remain blocked from re-engaging factory ACC.
  assert(chrysler_long_filter_button_byte(
    0x9FU, CHRYSLER_LONG_OWNER_OPENPILOT) == 0x83U);
  assert(chrysler_long_filter_button_byte(
    0x9EU, CHRYSLER_LONG_OWNER_OFF) == 0x9EU);

  // A healthy 25 Hz private snapshot remains fresh throughout the independent
  // 50 Hz stock receive cadence. Liveness depends on elapsed time, not on a
  // comparison between the two unrelated frame counts.
  uint32_t last_private_ts = 0U;
  for (uint32_t stock_cycle = 0U; stock_cycle < 500U; stock_cycle++) {
    const uint32_t now = stock_cycle * 20000U;
    if ((stock_cycle % 2U) == 0U) {
      last_private_ts = now;
    }
    assert(chrysler_long_is_fresh(
      now, last_private_ts, true, CHRYSLER_LONG_BRAKE_TIMEOUT_US));
  }

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
    0, false, false, false, CHRYSLER_LONG_LOW_DRIVE));
  // A stopped hold is brake-only. The captured OEM GO state is a separate
  // neutral cycle, and neither state can request engine torque.
  assert(chrysler_long_commands_valid(
    true, true, true, true, false, 2866,
    1, false, false, CHRYSLER_LONG_TORQUE_ZERO_RAW,
    0, false, false, false, CHRYSLER_LONG_LOW_HOLD));
  assert(chrysler_long_commands_valid(
    true, true, true, false, true, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, false, CHRYSLER_LONG_TORQUE_ZERO_RAW,
    5, false, false, false, CHRYSLER_LONG_LOW_RELEASE));
  assert(chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, true,
    2310, 100, false, false, false, CHRYSLER_LONG_LOW_DRIVE));
  assert(chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, true,
    CHRYSLER_LONG_TORQUE_MAX_RAW, 100, false, false, false,
    CHRYSLER_LONG_LOW_DRIVE));

  // Standstill launch torque is authorized only after HOLD -> RELEASE -> GO.
  assert(chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, true,
    CHRYSLER_LONG_LAUNCH_TORQUE_MAX_RAW, 0, false, false, false,
    CHRYSLER_LONG_LOW_CREEP));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, true,
    CHRYSLER_LONG_LAUNCH_TORQUE_MAX_RAW + 1, 0, false, false, false,
    CHRYSLER_LONG_LOW_CREEP));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, true,
    2100, 0, false, false, false, CHRYSLER_LONG_LOW_DRIVE));

  assert(!chrysler_long_commands_valid(
    false, true, true, false, false, 2866, 1, false, false,
    2000, 100, false, false, false, CHRYSLER_LONG_LOW_DRIVE));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_MIN_RAW - 1,
    1, false, false,
    2000, 100, false, false, false, CHRYSLER_LONG_LOW_DRIVE));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, true, false,
    2000, 100, false, false, false, CHRYSLER_LONG_LOW_DRIVE));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, true,
    CHRYSLER_LONG_TORQUE_MAX_RAW + 1, 100, false, false, false,
    CHRYSLER_LONG_LOW_DRIVE));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, false, false,
    2000, 100, true, false, false, CHRYSLER_LONG_LOW_DRIVE));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, false, false,
    2000, 100, false, true, false, CHRYSLER_LONG_LOW_DRIVE));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, false, false,
    2000, 100, false, false, true, CHRYSLER_LONG_LOW_DRIVE));
  assert(!chrysler_long_commands_valid(
    true, true, true, true, false, 2866, 1, false, false,
    2000, CHRYSLER_LONG_STOP_GO_SPEED_MAX_RAW + 1,
    false, false, false, CHRYSLER_LONG_LOW_DRIVE));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, true, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, false, 2000, CHRYSLER_LONG_STOP_GO_SPEED_MAX_RAW + 1,
    false, false, false, CHRYSLER_LONG_LOW_DRIVE));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, true, 2100, CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW - 1,
    false, false, false, CHRYSLER_LONG_LOW_DRIVE));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, true, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, true, 2100, CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW,
    false, false, false, CHRYSLER_LONG_LOW_DRIVE));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 2866, 1, false, true,
    2100, 100, false, false, false, CHRYSLER_LONG_LOW_DRIVE));

  assert(chrysler_long_speed_valid_for_command(
    true, false, 1, false, CHRYSLER_LONG_STOP_GO_SPEED_MAX_RAW,
    CHRYSLER_LONG_LOW_HOLD));
  assert(!chrysler_long_speed_valid_for_command(
    true, false, 1, false, CHRYSLER_LONG_STOP_GO_SPEED_MAX_RAW + 1,
    CHRYSLER_LONG_LOW_HOLD));
  assert(chrysler_long_speed_valid_for_command(
    false, false, 0, true, CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW,
    CHRYSLER_LONG_LOW_DRIVE));
  assert(!chrysler_long_speed_valid_for_command(
    false, false, 0, true, CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW - 1,
    CHRYSLER_LONG_LOW_DRIVE));
  assert(chrysler_long_speed_valid_for_command(
    false, false, 0, true, 0, CHRYSLER_LONG_LOW_CREEP));
  assert(chrysler_long_speed_valid_for_command(
    false, false, 1, false, 0, CHRYSLER_LONG_LOW_DRIVE));

  uint8_t low_state = CHRYSLER_LONG_LOW_DRIVE;
  uint8_t go_cycles = 0U;
  uint8_t next_low_state = 0U;
  uint8_t next_go_cycles = 0U;
  // Ordinary low-speed braking cannot authorize GO until STOP/HOLD was seen.
  assert(chrysler_long_low_speed_transition(
    low_state, go_cycles, false, false, 1, false, 5,
    &next_low_state, &next_go_cycles));
  assert(next_low_state == CHRYSLER_LONG_LOW_DRIVE);
  assert(!chrysler_long_low_speed_transition(
    next_low_state, next_go_cycles, false, true, 0, false, 5,
    &low_state, &go_cycles));
  assert(chrysler_long_low_speed_transition(
    next_low_state, next_go_cycles, true, false, 1, false, 0,
    &low_state, &go_cycles));
  assert(low_state == CHRYSLER_LONG_LOW_HOLD);
  // GO cannot overlap the hold; a separate release cycle is mandatory.
  assert(!chrysler_long_low_speed_transition(
    low_state, go_cycles, false, true, 0, false, 0,
    &next_low_state, &next_go_cycles));
  // Residual brake release ramp cycles remain HOLD and cannot authorize GO.
  assert(chrysler_long_low_speed_transition(
    low_state, go_cycles, false, false, 1, false, 0,
    &low_state, &go_cycles));
  assert(low_state == CHRYSLER_LONG_LOW_HOLD);
  assert(!chrysler_long_low_speed_transition(
    low_state, go_cycles, false, true, 0, false, 0,
    &next_low_state, &next_go_cycles));
  // One complete neutral cycle is the only HOLD -> RELEASE transition.
  assert(chrysler_long_low_speed_transition(
    low_state, go_cycles, false, false, 0, false, 0,
    &low_state, &go_cycles));
  assert(low_state == CHRYSLER_LONG_LOW_RELEASE);
  for (uint8_t cycle = 1U;
       cycle <= CHRYSLER_LONG_LOW_GO_MAX_CYCLES; cycle++) {
    assert(chrysler_long_low_speed_transition(
      low_state, go_cycles, false, true, 0, false, 0,
      &next_low_state, &next_go_cycles));
    low_state = next_low_state;
    go_cycles = next_go_cycles;
    assert(low_state == CHRYSLER_LONG_LOW_GO);
    assert(go_cycles == cycle);
  }
  assert(!chrysler_long_low_speed_transition(
    low_state, go_cycles, false, true, 0, false, 0,
    &next_low_state, &next_go_cycles));
  assert(chrysler_long_low_speed_transition(
    low_state, go_cycles, false, false, 0, false, 0,
    &low_state, &go_cycles));
  assert(low_state == CHRYSLER_LONG_LOW_CREEP);
  assert(!chrysler_long_low_speed_transition(
    low_state, go_cycles, false, true, 0, false, 0,
    &next_low_state, &next_go_cycles));
  assert(chrysler_long_low_speed_transition(
    low_state, go_cycles, false, false, 0, true, 0,
    &next_low_state, &next_go_cycles));
  assert(next_low_state == CHRYSLER_LONG_LOW_CREEP);
  assert(chrysler_long_low_speed_transition(
    low_state, go_cycles, false, false, 0, true,
    CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW,
    &low_state, &go_cycles));
  assert(low_state == CHRYSLER_LONG_LOW_DRIVE);
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, 3200, 0, false, false,
    2000, 100, false, false, false, CHRYSLER_LONG_LOW_DRIVE));
  assert(!chrysler_long_commands_valid(
    true, true, true, false, false, CHRYSLER_LONG_DECEL_INACTIVE_RAW,
    0, false, false,
    2100, 100, false, false, false, CHRYSLER_LONG_LOW_DRIVE));

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

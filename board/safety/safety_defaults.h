#include "../chrysler_long_guard.h"

static bool chrysler_long_host_requested = false;
static bool chrysler_long_driver_brake = false;
static bool chrysler_long_driver_gas = false;
static bool chrysler_long_brake_valid = false;
static bool chrysler_long_dash_valid = false;
static bool chrysler_long_torque_valid = false;
static bool chrysler_long_brake_counter_seen = false;
static bool chrysler_long_dash_counter_seen = false;
static bool chrysler_long_torque_counter_seen = false;
static bool chrysler_long_brake_counter_valid = false;
static bool chrysler_long_dash_counter_valid = false;
static bool chrysler_long_torque_counter_valid = false;
static bool chrysler_long_brake_checksum_valid = false;
static bool chrysler_long_dash_checksum_valid = false;
static bool chrysler_long_torque_checksum_valid = false;
static bool chrysler_long_speed_valid = false;
static bool chrysler_long_gas_pedal_valid = false;
static bool chrysler_long_brake_pedal_valid = false;
static bool chrysler_long_stock_acc_valid = false;
static bool chrysler_long_dashboard_valid = false;
static bool chrysler_long_dashboard_fault = false;
static bool chrysler_long_cancel_injected = false;
static bool chrysler_long_owner_diag_stock_valid = false;
static bool chrysler_long_owner_diag_stock_available = false;
static bool chrysler_long_owner_diag_stock_active = false;
static bool chrysler_long_owner_diag_stock_collision = false;
static bool chrysler_steering_stock_counter_seen = false;
static bool chrysler_long_committed_valid = false;
static bool chrysler_long_staged_host_requested = false;
static bool chrysler_long_staged_acc_available = false;
static bool chrysler_long_staged_acc_enabled = false;
static bool chrysler_long_staged_acc_stop = false;
static bool chrysler_long_staged_acc_go = false;
static bool chrysler_long_staged_acc_brk_prep = false;
static bool chrysler_long_staged_engine_torque_request_max = false;
static uint32_t chrysler_long_last_brake_ts = 0U;
static uint32_t chrysler_long_last_dash_ts = 0U;
static uint32_t chrysler_long_last_torque_ts = 0U;
static uint32_t chrysler_long_last_commit_ts = 0U;
static uint32_t chrysler_long_last_speed_ts = 0U;
static uint32_t chrysler_long_last_gas_pedal_ts = 0U;
static uint32_t chrysler_long_last_brake_pedal_ts = 0U;
static uint32_t chrysler_long_last_stock_acc_ts = 0U;
static uint32_t chrysler_long_last_dashboard_ts = 0U;
static uint32_t chrysler_long_cancel_start_ts = 0U;
static int chrysler_long_brake_counter = 0;
static int chrysler_long_dash_counter = 0;
static int chrysler_long_torque_counter = 0;
static int chrysler_long_vehicle_speed_raw = 0;
static int chrysler_long_staged_acc_decel_cmd = CHRYSLER_LONG_DECEL_INACTIVE_RAW;
static int chrysler_long_staged_command_type = 0;
static int chrysler_long_staged_engine_torque_raw = CHRYSLER_LONG_TORQUE_ZERO_RAW;
static int chrysler_long_committed_counter = 0;
static int chrysler_steering_stock_counter = 0;
static int chrysler_steering_stock_source_bus = -1;
static int chrysler_long_stock_acc_fault = 0;
static int chrysler_long_owner_diag_stock_fault = 0;
static uint8_t chrysler_long_owner_state = CHRYSLER_LONG_OWNER_OFF;
static uint8_t chrysler_long_inactive_confirm_frames = 0U;
static uint8_t chrysler_long_low_speed_state = CHRYSLER_LONG_LOW_DRIVE;
static uint8_t chrysler_long_low_speed_go_cycles = 0U;
static uint16_t chrysler_long_guard_failure_mask = 0xFFFFU;
static uint8_t chrysler_long_diag_status = CHRYSLER_LONG_DIAG_SIGNATURE;
static uint16_t chrysler_long_diag_failure_mask = 0xFFFFU;
static uint8_t chrysler_long_diag_counters = 0U;
static uint16_t chrysler_long_diag_stock_engine_word = 0U;
static uint16_t chrysler_long_diag_output_engine_word = 0U;
static uint16_t chrysler_long_diag_stock_accel_word = 0U;

static void chrysler_steering_update_guard(void) {
  steer_type = chrysler_steer_mode_from_stock_das3(
    chrysler_steering_stock_source_bus,
    TIM2->CNT,
    chrysler_long_last_stock_acc_ts,
    chrysler_long_stock_acc_valid,
    org_acc_available);
}

static void chrysler_long_invalidate_committed_cycle(void) {
  chrysler_long_committed_valid = false;
  chrysler_long_owner_state = CHRYSLER_LONG_OWNER_OFF;
  chrysler_long_inactive_confirm_frames = 0U;
  chrysler_long_cancel_injected = false;
  chrysler_long_cancel_start_ts = 0U;
  chrysler_long_low_speed_state = CHRYSLER_LONG_LOW_DRIVE;
  chrysler_long_low_speed_go_cycles = 0U;
}

static void chrysler_long_try_commit_staged_cycle(const uint32_t now) {
  if (!chrysler_long_staged_cycle_ready(
      chrysler_long_brake_valid, chrysler_long_brake_counter,
      chrysler_long_dash_valid, chrysler_long_dash_counter,
      chrysler_long_torque_valid, chrysler_long_torque_counter)) {
    return;
  }

  const bool context_fresh =
    chrysler_long_is_fresh(
      now, chrysler_long_last_speed_ts, chrysler_long_speed_valid,
      CHRYSLER_LONG_SPEED_TIMEOUT_US) &&
    chrysler_long_is_fresh(
      now, chrysler_long_last_gas_pedal_ts, chrysler_long_gas_pedal_valid,
      CHRYSLER_LONG_GAS_PEDAL_TIMEOUT_US) &&
    chrysler_long_is_fresh(
      now, chrysler_long_last_brake_pedal_ts,
      chrysler_long_brake_pedal_valid,
      CHRYSLER_LONG_BRAKE_PEDAL_TIMEOUT_US) &&
    chrysler_long_is_fresh(
      now, chrysler_long_last_stock_acc_ts, chrysler_long_stock_acc_valid,
      CHRYSLER_LONG_STOCK_ACC_TIMEOUT_US) &&
    chrysler_long_dashboard_ready(
      now, chrysler_long_last_dashboard_ts, chrysler_long_dashboard_valid,
      chrysler_long_dashboard_fault);
  const bool staged_commands_valid = chrysler_long_commands_valid(
    chrysler_long_staged_host_requested,
    chrysler_long_staged_acc_available,
    chrysler_long_staged_acc_enabled,
    chrysler_long_staged_acc_stop,
    chrysler_long_staged_acc_go,
    chrysler_long_staged_acc_decel_cmd,
    chrysler_long_staged_command_type,
    chrysler_long_staged_acc_brk_prep,
    chrysler_long_staged_engine_torque_request_max,
    chrysler_long_staged_engine_torque_raw,
    chrysler_long_vehicle_speed_raw,
    chrysler_long_driver_brake,
    chrysler_long_driver_gas,
    org_collision_active,
    chrysler_long_low_speed_state);
  uint8_t next_low_speed_state = chrysler_long_low_speed_state;
  uint8_t next_low_speed_go_cycles = chrysler_long_low_speed_go_cycles;
  const bool low_speed_transition_valid =
    chrysler_long_low_speed_transition(
      chrysler_long_low_speed_state,
      chrysler_long_low_speed_go_cycles,
      chrysler_long_staged_acc_stop,
      chrysler_long_staged_acc_go,
      chrysler_long_staged_command_type,
      chrysler_long_staged_engine_torque_request_max,
      chrysler_long_vehicle_speed_raw,
      &next_low_speed_state,
      &next_low_speed_go_cycles);
  if (!context_fresh || !org_acc_available || !staged_commands_valid ||
      !low_speed_transition_valid) {
    // A complete but unsafe cycle supersedes the old command with a fail-off
    // decision; it must never leave the prior actuation snapshot armed.
    chrysler_long_invalidate_committed_cycle();
    return;
  }

  // Publish the complete counter-matched snapshot in one operation from the
  // guard's point of view. Until this point, the previous committed command
  // remains active and cannot be mixed with fields from the next cycle.
  chrysler_long_host_requested = chrysler_long_staged_host_requested;
  acc_available = chrysler_long_staged_acc_available;
  acc_enabled = chrysler_long_staged_acc_enabled;
  acc_stop = chrysler_long_staged_acc_stop;
  acc_go = chrysler_long_staged_acc_go;
  acc_decel_cmd = chrysler_long_staged_acc_decel_cmd;
  command_type = chrysler_long_staged_command_type;
  acc_brk_prep = chrysler_long_staged_acc_brk_prep;
  engine_torque_request_max =
    chrysler_long_staged_engine_torque_request_max;
  engine_torque_raw = chrysler_long_staged_engine_torque_raw;
  chrysler_long_low_speed_state = next_low_speed_state;
  chrysler_long_low_speed_go_cycles = next_low_speed_go_cycles;
  chrysler_long_committed_counter = chrysler_long_brake_counter;
  chrysler_long_last_commit_ts = now;
  chrysler_long_committed_valid = true;
}

static void chrysler_long_update_guard(void) {
  const uint32_t now = TIM2->CNT;
  const bool committed_fresh =
    chrysler_long_is_fresh(now, chrysler_long_last_commit_ts,
                           chrysler_long_committed_valid,
                           CHRYSLER_LONG_BRAKE_TIMEOUT_US);
  // Preserve the existing diagnostic layout. These three bits now report the
  // freshness of the one atomic command snapshot instead of transient staging
  // state from the individual CAN frames.
  const bool brake_fresh = committed_fresh;
  const bool dash_fresh = committed_fresh;
  const bool torque_fresh = committed_fresh;
  const bool speed_fresh =
    chrysler_long_is_fresh(now, chrysler_long_last_speed_ts,
                           chrysler_long_speed_valid, CHRYSLER_LONG_SPEED_TIMEOUT_US);
  const bool gas_fresh =
    chrysler_long_is_fresh(now, chrysler_long_last_gas_pedal_ts,
                           chrysler_long_gas_pedal_valid, CHRYSLER_LONG_GAS_PEDAL_TIMEOUT_US);
  const bool brake_pedal_fresh =
    chrysler_long_is_fresh(now, chrysler_long_last_brake_pedal_ts,
                           chrysler_long_brake_pedal_valid, CHRYSLER_LONG_BRAKE_PEDAL_TIMEOUT_US);
  const bool stock_acc_fresh =
    chrysler_long_is_fresh(now, chrysler_long_last_stock_acc_ts,
                           chrysler_long_stock_acc_valid, CHRYSLER_LONG_STOCK_ACC_TIMEOUT_US);
  // DAS_4 is the dashboard fault source that actually asserted in b6v. Fold
  // its freshness and fault bit into the existing stock-ACC readiness gate so
  // the diagnostic layout remains stable while the actuation gate tightens.
  const bool dashboard_ready = chrysler_long_dashboard_ready(
    now, chrysler_long_last_dashboard_ts, chrysler_long_dashboard_valid,
    chrysler_long_dashboard_fault);
  const bool stock_acc_ready =
    stock_acc_fresh && dashboard_ready && org_acc_available;
  const bool counters_aligned = chrysler_long_committed_valid;
  const bool private_integrity_valid = chrysler_long_committed_valid;
  const bool messages_fresh =
    brake_fresh &&
    dash_fresh &&
    torque_fresh &&
    speed_fresh &&
    gas_fresh &&
    brake_pedal_fresh &&
    stock_acc_ready &&
    counters_aligned &&
    private_integrity_valid;

  const bool commands_valid = chrysler_long_commands_valid(
    chrysler_long_host_requested,
    acc_available,
    acc_enabled,
    acc_stop,
    acc_go,
    acc_decel_cmd,
    command_type,
    acc_brk_prep,
    engine_torque_request_max,
    engine_torque_raw,
    chrysler_long_vehicle_speed_raw,
    chrysler_long_driver_brake,
    chrysler_long_driver_gas,
    org_collision_active,
    chrysler_long_low_speed_state);

  chrysler_long_guard_failure_mask = chrysler_long_diagnostic_mask(
    CHRYSLER_LONG_ACTUATION != 0U,
    chrysler_long_host_requested,
    brake_fresh,
    dash_fresh,
    torque_fresh,
    speed_fresh,
    gas_fresh,
    brake_pedal_fresh,
    stock_acc_ready,
    counters_aligned,
    private_integrity_valid,
    chrysler_long_speed_valid_for_command(
      acc_stop, acc_go, command_type, engine_torque_request_max,
      chrysler_long_vehicle_speed_raw, chrysler_long_low_speed_state),
    chrysler_long_driver_brake,
    chrysler_long_driver_gas,
    org_collision_active,
    commands_valid);
  // Keep the b6y actuation decision independent from diagnostic packing.
  is_oplong_enabled = (CHRYSLER_LONG_ACTUATION != 0U) &&
                      messages_fresh && commands_valid;
}

static uint8_t fca_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  /* This function does not want the checksum byte in the input data.
  jeep chrysler canbus checksum from http://illmatics.com/Remote%20Car%20Hacking.pdf */
  uint8_t checksum = 0xFF;
  int len = GET_LEN(to_push);
  for (int j = 0; j < (len - 1); j++) {
    uint8_t shift = 0x80;
    uint8_t curr = (uint8_t)GET_BYTE(to_push, j);
    for (int i=0; i<8; i++) {
      uint8_t bit_sum = curr & shift;
      uint8_t temp_chk = checksum & 0x80U;
      if (bit_sum != 0U) {
        bit_sum = 0x1C;
        if (temp_chk != 0U) {
          bit_sum = 1;
        }
        checksum = checksum << 1;
        temp_chk = checksum | 1U;
        bit_sum ^= temp_chk;
      } else {
        if (temp_chk != 0U) {
          bit_sum = 0x1D;
        }
        checksum = checksum << 1;
        bit_sum ^= checksum;
      }
      checksum = bit_sum;
      shift = shift >> 1;
    }
  }
  return ~checksum;
}

static void send_steer_enable_speed(CAN_FIFOMailBox_TypeDef *to_fwd){
  int crc;
  int kph_factor = 128;
  int eps_cutoff_speed;
  int lkas_enable_speed = 65 * kph_factor;
  int apa_enable_speed = 0 * kph_factor;
  int veh_speed = GET_BYTE(to_fwd, 4) | GET_BYTE(to_fwd, 5) << 8;

  // Re-evaluate immediately before modifying the frame so stale or invalid
  // CAN2 DAS_3 state fails disabled even when no newer DAS_3 has arrived.
  chrysler_steering_update_guard();

  eps_cutoff_speed = veh_speed;

  if(steer_type == CHRYSLER_STEER_MODE_APA) {
    eps_cutoff_speed = apa_enable_speed >> 8 | ((apa_enable_speed << 8) & 0xFFFF);  //2kph with 128 factor
  }
  else if (steer_type == CHRYSLER_STEER_MODE_LKAS) {
    eps_cutoff_speed = lkas_enable_speed >> 8 | ((lkas_enable_speed << 8) & 0xFFFF);  //65kph with 128 factor
  }

  to_fwd->RDHR &= 0x00FF0000;  //clear speed and Checksum
  to_fwd->RDHR |= eps_cutoff_speed;       //replace speed
  crc = fca_compute_checksum(to_fwd);
  to_fwd->RDHR |= (((crc << 8) << 8) << 8);   //replace Checksum
};

static void send_trans_apa_signature(CAN_FIFOMailBox_TypeDef *to_fwd){
  int gear_R = 0xB;
  if (steer_type == CHRYSLER_STEER_MODE_APA) {
    to_fwd->RDLR &= 0xFFFFF0FF;  //clear speed and Checksum
    to_fwd->RDLR |= gear_R << 8;  //replace gear
  }
}
static void send_shifter_apa_signature(CAN_FIFOMailBox_TypeDef *to_fwd){
  int shifter_R = 0x1;
  if (steer_type == CHRYSLER_STEER_MODE_APA) {
    to_fwd->RDLR &= 0xFFFFFFE0;  //clear speed and Checksum
    to_fwd->RDLR |= shifter_R << 2;  //replace shifter
  }
}

static void send_rev_apa_signature(CAN_FIFOMailBox_TypeDef *to_fwd){
  if (steer_type == CHRYSLER_STEER_MODE_APA) {
    to_fwd->RDLR &= 0xFFFFFFEF;  //clear REV and Checksum
  }
}

static void send_wspd_apa_signature(CAN_FIFOMailBox_TypeDef *to_fwd){
  if (steer_type == CHRYSLER_STEER_MODE_APA) {
    to_fwd->RDLR &= 0x00000000;  //clear speed and Checksum
  }
}

static void send_count_apa_signature(CAN_FIFOMailBox_TypeDef *to_fwd){
  if (steer_type == CHRYSLER_STEER_MODE_APA) {
    to_fwd->RDLR &= 0x00000000;  //clear speed and Checksum
    to_fwd->RDHR &= 0x00000000;  //clear speed and Checksum
  }
}

static void send_xxx_apa_signature(CAN_FIFOMailBox_TypeDef *to_fwd){
  to_fwd->RDLR &= 0x00000000;  //clear speed and Checksum
}

static void send_apa_signature(CAN_FIFOMailBox_TypeDef *to_fwd){
  int crc;
  int multi = 4; // steering torq multiplier
  int apa_torq = ((lkas_torq - 1024) * multi/4) + 1024;  //LKAS torq 768 to 1280 +-0.5NM  512  //APA torq 896 to 1152 +-1NM 128 0x80

  if ((steer_type == CHRYSLER_STEER_MODE_APA) && is_op_active) {
    to_fwd->RDLR &= 0x00000000;  //clear everything for new apa
    to_fwd->RDLR |= 0x50;  //replace apa req to true
    to_fwd->RDLR |= 0x20 << 8 << 8;  //replace apa type = 1
    to_fwd->RDLR |= apa_torq >> 8;  //replace torq
    to_fwd->RDLR |= (apa_torq & 0xFF) << 8;  //replace torq
  }
  to_fwd->RDHR &= 0x00FF0000;  //clear everything except counter
  crc = fca_compute_checksum(to_fwd);
  to_fwd->RDHR |= (((crc << 8) << 8) << 8);   //replace Checksum
};

static void send_acc_decel_msg(CAN_FIFOMailBox_TypeDef *to_fwd){
  int crc;
  chrysler_long_diag_stock_engine_word =
    (uint16_t)(to_fwd->RDLR & 0xFFFFU);
  chrysler_long_diag_stock_accel_word =
    (uint16_t)((to_fwd->RDLR >> 16) & 0xFFFFU);
  chrysler_long_update_guard();

  // Use every ownership-relevant field from this exact stock frame. The
  // forwarding path runs before rx_hook updates the cached stock state.
  const bool stock_available = (GET_BYTE(to_fwd, 2) >> 4) & 0x1;
  const bool stock_active = (GET_BYTE(to_fwd, 2) >> 5) & 0x1;
  const int stock_command_type = (GET_BYTE(to_fwd, 4) >> 4) & 0x7;
  const int stock_fault = (GET_BYTE(to_fwd, 5) >> 6) & 0x3;
  const bool stock_collision =
    (GET_BYTE(to_fwd, 6) & 0x1) || (stock_command_type > 1);
  const int stock_counter = (GET_BYTE(to_fwd, 6) >> 4) & 0xF;
  const bool stock_checksum_valid =
    (GET_LEN(to_fwd) == 8) &&
    (GET_BYTE(to_fwd, 7) == fca_compute_checksum(to_fwd));
  const bool current_stock_valid = chrysler_long_current_stock_frame_valid(
    chrysler_steering_stock_counter_seen,
    chrysler_steering_stock_counter,
    stock_counter,
    GET_LEN(to_fwd),
    stock_checksum_valid);
  const bool dashboard_ready = chrysler_long_dashboard_ready(
    TIM2->CNT, chrysler_long_last_dashboard_ts,
    chrysler_long_dashboard_valid, chrysler_long_dashboard_fault);
  const bool guarded_stock_valid = current_stock_valid && dashboard_ready;
  const int combined_stock_fault =
    stock_fault | (chrysler_long_dashboard_fault ? 1 : 0);

  chrysler_long_owner_diag_stock_valid = guarded_stock_valid;
  chrysler_long_owner_diag_stock_available = stock_available;
  chrysler_long_owner_diag_stock_active = stock_active;
  chrysler_long_owner_diag_stock_fault = combined_stock_fault;
  chrysler_long_owner_diag_stock_collision = stock_collision;

  const uint8_t previous_owner_state = chrysler_long_owner_state;
  const bool valid_inactive_candidate =
    (chrysler_long_owner_state == CHRYSLER_LONG_OWNER_OFF) &&
    is_oplong_enabled && guarded_stock_valid && stock_available &&
    !stock_active && (combined_stock_fault == 0) && !stock_collision;
  chrysler_long_inactive_confirm_frames =
    chrysler_long_update_inactive_confirmation(
      chrysler_long_inactive_confirm_frames, valid_inactive_candidate);
  const bool stock_inactive_confirmed =
    chrysler_long_inactive_confirmation_complete(
      chrysler_long_inactive_confirm_frames);
  chrysler_long_owner_state = chrysler_long_next_owner_state(
    chrysler_long_owner_state, is_oplong_enabled, guarded_stock_valid,
    stock_available,
    stock_active, stock_inactive_confirmed, combined_stock_fault,
    stock_collision);
  if (chrysler_long_owner_state != CHRYSLER_LONG_OWNER_OFF) {
    chrysler_long_inactive_confirm_frames = 0U;
  }
  if ((chrysler_long_owner_state == CHRYSLER_LONG_OWNER_CANCELING) &&
      (previous_owner_state != CHRYSLER_LONG_OWNER_CANCELING)) {
    chrysler_long_cancel_start_ts = TIM2->CNT;
  }
  if (chrysler_long_cancel_timed_out(
      chrysler_long_owner_state, TIM2->CNT,
      chrysler_long_cancel_start_ts)) {
    chrysler_long_owner_state = CHRYSLER_LONG_OWNER_FAILED;
    chrysler_long_cancel_injected = false;
  }
  const bool applied = chrysler_long_should_substitute_das3(
    chrysler_long_owner_state, is_oplong_enabled, guarded_stock_valid,
    combined_stock_fault,
    stock_collision, stock_available, stock_active);
  chrysler_long_diag_status =
    CHRYSLER_LONG_DIAG_SIGNATURE |
    (applied ? CHRYSLER_LONG_DIAG_APPLIED : 0U) |
    (chrysler_long_host_requested ? CHRYSLER_LONG_DIAG_HOST_REQUESTED : 0U) |
    ((command_type == 1) ? CHRYSLER_LONG_DIAG_BRAKE_REQUESTED : 0U) |
    (engine_torque_request_max ? CHRYSLER_LONG_DIAG_ENGINE_REQUESTED : 0U);
  chrysler_long_diag_failure_mask = chrysler_long_guard_failure_mask;
  chrysler_long_diag_counters =
    (uint8_t)(((chrysler_long_committed_counter & 0xF) << 4) |
              (stock_counter & 0xF));

  if (applied) {
    // Preserve the two stock DAS_3 byte-2 bits that are outside the private
    // protocol. Replace propulsion and braking in this one diesel DAS_3 frame.
    to_fwd->RDLR &= 0x00C00000U;
    to_fwd->RDLR |= (uint32_t)((engine_torque_raw >> 8) & 0x1F);
    to_fwd->RDLR |= (uint32_t)(engine_torque_raw & 0xFF) << 8;
    to_fwd->RDLR |= (uint32_t)engine_torque_request_max << 7;

    to_fwd->RDLR |= (uint32_t)acc_stop << 5;
    to_fwd->RDLR |= (uint32_t)acc_go << 6;
    to_fwd->RDLR |= (uint32_t)((acc_decel_cmd >> 8) & 0xF) << 16;
    to_fwd->RDLR |= (uint32_t)acc_available << 20;
    to_fwd->RDLR |= (uint32_t)acc_enabled << 21;
    to_fwd->RDLR |= (uint32_t)(acc_decel_cmd & 0xFF) << 24;

    // Preserve stock fault, collision, counter, and unrelated bits.
    to_fwd->RDHR &= ~((uint32_t)0x70U | ((uint32_t)1U << 17));
    to_fwd->RDHR |= (uint32_t)command_type << 4;
    to_fwd->RDHR |= (uint32_t)acc_brk_prep << 17;

    to_fwd->RDHR &= 0x00FFFFFFU;
    crc = fca_compute_checksum(to_fwd);
    to_fwd->RDHR |= (uint32_t)crc << 24;
  }
  else { //pass through
    to_fwd->RDLR |= 0x00000000;
    to_fwd->RDHR |= 0x00000000;
  }
  chrysler_long_diag_output_engine_word =
    (uint16_t)(to_fwd->RDLR & 0xFFFFU);
}

static void send_acc_dash_msg(CAN_FIFOMailBox_TypeDef *to_fwd){
  // The private 0x1F7 carries only the host enable plus protocol integrity.
  // Factory DAS_4 dashboard state is never replaced. Observe its native fault
  // bit before forwarding, and revoke the committed command immediately when
  // the frame is malformed or the dashboard reports an ACC/FCW fault.
  chrysler_long_dashboard_valid = GET_LEN(to_fwd) == 8;
  chrysler_long_dashboard_fault =
    !chrysler_long_dashboard_valid ||
    chrysler_long_dashboard_fault_from_byte6(GET_BYTE(to_fwd, 6));
  chrysler_long_last_dashboard_ts = TIM2->CNT;
  if (chrysler_long_dashboard_fault) {
    chrysler_long_invalidate_committed_cycle();
  }
  chrysler_long_update_guard();
  to_fwd->RDLR |= 0x00000000U;
  to_fwd->RDHR |= 0x00000000U;
}

static void send_acc_accel_msg(CAN_FIFOMailBox_TypeDef *to_fwd){
  // This EcoDiesel did not use DAS_5 wheel torque in 130.2 minutes of stock
  // ACC logs. Propulsion belongs in DAS_3, so always pass DAS_5 through.
  chrysler_long_update_guard();
  to_fwd->RDLR |= 0x00000000U;
  to_fwd->RDHR |= 0x00000000U;
}

static void send_wheel_button_msg(CAN_FIFOMailBox_TypeDef *to_fwd){
  chrysler_long_update_guard();
  const uint8_t original_buttons = GET_BYTE(to_fwd, 0);
  // Physical or host-generated Cancel and ACC main presses revoke ownership
  // before the next DAS_3 frame can be modified.
  if ((original_buttons & 0x01U) || (original_buttons & 0xC0U)) {
    chrysler_long_invalidate_committed_cycle();
  }
  const uint8_t filtered_buttons = chrysler_long_filter_button_byte(
    original_buttons, chrysler_long_owner_state);
  chrysler_long_cancel_injected =
    (chrysler_long_owner_state == CHRYSLER_LONG_OWNER_CANCELING) &&
    ((original_buttons & 0x01U) == 0U) &&
    ((filtered_buttons & 0x01U) != 0U);
  to_fwd->RDLR &= ~0xFFU;
  to_fwd->RDLR |= filtered_buttons;
  // CRUISE_BUTTONS is three bytes: retain its counter in byte 1 and rebuild
  // byte 2 after changing byte 0.
  to_fwd->RDLR &= 0x0000FFFFU;
  const uint8_t crc = fca_compute_checksum(to_fwd);
  to_fwd->RDLR |= (uint32_t)crc << 16;
}

static void create_chrysler_wp_status_diagnostic(
    CAN_FIFOMailBox_TypeDef *to_send) {
  to_send->RIR = (0x4FFU << 21) | 1U;
  to_send->RDTR = 4U;
  to_send->RDLR = chrysler_long_status_diagnostic_word(
    chrysler_long_diag_status,
    chrysler_long_diag_failure_mask,
    chrysler_long_diag_counters);
  to_send->RDHR = 0U;
}

static void create_chrysler_wp_command_diagnostic(
    CAN_FIFOMailBox_TypeDef *to_send) {
  to_send->RIR = (0x4FEU << 21) | 1U;
  to_send->RDTR = 8U;
  to_send->RDLR = chrysler_long_command_diagnostic_low(
    chrysler_long_diag_stock_engine_word);
  to_send->RDHR = chrysler_long_command_diagnostic_high(
    chrysler_long_diag_output_engine_word,
    chrysler_long_diag_stock_accel_word);
}

static void create_chrysler_wp_owner_diagnostic(
    CAN_FIFOMailBox_TypeDef *to_send) {
  to_send->RIR = (0x4FDU << 21) | 1U;
  to_send->RDTR = 4U;
  to_send->RDLR = chrysler_long_owner_diagnostic_word(
    chrysler_long_owner_state,
    chrysler_long_owner_diag_stock_valid,
    chrysler_long_owner_diag_stock_available,
    chrysler_long_owner_diag_stock_active,
    chrysler_long_owner_diag_stock_fault,
    chrysler_long_owner_diag_stock_collision,
    chrysler_long_cancel_injected);
  to_send->RDHR = 0U;
}

int default_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);
  int bus_num = GET_BUS(to_push);

  if ((addr == 658) && (bus_num == 0)) {
    is_op_active = (GET_BYTE(to_push, 0) >> 4) & 0x1;
    lkas_torq = ((GET_BYTE(to_push, 0) & 0x7) << 8) | GET_BYTE(to_push, 1);
    counter_658 += 1;
  }

  if ((addr == 514) && (bus_num == 0)) {
    const int speed_left_raw = (GET_BYTE(to_push, 0) << 4) |
                               (GET_BYTE(to_push, 1) >> 4);
    const int speed_right_raw = (GET_BYTE(to_push, 2) << 4) |
                                (GET_BYTE(to_push, 3) >> 4);
    chrysler_long_vehicle_speed_raw = (speed_left_raw + speed_right_raw) / 2;
    chrysler_long_last_speed_ts = TIM2->CNT;
    chrysler_long_speed_valid = true;
    chrysler_long_update_guard();
  }

  if ((addr == 284) && (bus_num == 0)) {
    // Longitudinal command liveness is enforced by the monotonic 100 ms
    // timestamp watchdogs above. Do not compare the 50 Hz stock 0x11C count
    // with the independent 25 Hz private 0x1F6 count: that legacy comparison
    // necessarily drifted and invalidated a healthy command once per second.
    if (counter_658 > 0) {
        counter_284_658 += 2;
        if (counter_284_658 - counter_658 > 25){
            is_op_active = false;
            steer_type = CHRYSLER_STEER_MODE_DISABLED;
            counter_658 = 0;
            counter_284_658 = 0;
        }
    }
  }

  if ((addr == 502) && (bus_num == 0)) {
    const uint32_t now = TIM2->CNT;
    if ((uint32_t)(now - chrysler_long_last_brake_ts) >
        CHRYSLER_LONG_BRAKE_TIMEOUT_US) {
      chrysler_long_brake_counter_seen = false;
    }
    const int current_counter = (GET_BYTE(to_push, 6) >> 4) & 0xF;
    chrysler_long_brake_checksum_valid =
      (GET_LEN(to_push) == 8) &&
      (GET_BYTE(to_push, 7) == fca_compute_checksum(to_push));
    chrysler_long_brake_counter_valid =
      chrysler_long_brake_checksum_valid &&
      chrysler_long_counter_step_valid(
        &chrysler_long_brake_counter_seen,
        &chrysler_long_brake_counter, current_counter);
    chrysler_long_brake_valid =
      chrysler_long_brake_checksum_valid && chrysler_long_brake_counter_valid;
    if (chrysler_long_brake_valid) {
      chrysler_long_staged_acc_stop = (GET_BYTE(to_push, 0) >> 5) & 0x1;
      chrysler_long_staged_acc_go = (GET_BYTE(to_push, 0) >> 6) & 0x1;
      chrysler_long_staged_acc_available =
        (GET_BYTE(to_push, 2) >> 4) & 0x1;
      chrysler_long_staged_acc_enabled =
        (GET_BYTE(to_push, 2) >> 5) & 0x1;
      chrysler_long_staged_acc_decel_cmd =
        ((GET_BYTE(to_push, 2) & 0xF) << 8) | GET_BYTE(to_push, 3);
      chrysler_long_staged_command_type =
        (GET_BYTE(to_push, 4) >> 4) & 0x7;
      chrysler_long_staged_acc_brk_prep =
        (GET_BYTE(to_push, 6) >> 1) & 0x1;
    } else {
      // A malformed command is not a harmless partial update. Revoke the
      // committed snapshot immediately and require a complete valid cycle.
      chrysler_long_invalidate_committed_cycle();
    }
    chrysler_long_last_brake_ts = now;
    chrysler_long_try_commit_staged_cycle(now);
    chrysler_long_update_guard();
  }

  if ((addr == 503) && (bus_num == 0)) {
    const uint32_t now = TIM2->CNT;
    if ((uint32_t)(now - chrysler_long_last_dash_ts) >
        CHRYSLER_LONG_DASH_TIMEOUT_US) {
      chrysler_long_dash_counter_seen = false;
    }
    const int current_counter = (GET_BYTE(to_push, 6) >> 4) & 0xF;
    chrysler_long_dash_checksum_valid =
      (GET_LEN(to_push) == 8) &&
      (GET_BYTE(to_push, 7) == fca_compute_checksum(to_push));
    chrysler_long_dash_counter_valid =
      chrysler_long_dash_checksum_valid &&
      chrysler_long_counter_step_valid(
        &chrysler_long_dash_counter_seen,
        &chrysler_long_dash_counter, current_counter);
    chrysler_long_dash_valid =
      chrysler_long_dash_checksum_valid && chrysler_long_dash_counter_valid;
    if (chrysler_long_dash_valid) {
      chrysler_long_staged_host_requested = GET_BYTE(to_push, 3) & 0x1;
    } else {
      chrysler_long_invalidate_committed_cycle();
    }
    chrysler_long_last_dash_ts = now;
    chrysler_long_try_commit_staged_cycle(now);
    chrysler_long_update_guard();
  }

  if ((addr == 626) && (bus_num == 0)) {
    const uint32_t now = TIM2->CNT;
    if ((uint32_t)(now - chrysler_long_last_torque_ts) >
        CHRYSLER_LONG_TORQUE_TIMEOUT_US) {
      chrysler_long_torque_counter_seen = false;
    }
    const int current_counter = (GET_BYTE(to_push, 6) >> 4) & 0xF;
    chrysler_long_torque_checksum_valid =
      (GET_LEN(to_push) == 8) &&
      (GET_BYTE(to_push, 7) == fca_compute_checksum(to_push));
    chrysler_long_torque_counter_valid =
      chrysler_long_torque_checksum_valid &&
      chrysler_long_counter_step_valid(
        &chrysler_long_torque_counter_seen,
        &chrysler_long_torque_counter, current_counter);
    chrysler_long_torque_valid =
      chrysler_long_torque_checksum_valid && chrysler_long_torque_counter_valid;
    if (chrysler_long_torque_valid) {
      chrysler_long_staged_engine_torque_request_max =
        (GET_BYTE(to_push, 4) >> 7) & 0x1;
      chrysler_long_staged_engine_torque_raw =
        (GET_BYTE(to_push, 4) & 0x7F) << 8 | GET_BYTE(to_push, 5);
    } else {
      chrysler_long_invalidate_committed_cycle();
    }
    chrysler_long_last_torque_ts = now;
    chrysler_long_try_commit_staged_cycle(now);
    chrysler_long_update_guard();
  }

  if ((addr == 559) && (bus_num == 0)) {
    chrysler_long_driver_gas = GET_BYTE(to_push, 0) != 0;
    chrysler_long_last_gas_pedal_ts = TIM2->CNT;
    chrysler_long_gas_pedal_valid = true;
    if (chrysler_long_driver_gas) {
      chrysler_long_invalidate_committed_cycle();
    }
    chrysler_long_update_guard();
  }

  if ((addr == 320) && (bus_num == 0)) {
    chrysler_long_driver_brake = ((GET_BYTE(to_push, 0) >> 2) & 0x3) == 1;
    chrysler_long_last_brake_pedal_ts = TIM2->CNT;
    chrysler_long_brake_pedal_valid = true;
    if (chrysler_long_driver_brake) {
      chrysler_long_invalidate_committed_cycle();
    }
    chrysler_long_update_guard();
  }

  if ((addr == 500) && (bus_num == CHRYSLER_STEER_STOCK_BUS)) {
    const uint32_t now = TIM2->CNT;
    if ((uint32_t)(now - chrysler_long_last_stock_acc_ts) >
        CHRYSLER_STEER_STOCK_TIMEOUT_US) {
      chrysler_steering_stock_counter_seen = false;
    }

    const bool checksum_valid =
      (GET_LEN(to_push) == 8) &&
      (GET_BYTE(to_push, 7) == fca_compute_checksum(to_push));
    const int current_counter = (GET_BYTE(to_push, 6) >> 4) & 0xF;
    const bool counter_valid =
      checksum_valid &&
      chrysler_long_counter_step_valid(
        &chrysler_steering_stock_counter_seen,
        &chrysler_steering_stock_counter,
        current_counter);

    chrysler_steering_stock_source_bus = bus_num;
    chrysler_long_last_stock_acc_ts = now;
    chrysler_long_stock_acc_valid =
      chrysler_steer_stock_das3_integrity_valid(
        bus_num, GET_LEN(to_push), checksum_valid, counter_valid);

    if (chrysler_long_stock_acc_valid) {
      org_acc_available = (GET_BYTE(to_push, 2) >> 4) & 0x1;
      org_cmd_type = (GET_BYTE(to_push, 4) >> 4) & 0x7;
      chrysler_long_stock_acc_fault =
        (GET_BYTE(to_push, 5) >> 6) & 0x3;
      org_brk_pul = GET_BYTE(to_push, 6) & 0x1;
      org_collision_active = org_brk_pul || (org_cmd_type > 1);
    } else {
      // Invalid stock state must not retain a previously permissive steering
      // or longitudinal decision.
      org_acc_available = false;
      chrysler_long_stock_acc_fault = 3;
      org_collision_active = true;
    }
    if (!chrysler_long_stock_acc_valid || !org_acc_available ||
        (chrysler_long_stock_acc_fault != 0) || org_collision_active) {
      chrysler_long_invalidate_committed_cycle();
    }
    chrysler_steering_update_guard();
    chrysler_long_update_guard();
  }
  return true;
}

// *** no output safety mode ***

static void nooutput_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();
}

static int nooutput_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  UNUSED(to_send);
  return false;
}

static int nooutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return false;
}

static int default_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  UNUSED(to_fwd);
  UNUSED(bus_num);

  return -1;
}

const safety_hooks nooutput_hooks = {
  .init = nooutput_init,
  .rx = default_rx_hook,
  .tx = nooutput_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};

// *** all output safety mode ***

static void alloutput_init(int16_t param) {
  UNUSED(param);
  controls_allowed = true;
  relay_malfunction_reset();
}

static int alloutput_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  UNUSED(to_send);
  return true;
}

static int alloutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return true;
}

const safety_hooks alloutput_hooks = {
  .init = alloutput_init,
  .rx = default_rx_hook,
  .tx = alloutput_tx_hook,
  .tx_lin = alloutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};

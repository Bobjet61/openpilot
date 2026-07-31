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
static uint16_t chrysler_long_guard_failure_mask = 0xFFFFU;
static uint8_t chrysler_long_diag_status = CHRYSLER_LONG_DIAG_SIGNATURE;
static uint16_t chrysler_long_diag_failure_mask = 0xFFFFU;
static uint8_t chrysler_long_diag_counters = 0U;

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
      CHRYSLER_LONG_STOCK_ACC_TIMEOUT_US);
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
    org_collision_active);
  if (!context_fresh || !org_acc_available || !staged_commands_valid) {
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
  const bool stock_acc_ready = stock_acc_fresh && org_acc_available;
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
    org_collision_active);

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
    chrysler_long_vehicle_speed_raw >= CHRYSLER_LONG_MOVING_SPEED_MIN_RAW,
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
  chrysler_long_update_guard();

  const bool applied = is_oplong_enabled && !org_collision_active;
  const int stock_counter = (GET_BYTE(to_fwd, 6) >> 4) & 0xF;
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
}

static void send_acc_dash_msg(CAN_FIFOMailBox_TypeDef *to_fwd){
  // The private 0x1F7 carries only the host enable plus protocol integrity.
  // Factory DAS_4 dashboard state is never replaced.
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
  // Preserve physical and host-generated 0x23B commands exactly. In
  // particular, never synthesize the ACC-off bit when longitudinal engages.
  to_fwd->RDLR = chrysler_long_wheel_button_passthrough(to_fwd->RDLR);
  to_fwd->RDHR = chrysler_long_wheel_button_passthrough(to_fwd->RDHR);
}

void chrysler_wp(void) {
  CAN1->sTxMailBox[0].TDLR =
    (uint32_t)chrysler_long_diag_status |
    (uint32_t)(chrysler_long_diag_failure_mask & 0xFFU) << 8 |
    (uint32_t)((chrysler_long_diag_failure_mask >> 8) & 0xFFU) << 16 |
    (uint32_t)chrysler_long_diag_counters << 24;
  CAN1->sTxMailBox[0].TDTR = 4;
  CAN1->sTxMailBox[0].TIR = (0x4FFU << 21) | 1U;
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
    if (chrysler_long_vehicle_speed_raw <
        CHRYSLER_LONG_MOVING_SPEED_MIN_RAW) {
      chrysler_long_invalidate_committed_cycle();
    }
    chrysler_long_update_guard();
  }

  if ((addr == 284) && (bus_num == 0)) {
    if (counter_502 > 0) {
        counter_284_502 += 1;
        if (counter_284_502 - counter_502 > 25) {
            chrysler_long_brake_valid = false;
            chrysler_long_invalidate_committed_cycle();
            acc_enabled = false;
            counter_502 = 0;
            counter_284_502 = 0;
            chrysler_long_update_guard();
        }
    }

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
    counter_502 += 1;
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
      org_brk_pul = GET_BYTE(to_push, 6) & 0x1;
      org_collision_active = org_brk_pul || (org_cmd_type > 1);
    } else {
      // Invalid stock state must not retain a previously permissive steering
      // or longitudinal decision.
      org_acc_available = false;
      org_collision_active = true;
    }
    if (!chrysler_long_stock_acc_valid || !org_acc_available ||
        org_collision_active) {
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

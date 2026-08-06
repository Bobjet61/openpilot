const SteeringLimits CHRYSLER_STEERING_LIMITS = {
  .max_steer = 261,
  .max_rt_delta = 112,
  .max_rt_interval = 250000,
  .max_rate_up = 3,
  .max_rate_down = 3,
  .max_torque_error = 80,
  .type = TorqueMotorLimited,
};

const SteeringLimits CHRYSLER_RAM_DT_STEERING_LIMITS = {
  .max_steer = 350,
  .max_rt_delta = 112,
  .max_rt_interval = 250000,
  .max_rate_up = 6,
  .max_rate_down = 6,
  .max_torque_error = 80,
  .type = TorqueMotorLimited,
};

const SteeringLimits CHRYSLER_RAM_HD_STEERING_LIMITS = {
  .max_steer = 361,
  .max_rt_delta = 182,
  .max_rt_interval = 250000,
  .max_rate_up = 14,
  .max_rate_down = 14,
  .max_torque_error = 80,
  .type = TorqueMotorLimited,
};

const SteeringLimits CHRYSLER_JEEP_RATE4_STEERING_LIMITS = {
  .max_steer = 261,
  .max_rt_delta = 112,
  .max_rt_interval = 250000,
  .max_rate_up = 4,
  .max_rate_down = 4,
  .max_torque_error = 80,
  .type = TorqueMotorLimited,
};

const SteeringLimits CHRYSLER_JEEP_RATE5_STEERING_LIMITS = {
  .max_steer = 261,
  .max_rt_delta = 112,
  .max_rt_interval = 250000,
  .max_rate_up = 5,
  .max_rate_down = 5,
  .max_torque_error = 80,
  .type = TorqueMotorLimited,
};

#define CHRYSLER_LONG_BRAKE_ADDR 0x1F6U
#define CHRYSLER_LONG_DASH_ADDR 0x1F7U
#define CHRYSLER_LONG_TORQUE_ADDR 0x272U
#define CHRYSLER_LONG_DAS_4_ADDR 0x1F5U
#define CHRYSLER_LONG_DECEL_MIN_RAW 2661
#define CHRYSLER_LONG_DECEL_MAX_RAW 3275
#define CHRYSLER_LONG_DECEL_INACTIVE_RAW 4094
#define CHRYSLER_LONG_TORQUE_ZERO_RAW 2000
// b6v asserted the DAS_4 dashboard fault while a 500 Nm request was saturated,
// while an earlier 439.25 Nm peak remained fault-free. The private DAS_3
// scaling is raw * 0.25 - 500 Nm, so raw 3760 independently enforces b6w's
// common 440 Nm ceiling.
#define CHRYSLER_LONG_TORQUE_MAX_RAW 3760
#define CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW 11
#define CHRYSLER_LONG_LAUNCH_TORQUE_MAX_RAW 2800
#define CHRYSLER_LONG_STOP_GO_SPEED_MAX_RAW 12
#define CHRYSLER_LONG_LOW_GO_MAX_CYCLES 4U
#define CHRYSLER_LONG_SOURCE_TIMEOUT_US 100000U
#define CHRYSLER_LONG_MIN_CYCLE_INTERVAL_US 15000U
#define CHRYSLER_LONG_COUNTER_RESET_US 100000U
#define CHRYSLER_B6Y_HOLD_DECEL_RAW 2866
#define CHRYSLER_B6Y_HOLD_TIMEOUT_US 100000U

typedef struct {
  const int EPS_2;
  const int ESP_1;
  const int ESP_8;
  const int ECM_5;
  const int DAS_3;
  const int DAS_6;
  const int LKAS_COMMAND;
  const int LKAS_HEARTBIT;
  const int CRUISE_BUTTONS;
  const int CENTER_STACK_1;
  const int CENTER_STACK_2;
} ChryslerAddrs;

// CAN messages for Chrysler/Jeep platforms
const ChryslerAddrs CHRYSLER_ADDRS = {
  .EPS_2            = 0x220,  // EPS driver input torque
  .ESP_1            = 0x140,  // Brake pedal and vehicle speed
  .ESP_8            = 0x11C,  // Brake pedal and vehicle speed
  .ECM_5            = 0x22F,  // Throttle position sensor
  .DAS_3            = 0x1F4,  // ACC engagement states from DASM
  .DAS_6            = 0x2A6,  // LKAS HUD and auto headlight control from DASM
  .LKAS_COMMAND     = 0x292,  // LKAS controls from DASM
  .CRUISE_BUTTONS   = 0x23B,  // Cruise control buttons
  .LKAS_HEARTBIT    = 0x2D9,  // LKAS HEARTBIT from DASM
  .CENTER_STACK_1   = 0x330,  // LKAS Button
  .CENTER_STACK_2   = 0x28A,  // LKAS Button
};

// CAN messages for the 5th gen RAM DT platform
const ChryslerAddrs CHRYSLER_RAM_DT_ADDRS = {
  .EPS_2            = 0x31,   // EPS driver input torque
  .ESP_1            = 0x83,   // Brake pedal and vehicle speed
  .ESP_8            = 0x79,   // Brake pedal and vehicle speed
  .ECM_5            = 0x9D,   // Throttle position sensor
  .DAS_3            = 0x99,   // ACC engagement states from DASM
  .DAS_6            = 0xFA,   // LKAS HUD and auto headlight control from DASM
  .LKAS_COMMAND     = 0xA6,   // LKAS controls from DASM
  .CRUISE_BUTTONS   = 0xB1,   // Cruise control buttons
  .CENTER_STACK_1   = 0xDD,   // LKAS Button
  .CENTER_STACK_2   = 0x28A,  // LKAS Button
};

// CAN messages for the 5th gen RAM HD platform
const ChryslerAddrs CHRYSLER_RAM_HD_ADDRS = {
  .EPS_2            = 0x220,  // EPS driver input torque
  .ESP_1            = 0x140,  // Brake pedal and vehicle speed
  .ESP_8            = 0x11C,  // Brake pedal and vehicle speed
  .ECM_5            = 0x22F,  // Throttle position sensor
  .DAS_3            = 0x1F4,  // ACC engagement states from DASM
  .DAS_6            = 0x275,  // LKAS HUD and auto headlight control from DASM
  .LKAS_COMMAND     = 0x276,  // LKAS controls from DASM
  .CRUISE_BUTTONS   = 0x23A,  // Cruise control buttons
  .CENTER_STACK_1   = 0x330,  // LKAS Button
  .CENTER_STACK_2   = 0x28A,  // LKAS Button
};

const CanMsg CHRYSLER_TX_MSGS[] = {
  {CHRYSLER_ADDRS.CRUISE_BUTTONS, 0, 3},
  {CHRYSLER_ADDRS.LKAS_COMMAND, 0, 6},
  {CHRYSLER_ADDRS.DAS_6, 0, 8},
  {CHRYSLER_ADDRS.DAS_3, 0, 8},
  {CHRYSLER_ADDRS.LKAS_HEARTBIT, 0, 5},
};

const CanMsg CHRYSLER_LONG_SHADOW_TX_MSGS[] = {
  {CHRYSLER_ADDRS.CRUISE_BUTTONS, 0, 3},
  {CHRYSLER_ADDRS.LKAS_COMMAND, 0, 6},
  {CHRYSLER_ADDRS.DAS_6, 0, 8},
  {CHRYSLER_ADDRS.DAS_3, 0, 8},
  {CHRYSLER_ADDRS.LKAS_HEARTBIT, 0, 5},
  {CHRYSLER_LONG_BRAKE_ADDR, 0, 8},
  {CHRYSLER_LONG_DASH_ADDR, 0, 8},
  {CHRYSLER_LONG_TORQUE_ADDR, 0, 8},
};

const CanMsg CHRYSLER_RAM_DT_TX_MSGS[] = {
  {CHRYSLER_RAM_DT_ADDRS.CRUISE_BUTTONS, 2, 3},
  {CHRYSLER_RAM_DT_ADDRS.LKAS_COMMAND, 0, 8},
  {CHRYSLER_RAM_DT_ADDRS.DAS_6, 0, 8},
};

const CanMsg CHRYSLER_RAM_HD_TX_MSGS[] = {
  {CHRYSLER_RAM_HD_ADDRS.CRUISE_BUTTONS, 2, 3},
  {CHRYSLER_RAM_HD_ADDRS.LKAS_COMMAND, 0, 8},
  {CHRYSLER_RAM_HD_ADDRS.DAS_6, 0, 8},
};

RxCheck chrysler_rx_checks[] = {
  {.msg = {{CHRYSLER_ADDRS.EPS_2, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_ADDRS.ESP_1, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  //{.msg = {{ESP_8, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}}},
  {.msg = {{514, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_ADDRS.ECM_5, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_ADDRS.DAS_3, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
};

// Full longitudinal control uses physical SET/RES releases to re-arm after a
// pedal override. Keep that input behind the same checksum/counter validation
// as the other safety-critical vehicle sources without changing stock modes.
RxCheck chrysler_long_rx_checks[] = {
  {.msg = {{CHRYSLER_ADDRS.EPS_2, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_ADDRS.ESP_1, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{514, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_ADDRS.ECM_5, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_ADDRS.DAS_3, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_ADDRS.CRUISE_BUTTONS, 0, 3, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  // DAS_4 has no checksum or counter in the FCA DBC. Its 50 Hz liveness and
  // dashboard ACC fault bit are nevertheless independent longitudinal gates.
  {.msg = {{CHRYSLER_LONG_DAS_4_ADDR, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 50U}, { 0 }, { 0 }}},
};

RxCheck chrysler_ram_dt_rx_checks[] = {
  {.msg = {{CHRYSLER_RAM_DT_ADDRS.EPS_2, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_RAM_DT_ADDRS.ESP_1, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_RAM_DT_ADDRS.ESP_8, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_RAM_DT_ADDRS.ECM_5, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_RAM_DT_ADDRS.DAS_3, 2, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
};

RxCheck chrysler_ram_hd_rx_checks[] = {
  {.msg = {{CHRYSLER_RAM_HD_ADDRS.EPS_2, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_RAM_HD_ADDRS.ESP_1, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_RAM_HD_ADDRS.ESP_8, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_RAM_HD_ADDRS.ECM_5, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{CHRYSLER_RAM_HD_ADDRS.DAS_3, 2, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
};



const uint32_t CHRYSLER_PARAM_RAM_DT = 1U;  // set for Ram DT platform
const uint32_t CHRYSLER_PARAM_RAM_HD = 2U;  // set for Ram HD platform
const uint32_t CHRYSLER_PARAM_JEEP_LONG_SHADOW = 4U;
const uint32_t CHRYSLER_PARAM_JEEP_RATE4 = 8U;
const uint32_t CHRYSLER_PARAM_JEEP_LONG_DIAGNOSTIC = 16U;
const uint32_t CHRYSLER_PARAM_JEEP_RATE5 = 32U;
const uint32_t CHRYSLER_PARAM_JEEP_LONG_ACTUATION = 64U;

typedef enum {
  CHRYSLER_RAM_DT,
  CHRYSLER_RAM_HD,
  CHRYSLER_PACIFICA,  // plus Jeep
} ChryslerPlatform;
ChryslerPlatform chrysler_platform = CHRYSLER_PACIFICA;
const ChryslerAddrs *chrysler_addrs = &CHRYSLER_ADDRS;
static uint8_t chrysler_das_3_last[8] = {0};
static bool chrysler_das_3_last_valid = false;
static bool chrysler_long_shadow_enabled = false;
static bool chrysler_jeep_rate4_enabled = false;
static bool chrysler_jeep_rate5_enabled = false;
static bool chrysler_long_diagnostic_enabled = false;
static bool chrysler_long_actuation_enabled = false;
static bool chrysler_long_stock_collision = false;
static bool chrysler_long_dashboard_fault = false;
static bool chrysler_long_dashboard_seen = false;
static bool chrysler_long_speed_seen = false;
static bool chrysler_long_gas_seen = false;
static bool chrysler_long_brake_seen = false;
static bool chrysler_long_stock_seen = false;
static uint32_t chrysler_long_speed_ts = 0U;
static uint32_t chrysler_long_gas_ts = 0U;
static uint32_t chrysler_long_brake_ts = 0U;
static uint32_t chrysler_long_stock_ts = 0U;
static uint32_t chrysler_long_dashboard_ts = 0U;
static uint8_t chrysler_long_stage = 0U;
static uint8_t chrysler_long_cycle_counter = 0U;
static uint8_t chrysler_long_last_counter = 0U;
static bool chrysler_long_counter_seen = false;
static bool chrysler_long_brake_active = false;
static bool chrysler_long_cycle_stop = false;
static bool chrysler_long_cycle_go = false;
static int chrysler_long_vehicle_speed_raw = 0;
static uint8_t chrysler_long_low_speed_state = 0U;
static uint8_t chrysler_long_low_speed_go_cycles = 0U;
static uint32_t chrysler_long_last_cycle_ts = 0U;
static bool chrysler_b6y_hold_active = false;
static uint32_t chrysler_b6y_hold_last_ts = 0U;
static uint8_t chrysler_long_enable_button_prev = 0U;
static bool chrysler_long_enable_press_qualified = false;

typedef enum {
  CHRYSLER_LONG_REJECT_NONE = 0U,
  CHRYSLER_LONG_REJECT_SHADOW_DISABLED = 1U,
  CHRYSLER_LONG_REJECT_PLATFORM = 2U,
  CHRYSLER_LONG_REJECT_BRAKE_STAGE = 3U,
  CHRYSLER_LONG_REJECT_CONTROLS = 4U,
  CHRYSLER_LONG_REJECT_CONTROLS_LONG = 5U,
  CHRYSLER_LONG_REJECT_ACC_MAIN = 6U,
  CHRYSLER_LONG_REJECT_STOPPED = 7U,
  CHRYSLER_LONG_REJECT_GAS = 8U,
  CHRYSLER_LONG_REJECT_BRAKE = 9U,
  CHRYSLER_LONG_REJECT_COLLISION = 10U,
  CHRYSLER_LONG_REJECT_SPEED_MISSING = 11U,
  CHRYSLER_LONG_REJECT_SPEED_STALE = 12U,
  CHRYSLER_LONG_REJECT_GAS_MISSING = 13U,
  CHRYSLER_LONG_REJECT_GAS_STALE = 14U,
  CHRYSLER_LONG_REJECT_BRAKE_MISSING = 15U,
  CHRYSLER_LONG_REJECT_BRAKE_STALE = 16U,
  CHRYSLER_LONG_REJECT_STOCK_MISSING = 17U,
  CHRYSLER_LONG_REJECT_STOCK_STALE = 18U,
  CHRYSLER_LONG_REJECT_BRAKE_CHECKSUM = 19U,
  CHRYSLER_LONG_REJECT_BRAKE_PAYLOAD = 20U,
  CHRYSLER_LONG_REJECT_BRAKE_COMMAND = 21U,
  CHRYSLER_LONG_REJECT_INTERVAL = 22U,
  CHRYSLER_LONG_REJECT_COUNTER = 23U,
  CHRYSLER_LONG_REJECT_DASH_STAGE = 24U,
  CHRYSLER_LONG_REJECT_DASH_COUNTER = 25U,
  CHRYSLER_LONG_REJECT_DASH_CHECKSUM = 26U,
  CHRYSLER_LONG_REJECT_DASH_PAYLOAD = 27U,
  CHRYSLER_LONG_REJECT_TORQUE_STAGE = 28U,
  CHRYSLER_LONG_REJECT_TORQUE_COUNTER = 29U,
  CHRYSLER_LONG_REJECT_TORQUE_CHECKSUM = 30U,
  CHRYSLER_LONG_REJECT_TORQUE_PAYLOAD = 31U,
  CHRYSLER_LONG_REJECT_TORQUE_CONFLICT = 32U,
  CHRYSLER_LONG_REJECT_LOW_SPEED_SEQUENCE = 33U,
  CHRYSLER_LONG_REJECT_DASHBOARD_FAULT = 34U,
  CHRYSLER_LONG_REJECT_DASHBOARD_MISSING = 35U,
  CHRYSLER_LONG_REJECT_DASHBOARD_STALE = 36U,
} ChryslerLongRejectReason;

typedef enum {
  CHRYSLER_LONG_LOW_DRIVE = 0U,
  CHRYSLER_LONG_LOW_HOLD = 1U,
  CHRYSLER_LONG_LOW_RELEASE = 2U,
  CHRYSLER_LONG_LOW_GO = 3U,
  CHRYSLER_LONG_LOW_CREEP = 4U,
} ChryslerLongLowSpeedState;

static bool chrysler_long_low_speed_transition(
    const uint8_t current_state,
    const uint8_t current_go_cycles,
    const bool stop_request,
    const bool go_request,
    const bool brake_request,
    const bool engine_request,
    uint8_t *next_state,
    uint8_t *next_go_cycles) {
  uint8_t state = current_state;
  uint8_t go_cycles = current_go_cycles;

  if (chrysler_long_vehicle_speed_raw >
      CHRYSLER_LONG_STOP_GO_SPEED_MAX_RAW) {
    state = CHRYSLER_LONG_LOW_DRIVE;
    go_cycles = 0U;
  } else if (engine_request) {
    // The complete factory capture applies bounded engine torque immediately
    // after GO, before wheel speed reaches 0.78 m/s. Admit that one case only
    // from CREEP; ordinary low-speed DRIVE cannot create launch torque.
    if (stop_request || go_request || brake_request) {
      return false;
    }
    if (chrysler_long_vehicle_speed_raw >=
        CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW) {
      state = CHRYSLER_LONG_LOW_DRIVE;
      go_cycles = 0U;
    } else if (current_state != CHRYSLER_LONG_LOW_CREEP) {
      return false;
    }
  } else if (stop_request && brake_request && !go_request) {
    state = CHRYSLER_LONG_LOW_HOLD;
    go_cycles = 0U;
  } else if (current_state == CHRYSLER_LONG_LOW_DRIVE) {
    if (go_request) {
      return false;
    }
  } else if (current_state == CHRYSLER_LONG_LOW_HOLD) {
    if (go_request) {
      return false;
    }
    // A decaying brake command is still HOLD. Require one complete neutral
    // private cycle before the subsequent GO cycle can be accepted.
    if (!brake_request && !engine_request) {
      state = CHRYSLER_LONG_LOW_RELEASE;
      go_cycles = 0U;
    }
  } else if (current_state == CHRYSLER_LONG_LOW_RELEASE) {
    if (brake_request) {
      state = CHRYSLER_LONG_LOW_HOLD;
      go_cycles = 0U;
    }
    if (go_request) {
      if (brake_request || engine_request) {
        return false;
      }
      state = CHRYSLER_LONG_LOW_GO;
      go_cycles = 1U;
    }
  } else if (current_state == CHRYSLER_LONG_LOW_GO) {
    if (go_request) {
      if (brake_request || engine_request ||
          (current_go_cycles >= CHRYSLER_LONG_LOW_GO_MAX_CYCLES)) {
        return false;
      }
      go_cycles = current_go_cycles + 1U;
    } else {
      state = CHRYSLER_LONG_LOW_CREEP;
      go_cycles = 0U;
    }
  } else if (current_state == CHRYSLER_LONG_LOW_CREEP) {
    if (go_request) {
      return false;
    }
  } else {
    return false;
  }

  *next_state = state;
  *next_go_cycles = go_cycles;
  return true;
}

static void chrysler_long_set_reject(const uint8_t reason,
                                     const uint32_t detail) {
  if (chrysler_long_diagnostic_enabled) {
    safety_tx_reject_reason = reason;
    safety_tx_reject_detail = detail;
  }
}

static uint32_t chrysler_get_checksum(const CANPacket_t *to_push) {
  int checksum_byte = GET_LEN(to_push) - 1U;
  return (uint8_t)(GET_BYTE(to_push, checksum_byte));
}

static uint32_t chrysler_compute_checksum(const CANPacket_t *to_push) {
  // TODO: clean this up
  // http://illmatics.com/Remote%20Car%20Hacking.pdf
  uint8_t checksum = 0xFFU;
  int len = GET_LEN(to_push);
  for (int j = 0; j < (len - 1); j++) {
    uint8_t shift = 0x80U;
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
  return (uint8_t)(~checksum);
}

static uint8_t chrysler_get_counter(const CANPacket_t *to_push) {
  return (GET_ADDR(to_push) == chrysler_addrs->CRUISE_BUTTONS) ?
    (uint8_t)(GET_BYTE(to_push, 1) >> 4) :
    (uint8_t)(GET_BYTE(to_push, 6) >> 4);
}

static bool chrysler_long_fresh(const uint32_t now, const uint32_t last,
                                const bool seen) {
  return seen &&
         (get_ts_elapsed(now, last) <= CHRYSLER_LONG_SOURCE_TIMEOUT_US);
}

static void chrysler_b6y_hold_clear(void) {
  chrysler_b6y_hold_active = false;
}

static bool chrysler_b6y_hold_sources_valid(void) {
  const uint32_t now = microsecond_timer_get();
  return chrysler_long_actuation_enabled &&
         (chrysler_platform == CHRYSLER_PACIFICA) &&
         chrysler_das_3_last_valid &&
         acc_main_on && !vehicle_moving &&
         !gas_pressed && !brake_pressed &&
         !chrysler_long_stock_collision &&
         !chrysler_long_dashboard_fault &&
         chrysler_long_fresh(now, chrysler_long_speed_ts,
                             chrysler_long_speed_seen) &&
         chrysler_long_fresh(now, chrysler_long_gas_ts,
                             chrysler_long_gas_seen) &&
         chrysler_long_fresh(now, chrysler_long_brake_ts,
                             chrysler_long_brake_seen) &&
         chrysler_long_fresh(now, chrysler_long_stock_ts,
                             chrysler_long_stock_seen) &&
         chrysler_long_fresh(now, chrysler_long_dashboard_ts,
                             chrysler_long_dashboard_seen);
}

static bool chrysler_long_rearm_sources_valid(void) {
  const uint32_t now = microsecond_timer_get();
  const int required_rx_checks[] = {1, 2, 3, 4, 5, 6};
  bool rx_integrity_valid = true;
  for (uint8_t i = 0U;
       i < (sizeof(required_rx_checks) / sizeof(required_rx_checks[0]));
       i++) {
    const RxStatus status =
      chrysler_long_rx_checks[required_rx_checks[i]].status;
    rx_integrity_valid = rx_integrity_valid && status.msg_seen &&
                         status.valid_checksum &&
                         status.valid_quality_flag &&
                         (status.wrong_counters < MAX_WRONG_COUNTERS) &&
                         !status.lagging;
  }
  return chrysler_long_actuation_enabled &&
         (chrysler_platform == CHRYSLER_PACIFICA) &&
         rx_integrity_valid &&
         chrysler_das_3_last_valid && acc_main_on &&
         !gas_pressed && !brake_pressed &&
         !chrysler_long_stock_collision &&
         !chrysler_long_dashboard_fault &&
         chrysler_long_fresh(now, chrysler_long_speed_ts,
                             chrysler_long_speed_seen) &&
         chrysler_long_fresh(now, chrysler_long_gas_ts,
                             chrysler_long_gas_seen) &&
         chrysler_long_fresh(now, chrysler_long_brake_ts,
                             chrysler_long_brake_seen) &&
         chrysler_long_fresh(now, chrysler_long_stock_ts,
                             chrysler_long_stock_seen) &&
         chrysler_long_fresh(now, chrysler_long_dashboard_ts,
                             chrysler_long_dashboard_seen);
}

static bool chrysler_b6y_hold_recent(void) {
  const uint32_t now = microsecond_timer_get();
  return chrysler_b6y_hold_active &&
         chrysler_b6y_hold_sources_valid() &&
         (get_ts_elapsed(now, chrysler_b6y_hold_last_ts) <=
          CHRYSLER_B6Y_HOLD_TIMEOUT_US);
}

static uint8_t chrysler_long_source_reject_reason(uint32_t *detail) {
  const uint32_t now = microsecond_timer_get();
  uint8_t reason = CHRYSLER_LONG_REJECT_NONE;
  *detail = 0U;

  if (chrysler_long_dashboard_fault) {
    reason = CHRYSLER_LONG_REJECT_DASHBOARD_FAULT;
  } else if (!chrysler_long_dashboard_seen) {
    reason = CHRYSLER_LONG_REJECT_DASHBOARD_MISSING;
  } else if (!chrysler_long_fresh(now, chrysler_long_dashboard_ts, true)) {
    reason = CHRYSLER_LONG_REJECT_DASHBOARD_STALE;
    *detail = get_ts_elapsed(now, chrysler_long_dashboard_ts);
  } else if (!controls_allowed) {
    reason = CHRYSLER_LONG_REJECT_CONTROLS;
  } else if (!controls_allowed_long) {
    reason = CHRYSLER_LONG_REJECT_CONTROLS_LONG;
  } else if (!acc_main_on) {
    reason = CHRYSLER_LONG_REJECT_ACC_MAIN;
  } else if (gas_pressed) {
    reason = CHRYSLER_LONG_REJECT_GAS;
  } else if (brake_pressed) {
    reason = CHRYSLER_LONG_REJECT_BRAKE;
  } else if (chrysler_long_stock_collision) {
    reason = CHRYSLER_LONG_REJECT_COLLISION;
  } else if (!chrysler_long_speed_seen) {
    reason = CHRYSLER_LONG_REJECT_SPEED_MISSING;
  } else if (!chrysler_long_fresh(now, chrysler_long_speed_ts, true)) {
    reason = CHRYSLER_LONG_REJECT_SPEED_STALE;
    *detail = get_ts_elapsed(now, chrysler_long_speed_ts);
  } else if (!chrysler_long_gas_seen) {
    reason = CHRYSLER_LONG_REJECT_GAS_MISSING;
  } else if (!chrysler_long_fresh(now, chrysler_long_gas_ts, true)) {
    reason = CHRYSLER_LONG_REJECT_GAS_STALE;
    *detail = get_ts_elapsed(now, chrysler_long_gas_ts);
  } else if (!chrysler_long_brake_seen) {
    reason = CHRYSLER_LONG_REJECT_BRAKE_MISSING;
  } else if (!chrysler_long_fresh(now, chrysler_long_brake_ts, true)) {
    reason = CHRYSLER_LONG_REJECT_BRAKE_STALE;
    *detail = get_ts_elapsed(now, chrysler_long_brake_ts);
  } else if (!chrysler_long_stock_seen) {
    reason = CHRYSLER_LONG_REJECT_STOCK_MISSING;
  } else if (!chrysler_long_fresh(now, chrysler_long_stock_ts, true)) {
    reason = CHRYSLER_LONG_REJECT_STOCK_STALE;
    *detail = get_ts_elapsed(now, chrysler_long_stock_ts);
  } else {
  }

  return reason;
}

static void chrysler_long_reset_pending(void) {
  chrysler_long_stage = 0U;
  chrysler_long_brake_active = false;
  chrysler_long_cycle_stop = false;
  chrysler_long_cycle_go = false;
}

static bool chrysler_long_checksum_valid(const CANPacket_t *to_send) {
  return (GET_LEN(to_send) == 8) &&
         (GET_BYTE(to_send, 7) == chrysler_compute_checksum(to_send));
}

static bool chrysler_long_brake_tx_allowed(const CANPacket_t *to_send) {
  const bool shadow_valid = chrysler_long_shadow_enabled;
  const bool platform_valid = chrysler_platform == CHRYSLER_PACIFICA;
  const bool stage_valid = chrysler_long_stage == 0U;
  uint32_t source_detail = 0U;
  uint8_t source_reason = CHRYSLER_LONG_REJECT_NONE;
  if (shadow_valid && platform_valid && stage_valid) {
    source_reason = chrysler_long_source_reject_reason(&source_detail);
  }
  const bool source_valid = source_reason == CHRYSLER_LONG_REJECT_NONE;
  const bool checksum_valid =
    shadow_valid && platform_valid && stage_valid && source_valid &&
    chrysler_long_checksum_valid(to_send);

  const uint8_t counter = (GET_BYTE(to_send, 6) >> 4) & 0xFU;
  const int decel_raw = ((GET_BYTE(to_send, 2) & 0xFU) << 8) |
                        GET_BYTE(to_send, 3);
  const int command_type = (GET_BYTE(to_send, 4) >> 4) & 0x7U;
  const bool stop_request = GET_BIT(to_send, 5U);
  const bool go_request = GET_BIT(to_send, 6U);
  const bool acc_available_cmd = GET_BIT(to_send, 20U);
  const bool acc_enabled_cmd = GET_BIT(to_send, 21U);
  const bool brake_fields_valid =
    ((GET_BYTE(to_send, 0) & 0x9FU) == 0U) &&
    (GET_BYTE(to_send, 1) == 0U) &&
    ((GET_BYTE(to_send, 2) & 0xC0U) == 0U) &&
    ((GET_BYTE(to_send, 4) & 0x8FU) == 0U) &&
    (GET_BYTE(to_send, 5) == 0U) &&
    ((GET_BYTE(to_send, 6) & 0xFU) == 0U) &&
    acc_available_cmd && acc_enabled_cmd &&
    !(stop_request && go_request);
  bool command_valid = false;
  if (command_type == 1) {
    command_valid =
      !go_request &&
      (!stop_request ||
       (chrysler_long_vehicle_speed_raw <=
        CHRYSLER_LONG_STOP_GO_SPEED_MAX_RAW)) &&
      (decel_raw >= CHRYSLER_LONG_DECEL_MIN_RAW) &&
      (decel_raw <= CHRYSLER_LONG_DECEL_MAX_RAW);
  } else {
    command_valid =
      (command_type == 0) &&
      !stop_request &&
      (!go_request ||
       (chrysler_long_vehicle_speed_raw <=
        CHRYSLER_LONG_STOP_GO_SPEED_MAX_RAW)) &&
      (decel_raw == CHRYSLER_LONG_DECEL_INACTIVE_RAW);
  }
  uint8_t precheck_low_speed_state = chrysler_long_low_speed_state;
  uint8_t precheck_low_speed_go_cycles = chrysler_long_low_speed_go_cycles;
  const bool low_speed_sequence_valid =
    chrysler_long_low_speed_transition(
      chrysler_long_low_speed_state,
      chrysler_long_low_speed_go_cycles,
      stop_request,
      go_request,
      command_type == 1,
      false,
      &precheck_low_speed_state,
      &precheck_low_speed_go_cycles);

  bool allowed = shadow_valid && platform_valid && stage_valid &&
                 source_valid && checksum_valid && brake_fields_valid &&
                 command_valid && low_speed_sequence_valid;
  const uint32_t now = microsecond_timer_get();
  uint32_t elapsed = 0U;
  bool interval_valid = true;
  bool counter_valid = true;
  if (allowed && chrysler_long_counter_seen) {
    elapsed = get_ts_elapsed(now, chrysler_long_last_cycle_ts);
    interval_valid = elapsed >= CHRYSLER_LONG_MIN_CYCLE_INTERVAL_US;
    if (elapsed <= CHRYSLER_LONG_COUNTER_RESET_US) {
      counter_valid =
        counter == ((chrysler_long_last_counter + 1U) & 0xFU);
    }
    allowed = interval_valid && counter_valid;
  }

  if (allowed) {
    chrysler_long_cycle_counter = (uint8_t)counter;
    chrysler_long_last_counter = (uint8_t)counter;
    chrysler_long_counter_seen = true;
    chrysler_long_brake_active = command_type == 1;
    chrysler_long_cycle_stop = stop_request;
    chrysler_long_cycle_go = go_request;
    chrysler_long_last_cycle_ts = now;
    chrysler_long_stage = 1U;
  } else {
    if (!shadow_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_SHADOW_DISABLED, 0U);
    } else if (!platform_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_PLATFORM,
                               (uint32_t)chrysler_platform);
    } else if (!stage_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_BRAKE_STAGE,
                               (uint32_t)chrysler_long_stage);
    } else if (!source_valid) {
      chrysler_long_set_reject(source_reason, source_detail);
    } else if (!checksum_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_BRAKE_CHECKSUM, 0U);
    } else if (!brake_fields_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_BRAKE_PAYLOAD, 0U);
    } else if (!command_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_BRAKE_COMMAND,
                               (uint32_t)decel_raw);
    } else if (!low_speed_sequence_valid) {
      chrysler_long_set_reject(
        CHRYSLER_LONG_REJECT_LOW_SPEED_SEQUENCE,
        ((uint32_t)chrysler_long_low_speed_state << 8U) |
        (uint32_t)chrysler_long_low_speed_go_cycles);
    } else if (!interval_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_INTERVAL, elapsed);
      // The frame remains blocked, but retain its valid input counter so an
      // intentional rate-floor rejection does not poison the following
      // counter sequence. Keep last_cycle_ts unchanged: accepted private
      // frames must still remain at least 15 ms apart.
      if (counter_valid) {
        chrysler_long_last_counter = (uint8_t)counter;
      }
    } else if (!counter_valid) {
      const uint32_t expected =
        (uint32_t)((chrysler_long_last_counter + 1U) & 0xFU);
      chrysler_long_set_reject(
        CHRYSLER_LONG_REJECT_COUNTER,
        (expected << 8U) | (uint32_t)counter);
    } else {
    }
    chrysler_long_reset_pending();
  }
  return allowed;
}

static bool chrysler_long_dash_tx_allowed(const CANPacket_t *to_send) {
  const uint8_t counter = (GET_BYTE(to_send, 6) >> 4) & 0xFU;
  const bool shadow_valid = chrysler_long_shadow_enabled;
  const bool stage_valid = chrysler_long_stage == 1U;
  const bool counter_valid = counter == chrysler_long_cycle_counter;
  const bool checksum_valid = chrysler_long_checksum_valid(to_send);
  uint32_t source_detail = 0U;
  const uint8_t source_reason =
    chrysler_long_source_reject_reason(&source_detail);
  const bool source_valid = source_reason == CHRYSLER_LONG_REJECT_NONE;
  const bool payload_valid =
    (GET_BYTE(to_send, 0) == 0U) &&
    (GET_BYTE(to_send, 1) == 0U) &&
    (GET_BYTE(to_send, 2) == 0U) &&
    (GET_BYTE(to_send, 3) ==
      (chrysler_long_actuation_enabled ? 1U : 0U)) &&
    (GET_BYTE(to_send, 4) == 0U) &&
    (GET_BYTE(to_send, 5) == 0U) &&
    ((GET_BYTE(to_send, 6) & 0xFU) == 0U);
  const bool allowed = shadow_valid && stage_valid && counter_valid &&
                       checksum_valid && source_valid && payload_valid;

  if (allowed) {
    chrysler_long_stage = 2U;
  } else {
    if (!shadow_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_SHADOW_DISABLED, 0U);
    } else if (!stage_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_DASH_STAGE,
                               (uint32_t)chrysler_long_stage);
    } else if (!counter_valid) {
      chrysler_long_set_reject(
        CHRYSLER_LONG_REJECT_DASH_COUNTER,
        ((uint32_t)chrysler_long_cycle_counter << 8U) | (uint32_t)counter);
    } else if (!checksum_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_DASH_CHECKSUM, 0U);
    } else if (!source_valid) {
      chrysler_long_set_reject(source_reason, source_detail);
    } else if (!payload_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_DASH_PAYLOAD, 0U);
    } else {
    }
    chrysler_long_reset_pending();
  }
  return allowed;
}

static bool chrysler_long_torque_tx_allowed(const CANPacket_t *to_send) {
  const uint8_t counter = (GET_BYTE(to_send, 6) >> 4) & 0xFU;
  const bool engine_request = GET_BIT(to_send, 39U);
  const int torque_raw = ((GET_BYTE(to_send, 4) & 0x7FU) << 8) |
                         GET_BYTE(to_send, 5);
  const bool shadow_valid = chrysler_long_shadow_enabled;
  const bool stage_valid = chrysler_long_stage == 2U;
  const bool counter_valid = counter == chrysler_long_cycle_counter;
  const bool checksum_valid = chrysler_long_checksum_valid(to_send);
  uint32_t source_detail = 0U;
  const uint8_t source_reason =
    chrysler_long_source_reject_reason(&source_detail);
  const bool source_valid = source_reason == CHRYSLER_LONG_REJECT_NONE;
  const bool payload_valid =
    (GET_BYTE(to_send, 0) == 0U) &&
    (GET_BYTE(to_send, 1) == 0U) &&
    (GET_BYTE(to_send, 2) == 0U) &&
    (GET_BYTE(to_send, 3) == 0U) &&
    ((GET_BYTE(to_send, 6) & 0xFU) == 0U);
  const bool conflict_valid =
    !(chrysler_long_brake_active && engine_request) &&
    !(engine_request &&
      (chrysler_long_cycle_stop || chrysler_long_cycle_go));
  uint8_t next_low_speed_state = chrysler_long_low_speed_state;
  uint8_t next_low_speed_go_cycles = chrysler_long_low_speed_go_cycles;
  const bool low_speed_transition_valid =
    chrysler_long_low_speed_transition(
      chrysler_long_low_speed_state,
      chrysler_long_low_speed_go_cycles,
      chrysler_long_cycle_stop,
      chrysler_long_cycle_go,
      chrysler_long_brake_active,
      engine_request,
      &next_low_speed_state,
      &next_low_speed_go_cycles);

  bool command_valid = false;
  if (engine_request) {
    const bool running_torque_valid =
      (chrysler_long_vehicle_speed_raw >=
       CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW) &&
      (torque_raw <= CHRYSLER_LONG_TORQUE_MAX_RAW);
    const bool launch_torque_valid =
      (chrysler_long_vehicle_speed_raw <
       CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW) &&
      (chrysler_long_low_speed_state == CHRYSLER_LONG_LOW_CREEP) &&
      (torque_raw <= CHRYSLER_LONG_LAUNCH_TORQUE_MAX_RAW);
    command_valid =
      (torque_raw >= CHRYSLER_LONG_TORQUE_ZERO_RAW) &&
      (running_torque_valid || launch_torque_valid);
  } else {
    command_valid = torque_raw == CHRYSLER_LONG_TORQUE_ZERO_RAW;
  }
  const bool allowed = shadow_valid && stage_valid && counter_valid &&
                       checksum_valid && source_valid && payload_valid &&
                       conflict_valid && low_speed_transition_valid &&
                       command_valid;

  if (allowed) {
    chrysler_long_low_speed_state = next_low_speed_state;
    chrysler_long_low_speed_go_cycles = next_low_speed_go_cycles;
  } else {
    if (!shadow_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_SHADOW_DISABLED, 0U);
    } else if (!stage_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_TORQUE_STAGE,
                               (uint32_t)chrysler_long_stage);
    } else if (!counter_valid) {
      chrysler_long_set_reject(
        CHRYSLER_LONG_REJECT_TORQUE_COUNTER,
        ((uint32_t)chrysler_long_cycle_counter << 8U) | (uint32_t)counter);
    } else if (!checksum_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_TORQUE_CHECKSUM, 0U);
    } else if (!source_valid) {
      chrysler_long_set_reject(source_reason, source_detail);
    } else if (!payload_valid || !command_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_TORQUE_PAYLOAD,
                               (uint32_t)torque_raw);
    } else if (!conflict_valid) {
      chrysler_long_set_reject(CHRYSLER_LONG_REJECT_TORQUE_CONFLICT, 0U);
    } else if (!low_speed_transition_valid) {
      chrysler_long_set_reject(
        CHRYSLER_LONG_REJECT_LOW_SPEED_SEQUENCE,
        ((uint32_t)chrysler_long_low_speed_state << 8U) |
        (uint32_t)chrysler_long_low_speed_go_cycles);
    } else {
    }
  }

  chrysler_long_reset_pending();
  return allowed;
}

static void chrysler_rx_hook(const CANPacket_t *to_push) {
  const int bus = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);

  // DAS_4 carries the dashboard ACC/FCW fault actually asserted by this Jeep.
  // Revoke only longitudinal permission immediately; a later clear frame does
  // not re-arm actuation without a new, clean physical SET/RES action.
  if ((bus == 0) && (addr == CHRYSLER_LONG_DAS_4_ADDR) &&
      (GET_LEN(to_push) == 8)) {
    chrysler_long_dashboard_fault = GET_BIT(to_push, 50U) != 0U;
    chrysler_long_dashboard_seen = true;
    chrysler_long_dashboard_ts = microsecond_timer_get();
    if (chrysler_long_dashboard_fault &&
        chrysler_long_actuation_enabled) {
      controls_allowed_long = false;
      chrysler_long_enable_press_qualified = false;
      chrysler_b6y_hold_clear();
      chrysler_long_reset_pending();
    }
  }

  // Measured EPS torque
  if ((bus == 0) && (addr == chrysler_addrs->EPS_2)) {
    int torque_meas_new = ((GET_BYTE(to_push, 4) & 0x7U) << 8) + GET_BYTE(to_push, 5) - 1024U;
    update_sample(&torque_meas, torque_meas_new);
  }

  // enter controls on rising edge of ACC, exit controls on ACC off
  const int das_3_bus = (chrysler_platform == CHRYSLER_PACIFICA) ? 0 : 2;
  if ((bus == das_3_bus) && (addr == chrysler_addrs->DAS_3)) {
    for (int i = 0; i < 8; i++) {
      chrysler_das_3_last[i] = GET_BYTE(to_push, i);
    }
    chrysler_das_3_last_valid = true;

    bool cruise_engaged = GET_BIT(to_push, 21U);
    if (chrysler_long_actuation_enabled) {
      // Factory ACC is intentionally inactive while the White Panda owns
      // DAS_3. Preserve the normal rising-edge enable, but do not revoke a
      // prior physical SET/RES enable merely because stock ACC stays inactive.
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = true;
        controls_allowed_long = true;
      }
      cruise_engaged_prev = cruise_engaged;
    } else {
      pcm_cruise_check(cruise_engaged);
    }

    acc_main_on = GET_BIT(to_push, 20U) != 0U;
    mads_acc_main_check(acc_main_on);
    chrysler_long_stock_collision =
      ((GET_BYTE(to_push, 6) & 0x1U) != 0U) ||
      (((GET_BYTE(to_push, 4) >> 4) & 0x7U) > 1U);
    chrysler_long_stock_ts = microsecond_timer_get();
    chrysler_long_stock_seen = true;
    if (cruise_engaged || !acc_main_on ||
        chrysler_long_stock_collision) {
      chrysler_b6y_hold_clear();
    }
    if (!acc_main_on || chrysler_long_stock_collision) {
      chrysler_long_enable_press_qualified = false;
      controls_allowed_long = false;
    }
  }

  if ((bus == 0) && (addr == chrysler_addrs->CRUISE_BUTTONS) &&
      (GET_LEN(to_push) == 3) &&
      chrysler_long_actuation_enabled) {
    const uint8_t enable_buttons = GET_BYTE(to_push, 0) & 0x1CU;
    const bool single_enable_button =
      (enable_buttons == 0x04U) || (enable_buttons == 0x08U) ||
      (enable_buttons == 0x10U);
    const bool cancel_pressed = (GET_BYTE(to_push, 0) & 0x01U) != 0U;

    if (cancel_pressed) {
      controls_allowed_long = false;
      chrysler_long_enable_press_qualified = false;
    } else if ((chrysler_long_enable_button_prev == 0U) &&
               single_enable_button) {
      // Qualification occurs on the press, not only on release, so a button
      // held through a pedal transition cannot silently re-arm actuation.
      chrysler_long_enable_press_qualified =
        chrysler_long_rearm_sources_valid();
    } else if ((chrysler_long_enable_button_prev != 0U) &&
               (enable_buttons == 0U)) {
      if (chrysler_long_enable_press_qualified &&
          chrysler_long_rearm_sources_valid()) {
        controls_allowed = true;
        controls_allowed_long = true;
      }
      chrysler_long_enable_press_qualified = false;
    } else if (!single_enable_button && (enable_buttons != 0U)) {
      chrysler_long_enable_press_qualified = false;
    } else {
    }
    chrysler_long_enable_button_prev = enable_buttons;
  }

  // TODO: use the same message for both
  // update vehicle moving
  if ((chrysler_platform != CHRYSLER_PACIFICA) && (bus == 0) && (addr == chrysler_addrs->ESP_8)) {
    vehicle_moving = ((GET_BYTE(to_push, 4) << 8) + GET_BYTE(to_push, 5)) != 0U;
  }
  if ((chrysler_platform == CHRYSLER_PACIFICA) && (bus == 0) && (addr == 514)) {
    int speed_l = (GET_BYTE(to_push, 0) << 4) + (GET_BYTE(to_push, 1) >> 4);
    int speed_r = (GET_BYTE(to_push, 2) << 4) + (GET_BYTE(to_push, 3) >> 4);
    chrysler_long_vehicle_speed_raw = (speed_l + speed_r) / 2;
    vehicle_moving = (speed_l != 0) || (speed_r != 0);
    chrysler_long_speed_ts = microsecond_timer_get();
    chrysler_long_speed_seen = true;
    if (vehicle_moving) {
      chrysler_b6y_hold_clear();
    }
  }

  // exit controls on rising edge of gas press
  if ((bus == 0) && (addr == chrysler_addrs->ECM_5)) {
    gas_pressed = GET_BYTE(to_push, 0U) != 0U;
    chrysler_long_gas_ts = microsecond_timer_get();
    chrysler_long_gas_seen = true;
    if (gas_pressed) {
      chrysler_b6y_hold_clear();
      chrysler_long_enable_press_qualified = false;
    }
  }

  // exit controls on rising edge of brake press
  if ((bus == 0) && (addr == chrysler_addrs->ESP_1)) {
    brake_pressed = ((GET_BYTE(to_push, 0U) & 0xFU) >> 2U) == 1U;
    chrysler_long_brake_ts = microsecond_timer_get();
    chrysler_long_brake_seen = true;
    if (brake_pressed) {
      chrysler_b6y_hold_clear();
      chrysler_long_enable_press_qualified = false;
    }
  }

  generic_rx_checks((bus == 0) && (addr == chrysler_addrs->LKAS_COMMAND));
}

static bool chrysler_das_3_tx_allowed(const CANPacket_t *to_send) {
  if (!chrysler_b6y_hold_sources_valid()) {
    chrysler_b6y_hold_clear();
    return false;
  }

  const uint8_t allowed_masks[8] = {0x60U, 0x00U, 0x3FU, 0xFFU, 0x7FU, 0x00U, 0xF2U, 0xFFU};
  for (int i = 0; i < 8; i++) {
    if (((GET_BYTE(to_send, i) ^ chrysler_das_3_last[i]) & (uint8_t)(~allowed_masks[i])) != 0U) {
      return false;
    }
  }

  const int decel_raw = ((GET_BYTE(to_send, 2) & 0xFU) << 8) | GET_BYTE(to_send, 3);
  const int counter = GET_BYTE(to_send, 6) >> 4;
  const int last_counter = chrysler_das_3_last[6] >> 4;
  const int counter_delta = (counter - last_counter) & 0xFU;

  const bool allowed =
    ((GET_BYTE(to_send, 0) & 0x60U) == 0U) &&
    ((GET_BYTE(to_send, 2) & 0x30U) == 0x30U) &&
    ((GET_BYTE(to_send, 4) & 0xFU) == 2U) &&
    (((GET_BYTE(to_send, 4) >> 4) & 0x7U) == 1U) &&
    ((GET_BYTE(to_send, 6) & 0x2U) == 0U) &&
    (decel_raw == CHRYSLER_B6Y_HOLD_DECEL_RAW) &&
    ((counter_delta == 2) || (counter_delta == 3)) &&
    (GET_BYTE(to_send, 7) == chrysler_compute_checksum(to_send));

  if (allowed) {
    chrysler_b6y_hold_active = true;
    chrysler_b6y_hold_last_ts = microsecond_timer_get();
  } else {
    chrysler_b6y_hold_clear();
  }
  return allowed;
}

static bool chrysler_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);

  if (addr == chrysler_addrs->DAS_3) {
    tx = chrysler_das_3_tx_allowed(to_send);
  }
  if (addr == CHRYSLER_LONG_BRAKE_ADDR) {
    tx = chrysler_long_brake_tx_allowed(to_send);
  }
  if (addr == CHRYSLER_LONG_DASH_ADDR) {
    tx = chrysler_long_dash_tx_allowed(to_send);
  }
  if (addr == CHRYSLER_LONG_TORQUE_ADDR) {
    tx = chrysler_long_torque_tx_allowed(to_send);
  }

  // STEERING
  if (addr == chrysler_addrs->LKAS_COMMAND) {
    int start_byte = (chrysler_platform == CHRYSLER_PACIFICA) ? 0 : 1;
    int desired_torque = ((GET_BYTE(to_send, start_byte) & 0x7U) << 8) | GET_BYTE(to_send, start_byte + 1);
    desired_torque -= 1024;

    const SteeringLimits limits = (chrysler_platform == CHRYSLER_PACIFICA) ?
                                  (chrysler_jeep_rate5_enabled ?
                                   CHRYSLER_JEEP_RATE5_STEERING_LIMITS :
                                   (chrysler_jeep_rate4_enabled ?
                                    CHRYSLER_JEEP_RATE4_STEERING_LIMITS :
                                    CHRYSLER_STEERING_LIMITS)) :
                                  (chrysler_platform == CHRYSLER_RAM_DT) ? CHRYSLER_RAM_DT_STEERING_LIMITS : CHRYSLER_RAM_HD_STEERING_LIMITS;

    bool steer_req = (chrysler_platform == CHRYSLER_PACIFICA) ? GET_BIT(to_send, 4U) : (GET_BYTE(to_send, 3) & 0x7U) == 2U;
    if (steer_torque_cmd_checks(desired_torque, steer_req, limits)) {
      tx = false;
    }
  }

  // FORCE CANCEL: only the cancel button press is allowed
  if ((addr == chrysler_addrs->CRUISE_BUTTONS) && (chrysler_platform == CHRYSLER_PACIFICA)) {
    const bool is_cancel = GET_BYTE(to_send, 0) == 1U;
    const bool is_resume = GET_BYTE(to_send, 0) == 0x10U;
    const bool is_accel = GET_BIT(to_send, 2);
    const bool is_decel = GET_BIT(to_send, 3);
    // The controls_allowed bypass is valid only immediately after an accepted
    // exact b6 hold frame. It cannot authorize standstill propulsion.
    const bool allow_resume_standstill =
      is_resume && chrysler_b6y_hold_recent();
    const bool allowed = is_cancel || allow_resume_standstill ||
                         ((is_resume || is_accel || is_decel) &&
                          controls_allowed && controls_allowed_long);
    if (is_cancel) {
      chrysler_b6y_hold_clear();
    }
    if (!allowed) {
      tx = false;
    }
  }

  return tx;
}

static int chrysler_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  // forward all messages from car to camera except CRUISE_BUTTONS messages for Ram platforms
  const bool is_cruise_buttons = ((addr == chrysler_addrs->CRUISE_BUTTONS) && (chrysler_platform != CHRYSLER_PACIFICA));
  // forward to camera
  if ((bus_num == 0) && !is_cruise_buttons) {
    bus_fwd = 2;
  }

  // forward all messages from camera except LKAS messages and HeartBit
  const bool is_lkas = ((addr == chrysler_addrs->LKAS_COMMAND) || (addr == chrysler_addrs->DAS_6) || (addr == chrysler_addrs->LKAS_HEARTBIT));
  if ((bus_num == 2) && !is_lkas){
    bus_fwd = 0;
  }

  return bus_fwd;
}

static safety_config chrysler_init(uint16_t param) {
  safety_config ret;
  chrysler_das_3_last_valid = false;
  chrysler_long_shadow_enabled = false;
  chrysler_jeep_rate4_enabled = false;
  chrysler_jeep_rate5_enabled = false;
  chrysler_long_diagnostic_enabled = false;
  chrysler_long_actuation_enabled = false;
  chrysler_long_stock_collision = false;
  chrysler_long_speed_seen = false;
  chrysler_long_gas_seen = false;
  chrysler_long_brake_seen = false;
  chrysler_long_stock_seen = false;
  chrysler_long_counter_seen = false;
  chrysler_long_last_counter = 0U;
  chrysler_long_cycle_counter = 0U;
  chrysler_long_last_cycle_ts = 0U;
  chrysler_long_vehicle_speed_raw = 0;
  chrysler_long_low_speed_state = CHRYSLER_LONG_LOW_DRIVE;
  chrysler_long_low_speed_go_cycles = 0U;
  chrysler_b6y_hold_last_ts = 0U;
  chrysler_long_enable_button_prev = 0U;
  chrysler_long_enable_press_qualified = false;
  chrysler_long_dashboard_fault = false;
  chrysler_long_dashboard_seen = false;
  chrysler_long_dashboard_ts = 0U;
  chrysler_b6y_hold_clear();
  chrysler_long_reset_pending();

  bool enable_ram_dt = GET_FLAG(param, CHRYSLER_PARAM_RAM_DT);
  if (enable_ram_dt) {
    chrysler_platform = CHRYSLER_RAM_DT;
    chrysler_addrs = &CHRYSLER_RAM_DT_ADDRS;
    ret = BUILD_SAFETY_CFG(chrysler_ram_dt_rx_checks, CHRYSLER_RAM_DT_TX_MSGS);
#ifdef ALLOW_DEBUG
  } else if (GET_FLAG(param, CHRYSLER_PARAM_RAM_HD)) {
    chrysler_platform = CHRYSLER_RAM_HD;
    chrysler_addrs = &CHRYSLER_RAM_HD_ADDRS;
    ret = BUILD_SAFETY_CFG(chrysler_ram_hd_rx_checks, CHRYSLER_RAM_HD_TX_MSGS);
#endif
  } else {
    chrysler_platform = CHRYSLER_PACIFICA;
    chrysler_addrs = &CHRYSLER_ADDRS;
    const bool jeep_rate4_requested =
      GET_FLAG(param, CHRYSLER_PARAM_JEEP_RATE4);
    const bool jeep_rate5_requested =
      GET_FLAG(param, CHRYSLER_PARAM_JEEP_RATE5);
    // Conflicting response flags fall closed to the stock rate-3 envelope.
    chrysler_jeep_rate4_enabled =
      jeep_rate4_requested && !jeep_rate5_requested;
    chrysler_jeep_rate5_enabled =
      jeep_rate5_requested && !jeep_rate4_requested;
    chrysler_long_shadow_enabled =
      GET_FLAG(param, CHRYSLER_PARAM_JEEP_LONG_SHADOW);
    chrysler_long_diagnostic_enabled =
      GET_FLAG(param, CHRYSLER_PARAM_JEEP_LONG_DIAGNOSTIC);
    chrysler_long_actuation_enabled =
      chrysler_long_shadow_enabled &&
      GET_FLAG(param, CHRYSLER_PARAM_JEEP_LONG_ACTUATION);
    if (chrysler_long_shadow_enabled) {
      ret = BUILD_SAFETY_CFG(chrysler_long_rx_checks,
                             CHRYSLER_LONG_SHADOW_TX_MSGS);
    } else {
      ret = BUILD_SAFETY_CFG(chrysler_rx_checks, CHRYSLER_TX_MSGS);
    }
  }
  return ret;
}

const safety_hooks chrysler_hooks = {
  .init = chrysler_init,
  .rx = chrysler_rx_hook,
  .tx = chrysler_tx_hook,
  .fwd = chrysler_fwd_hook,
  .get_counter = chrysler_get_counter,
  .get_checksum = chrysler_get_checksum,
  .compute_checksum = chrysler_compute_checksum,
};

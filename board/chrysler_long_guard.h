#ifndef CHRYSLER_LONG_GUARD_H
#define CHRYSLER_LONG_GUARD_H

#include <stdbool.h>
#include <stdint.h>

// wp-b6p calibrated actuation build. Runtime substitution remains behind every
// freshness, integrity, pedal, collision, speed, command-envelope, and
// single-owner handoff guard below and cannot be enabled or altered with a
// compiler flag.
#ifdef CHRYSLER_LONG_ACTUATION
#error "CHRYSLER_LONG_ACTUATION must not be set from the build command"
#endif
#define CHRYSLER_LONG_ACTUATION 1U

#define CHRYSLER_LONG_BRAKE_TIMEOUT_US 100000U
#define CHRYSLER_LONG_DASH_TIMEOUT_US 250000U
#define CHRYSLER_LONG_TORQUE_TIMEOUT_US 100000U
#define CHRYSLER_LONG_SPEED_TIMEOUT_US 100000U
#define CHRYSLER_LONG_GAS_PEDAL_TIMEOUT_US 100000U
#define CHRYSLER_LONG_BRAKE_PEDAL_TIMEOUT_US 100000U
#define CHRYSLER_LONG_STOCK_ACC_TIMEOUT_US 100000U
// Route 28 showed valid factory-cancel acknowledgements straddling the old
// 300 ms limit (0.299-0.322 s). Canceling still passes factory DAS_3 unchanged
// and cannot actuate openpilot commands, so allow bounded scheduling margin.
#define CHRYSLER_LONG_CANCEL_TIMEOUT_US 800000U

// The FCA DAS_3 source is isolated on physical White Panda CAN2, which the
// firmware numbers as bus 1. Only a fresh, integrity-checked stock DAS_3 from
// that bus may select normal LKAS steering. APA is deliberately never selected
// by this guard.
#define CHRYSLER_STEER_STOCK_BUS 1
#define CHRYSLER_STEER_MODE_LKAS 1
#define CHRYSLER_STEER_MODE_APA 2
#define CHRYSLER_STEER_MODE_DISABLED 3
#define CHRYSLER_STEER_STOCK_TIMEOUT_US CHRYSLER_LONG_STOCK_ACC_TIMEOUT_US

// ACC_DECEL_CMD: raw * 0.004885 - 16 m/s^2.
#define CHRYSLER_LONG_DECEL_MIN_RAW 2661  // -3.001015 m/s^2, stock p01
#define CHRYSLER_LONG_DECEL_BRAKE_MAX_RAW 3275  // approximately 0 m/s^2
#define CHRYSLER_LONG_DECEL_INACTIVE_RAW 4094   // stock no-brake sentinel

// A stock-ACC uphill capture sustained 503.25 Nm median and reached 535.5 Nm
// with both pedals released. The private DAS_3 scaling is raw * 0.25 - 500 Nm,
// so raw 4000 independently enforces the common 500 Nm b6s ceiling.
#define CHRYSLER_LONG_TORQUE_ZERO_RAW 2000
#define CHRYSLER_LONG_TORQUE_MAX_RAW 4000

// SPEED_1 raw * 0.071028 m/s. Running torque uses the independently measured
// raw-11 threshold. The complete capture also showed a bounded engine request
// immediately after GO, so a separate 200 Nm launch ceiling is permitted only
// from the guarded CREEP state.
#define CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW 11  // approximately 0.78 m/s
#define CHRYSLER_LONG_LAUNCH_TORQUE_MAX_RAW 2800 // 200 Nm
#define CHRYSLER_LONG_STOP_GO_SPEED_MAX_RAW 12 // approximately 0.85 m/s

// The existing 0x4FF White Panda beacon remains four bytes long. b6h emits it
// once per stock 50 Hz DAS_3 frame through the normal transmit queue rather
// than writing a hardware mailbox for every CAN1 receive interrupt.
#define CHRYSLER_LONG_DIAG_SIGNATURE 0xB0U
#define CHRYSLER_LONG_DIAG_APPLIED (1U << 0)
#define CHRYSLER_LONG_DIAG_HOST_REQUESTED (1U << 1)
#define CHRYSLER_LONG_DIAG_BRAKE_REQUESTED (1U << 2)
#define CHRYSLER_LONG_DIAG_ENGINE_REQUESTED (1U << 3)

// A separate diagnostic-only frame records the factory command before
// substitution and the command actually forwarded to the vehicle. The first
// two bytes identify the layout; the remaining bytes preserve the original
// DAS_3 byte pairs exactly so analysis does not depend on firmware rounding.
#define CHRYSLER_LONG_COMMAND_DIAG_SIGNATURE 0xC1U
#define CHRYSLER_LONG_COMMAND_DIAG_VERSION 1U

// A separate ownership record makes the stock-ACC handoff auditable without
// overloading the established 0x4FE/0x4FF diagnostic layouts.
#define CHRYSLER_LONG_OWNER_DIAG_SIGNATURE 0xD0U
#define CHRYSLER_LONG_OWNER_OFF 0U
#define CHRYSLER_LONG_OWNER_CANCELING 1U
#define CHRYSLER_LONG_OWNER_OPENPILOT 2U
#define CHRYSLER_LONG_OWNER_FAILED 3U
#define CHRYSLER_LONG_OWNER_DIAG_STOCK_AVAILABLE (1U << 0)
#define CHRYSLER_LONG_OWNER_DIAG_STOCK_ACTIVE (1U << 1)
#define CHRYSLER_LONG_OWNER_DIAG_STOCK_COLLISION (1U << 2)
#define CHRYSLER_LONG_OWNER_DIAG_CANCEL_INJECTED (1U << 3)
#define CHRYSLER_LONG_OWNER_DIAG_STOCK_VALID (1U << 4)

// If factory ACC is already inactive when the host requests ownership, there
// is no active controller to Cancel. Require five consecutive integrity-
// checked 50 Hz inactive frames before taking ownership directly.
#define CHRYSLER_LONG_INACTIVE_CONFIRM_FRAMES 5U

#define CHRYSLER_LONG_LOW_DRIVE 0U
#define CHRYSLER_LONG_LOW_HOLD 1U
#define CHRYSLER_LONG_LOW_RELEASE 2U
#define CHRYSLER_LONG_LOW_GO 3U
#define CHRYSLER_LONG_LOW_CREEP 4U
#define CHRYSLER_LONG_LOW_GO_MAX_CYCLES 4U

static inline uint32_t chrysler_long_status_diagnostic_word(
    const uint8_t status,
    const uint16_t failure_mask,
    const uint8_t counters) {
  return (uint32_t)status |
         (uint32_t)(failure_mask & 0xFFU) << 8 |
         (uint32_t)((failure_mask >> 8) & 0xFFU) << 16 |
         (uint32_t)counters << 24;
}

static inline uint32_t chrysler_long_command_diagnostic_low(
    const uint16_t stock_engine_word) {
  return (uint32_t)CHRYSLER_LONG_COMMAND_DIAG_SIGNATURE |
         (uint32_t)CHRYSLER_LONG_COMMAND_DIAG_VERSION << 8 |
         (uint32_t)stock_engine_word << 16;
}

static inline uint32_t chrysler_long_command_diagnostic_high(
    const uint16_t output_engine_word,
    const uint16_t stock_accel_word) {
  return (uint32_t)output_engine_word |
         (uint32_t)stock_accel_word << 16;
}

static inline uint32_t chrysler_long_owner_diagnostic_word(
    const uint8_t owner_state,
    const bool stock_valid,
    const bool stock_available,
    const bool stock_active,
    const int stock_fault,
    const bool stock_collision,
    const bool cancel_injected) {
  uint8_t stock_status = 0U;
  stock_status |= stock_valid ?
    CHRYSLER_LONG_OWNER_DIAG_STOCK_VALID : 0U;
  stock_status |= stock_available ?
    CHRYSLER_LONG_OWNER_DIAG_STOCK_AVAILABLE : 0U;
  stock_status |= stock_active ?
    CHRYSLER_LONG_OWNER_DIAG_STOCK_ACTIVE : 0U;
  stock_status |= stock_collision ?
    CHRYSLER_LONG_OWNER_DIAG_STOCK_COLLISION : 0U;
  stock_status |= cancel_injected ?
    CHRYSLER_LONG_OWNER_DIAG_CANCEL_INJECTED : 0U;

  return (uint32_t)(CHRYSLER_LONG_OWNER_DIAG_SIGNATURE |
                    (owner_state & 0x0FU)) |
         (uint32_t)stock_status << 8 |
         (uint32_t)(stock_fault & 0x3) << 16;
}

#define CHRYSLER_LONG_DIAG_ACTUATION_DISABLED (1U << 0)
#define CHRYSLER_LONG_DIAG_HOST_NOT_REQUESTED (1U << 1)
#define CHRYSLER_LONG_DIAG_BRAKE_NOT_FRESH (1U << 2)
#define CHRYSLER_LONG_DIAG_DASH_NOT_FRESH (1U << 3)
#define CHRYSLER_LONG_DIAG_TORQUE_NOT_FRESH (1U << 4)
#define CHRYSLER_LONG_DIAG_SPEED_NOT_FRESH (1U << 5)
#define CHRYSLER_LONG_DIAG_GAS_NOT_FRESH (1U << 6)
#define CHRYSLER_LONG_DIAG_BRAKE_PEDAL_NOT_FRESH (1U << 7)
#define CHRYSLER_LONG_DIAG_STOCK_ACC_NOT_FRESH (1U << 8)
#define CHRYSLER_LONG_DIAG_COUNTERS_MISALIGNED (1U << 9)
#define CHRYSLER_LONG_DIAG_PRIVATE_INTEGRITY (1U << 10)
#define CHRYSLER_LONG_DIAG_SPEED_TOO_LOW (1U << 11)
#define CHRYSLER_LONG_DIAG_DRIVER_BRAKE (1U << 12)
#define CHRYSLER_LONG_DIAG_DRIVER_GAS (1U << 13)
#define CHRYSLER_LONG_DIAG_COLLISION (1U << 14)
#define CHRYSLER_LONG_DIAG_COMMAND_ENVELOPE (1U << 15)

static inline bool chrysler_long_is_fresh(const uint32_t now, const uint32_t last,
                                          const bool valid, const uint32_t timeout_us) {
  return valid && ((uint32_t)(now - last) <= timeout_us);
}

static inline bool chrysler_long_counter_step_valid(bool *seen, int *last,
                                                     const int current) {
  const bool valid = !*seen || (current == ((*last + 1) & 0xF));
  *seen = true;
  *last = current;
  return valid;
}

static inline int chrysler_steer_mode_from_stock_das3(
    const int source_bus,
    const uint32_t now,
    const uint32_t last,
    const bool frame_valid,
    const bool acc_available) {
  const bool valid_can2_source = source_bus == CHRYSLER_STEER_STOCK_BUS;
  const bool fresh = chrysler_long_is_fresh(
    now, last, frame_valid, CHRYSLER_STEER_STOCK_TIMEOUT_US);

  return (valid_can2_source && fresh && acc_available) ?
    CHRYSLER_STEER_MODE_LKAS : CHRYSLER_STEER_MODE_DISABLED;
}

static inline bool chrysler_steer_stock_das3_integrity_valid(
    const int source_bus,
    const int length,
    const bool checksum_valid,
    const bool counter_valid) {
  return (source_bus == CHRYSLER_STEER_STOCK_BUS) &&
         (length == 8) && checksum_valid && counter_valid;
}

static inline uint8_t chrysler_long_update_inactive_confirmation(
    const uint8_t current_count,
    const bool valid_inactive_candidate) {
  if (!valid_inactive_candidate) {
    return 0U;
  }
  return (current_count < CHRYSLER_LONG_INACTIVE_CONFIRM_FRAMES) ?
    (uint8_t)(current_count + 1U) : current_count;
}

static inline bool chrysler_long_inactive_confirmation_complete(
    const uint8_t count) {
  return count >= CHRYSLER_LONG_INACTIVE_CONFIRM_FRAMES;
}

// Factory ACC must not remain an active second longitudinal controller. Start
// by requesting a normal Cancel while factory ACC retains main availability.
// Full substitution is permitted only after a subsequent stock DAS_3 confirms
// that factory ACC is inactive. Any loss of host authority, factory
// availability, or a stock fault/collision immediately drops ownership.
static inline uint8_t chrysler_long_next_owner_state(
    const uint8_t owner_state,
    const bool guard_enabled,
    const bool stock_frame_valid,
    const bool stock_available,
    const bool stock_active,
    const bool stock_inactive_confirmed,
    const int stock_fault,
    const bool stock_collision) {
  if (!guard_enabled) {
    return CHRYSLER_LONG_OWNER_OFF;
  }

  if (owner_state == CHRYSLER_LONG_OWNER_FAILED) {
    // Require host authority to be released before another handoff attempt.
    return CHRYSLER_LONG_OWNER_FAILED;
  }

  if (!stock_frame_valid || !stock_available || (stock_fault != 0) ||
      stock_collision) {
    return CHRYSLER_LONG_OWNER_OFF;
  }

  if (owner_state == CHRYSLER_LONG_OWNER_OFF) {
    return stock_active ? CHRYSLER_LONG_OWNER_CANCELING :
      (stock_inactive_confirmed ? CHRYSLER_LONG_OWNER_OPENPILOT :
       CHRYSLER_LONG_OWNER_OFF);
  }
  if (owner_state == CHRYSLER_LONG_OWNER_CANCELING) {
    return stock_active ?
      CHRYSLER_LONG_OWNER_CANCELING : CHRYSLER_LONG_OWNER_OPENPILOT;
  }
  if (owner_state == CHRYSLER_LONG_OWNER_OPENPILOT) {
    // An unexpected factory re-engagement suspends substitution before the
    // current frame reaches the vehicle and restarts the Cancel handshake.
    return stock_active ?
      CHRYSLER_LONG_OWNER_CANCELING : CHRYSLER_LONG_OWNER_OPENPILOT;
  }
  return CHRYSLER_LONG_OWNER_OFF;
}

static inline bool chrysler_long_current_stock_frame_valid(
    const bool previous_counter_seen,
    const int previous_counter,
    const int current_counter,
    const int length,
    const bool checksum_valid) {
  return previous_counter_seen && (length == 8) && checksum_valid &&
         (current_counter == ((previous_counter + 1) & 0xF));
}

static inline bool chrysler_long_cancel_timed_out(
    const uint8_t owner_state,
    const uint32_t now,
    const uint32_t cancel_start_ts) {
  return (owner_state == CHRYSLER_LONG_OWNER_CANCELING) &&
         ((uint32_t)(now - cancel_start_ts) >
          CHRYSLER_LONG_CANCEL_TIMEOUT_US);
}

static inline bool chrysler_long_should_substitute_das3(
    const uint8_t owner_state,
    const bool guard_enabled,
    const bool stock_frame_valid,
    const int stock_fault,
    const bool stock_collision,
    const bool stock_available,
    const bool stock_active) {
  return (CHRYSLER_LONG_ACTUATION != 0U) &&
         (owner_state == CHRYSLER_LONG_OWNER_OPENPILOT) &&
         guard_enabled && stock_frame_valid && stock_available && !stock_active &&
         (stock_fault == 0) && !stock_collision;
}

// While canceling, request a normal ACC Cancel from the isolated factory ACC
// source. During openpilot ownership, prevent Set/Resume and speed-adjust
// buttons from silently reactivating the factory controller. Main and distance
// buttons remain untouched, and a physical Cancel always passes through.
static inline uint8_t chrysler_long_filter_button_byte(
    const uint8_t buttons,
    const uint8_t owner_state) {
  if ((owner_state != CHRYSLER_LONG_OWNER_CANCELING) &&
      (owner_state != CHRYSLER_LONG_OWNER_OPENPILOT)) {
    return buttons;
  }
  uint8_t filtered = buttons & (uint8_t)~0x1CU;
  if (owner_state == CHRYSLER_LONG_OWNER_CANCELING) {
    filtered |= 0x01U;
  }
  return filtered;
}

static inline bool chrysler_long_counters_aligned(
    const bool brake_seen, const int brake_counter,
    const bool dash_seen, const int dash_counter,
    const bool torque_seen, const int torque_counter) {
  return brake_seen && dash_seen && torque_seen &&
         (brake_counter == dash_counter) && (dash_counter == torque_counter);
}

// The three private messages form one logical command. A partial next cycle
// must never replace or invalidate the last complete cycle simply because CAN
// arbitration delivered its brake, dash, and torque frames one at a time.
static inline bool chrysler_long_staged_cycle_ready(
    const bool brake_valid, const int brake_counter,
    const bool dash_valid, const int dash_counter,
    const bool torque_valid, const int torque_counter) {
  return brake_valid && dash_valid && torque_valid &&
         (brake_counter == dash_counter) &&
         (dash_counter == torque_counter);
}

static inline bool chrysler_long_commands_valid(
    const bool host_requested,
    const bool acc_available_cmd,
    const bool acc_enabled_cmd,
    const bool acc_stop_cmd,
    const bool acc_go_cmd,
    const int decel_raw,
    const int command_type_cmd,
    const bool brake_prep_cmd,
    const bool engine_request_cmd,
    const int torque_raw,
    const int speed_raw,
    const bool driver_brake,
    const bool driver_gas,
    const bool stock_collision,
    const uint8_t low_speed_state) {
  bool valid = host_requested && acc_available_cmd && acc_enabled_cmd &&
               !driver_brake && !driver_gas && !stock_collision;

  valid = valid && !(acc_stop_cmd && acc_go_cmd);
  valid = valid && (command_type_cmd >= 0) && (command_type_cmd <= 1);

  if (command_type_cmd == 1) {
    valid = valid && !engine_request_cmd;
    valid = valid && !brake_prep_cmd;
    valid = valid && !acc_go_cmd;
    valid = valid && (!acc_stop_cmd ||
                      (speed_raw <= CHRYSLER_LONG_STOP_GO_SPEED_MAX_RAW));
    valid = valid && (decel_raw >= CHRYSLER_LONG_DECEL_MIN_RAW);
    valid = valid && (decel_raw <= CHRYSLER_LONG_DECEL_BRAKE_MAX_RAW);
  } else {
    valid = valid && !brake_prep_cmd;
    valid = valid && (decel_raw == CHRYSLER_LONG_DECEL_INACTIVE_RAW);
    if (engine_request_cmd) {
      valid = valid && !acc_stop_cmd && !acc_go_cmd;
      valid = valid && (torque_raw >= CHRYSLER_LONG_TORQUE_ZERO_RAW);
      const bool running_torque_valid =
        (speed_raw >= CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW) &&
        (torque_raw <= CHRYSLER_LONG_TORQUE_MAX_RAW);
      const bool launch_torque_valid =
        (speed_raw < CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW) &&
        (low_speed_state == CHRYSLER_LONG_LOW_CREEP) &&
        (torque_raw <= CHRYSLER_LONG_LAUNCH_TORQUE_MAX_RAW);
      valid = valid && (running_torque_valid || launch_torque_valid);
    } else {
      valid = valid && !acc_stop_cmd;
      valid = valid && (!acc_go_cmd ||
                        (speed_raw <= CHRYSLER_LONG_STOP_GO_SPEED_MAX_RAW));
      valid = valid && (torque_raw == CHRYSLER_LONG_TORQUE_ZERO_RAW);
    }
  }

  return valid;
}

static inline bool chrysler_long_speed_valid_for_command(
    const bool acc_stop_cmd,
    const bool acc_go_cmd,
    const int command_type_cmd,
    const bool engine_request_cmd,
    const int speed_raw,
    const uint8_t low_speed_state) {
  if (acc_stop_cmd || acc_go_cmd) {
    return speed_raw <= CHRYSLER_LONG_STOP_GO_SPEED_MAX_RAW;
  }
  if ((command_type_cmd == 0) && engine_request_cmd) {
    return (speed_raw >= CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW) ||
           (low_speed_state == CHRYSLER_LONG_LOW_CREEP);
  }
  return true;
}

// Independently enforce the order observed in the OEM stop/go capture. A GO
// request is accepted only after a stopped brake hold and at least one release
// cycle. It is bounded to four private 25 Hz cycles (160 ms maximum), cannot be
// repeated from creep, and never overlaps a brake or engine request.
static inline bool chrysler_long_low_speed_transition(
    const uint8_t current_state,
    const uint8_t current_go_cycles,
    const bool acc_stop_cmd,
    const bool acc_go_cmd,
    const int command_type_cmd,
    const bool engine_request_cmd,
    const int speed_raw,
    uint8_t *next_state,
    uint8_t *next_go_cycles) {
  uint8_t state = current_state;
  uint8_t go_cycles = current_go_cycles;
  const bool brake_request = command_type_cmd == 1;

  if (speed_raw > CHRYSLER_LONG_STOP_GO_SPEED_MAX_RAW) {
    state = CHRYSLER_LONG_LOW_DRIVE;
    go_cycles = 0U;
  } else if (engine_request_cmd) {
    if (acc_stop_cmd || acc_go_cmd || brake_request) {
      return false;
    }
    if (speed_raw >= CHRYSLER_LONG_ENGINE_SPEED_MIN_RAW) {
      state = CHRYSLER_LONG_LOW_DRIVE;
      go_cycles = 0U;
    } else if (current_state != CHRYSLER_LONG_LOW_CREEP) {
      return false;
    }
  } else if (acc_stop_cmd && brake_request && !acc_go_cmd) {
    state = CHRYSLER_LONG_LOW_HOLD;
    go_cycles = 0U;
  } else if (current_state == CHRYSLER_LONG_LOW_DRIVE) {
    if (acc_go_cmd) {
      return false;
    }
  } else if (current_state == CHRYSLER_LONG_LOW_HOLD) {
    if (acc_go_cmd) {
      return false;
    }
    // Residual braking during the host's release ramp does not count as a
    // released cycle. Stay latched in HOLD until one complete neutral private
    // snapshot has been committed.
    if (!brake_request && !engine_request_cmd) {
      state = CHRYSLER_LONG_LOW_RELEASE;
      go_cycles = 0U;
    }
  } else if (current_state == CHRYSLER_LONG_LOW_RELEASE) {
    if (brake_request) {
      state = CHRYSLER_LONG_LOW_HOLD;
      go_cycles = 0U;
    }
    if (acc_go_cmd) {
      if (brake_request || engine_request_cmd) {
        return false;
      }
      state = CHRYSLER_LONG_LOW_GO;
      go_cycles = 1U;
    }
  } else if (current_state == CHRYSLER_LONG_LOW_GO) {
    if (acc_go_cmd) {
      if (brake_request || engine_request_cmd ||
          (current_go_cycles >= CHRYSLER_LONG_LOW_GO_MAX_CYCLES)) {
        return false;
      }
      go_cycles = current_go_cycles + 1U;
    } else {
      state = CHRYSLER_LONG_LOW_CREEP;
      go_cycles = 0U;
    }
  } else if (current_state == CHRYSLER_LONG_LOW_CREEP) {
    if (acc_go_cmd) {
      return false;
    }
  } else {
    return false;
  }

  *next_state = state;
  *next_go_cycles = go_cycles;
  return true;
}

static inline uint16_t chrysler_long_diagnostic_mask(
    const bool actuation_enabled,
    const bool host_requested,
    const bool brake_fresh,
    const bool dash_fresh,
    const bool torque_fresh,
    const bool speed_fresh,
    const bool gas_fresh,
    const bool brake_pedal_fresh,
    const bool stock_acc_fresh,
    const bool counters_aligned,
    const bool private_integrity_valid,
    const bool speed_high_enough,
    const bool driver_brake,
    const bool driver_gas,
    const bool stock_collision,
    const bool commands_valid) {
  uint16_t mask = 0U;
  mask |= actuation_enabled ? 0U : CHRYSLER_LONG_DIAG_ACTUATION_DISABLED;
  mask |= host_requested ? 0U : CHRYSLER_LONG_DIAG_HOST_NOT_REQUESTED;
  mask |= brake_fresh ? 0U : CHRYSLER_LONG_DIAG_BRAKE_NOT_FRESH;
  mask |= dash_fresh ? 0U : CHRYSLER_LONG_DIAG_DASH_NOT_FRESH;
  mask |= torque_fresh ? 0U : CHRYSLER_LONG_DIAG_TORQUE_NOT_FRESH;
  mask |= speed_fresh ? 0U : CHRYSLER_LONG_DIAG_SPEED_NOT_FRESH;
  mask |= gas_fresh ? 0U : CHRYSLER_LONG_DIAG_GAS_NOT_FRESH;
  mask |= brake_pedal_fresh ? 0U : CHRYSLER_LONG_DIAG_BRAKE_PEDAL_NOT_FRESH;
  mask |= stock_acc_fresh ? 0U : CHRYSLER_LONG_DIAG_STOCK_ACC_NOT_FRESH;
  mask |= counters_aligned ? 0U : CHRYSLER_LONG_DIAG_COUNTERS_MISALIGNED;
  mask |= private_integrity_valid ? 0U : CHRYSLER_LONG_DIAG_PRIVATE_INTEGRITY;
  mask |= speed_high_enough ? 0U : CHRYSLER_LONG_DIAG_SPEED_TOO_LOW;
  mask |= driver_brake ? CHRYSLER_LONG_DIAG_DRIVER_BRAKE : 0U;
  mask |= driver_gas ? CHRYSLER_LONG_DIAG_DRIVER_GAS : 0U;
  mask |= stock_collision ? CHRYSLER_LONG_DIAG_COLLISION : 0U;
  mask |= commands_valid ? 0U : CHRYSLER_LONG_DIAG_COMMAND_ENVELOPE;
  return mask;
}

#endif

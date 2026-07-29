#ifndef CHRYSLER_LONG_GUARD_H
#define CHRYSLER_LONG_GUARD_H

#include <stdbool.h>
#include <stdint.h>

// b8y actuation build. This remains source-controlled and cannot be enabled or
// altered with a compiler flag. Runtime output still requires every freshness,
// integrity, pedal, collision, speed, and command-envelope check below.
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

// ACC_DECEL_CMD: raw * 0.004885 - 16 m/s^2.
#define CHRYSLER_LONG_DECEL_MIN_RAW 2661  // -3.001015 m/s^2, stock p01
#define CHRYSLER_LONG_DECEL_BRAKE_MAX_RAW 3275  // approximately 0 m/s^2
#define CHRYSLER_LONG_DECEL_INACTIVE_RAW 4094   // stock no-brake sentinel

// Private engine-torque command uses the stock DAS_3 scaling:
// raw * 0.25 - 500 Nm. The calibrated shadow ceiling is 100 Nm.
#define CHRYSLER_LONG_TORQUE_ZERO_RAW 2000
#define CHRYSLER_LONG_TORQUE_MAX_RAW 2400

// SPEED_1 raw * 0.071028 m/s. The initial scope is moving-only and excludes
// standstill, stop, go, brake preparation, and hold behavior.
#define CHRYSLER_LONG_MOVING_SPEED_MIN_RAW 29  // approximately 2.06 m/s

// The existing 0x4FF White Panda beacon remains four bytes long. Its first
// byte keeps a fixed high-nibble signature and four outcome bits; the middle
// two bytes carry this diagnostic failure mask. This changes no CAN identifier,
// payload length, or actuation field and never participates in guard logic.
#define CHRYSLER_LONG_DIAG_SIGNATURE 0xB0U
#define CHRYSLER_LONG_DIAG_APPLIED (1U << 0)
#define CHRYSLER_LONG_DIAG_HOST_REQUESTED (1U << 1)
#define CHRYSLER_LONG_DIAG_BRAKE_REQUESTED (1U << 2)
#define CHRYSLER_LONG_DIAG_ENGINE_REQUESTED (1U << 3)

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

static inline bool chrysler_long_counters_aligned(
    const bool brake_seen, const int brake_counter,
    const bool dash_seen, const int dash_counter,
    const bool torque_seen, const int torque_counter) {
  return brake_seen && dash_seen && torque_seen &&
         (brake_counter == dash_counter) && (dash_counter == torque_counter);
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
    const bool stock_collision) {
  bool valid = host_requested && acc_available_cmd && acc_enabled_cmd &&
               !driver_brake && !driver_gas && !stock_collision;

  valid = valid && !acc_stop_cmd && !acc_go_cmd;
  valid = valid && (speed_raw >= CHRYSLER_LONG_MOVING_SPEED_MIN_RAW);
  valid = valid && (command_type_cmd >= 0) && (command_type_cmd <= 1);

  if (command_type_cmd == 1) {
    valid = valid && !engine_request_cmd;
    valid = valid && !brake_prep_cmd;
    valid = valid && (decel_raw >= CHRYSLER_LONG_DECEL_MIN_RAW);
    valid = valid && (decel_raw <= CHRYSLER_LONG_DECEL_BRAKE_MAX_RAW);
  } else {
    valid = valid && !brake_prep_cmd;
    valid = valid && (decel_raw == CHRYSLER_LONG_DECEL_INACTIVE_RAW);
    if (engine_request_cmd) {
      valid = valid && (torque_raw >= CHRYSLER_LONG_TORQUE_ZERO_RAW);
      valid = valid && (torque_raw <= CHRYSLER_LONG_TORQUE_MAX_RAW);
    } else {
      valid = valid && (torque_raw == CHRYSLER_LONG_TORQUE_ZERO_RAW);
    }
  }

  return valid;
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

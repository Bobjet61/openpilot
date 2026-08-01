#ifndef CHRYSLER_LONG_GUARD_H
#define CHRYSLER_LONG_GUARD_H

#include <stdbool.h>
#include <stdint.h>

// wp-b6k recovery build. Factory ACC owns longitudinal control and every
// intercepted factory command must pass through unchanged. This remains
// source-controlled and cannot be enabled or altered with a compiler flag.
#ifdef CHRYSLER_LONG_ACTUATION
#error "CHRYSLER_LONG_ACTUATION must not be set from the build command"
#endif
#define CHRYSLER_LONG_ACTUATION 0U

#define CHRYSLER_LONG_BRAKE_TIMEOUT_US 100000U
#define CHRYSLER_LONG_DASH_TIMEOUT_US 250000U
#define CHRYSLER_LONG_TORQUE_TIMEOUT_US 100000U
#define CHRYSLER_LONG_SPEED_TIMEOUT_US 100000U
#define CHRYSLER_LONG_GAS_PEDAL_TIMEOUT_US 100000U
#define CHRYSLER_LONG_BRAKE_PEDAL_TIMEOUT_US 100000U
#define CHRYSLER_LONG_STOCK_ACC_TIMEOUT_US 100000U

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

// Private engine-torque command uses the stock DAS_3 scaling:
// raw * 0.25 - 500 Nm. The calibrated shadow ceiling is 100 Nm.
#define CHRYSLER_LONG_TORQUE_ZERO_RAW 2000
#define CHRYSLER_LONG_TORQUE_MAX_RAW 2400

// SPEED_1 raw * 0.071028 m/s. The initial scope is moving-only and excludes
// standstill, stop, go, brake preparation, and hold behavior.
#define CHRYSLER_LONG_MOVING_SPEED_MIN_RAW 29  // approximately 2.06 m/s

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

// Full-long keeps factory ACC-main availability as an independent permission.
// Steering-wheel button traffic must therefore remain bit-for-bit unchanged;
// synthesizing ACC-off here removes that permission and forces wrongCarMode.
static inline uint32_t chrysler_long_wheel_button_passthrough(
    const uint32_t word) {
  return word;
}

// The factory ACC module supervises its normal braking request in DAS_3. The
// two b6i road faults occurred after command type 1 was replaced by an openpilot
// propulsion command. Defer that frame to factory ACC immediately; collision
// and AEB frames remain covered by the existing stock_collision guard.
static inline bool chrysler_long_should_substitute_das3(
    const bool guard_enabled,
    const bool stock_collision,
    const int stock_command_type) {
  return (CHRYSLER_LONG_ACTUATION != 0U) && guard_enabled &&
         !stock_collision && (stock_command_type == 0);
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

#ifndef CHRYSLER_LONG_GUARD_H
#define CHRYSLER_LONG_GUARD_H

#include <stdbool.h>
#include <stdint.h>

// Hard-coded off for source review and bench validation. This is deliberately
// not overrideable with a compiler flag.
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

static inline bool chrysler_long_is_fresh(const uint32_t now, const uint32_t last,
                                          const bool valid, const uint32_t timeout_us) {
  return valid && ((uint32_t)(now - last) <= timeout_us);
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

#endif

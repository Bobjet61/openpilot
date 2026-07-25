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
#define CHRYSLER_LONG_DECEL_MIN_RAW 2559  // approximately -3.5 m/s^2
#define CHRYSLER_LONG_DECEL_ZERO_RAW 3275

// ACC_TORQ: raw - 7767 Nm. The shadow host caps acceleration at 1 m/s^2
// and estimates less than 80 Nm for this non-hybrid Jeep. Keep 100 Nm as
// the provisional absolute ceiling pending logged calibration.
#define CHRYSLER_LONG_TORQUE_ZERO_RAW 7767
#define CHRYSLER_LONG_TORQUE_MAX_RAW 7867

// Stop/go state is only valid near standstill. ESP_8 speed is 1/128 km/h.
#define CHRYSLER_LONG_STOP_SPEED_MAX_RAW 206  // approximately 1 mph

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

  valid = valid && !(acc_stop_cmd && acc_go_cmd);
  valid = valid && (command_type_cmd >= 0) && (command_type_cmd <= 1);

  if (acc_stop_cmd || acc_go_cmd) {
    valid = valid && (speed_raw <= CHRYSLER_LONG_STOP_SPEED_MAX_RAW);
  }

  if (command_type_cmd == 1) {
    valid = valid && !engine_request_cmd;
    valid = valid && brake_prep_cmd;
    valid = valid && (decel_raw >= CHRYSLER_LONG_DECEL_MIN_RAW);
    valid = valid && (decel_raw <= CHRYSLER_LONG_DECEL_ZERO_RAW);
    valid = valid && !acc_go_cmd;
  } else {
    valid = valid && !brake_prep_cmd;
    valid = valid && !acc_stop_cmd;
    if (engine_request_cmd) {
      valid = valid && (torque_raw >= CHRYSLER_LONG_TORQUE_ZERO_RAW);
      valid = valid && (torque_raw <= CHRYSLER_LONG_TORQUE_MAX_RAW);
    }
  }

  return valid;
}

#endif

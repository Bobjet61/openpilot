#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "../board/chrysler_long_guard.h"


int main(void) {
  const uint32_t now = 1000000U;
  const uint32_t fresh = now - CHRYSLER_STEER_STOCK_TIMEOUT_US;

  assert(chrysler_steer_mode_from_stock_das3(
    CHRYSLER_STEER_STOCK_BUS, now, fresh, true, true) ==
    CHRYSLER_STEER_MODE_LKAS);

  // The next microsecond must fail disabled.
  assert(chrysler_steer_mode_from_stock_das3(
    CHRYSLER_STEER_STOCK_BUS, now, fresh - 1U, true, true) ==
    CHRYSLER_STEER_MODE_DISABLED);
  assert(chrysler_steer_mode_from_stock_das3(
    CHRYSLER_STEER_STOCK_BUS, now, fresh, false, true) ==
    CHRYSLER_STEER_MODE_DISABLED);
  assert(chrysler_steer_mode_from_stock_das3(
    CHRYSLER_STEER_STOCK_BUS, now, fresh, true, false) ==
    CHRYSLER_STEER_MODE_DISABLED);

  // Main-CAN and EPS-CAN copies cannot authorize steering.
  assert(chrysler_steer_mode_from_stock_das3(
    0, now, fresh, true, true) == CHRYSLER_STEER_MODE_DISABLED);
  assert(chrysler_steer_mode_from_stock_das3(
    2, now, fresh, true, true) == CHRYSLER_STEER_MODE_DISABLED);

  // Timestamp wraparound remains fresh when the unsigned delta is small.
  assert(chrysler_steer_mode_from_stock_das3(
    CHRYSLER_STEER_STOCK_BUS, 50U, 0xFFFFFFF0U, true, true) ==
    CHRYSLER_STEER_MODE_LKAS);

  assert(chrysler_steer_stock_das3_integrity_valid(
    CHRYSLER_STEER_STOCK_BUS, 8, true, true));
  assert(!chrysler_steer_stock_das3_integrity_valid(
    0, 8, true, true));
  assert(!chrysler_steer_stock_das3_integrity_valid(
    CHRYSLER_STEER_STOCK_BUS, 7, true, true));
  assert(!chrysler_steer_stock_das3_integrity_valid(
    CHRYSLER_STEER_STOCK_BUS, 8, false, true));
  assert(!chrysler_steer_stock_das3_integrity_valid(
    CHRYSLER_STEER_STOCK_BUS, 8, true, false));

  // Exercise all boolean inputs and all Panda buses: this guard has no APA
  // result under any combination.
  for (int bus = 0; bus < 3; bus++) {
    for (int valid = 0; valid < 2; valid++) {
      for (int available = 0; available < 2; available++) {
        const int mode = chrysler_steer_mode_from_stock_das3(
          bus, now, fresh, valid != 0, available != 0);
        assert(mode != CHRYSLER_STEER_MODE_APA);
        assert((mode == CHRYSLER_STEER_MODE_LKAS) ||
               (mode == CHRYSLER_STEER_MODE_DISABLED));
      }
    }
  }

  // The same rolling-counter helper used by stock DAS_3 accepts wraparound
  // and rejects duplicates/skips.
  bool seen = false;
  int last = 0;
  assert(chrysler_long_counter_step_valid(&seen, &last, 14));
  assert(chrysler_long_counter_step_valid(&seen, &last, 15));
  assert(chrysler_long_counter_step_valid(&seen, &last, 0));
  assert(!chrysler_long_counter_step_valid(&seen, &last, 0));
  assert(!chrysler_long_counter_step_valid(&seen, &last, 2));
  assert(chrysler_long_counter_step_valid(&seen, &last, 3));

  // Factory ownership remains a transparent wheel-button bridge. Exercise
  // empty, full, and representative payload words through the current
  // mode-aware arbiter rather than the removed legacy passthrough helper.
  const uint32_t factory_payloads[] = {
    0x00000000U,
    0xFFFFFFFFU,
    0x12345678U,
    0x00ABC080U,
  };
  for (unsigned int i = 0U;
       i < sizeof(factory_payloads) / sizeof(factory_payloads[0]); i++) {
    const uint32_t payload = factory_payloads[i];
    const uint8_t buttons = (uint8_t)(payload & 0xFFU);
    assert(chrysler_long_arbitrate_wheel_button_payload(
      payload, buttons, buttons, CHRYSLER_LONG_OWNER_OFF) == payload);
  }

  return 0;
}

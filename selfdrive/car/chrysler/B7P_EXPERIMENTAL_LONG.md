# b7p Experimental OP Long activation

Date: 2026-08-12

This is the explicit vehicle-actuating successor to b7o. It retains b7o's
restart-gated, mutually exclusive mode selector and changes the compile-time
openpilot-long gate from disabled to enabled.

## Modes

| Experimental Longitudinal (Alpha) | Custom Stock Long | Vehicle behavior |
|---|---|---|
| Off | Off | Factory ACC |
| Off | On | Factory ACC plus guarded stop/go |
| On | Either | Experimental OP Long |

Experimental takes priority if both toggles are selected. Factory stop/go is
disabled in that state, so the two augmentation paths cannot be armed at the
same time. Toggle changes are sampled at startup and require a comma restart.

## Actuation boundaries

- The host emits active private longitudinal commands only while openpilot-long
  is selected and all vehicle-eligibility checks pass.
- Brake and propulsion are mutually exclusive, with a neutral coast interlock
  between them.
- Driver brake, driver gas, cancel, unavailable cruise, an ACC fault, stock
  AEB, non-forward gear, or inactive longitudinal control stops output.
- The embedded Panda receives the Jeep shadow, diagnostic, and actuation safety
  flags only in Experimental OP Long mode.
- The external White Panda independently validates freshness, counters,
  liveness, vehicle state, command bounds, and brake/propulsion exclusion.
- Factory and Factory + Stop/Go modes do not receive the OP Long actuation
  safety flag.

This branch does not raise the steering-torque ceiling and does not change the
bump-steer filter or steering response-5 tune.

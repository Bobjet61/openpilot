# Jeep longitudinal shadow branch

This branch recovers and packs the Chrysler Advanced White Panda command
protocol for offline/shadow validation:

- `0x1F6` brake/deceleration command;
- `0x1F7` dashboard state and White Panda longitudinal enable bit;
- `0x272` engine-torque command.

The frames are packed in memory and logged but never appended to `can_sends`.
The `OP_LONG_ENABLE` bit is always zero.
`JEEP_LONG_ACTUATION_COMPILED` is deliberately `False`, so
`openpilotLongitudinalControl` and `experimentalLongitudinalAvailable` remain
false.

Before actuation can be considered, the exact installed White Panda firmware
must be identified and the following must be implemented and replay-tested:

- strict host-frame allowlists, frequency checks, bounds, and counters in the
  comma 3X Panda safety model;
- independent White Panda command timeouts and real pedal state checks;
- bounded acceleration, deceleration, and jerk;
- stock AEB/collision pass-through;
- disengagement on brake, gas, CAN faults, stale commands, or invalid state;
- closed-course tests beginning with propulsion disabled.

The prior XPS patch is not a safety baseline: it relaxed steering error limits
and disabled receive checks.

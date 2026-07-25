# Jeep longitudinal shadow branch

This branch packs a calibrated Chrysler Advanced White Panda command protocol
for offline/shadow validation:

- `0x1F6` brake/deceleration command;
- `0x1F7` dashboard state and White Panda longitudinal enable bit;
- `0x272` private engine-torque command, mapped by the White Panda onto the
  Jeep's stock `0x1F4` DAS_3 engine-torque fields.

The frames are packed in memory and logged but never appended to `can_sends`.
The `OP_LONG_ENABLE` bit is always zero.
`JEEP_LONG_ACTUATION_COMPILED` is deliberately `False`, so
`openpilotLongitudinalControl` and `experimentalLongitudinalAvailable` remain
false.

Before actuation can be considered, the following must be implemented and
replay-tested:

- strict host-frame allowlists, frequency checks, bounds, and counters in the
  comma 3X Panda safety model;
- independent White Panda command timeouts and real pedal state checks;
- bounded acceleration, deceleration, and jerk;
- stock AEB/collision pass-through;
- disengagement on brake, gas, CAN faults, stale commands, or invalid state;
- closed-course tests beginning with propulsion disabled.

The prior XPS patch is not a safety baseline: it relaxed steering error limits
and disabled receive checks.

## Stock-log calibration

Passive analysis of 131 rlogs (130.2 minutes) found:

- propulsion uses DAS_3 `ENGINE_TORQUE_REQUEST`; no active DAS_5 wheel-torque
  request was observed while stock ACC was active;
- driver accelerator is `0x22F` ECM_5, not the provisional `0x134` signal;
- speed is sourced from `0x202` SPEED_1;
- brake-prep was asserted on only 12.3% of normal braking-request frames.

The hard-off shadow is therefore limited to moving-only braking from -3.0 to
0.0 m/s^2 and DAS_3 engine torque from 0 to 100 Nm. Stop/go, standstill hold,
and brake-prep are excluded pending separate validation.

All three private frames carry the same rolling four-bit counter and an FCA
checksum. The private dashboard frame contains only the hard-off request bit,
counter, and checksum; the factory DAS_4 dashboard frame is passed through
unchanged.

The offline replay opened 131 downloaded rlogs. All 284,095 eligible shadow
cycles passed the envelope, and every one of 5,619 samples per injected fault
class was rejected. The short end-of-drive segment has 51,085 readable events
followed by a corrupt/truncated final event; its readable portion was replayed
and the tail is a non-blocking data caveat. The comma 3X Panda still lacks an
independent matching transmit safety policy, so the branch remains hard-off
and is not bench-ready.

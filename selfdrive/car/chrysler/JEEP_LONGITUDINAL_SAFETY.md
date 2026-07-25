# Jeep longitudinal shadow branch

This branch packs a calibrated Chrysler Advanced White Panda command protocol
for offline/shadow validation:

- `0x1F6` brake/deceleration command;
- `0x1F7` dashboard state and White Panda longitudinal enable bit;
- `0x272` private engine-torque command, mapped by the White Panda onto the
  Jeep's stock `0x1F4` DAS_3 engine-torque fields.

The source contains a 50 Hz host-to-Panda transport path, but it is guarded by
the independent `JEEP_LONG_SHADOW_TRANSPORT_COMPILED = False` constant. With
that gate off, the three frames are packed in memory for shadow analysis but
are never appended to `can_sends`, and Chrysler Panda parameter bit 4 is not
requested.

`OP_LONG_ENABLE` follows the separate
`JEEP_LONG_ACTUATION_COMPILED = False` gate. Consequently, actual
longitudinal enable remains zero even in a future disconnected-bench-only
transport build. `openpilotLongitudinalControl` and
`experimentalLongitudinalAvailable` also remain false.

The matching comma 3X Panda policy independently requires safety parameter bit
4, validates the exact frame order, counter, checksum, frequency, source
freshness, pedals, stock collision state, and command bounds, and hard-rejects
`OP_LONG_ENABLE=1`. The White Panda has a separate hard-off command gate and
watchdogs. All three hard-off layers must be changed independently before
actuation is possible.

This host commit defines the matching Python safety-parameter flag but does
not replace or embed a Panda firmware image. The comma Panda policy remains a
separate local source branch, and no firmware artifact has been installed or
flashed.

Before actuation can be considered, the following must still be completed:

- design and execute an isolated, disconnected CAN bench test with the host
  transport gate enabled but all three actuation gates still off;
- verify the comma Panda and White Panda independently reject every malformed,
  stale, out-of-order, over-rate, or enabled command on real hardware;
- define a hardware power-cut and watchdog test procedure;
- only after review, design closed-course tests beginning with propulsion
  disabled.

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
and the tail is a non-blocking data caveat.

The independent comma 3X Panda policy, White Panda guard, and host transport
plumbing are now present in separate local branches. The committed host
transport and every actuation gate remain hard-off. This source is suitable
only for offline review and planning a disconnected bench test; it is not a
vehicle-test build.

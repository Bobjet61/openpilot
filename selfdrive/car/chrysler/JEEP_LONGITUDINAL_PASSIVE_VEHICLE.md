# Jeep longitudinal passive vehicle log

## Purpose

This procedure collects candidate longitudinal data during an ordinary drive
while the Jeep's stock ACC remains in control. It is not a longitudinal
actuation test.

The `b8l` host branch is based on `b7bm2`, so it retains the existing lateral
changes. It computes, rate-limits, packs, and logs candidate Jeep longitudinal
messages in memory. The committed branch does not append those private frames
to `can_sends`.

## Required hard-off state

Before any installation is considered, all of the following must be verified
from the exact source revision:

- host `JEEP_LONG_SHADOW_TRANSPORT_COMPILED = False`;
- host `JEEP_LONG_ACTUATION_COMPILED = False`;
- `experimentalLongitudinalAvailable = False`;
- `openpilotLongitudinalControl = False`;
- comma Panda longitudinal actuation remains compiled off;
- White Panda longitudinal actuation remains compiled off.

Do not install either experimental Panda firmware branch for this passive log.
Do not alter the White Panda, Chrysler Advanced adapter, star board, blue-marked
connector, or any vehicle wiring.

## Expected vehicle behavior

- Stock ACC remains responsible for propulsion and braking.
- Experimental Mode remains unavailable for this Jeep.
- Existing lateral steering behavior remains unchanged from `b7bm2`.
- Private addresses `0x1F6`, `0x1F7`, and `0x272` never appear on CAN.
- A `Jeep long shadow` diagnostic is recorded approximately once per second
  when the Chrysler Advanced White Panda flag is present.
- Every diagnostic reports `transport=False` and `host_enabled=False`.

## Passive collection

Use an ordinary familiar route and normal stock ACC operation. No staged
hazard, abrupt cut-in, close following, emergency braking, pedal conflict,
stop/go experiment, or unusual maneuver is requested for this log.

The driver remains responsible for the vehicle and should use the system only
as they normally do. End the drive normally, then download the rlog.

Stop using the branch and return to the known-good `b7bm2` branch if any of the
following occurs:

- Experimental Mode or openpilot longitudinal control becomes available;
- stock ACC, AEB, braking, throttle, or steering behavior changes;
- a new vehicle, Panda, CAN, or controls fault appears;
- the comma device repeatedly reboots or fails to start;
- any shadow diagnostic reports `transport=True` or `host_enabled=True`.

## Post-drive acceptance

Analyze the rlog before making another change:

1. Confirm private addresses `0x1F6`, `0x1F7`, and `0x272` are absent from every
   recorded CAN bus.
2. Confirm every shadow diagnostic reports both hard-off fields false.
3. Compare requested and rate-limited acceleration with stock `DAS_3`
   propulsion/braking fields.
4. Confirm driver pedals, stock AEB, invalid state, and disengagement make the
   shadow envelope ineligible and return its limited acceleration to zero.
5. Record any missing diagnostics, logging gaps, CAN faults, or behavioral
   change as a failed passive test.

Passing this procedure supports only the shadow calculation and logging path.
It does not authorize enabling transport, flashing experimental Panda firmware,
or commanding gas or brakes in the vehicle.

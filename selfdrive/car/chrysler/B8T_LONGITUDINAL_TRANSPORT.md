# b8t longitudinal transport test

`b8t` is based on `b8r`. It retains Jeep steering response rate 4, the
261-count steering ceiling, and the existing lateral tune.

This branch does not give openpilot control of the throttle or brakes.
Experimental longitudinal mode and openpilot longitudinal control remain
unavailable.

## What is transmitted

When the passive longitudinal planner is eligible, the host sends the three
private White Panda protocol frames at 50 Hz with a shared rolling counter and
FCA checksum:

- `0x1F6`: no stop/go request, inactive `+4.0 m/s^2` brake sentinel,
  command type 0, and brake preparation off;
- `0x1F7`: `OP_LONG_ENABLE=0`;
- `0x272`: engine request off and encoded zero torque.

The live candidate acceleration remains in memory and in the existing
diagnostic log. It is not placed in the transmitted transport frames.

The embedded comma Panda selects safety parameter 12: Jeep steering rate 4
plus Jeep longitudinal shadow transport. Its firmware hard-rejects
`OP_LONG_ENABLE=1`, requires the frames in brake/dashboard/torque order,
validates their shared counter and checksum, and checks fresh vehicle speed,
stock ACC, brake, and accelerator inputs.

The installed White Panda firmware absorbs private frames `0x1F6`, `0x1F7`,
and `0x272` from its comma-facing bus instead of forwarding them unchanged.
Because the dashboard frame is always disabled, its longitudinal rewrite
condition remains false and the factory ACC frames pass through unchanged.

## First test

1. Install `b8t` normally on the comma 3X.
2. Before driving, confirm Experimental Mode is still unavailable.
3. On a familiar low-traffic road, engage stock ACC for 30 to 60 seconds at a
   steady speed.
4. Confirm the Jeep accelerates and brakes exactly as stock ACC normally does.
5. Disengage with the brake pedal and confirm normal immediate cancellation.
6. Upload the route.

Stop the test and return to `b8r` if Experimental Mode appears, stock ACC
behavior changes, a brake/throttle/CAN fault appears, or pedal cancellation
does not behave normally.

The uploaded log should show branch `b8t`, safety parameter 12,
`openpilotLongitudinalControl=false`, and ordered neutral `0x1F6`, `0x1F7`,
`0x272` sendcan cycles. That proves the host and embedded Panda transport path.
It does not authorize active longitudinal control.

For direct confirmation at the White Panda, connect the laptop to its USB port
and run the workspace `monitor_b8t_transport.py`. The monitor only opens the
USB receive queue. It does not set a safety mode, transmit CAN, reset, or flash
the Panda. A pass requires complete brake/dashboard/torque cycles with valid
checksums, aligned counters, inactive braking, zero engine request, and
`OP_LONG_ENABLE=0`.

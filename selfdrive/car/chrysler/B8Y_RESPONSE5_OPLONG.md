# b8y response 5 and moving-only openpilot longitudinal

## Scope

`b8y` combines two separately bounded changes for the supported Jeep Grand
Cherokee with the Chrysler Advanced White Panda installation:

- Jeep steering response 5; and
- moving-only openpilot longitudinal control.

Steering response 5 changes only the per-command rise/fall limit from 4 to 5.
Maximum commanded steering torque remains 261, maximum real-time delta remains
112 per 250 ms, and maximum measured-torque error remains 80.

The initial longitudinal scope excludes standstill, stop, go, brake
preparation, automatic resume, and brake hold. The host will not request
longitudinal control below 2.1 m/s. The external White Panda independently
requires wheel-speed raw 29 or greater (approximately 2.06 m/s).

## Host command envelope

- acceleration range: -3.0 to +1.0 m/s^2;
- acceleration jerk limit: +1.0 m/s^3;
- deceleration jerk limit: -2.0 m/s^3;
- propulsion/braking deadband: 0.05 m/s^2;
- engine-torque request ceiling: 100 Nm;
- private command cycle: at most 50 Hz with an 18 ms host rate floor; and
- mutually exclusive propulsion and braking.

The host uses the production `CarControl.actuators.accel` result. A command is
packed only when controls and longitudinal control are active, the vehicle is
in a forward gear, stock ACC is available and active, there is no ACC fault,
pedal override, or stock AEB, the vehicle is above the moving-only threshold,
and the longitudinal plan is current and valid.

## Embedded comma Panda policy

The b8y safety parameter explicitly selects:

- Jeep response 5 (`32`);
- private longitudinal transport (`4`);
- rejection diagnostics (`16`); and
- longitudinal actuation (`64`).

The actuation bit is ineffective without the private-transport bit. Conflicting
response-4 and response-5 bits fall back to the stock response-3 steering
envelope.

Every private longitudinal cycle must arrive on bus 0 in brake/dashboard/torque
order with aligned consecutive counters, valid FCA checksums, at least 15 ms
between accepted cycles, exact reserved bits, bounded commands, and fresh
vehicle speed, gas, brake, and stock ACC sources. Controls, longitudinal
controls, ACC main, and vehicle motion must remain active, and gas, brake, or a
stock collision indication aborts the cycle. Source state is checked again at
the dashboard and torque stages so a mid-cycle change fails closed.

## External White Panda policy

The matched `wp-b8y-oplong` firmware independently requires:

- fresh private brake, dashboard, and torque commands;
- valid FCA checksums and consecutive aligned counters;
- fresh wheel speed, gas pedal, brake pedal, and stock ACC sources;
- host enable, ACC available/enabled, and speed above the moving-only floor;
- no driver gas, driver brake, or stock collision indication;
- no stop, go, or brake-preparation request;
- brake raw 2661 through 3275 or the exact inactive raw value 4094;
- engine torque raw 2000 through 2400; and
- no simultaneous propulsion and braking.

Only while every guard is true does the White Panda replace the propulsion and
deceleration fields in the factory DAS_3 frame. Fault, collision, counter, and
unrelated factory fields are preserved. If any guard becomes false, the White
Panda stops replacement and passes the factory frame through.

## Matched installation requirement

The host branch, embedded comma Panda firmware, and external White Panda
firmware are one matched set. Do not install only part of the set. This commit
does not flash either Panda and does not install the branch on a comma device.

## Matched source and artifacts

Embedded comma Panda source:

- branch: `panda-0971-b8y-rate5-oplong`;
- commit: `7afc0fe0ed17c18574e13672c256856fe034a4f9`;
- F4 signed firmware SHA-256:
  `b21f45a67b262bcaff3ad31fe0e222881bc3ca27b23d793ba225ce916a360a9c`;
- H7 signed firmware SHA-256:
  `9a32d07b686594035e9461856d88065e0d56144ee6d4808fd432c6c32b5cb477`; and
- embedded version: `DEV-7afc0fe0-DEBUG`.

External White Panda source:

- branch: `wp-b8y-oplong`;
- commit: `5e755d76de5b625986addc066bfbab6f9fcce1a6`;
- signed firmware SHA-256:
  `3f8509549e3626782b910ef4b75b1ad65d9c89afc4a73fd825224c88d772880d`;
  and
- embedded version: `v1.7.5-DEV-5e755d76-DEBUG`.

## Validation

- 51 Jeep-specific host tests passed, including production controller behavior,
  active/private-frame gating, longitudinal-plan validity, and real DBC byte
  packing.
- 26 focused comma Panda tests passed, including response-5 bounds, conflicting
  response flags, source freshness, command ordering, checksum/counter
  integrity, actuation-flag composition, and mid-cycle pedal interruption.
- The White Panda guard unit was compiled with `-Wall -Wextra -Werror` and
  passed its freshness, counter, command-envelope, pedal, collision, and exact
  moving-speed boundary assertions.
- Both comma Panda targets and the White Panda target compiled and signed from
  the source commits listed above.

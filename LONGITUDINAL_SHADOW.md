# Chrysler guarded longitudinal control

## wp-b6k recovery configuration

`wp-b6k` compiles `CHRYSLER_LONG_ACTUATION` to zero. The existing CAN2
isolation/routing and guarded LKAS steering path remain in place, but no
factory `DAS_3` longitudinal frame can be substituted. Factory ACC therefore
retains propulsion and braking ownership while the recovery build is active.

`wp-b6j` is the White Panda half of the Jeep moving-only longitudinal
controller. It is derived from the Chrysler Advanced firmware and assumes the
vehicle's factory `DAS_3` source is isolated on physical White Panda CAN2
(firmware bus 1). Do not use this firmware on a different wiring topology.

In the prior `wp-b6j` build, actuation was compiled on, but every forwarded
command remained behind the independent White Panda guard:

- freshness watchdogs for private `0x1F6`, `0x1F7`, and `0x272` commands,
  wheel speed, both driver pedals, and factory ACC/AEB state;
- driver brake and gas cancellation plus stock collision/AEB pass-through;
- a minimum moving speed of approximately 2.06 m/s;
- stop, go, standstill, brake preparation, and brake hold rejected;
- mutually exclusive braking and propulsion;
- a calibrated -3.0 m/s² braking ceiling and the existing 100 Nm `DAS_3`
  engine-torque ceiling;
- one shared rolling 4-bit counter and FCA checksum on the complete private
  command set, with incomplete, stale, corrupt, duplicate, or mismatched
  cycles rejected;
- factory `DAS_4`, `DAS_5`, and steering-wheel button traffic passed through
  unchanged; and
- factory `DAS_3` fault, collision, counter, and unrelated bits preserved.

## b6h diagnostics and transport

The host sends a complete command snapshot at a nominal 25 Hz. The White Panda
holds the last complete valid snapshot while processing the factory 50 Hz
`DAS_3` stream. This removes the short-interval cycles observed with the b6g
20 ms gate while remaining comfortably inside the 100 ms freshness watchdog.

The former direct-mailbox `0x4FF` beacon was emitted for almost every CAN1
receive interrupt. b6h queues it once per factory `DAS_3` frame instead. It
also queues diagnostic-only `0x4FE`, whose payload contains:

1. signature `0xC1` and layout version 1;
2. the exact factory `DAS_3` engine-command bytes before substitution;
3. the exact engine-command bytes forwarded to the vehicle; and
4. the exact factory ACC acceleration/availability bytes.

Neither diagnostic frame participates in a guard decision or carries an
actuation command.

## b6i watchdog correction

On-road b6h telemetry confirmed that the private command set arrived cleanly
at a nominal 25 Hz, but also exposed a legacy count-difference watchdog. It
compared the 50 Hz stock `0x11C` frame count with the independent 25 Hz private
`0x1F6` frame count, so the difference necessarily exceeded its threshold and
invalidated a healthy committed command approximately once every 1.04 seconds.

b6i removes only that mismatched-rate count comparison. The independent 100 ms
monotonic freshness watchdogs, shared rolling counter, checksums, atomic-cycle
commit, command envelope, pedal cancellation, speed floor, collision/AEB
pass-through, and factory fault preservation remain unchanged.

## b6j factory-brake arbitration

Two b6i drives produced repeatable factory ACC faults after the driver lowered
the selected speed. In each case, factory `DAS_3` requested normal braking for
about 1.1 to 1.3 seconds while the host was still requesting propulsion. The
private command replaced the factory brake request until the factory ACC
supervisor faulted.

b6j gives a current-frame factory braking request priority: whenever a valid
stock `DAS_3` has command type 1, that complete frame is forwarded unchanged.
Openpilot substitution resumes only after factory `DAS_3` stops requesting
braking and every existing White Panda guard permits it. Collision/AEB
pass-through, driver-pedal cancellation, the speed floor, freshness and
integrity checks, steering behavior, and the 100 Nm ceiling are unchanged.

This is deliberately hybrid longitudinal control. It is intended to preserve
factory-supervised braking while openpilot supplies permitted propulsion; it
does not provide full openpilot-controlled braking.

## b6i local validation on 2026-07-31

- the standalone guard suite passed with `-Wall -Wextra -Werror`;
- the complete ARM firmware compiled and linked with warnings treated as
  errors;
- the debug image signed successfully at 46,712 bytes, below the 49,152-byte
  firmware limit; and
- no b6i firmware was flashed during this validation.

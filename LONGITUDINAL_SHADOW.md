# Chrysler guarded longitudinal control

`wp-b6h` is the White Panda half of the Jeep moving-only longitudinal
controller. It is derived from the Chrysler Advanced firmware and assumes the
vehicle's factory `DAS_3` source is isolated on physical White Panda CAN2
(firmware bus 1). Do not use this firmware on a different wiring topology.

Actuation is compiled on, but every forwarded command remains behind the
independent White Panda guard:

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

## Local validation on 2026-07-31

- the standalone guard suite passed with `-Wall -Wextra -Werror`;
- the complete ARM firmware compiled and linked with warnings treated as
  errors;
- the debug image signed successfully at 46,776 bytes, below the 49,152-byte
  firmware limit; and
- no b6h firmware was flashed and no b6h host build was installed during this
  validation.

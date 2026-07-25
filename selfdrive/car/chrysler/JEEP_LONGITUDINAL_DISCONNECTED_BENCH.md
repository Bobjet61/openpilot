# Jeep longitudinal disconnected bench plan

## Status and scope

This document is a design and review checklist only. It does not authorize a
vehicle connection, firmware flash, or longitudinal actuation.

The first bench must be physically incapable of moving any vehicle component:

- no connection to the Jeep, its OBD port, star board, EPS branch, brake
  module, powertrain CAN, or any other vehicle wiring;
- no EPS, brake, throttle, engine, motor, pump, relay, or other actuator on any
  bench CAN segment;
- host shadow transport off in the committed vehicle branch;
- host longitudinal actuation off;
- comma 3X Panda longitudinal actuation off;
- White Panda longitudinal actuation off.

The blue-marked star-board connector in the installation photograph is a
vehicle EPS connection. It is prohibited on this bench. The connector's
position on the star board is not a pinout and must never be used to infer one.

Prefer a spare White Panda and spare harness. If the installed White Panda
must be used, it must first be completely removed from the vehicle harness and
the Jeep must be returned to its known-good stock star-board connection by a
qualified installer. Merely unplugging the blue-marked connector from one side
of the White Panda is not sufficient isolation. The comma 3X must likewise be
removed from every vehicle cable before bench power is applied.

## Exact source revisions

The reviewed sources are separate local branches:

- host: `b8l` at `d2ee7baa775f53029ef0be9293577b94c7adf39b`;
- comma Panda: `panda-0971-long-shadow` at
  `2816927555f53efef0905e686deba5adaa6cb001`;
- White Panda: `wp-chrysler-long-shadow` at
  `72505583b8`.

No reviewed host commit embeds either experimental Panda firmware image.
Building, signing, and flashing bench firmware are separate future steps.

## Blocking inventory

Do not make or power a bench cable until every field below is recorded:

- comma device model and serial: comma 3X / `____________`;
- comma harness-box part or revision: `____________`;
- comma OBD-C or bench-cable part/revision: `____________`;
- White Panda hardware revision and serial: `____________`;
- White Panda Chrysler adapter/harness revision: `____________`;
- available CAN analyzer/generator and channel count: `____________`;
- bench supply model, voltage range, current limit, and output fuse:
  `____________`;
- emergency-disconnect hardware: `____________`;
- USB power source and how its VBUS is cut by the same disconnect:
  `____________`;
- measured pin-to-pin continuity table for every proposed breakout:
  `____________`.

Photograph both ends and labels of every harness before recording continuity.
Do not rely on wire color, connector orientation, an Internet diagram, or the
slot occupied on the vehicle star board.

## Required equipment

- a current-limited bench supply suitable for the identified comma harness;
- a correctly rated inline fuse close to the supply;
- one latching, normally-open emergency disconnect that removes every DC and
  USB power source from the devices under test;
- fused breakout hardware with individually labeled CAN high, CAN low, ground,
  and power conductors;
- at least two active CAN generator channels and three capture channels, or an
  equivalent arrangement that records all three isolated segments at once;
- a digital multimeter for polarity, continuity, shorts, and unpowered CAN
  termination checks;
- two 120-ohm terminators per high-speed CAN segment unless measurements show
  that equivalent termination is already present;
- short twisted CAN pairs and a common signal reference appropriate for the
  selected isolated interfaces;
- optional oscilloscope or differential probe for physical-layer review.

Do not connect a laptop, analyzer, or USB hub that can keep either Panda
partially powered after the emergency disconnect opens. If USB isolation is
not available, place the entire powered USB hub behind the same disconnect and
verify that all DUT rails fall to zero.

## Logical topology

Use three small, independent 500 kbit/s classical-CAN segments. The White
Panda source initializes its three buses at 500 kbit/s; the exact mapping must
still be verified on the actual breakout before connection.

### Segment A: simulated vehicle/host side

Connect only:

- comma 3X Panda bus 0;
- White Panda bus 0;
- simulator/generator channel A;
- capture channel A.

This segment carries simulated Jeep source traffic and, in a future
transport-only build, private frames `0x1F6`, `0x1F7`, and `0x272`.

### Segment B: simulated stock-ACC side

Connect only:

- White Panda bus 1;
- simulator/generator channel B;
- capture channel B.

This segment supplies a synthetic stock `0x1F4` DAS_3 stream and any other
explicitly reviewed stock-side inputs. There is no real ACC module.

### Segment C: empty EPS side

Connect only:

- White Panda bus 2;
- capture channel C configured as an active receiver that acknowledges valid
  frames but transmits no application traffic.

No generator or actuator is required initially. This segment exists only to
prove what the White Panda would place on its EPS-facing side. It must never be
connected to the blue-marked Jeep EPS connector.

A listen-only analyzer does not acknowledge CAN frames. If capture channel C
cannot acknowledge, add a separate non-transmitting, ACK-capable CAN node so
the White Panda is not driven bus-off merely because segment C has no second
active controller.

## Connector mapping gate

The previously supplied 16-pin illustration is useful as a hypothesis, not as
authorization to wire a device. Connector views can be mirrored, and harness
boxes may remap physical pins to Panda bus numbers.

Before power is applied:

1. Isolate every cable from the Jeep and all electronics.
2. Record connector face, keying, and pin numbering from the exact hardware
   documentation.
3. Use continuity mode to map each proposed pin end-to-end.
4. Prove there is no continuity from any CAN conductor to supply positive.
5. Prove grounds and shields match the intended design.
6. Perform a receive-only loopback on one bus at a time to establish which
   physical pair is Panda bus 0, bus 1, and bus 2.
7. Label both ends permanently as A, B, or C.
8. Have a second review compare the written map with the measured cable.

Any mismatch stops the bench setup.

## Power and termination gate

Perform these checks with all devices unpowered:

1. Verify supply polarity at the disconnected DUT plug.
2. Verify the emergency disconnect opens supply positive and USB VBUS.
3. Verify there is no alternate or backfeed power path.
4. Measure CAN high to CAN low on each completed segment.
5. Adjust only removable termination until each segment measures
   approximately 60 ohms, representing two 120-ohm end terminations.
6. Verify neither CAN conductor is shorted to power or ground.
7. Start with the supply current limit at the lowest value that supports the
   identified hardware's documented startup requirement; do not guess a
   current or fuse rating.

High-speed CAN is normally terminated with 120 ohms at both physical ends.
Measure the assembled, unpowered network rather than assuming either Panda or
analyzer contains termination:

- <https://www.ni.com/en/support/documentation/supplemental/09/can-physical-layer-and-termination-guide.html>
- <https://kvaser.com/developer-blog/how-to-test-your-can-termination-works-correctly/>

## Bench configurations

### Configuration 0: present committed sources

Use only the currently committed hard-off sources. Expected behavior:

- host transport remains off;
- Panda shadow safety parameter bit 4 remains unset;
- no `0x1F6`, `0x1F7`, or `0x272` is transmitted by the comma device;
- both Panda actuation constants remain zero.

This is the first powered configuration and establishes the capture baseline.

### Configuration 1: future transport-only source

This configuration does not exist yet. It must be created in a separate branch
that changes only:

- `JEEP_LONG_SHADOW_TRANSPORT_COMPILED` from `False` to `True`.

The following must remain unchanged:

- `JEEP_LONG_ACTUATION_COMPILED = False`;
- comma Panda `CHRYSLER_JEEP_LONG_ACTUATION = 0U`;
- White Panda `CHRYSLER_LONG_ACTUATION = 0U`.

Expected behavior after valid simulated source traffic:

- comma Panda accepts only the three bounded private frames in exact
  brake-dashboard-torque order;
- all three share one rolling four-bit counter and valid FCA checksums;
- `OP_LONG_ENABLE` remains zero;
- White Panda keeps `is_oplong_enabled` false;
- stock frames entering segment B emerge unchanged on segment A;
- no actuator is present to receive any output.

## Synthetic source traffic

Start with neutral command requests and deterministic, recorded payloads.
Synthetic inputs must be generated from reviewed DBC definitions, not copied
blindly from live vehicle traffic.

The valid-case source set includes:

- segment A `0x202` wheel speed above the moving-only threshold;
- segment A `0x22F` accelerator at zero;
- segment A `0x140` brake pedal released;
- segment A `0x1F4` with stock ACC available/active, a safe command type, no
  collision/AEB indication, and a valid rolling counter/checksum;
- segment B stock `0x1F4` at its expected cadence.

Bring up one message family at a time while all private commands remain
neutral. Record the exact payload, cadence, counter, and checksum generator for
each family.

That list is the safety-critical minimum, not a complete openpilot vehicle
simulation. Running the full comma host also requires the fingerprint,
CarState, heartbeat, and engagement traffic expected by the selected branch.
Create that larger simulator manifest from the reviewed local rlogs, document
every included address, and keep the five safety-critical source families
above independently switchable for fault injection.

## Acceptance tests

Every test requires synchronized captures from segments A, B, and C, device
health telemetry, supply voltage/current, and a written expected result.

### Baseline and valid transport

1. Cold boot: no unexpected DUT transmission before explicit startup.
2. Committed source: zero private frames from comma.
3. Transport-only source: private trio appears at 50 Hz only after all source
   eligibility conditions are valid.
4. Verify exact order `0x1F6` -> `0x1F7` -> `0x272`.
5. Verify one shared counter, including `15` -> `0` wrap.
6. Independently recompute all three FCA checksums from captured bytes.
7. Verify dashboard `OP_LONG_ENABLE` is zero in every capture.
8. Compare every stock frame entering segment B with the corresponding frame
   leaving on segment A; with White Panda actuation hard-off, no controlled
   longitudinal field may change.

### comma Panda rejection

Request each fault through the comma device's normal Panda transmit path so
the frame cannot bypass its safety hook:

- wrong bus, address, or payload length;
- wrong frame order;
- duplicate, skipped, or cross-frame-mismatched counter;
- corrupt checksum or unused payload bit;
- cycle interval below 15 ms;
- stale speed, gas, brake, or stock-ACC source;
- stopped vehicle, driver gas, driver brake, or collision/AEB;
- stop, go, brake-prep, or invalid command type;
- braking and propulsion in the same cycle;
- deceleration or torque outside the reviewed envelope;
- `OP_LONG_ENABLE=1`.

Pass condition: the disallowed frame is absent from segment A, and no later
frame in that partial cycle leaks through.

### White Panda rejection

Inject the same malformed private cycles directly on segment A so they reach
the White Panda independently of comma Panda. Also remove or corrupt each
White Panda source input individually.

Pass condition:

- White Panda never sets `is_oplong_enabled`;
- stock segment-B traffic is passed through without controlled longitudinal
  modification;
- stale or invalid state returns to fail-silent within the source's defined
  watchdog interval;
- collision/AEB stock traffic is never suppressed or replaced.

### Power, heartbeat, and wiring faults

- open the emergency disconnect during neutral transport;
- disconnect host heartbeat or communication;
- remove segment A, B, and C one at a time;
- remove one termination at a time;
- reboot each device independently;
- restore power without restarting the simulator;
- unplug each source generator channel.

Pass condition:

- no private actuation-enable bit appears;
- outputs cease or return to stock pass-through as specified;
- power removal leaves all DUT rails unpowered with no USB backfeed;
- every restart begins fail-silent and requires an explicit test start.

Do not deliberately short CAN high, CAN low, power, or ground. Electrical
fault insertion beyond open-circuit and removable termination requires a
purpose-built, current-limited fault box and a separate reviewed procedure.

## Stop conditions

Open the emergency disconnect and stop testing immediately if any of the
following occurs:

- any physical connection to vehicle wiring is discovered;
- supply polarity, connector mapping, or termination is uncertain;
- unexpected current rise, heat, odor, smoke, or repeated device reset;
- `OP_LONG_ENABLE=1` appears anywhere;
- a controlled stock longitudinal field changes while a Panda actuation gate
  is off;
- a malformed, stale, out-of-order, over-rate, or out-of-range command passes;
- collision/AEB traffic is blocked or altered;
- a DUT remains powered after the emergency disconnect opens;
- captures cannot prove which node transmitted a frame.

After a stop condition, preserve captures and power logs. Do not retry by
loosening a safety check.

## Required evidence

The bench stage is complete only when the review folder contains:

- photographs of the isolated setup showing no vehicle connection;
- the measured continuity and connector map;
- unpowered resistance and short-check measurements for A, B, and C;
- supply, fuse, current-limit, and emergency-disconnect details;
- exact source commits and firmware hashes;
- raw timestamped captures from all three segments;
- independent checksum/counter verification output;
- a pass/fail table for every test above;
- power-cut and reboot timing;
- a written record of every anomaly and its disposition.

## Next decision

Only after this plan is reviewed and the blocking inventory is complete should
a separate three-character bench branch be created. That branch may enable
host shadow transport only. It must not enable host, comma Panda, or White
Panda longitudinal actuation, and it must not be installed in a vehicle.

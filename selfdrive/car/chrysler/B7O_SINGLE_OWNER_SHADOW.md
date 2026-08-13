# b7o single-owner OP Long shadow

`b7o` is a development-only branch for replacing the mixed factory-ACC bridge
with an exclusive longitudinal owner. Vehicle actuation is compiled out.

The first milestone answers three questions from recorded drives:

1. Can an openpilot owner be armed only after the stock owner is inactive?
2. Does every pedal, gear, restraint, collision, and fault input revoke output
   immediately?
3. How often would the shadow owner request braking and propulsion without ever
   transmitting those requests?

`jeep_single_owner_shadow.py` is a pure state machine. Its result always has
`actuation_allowed=False`. `tools/b7o_owner_replay.py` audits qlogs/rlogs and
reports ownership transitions, reasons, and hypothetical command counts while
emitting zero CAN frames.

This branch must not be installed for driving. A later activation commit will
require replay acceptance criteria, explicit counter/timing verification, and
separate review of both Panda safety layers.

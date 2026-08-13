# b7o single-owner OP Long shadow

`b7o` is a development-only branch for replacing the mixed factory-ACC bridge
with an exclusive longitudinal owner. Vehicle actuation is compiled out.

The first milestone answers three questions from recorded drives:

1. Can an openpilot owner be armed only after an explicit, healthy stock-command
   isolation boundary is present and the isolated command path is quiet?
2. Does every pedal, gear, restraint, collision, and fault input revoke output
   immediately?
3. How often would the shadow owner request braking and propulsion without ever
   transmitting those requests?

`jeep_single_owner_shadow.py` is a pure state machine. Its result always has
`actuation_allowed=False`. `tools/b7o_owner_replay.py` audits qlogs/rlogs and
reports ownership transitions, reasons, and hypothetical command counts while
emitting zero CAN frames.

`cruiseState.enabled` is deliberately not treated as proof of longitudinal
ownership. It describes cruise engagement, not whether factory actuator commands
can reach the vehicle. Normal replay therefore remains blocked with
`stock_not_isolated`. `--assume-stock-isolated` runs a clearly labeled
counterfactual topology to exercise the handoff logic and candidate commands;
it does not prove that the physical vehicle has that topology.

Before any activation work, recorded hardware diagnostics must independently
prove both the isolation state and absence of stock actuator commands. A stock
command reappearing after handoff revokes shadow output immediately and requires
an explicit controls reset before a complete new handoff interval. Handoff
confirmation uses monotonic elapsed time rather than sample counts, so replay
and future production-loop rates cannot silently change the safety interval.

This branch must not be installed for driving. A later activation commit will
require replay acceptance criteria, explicit counter/timing verification, and
separate review of both Panda safety layers.

# Chrysler longitudinal safety shadow

Base: exact installed White Panda firmware commit
`b11da4d31097e5d210f94f5b0fe76cb9726ef2b5`.

Actuation is hard-coded off. The branch adds an independent guard around the
recovered Chrysler Advanced longitudinal protocol:

- freshness watchdogs for `0x1F6`, `0x1F7`, `0x272`, real speed, both
  driver pedals, and the factory ACC/AEB state;
- driver brake and gas cancellation;
- stock collision/AEB pass-through;
- command-type validation;
- calibrated -3.0 m/s^2 braking and 100 Nm DAS_3 engine-torque ceilings;
- `0x22F` ECM_5 accelerator, `0x140` brake-pedal, and `0x202` wheel-speed
  sources selected from 130.2 minutes of stock Jeep logs;
- stop/go, standstill, brake-prep, and brake-hold requests rejected;
- mutually exclusive braking and propulsion;
- an FCA checksum plus one shared rolling 4-bit counter on all three private
  frames, with duplicate, skipped, corrupt, and cross-frame-mismatched cycles
  rejected;
- factory `0x1F7` DAS_4 dashboard data always passed through unchanged.

This branch is for source review, replay, and bench testing only. Do not flash
it to the vehicle.

Validation on 2026-07-25:

- the standalone guard test passed with `-Wall -Wextra -Werror`;
- the complete ARM firmware compiled, linked, and signed with warnings treated
  as errors;
- the resulting temporary image was 45,100 bytes, below the 49,152-byte limit;
- no firmware artifact was retained or flashed.
- offline replay opened all 131 downloaded rlogs and accepted all 284,095
  eligible shadow cycles with zero envelope or hard-off violations;
- all 5,619 sampled instances of each fault class were rejected, including
  duplicate counters and payload corruption;
- the short end-of-drive segment contains 51,085 readable events followed by a
  corrupt/truncated final event; its readable portion was replayed and the tail
  is retained as a non-blocking data caveat.

The stock logs also showed that this EcoDiesel requests propulsion through
`0x1F4` DAS_3 engine torque, not `0x271` DAS_5 wheel torque. The shadow rewrite
now leaves DAS_5 unchanged and places the bounded private torque request in
DAS_3. Before actuation can be considered, the comma 3X Panda still needs an
independent transmit safety policy, and recorded-drive replay plus an isolated
bench test must pass.

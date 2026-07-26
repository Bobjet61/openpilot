# b8w neutral Panda counter recovery

`b8w` keeps the complete `b8v` host behavior:

- steering and rate-4 limits are unchanged;
- private Jeep longitudinal payloads remain strictly neutral;
- the 18 ms host scheduler is unchanged;
- rejection diagnostics remain enabled;
- `JEEP_LONG_ACTUATION_COMPILED = False`;
- `openpilotLongitudinalControl = False`;
- `experimentalLongitudinalAvailable = False`.

The embedded Panda firmware is built from Panda source commit
`429b5d10dd` on branch `panda-0971-b8wrecover`.

The only behavioral change is inside the embedded Panda's rejected-frame
state. A well-formed, correctly ordered private brake frame arriving below the
15 ms rate floor is still blocked and never transmitted. The Panda remembers
that valid input counter without changing the timestamp of the last accepted
cycle. The next correctly ordered cycle may therefore recover immediately once
the accepted-frame spacing reaches 15 ms, instead of suffering the b8v 100 ms
counter-mismatch cascade.

Malformed, out-of-order, stale, state-ineligible, non-neutral, dashboard-only,
and torque-only frames remain blocked and cannot use this recovery path.


# b8x legacy brake-hold removal

`b8x` starts from `b8w` commit
`9d0bd7a0a20dd447e356acbf532a5e3e5c78e35a`.

The post-White-Panda validation routes showed that the old Jeep brake-hold
routine still tried to transmit `DAS_3` braking commands after stock ACC
became inactive at standstill. The internal Panda rejected every observed
attempt. That fail-closed result prevented vehicle actuation, but the old path
must not coexist with the staged private White Panda protocol.

This branch removes:

- the legacy `brake_hold()` state machine;
- its `DAS_3` command construction and transmission;
- its periodic automatic RESUME injections;
- the associated Chrysler state mirrors;
- the cruise-mismatch exception that depended on synthetic brake-hold state.

The ordinary controller path for explicit `CC.cruiseControl.cancel` and
`CC.cruiseControl.resume` requests remains unchanged.

All b8w safety boundaries remain in force:

- private Jeep transport frames are neutral;
- `JEEP_LONG_ACTUATION_COMPILED = False`;
- `openpilotLongitudinalControl = False`;
- `experimentalLongitudinalAvailable = False`;
- steering limits and the b8w internal Panda firmware are unchanged;
- the external White Panda longitudinal switch remains compile-time hard-off.

`b8x` is a cleanup and verification branch. It does not authorize active
brake or propulsion testing.

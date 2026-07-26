# b8v neutral Panda rejection diagnostics

`b8v` retains the complete `b8u` driving behavior:

- longitudinal actuation is compile-time disabled;
- every private `0x1F6`, `0x1F7`, and `0x272` frame is neutral;
- the transport is suppressed at standstill;
- the Jeep steering rate-4 envelope and lateral tune are unchanged.

The only functional addition is an opt-in Panda diagnostic flag (Chrysler
safety parameter bit 16). When the Panda rejects one of the private frames,
it replaces bytes 0 through 5 of the USB rejection receipt with:

| Byte | Meaning |
|---|---|
| 0 | Marker `0xD7` |
| 1 | Rejection reason |
| 2-5 | Little-endian detail value |
| 6-7 | Original counter/checksum bytes |

The payload is changed only after the final safety decision is rejection.
`can_send()` marks that packet `rejected=1` and returns it to the host; the
packet is never queued to a physical CAN controller. Accepted frames are not
modified.

Important reason values include:

- `5`: Panda's internal `controls_allowed_long` is false;
- `11-18`: a required source is missing or stale;
- `22`: the actual Panda interval was below 15 ms (detail is microseconds);
- `23`: counter mismatch (detail bytes encode expected and actual);
- `24-32`: dashboard/torque stage, counter, checksum, payload, or conflict
  rejection.

The branch remains diagnostic-only. It must not be used as evidence to enable
brake or throttle actuation until a road log identifies and resolves the first
rejection in each cascade.

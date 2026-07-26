# b8v Jeep neutral-transport rejection diagnostics

This Panda branch adds diagnosis to the existing hard-off Jeep longitudinal
transport policy. It does not enable longitudinal actuation, change any
accepted command limit, or relax any rejection condition.

The diagnostic behavior requires Chrysler safety parameter bit 16 in addition
to the existing longitudinal-shadow bit 4. On a rejected private `0x1F6`,
`0x1F7`, or `0x272` host packet, `safety_tx_hook()` writes a diagnostic marker
to bytes 0 through 5:

- byte 0: `0xD7`;
- byte 1: exact rejection reason;
- bytes 2-5: little-endian detail (such as Panda-measured interval);
- bytes 6-7: unchanged original counter and FCA checksum.

The mutation is performed only after the final transmit decision is false.
`can_send()` returns the packet to the host with its rejected flag set and
does not place it in a CAN transmit queue. Accepted packets remain unchanged.

Reasons distinguish the hidden `controls_allowed_long` state, all four source
presence/freshness checks, stopped/pedal/collision states, checksum and
payload failures, Panda-clock interval, counter mismatch, and the downstream
dashboard/torque stage checks.

`CHRYSLER_JEEP_LONG_ACTUATION` remains hard-coded to zero. The neutral payload
tests, diagnostic opt-in tests, rate-4 boundary tests, and F4/H7 firmware
builds must pass before packaging this firmware.

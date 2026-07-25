# Jeep longitudinal comma Panda shadow policy

Base: the exact source revision embedded in the currently installed comma 3X
Panda firmware, `a314899687917f7ab66ebe8b06ff0c19fed83d0e`.

This branch adds an independent transmit policy for the three private
host-to-White-Panda frames on bus 0:

- `0x1F6` bounded braking command;
- `0x1F7` hard-off request, counter, and checksum;
- `0x272` bounded engine-torque command.

The policy is selected only with Chrysler safety parameter bit 4. It requires
an exact three-frame order, a shared rolling four-bit counter, FCA checksums,
an 8-byte payload, source-message freshness, moving vehicle, stock ACC active,
no driver pedal, and no stock collision/AEB indication. It rejects stop/go,
brake preparation, standstill, simultaneous braking and propulsion, commands
outside -3.0 m/s² to 0 m/s² or 0 Nm to 100 Nm, and cycles faster than 15 ms.

`CHRYSLER_JEEP_LONG_ACTUATION` is hard-coded to zero and cannot be supplied by
a compiler flag. Consequently, `OP_LONG_ENABLE=1` is always rejected.

This source is for review, offline tests, and a disconnected CAN bench only.
Do not flash it to the comma 3X or use it in a vehicle.

Validation on 2026-07-25:

- all 9 focused hard-off longitudinal safety tests passed;
- the cross-safety-mode transmit regression also passed;
- both F4 and H7 firmware targets compiled and linked with warnings treated as
  errors;
- the temporary H7 payload was 70,696 bytes and its signed image was
  70,832 bytes;
- the exact unmodified base and this branch both have the same 6 failures in
  the older custom Chrysler test class, with 25 passes and 3 skips; these are
  pre-existing brake-hold/test-expectation mismatches, not regressions from
  this policy;
- no firmware artifact was retained, installed, or flashed.

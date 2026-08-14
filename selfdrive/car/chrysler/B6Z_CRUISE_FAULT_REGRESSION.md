# b6z cruise-fault regression candidate

## Vehicle recovery state

The vehicle remains on the known-good `b6m` host commit
`8df1a0e9c6c51338ff9e9764e21d16bac97b60b8` with the exact `wp-b6k`
external White Panda artifact. This b6z candidate has not been installed or
flashed.

## Fault evidence

The fresh b6x fault in route `00000033--9b2d23dc08`, segment 22, asserted
`DAS_4.ACC_FAULTED` at route time 1332.553 s. Before that fault:

- active radar assist briefly replaced the planner's `+2.0 m/s^2` request with
  `-0.15 m/s^2` (`matched_lead_brake`), then released it back to acceleration;
- the transmitted engine-torque command subsequently climbed to 440 Nm;
- the positive transport step reached 14.5 Nm per transmitted command; and
- the factory captures show that 440 Nm is not the Jeep's real uphill limit:
  factory operation reached approximately 503 Nm median, 535.5 Nm in one
  uphill capture, and 573.5 Nm in the longer capture.

The same 440 Nm command was used extensively without a fault in the b6w
route-31 capture (p95, p99, and maximum were all 440 Nm). Therefore 440 Nm by
itself is not identified as the fault trigger. The new radar-induced reversal
and the more aggressive rise are the important regressions.

## b6z changes

- Radar remains observable for diagnostics, but cannot replace the openpilot
  longitudinal request in `CarController`.
- The b6w positive-acceleration gain of 16.5 and rise rate of 300 Nm/s are
  restored. Offline replay reduces the largest positive transmitted step from
  14.5 Nm to 12.0 Nm.
- The first regression candidate retains the b6w-proven 440 Nm absolute
  ceiling. This is a diagnostic staging ceiling, not the intended final
  uphill authority.
- Host, embedded Panda safety, and external White Panda apply the same running
  ceiling: `min(440, 250 + 20 * speed_mps)` Nm. The independent Panda guards
  reject commands above that envelope even if the host is corrupted.
- The external White Panda continues to fail closed to factory pass-through
  after a dashboard fault. No pre-fault automatic pass-through was added,
  because silently removing openpilot's authority while it believes it is
  active would create a different hazard.

## Smooth-cruise regression work

Chrysler has no upstream openpilot-long calibration, so the Jeep inherited the
generic `Kp=1`, `Ki=1`, zero-deadband feedback tune and a placeholder 0.15 s
actuator delay. That aggressive feedback sits in front of a delayed diesel
powertrain and the custom torque/brake mapper, creating a speed-control limit
cycle even when the planner itself is smooth.

Recorded-trajectory replay used eight b6w qlog segments from routes 31 and 32.
The selected Jeep-specific tune keeps planner feed-forward at 1.0 and keeps the
unproven delay assumption at 0.15 s, while changing only feedback to `Kp=0.6`,
`Ki=0.2`, and a 0.1 m/s speed deadband. Against identical recorded inputs:

- total acceleration-command variation fell from 0.3883 to 0.2918 per second
  (about 25%);
- steady-cruise command variation fell from 16.6298 to 10.5617 (about 36%);
- mapped engine/brake direction changes within three seconds fell from 3 to 2;
- every recorded large speed deficit still requested the full 2.0 m/s^2; and
- during 526 planned-deceleration samples, mean output moved from an overly
  aggressive -1.8404 m/s^2 to -1.4159 m/s^2 against a -1.5141 m/s^2 planner
  target. The lower deceleration tail improved from -3.5 to -2.5742 m/s^2.

Raw vehicle pitch also drove the grade term to its 150 Nm ceiling frequently.
Increasing only the pitch filter time constant from 0.75 to 1.5 seconds cut
modeled grade-command variation about 22% and the largest qlog-scale step about
47%; sustained hills still reach the complete 150 Nm feed-forward range.

Two full-rate b6w events showed White Panda failure mask `0x001c`: all three
private command frames had become stale, causing a 440-to-0-to-440 Nm command
sequence in one event and a 169.75-to-0-to-174 Nm sequence in another. The
unchanged 25 ms host rate floor is now offered from the 100 Hz controller loop,
producing a nominal 33 Hz complete triplet instead of 25 Hz. Both Pandas retain
their existing minimum-interval and 100 ms freshness watchdogs; no stale
command is held longer and a genuine host stall still fails closed.

## Validation completed

- 52 Chrysler host longitudinal tests pass after the smooth-cruise and
  transport-frequency tests were added.
- Focused embedded Panda b6z safety tests pass.
- External White Panda guard test passes under strict C warnings.
- Embedded Panda F4 and H7 firmware compile with warnings treated as errors.
- External White Panda F4 firmware compiles and signs successfully.
- The b6x fault segment replays offline without publishing CAN.

Offline replay can identify regressions and enforce invariants, but it cannot
prove that the Jeep will not fault. A stationary validation followed by one
controlled road test is still required before increasing the ceiling toward
the factory-observed 500-575 Nm range.

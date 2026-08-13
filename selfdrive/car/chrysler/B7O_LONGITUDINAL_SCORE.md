# b7o longitudinal replay score

Date: 2026-08-12

This is an offline, non-actuating evaluation. Both replay tools read local
`rlog` files only, publish no CAN, and report `actuation_frames=0`. Jeep OP Long
transport remains compiled out on b7o. These results do not authorize a vehicle
installation or an actuating branch.

## Evidence set

The mapper score uses four prior experimental OP Long routes and the two recent
b7f factory-ACC routes. The controller screen replays the same trajectories at
100 Hz and preserves controller state across segments of the same route.

- Experimental: `0000003d--10856f3742`, `0000003e--3f6f0a64f4`,
  `0000003f--3351676c53`, and `00000040--7fb5d8a01a`.
- Factory ACC: `0000005d--69a092ece3` and `0000005e--bfc03be27d`.
- Combined controller screen: 31,247 active samples / 312.357 seconds,
  including 14,552 experimental and 16,333 factory samples (362 samples had
  no fresh ownership diagnostic and were retained only as inactive context).

## What the recorded behavior says

The current Jeep shadow mapper is substantially calmer than the recorded
experimental behavior on identical requests:

| Metric | Recorded behavior | Current shadow mapper |
|---|---:|---:|
| Mode entries/minute | 23.895 | 12.239 |
| Engine/brake reversals within 3 seconds | 15 | 2 |
| Direct engine-to-brake transitions | 0 | 0 |
| Torque step, p95 | n/a | 3.589 Nm |
| Brake step, p95 | n/a | 0.030 m/s² |

The factory reference does not support changing the mapper constants yet:

- An engine model including requested acceleration, speed, and filtered pitch
  fits all samples with R² 0.703, but route-held-out validation improves two
  routes and regresses a third by 15.32 Nm MAE.
- A linear brake fit has R² 0.234 and improves one held-out route while
  worsening another by 0.193 m/s² MAE.
- On recorded uphill demand, the current candidate reaches its 440 Nm ceiling.
  This shows that grade/load authority needs separate validation; it does not
  establish that a higher ceiling is safe or correct.

Therefore b7o makes no torque-ceiling, grade-gain, or brake-map change.

## Feedback-controller screen

The current production candidate (`Kp=0.60`, `Ki=0.20`, deadzone 0.10,
delay 0.15 s) reproduced the recorded experimental acceleration command with
0.0153 m/s² MAE. That close agreement validates the replay implementation.
Three feedback-only alternatives kept planner feed-forward, actuator delay,
acceleration limits, stopping state machine, and Jeep mapper unchanged.

| Tune | Kp | Ki | Command variation/s | Steady braking fraction | Unplanned-brake events / seconds | Known 3f negative area | Planned braking ≤ -0.4 | Large deficit ≥ 1.5 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Current | 0.60 | 0.20 | 0.4407 | 0.2814 | 6 / 12.906 s | -1.5027 m/s | 0.8727 | 1.0000 |
| Lower I | 0.60 | 0.10 | 0.4425 | 0.1896 | 4 / 6.490 s | -0.7634 m/s | 0.8948 | 1.0000 |
| Lower PI | 0.50 | 0.10 | 0.4099 | 0.1919 | 4 / 6.030 s | -0.7797 m/s | 0.8962 | 1.0000 |
| Soft | 0.40 | 0.05 | 0.3808 | 0.1288 | 2 / 1.840 s | -0.4490 m/s | 0.9227 | 1.0000 |

The known route-3f event is a controller feedback issue rather than a mapper
issue: the vehicle crossed its target speed while the planner remained nearly
neutral, but the controller accumulated a braking request. The soft tune cuts
that negative command area by about 70%, reduces overall command variation by
about 14%, and preserves the recorded large-deficit and planned-braking gates.

The soft tune is the leading **offline shadow candidate**, not an accepted
vehicle tune. Recorded-trajectory replay cannot predict the new closed-loop
vehicle trajectory, and the evidence contains only one route with the clearest
false-braking sequence. Production constants remain unchanged.

## Acceptance gates before any tune change

1. Repeat the screen on at least one additional route containing overshoot /
   false-braking behavior and one sustained uphill acceleration event.
2. Preserve 100% of large-speed-deficit samples at or above 1.5 m/s².
3. Do not reduce the fraction of planned-deceleration samples at or below
   -0.4 m/s² relative to the current tune.
4. Reduce both false-brake duration and negative command area on each eligible
   route, not merely in the aggregate.
5. Keep b7o non-actuating and retain every ownership/isolation invariant in
   `B7O_REPLAY_ACCEPTANCE.md`.
6. Treat any future closed-loop validation as a separate, explicitly reviewed
   phase; replay success alone is insufficient.

## Reproduction

`tools/b7o_longitudinal_score.py` scores the recorded command topology and
non-actuating mapper. `tools/b7o_controller_replay.py` compares feedback tunes.
Both require the local Windows Cap'n Proto dependencies used by the existing
openpilot log-analysis environment.

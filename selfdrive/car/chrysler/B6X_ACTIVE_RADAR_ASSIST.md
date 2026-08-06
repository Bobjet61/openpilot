# b6x: active matched-radar braking and automatic lead departure

`b6x` is based on `b6w` and uses the already validated passive Jeep radar
association for bounded active longitudinal assistance. It does not enable the
unfinished generic Chrysler radar interface or accept radar-only targets.

## Active braking

- Vision must independently report a lead with at least 0.80 probability.
- The factory radar observation must pass the existing range, relative-speed,
  absolute-target-speed, strength, and ambiguity gates.
- The same radar track must match for three complete radar cycles.
- Missing, stale, weak, ambiguous, or radar-only observations immediately fall
  back to the production planner.
- The assist may request earlier or stronger braking, but never weaker braking
  than the production planner.
- The radar contribution is capped at `-2.5 m/s^2`; the existing host and Panda
  brake envelope, torque/brake interlock, checksums, counters, and pedal/fault
  gates remain authoritative.

The active target uses a Jeep-only 2.0 second gap and a closing-speed estimate
bounded by both vision and radar. Radar may add at most 3.0 m/s of closing speed
beyond vision for one matched target.

## Stop-and-go

At true standstill, a lead departure must be measured by both sensors for four
complete cycles. That starts the existing guarded HOLD -> RELEASE -> GO ->
CREEP sequence. No physical or injected RESUME-button press is required.

The launch request is `0.45 m/s^2`, while actual launch torque remains capped
at the existing 200 Nm. If the target stops, the association becomes stale, or
the Jeep does not establish rolling speed, the existing state machine re-holds.

## Other b6x changes

- First SET in experimental mode initializes to current speed, clipped to the
  FCA minimum, instead of the generic 105 km/h (65 mph) default. Physical RES
  still reuses the previous set speed.
- Positive-request feed-forward changes from 16.5 to 40.0 Nm/(m/s^2), and the
  torque ramp changes from 300 to 360 Nm/s.
- The hard engine-torque ceiling remains 440 Nm.
- Steering response 5 and the b6w bump-steer tuning are unchanged.

## Validation

Focused host tests cover ambiguous/radar-only rejection, three-cycle braking
confirmation, planner priority when it is already more conservative, matched
lead launch, dual-sensor launch gating, stale-match fallback, and the complete
standstill-to-GO sequence without brake/engine overlap.

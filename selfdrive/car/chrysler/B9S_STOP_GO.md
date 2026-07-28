# B9S Jeep stop-and-go

`b9s` extends the tested `b8y` Jeep longitudinal path with a guarded
standstill hold and stock-ACC resume.

## Control handoff

- Above 2.06 m/s (4.6 mph), openpilot longitudinal control is unchanged.
- Below that threshold, stock ACC still performs the final approach.
- If stock ACC times out at standstill, the private White Panda path applies
  the previously recorded `-2.0 m/s²` brake-only hold.
- A resume request requires eight consecutive new radar cycles in which the
  passive Jeep radar track and the independent vision lead both report that
  the lead is pulling away.
- RESUME is sent while the brake-only hold remains active. The private hold is
  removed only after the unmodified stock DAS_4 state explicitly reports
  adaptive ACC active and takes over as the braking/launch gate. Attempts are
  capped at ten; after that the vehicle remains held until driver intervention.

## Fail-closed conditions

Driver cancel, gas, brake, a controls disengagement, a non-forward gear, ACC
unavailability or fault, and stock AEB immediately clear the host hold state.
Both Panda layers independently require fresh speed, pedal, and stock-ACC
state. At or below the moving threshold they accept only a brake cycle with
zero engine-torque request. Stop, go, and brake-preparation bits remain
forbidden.

## Steering

The applied Jeep steering calibration is 261 maximum torque units at rate 5.
The rate and real-time torque envelopes remain unchanged. The 270-unit road
test produced a permanent EPS fault, so 270 is retained only as telemetry and
has no CAN output.

# B9S Jeep stop-and-go

`b9s` extends the tested `b8y` Jeep longitudinal path with a guarded
standstill hold and stock-ACC resume.

## Control handoff

- Above 2.06 m/s (4.6 mph), openpilot longitudinal control is unchanged.
- Below that threshold, stock ACC still performs the final approach.
- If stock ACC times out at standstill, the private White Panda path applies
  the previously recorded `-2.0 m/s²` brake-only hold.
- A resume request requires four consecutive new radar cycles in which the
  passive Jeep radar track and the independent vision lead both report that
  the lead is pulling away.
- Hold is released for 0.5 seconds after each resume request and is restored
  if the vehicle does not launch. Attempts are capped at ten; after that the
  vehicle remains held until driver intervention.

## Fail-closed conditions

Driver cancel, gas, brake, a controls disengagement, a non-forward gear, ACC
unavailability or fault, and stock AEB immediately clear the host hold state.
Both Panda layers independently require fresh speed, pedal, and stock-ACC
state. At or below the moving threshold they accept only a brake cycle with
zero engine-torque request. Stop, go, and brake-preparation bits remain
forbidden.

## Steering

The applied steering calibration remains 261 maximum torque units at rate 5.
A 270-unit candidate runs as telemetry only and has no CAN output. The route
replay showed that raising the ceiling alone increased reversal lag, so the
candidate is not applied in this branch.

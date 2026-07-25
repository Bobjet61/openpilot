# Jeep APA5 research branch

This branch replaces the earlier 50 mph APA proposal with a factory-speed
envelope:

- full proposed APA authority at or below 4 mph;
- linear authority reduction from 4 to 5 mph;
- hard APA exit at 5 mph;
- re-entry only at or below 4 mph;
- host eligibility requires forward gear, cruise availability, lateral
  control, no brake input, no driver steering input, and no steering fault.

`APA5_ACTUATION_COMPILED` is deliberately `False`. The branch only logs the
envelope and commands `STEER_TYPE_NONE`. It is not an actuation or installation
branch.

The paired White Panda branch must independently enforce real-speed and
message-freshness gates. If the EPS will not accept APA while receiving real
speed, transmission, shifter, reverse, and wheel-speed state, APA development
must stop rather than falsifying those states.

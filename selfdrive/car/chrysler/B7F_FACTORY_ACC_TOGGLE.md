# b7f: restart-gated factory/openpilot longitudinal selector

`b7f` keeps the b6z diagnostic openpilot-long implementation available, but
places it behind sunnypilot's existing **Experimental Longitudinal Control
(Alpha)** offroad toggle.

## Modes

### Both toggles off (factory ACC)

- `openpilotLongitudinalControl` is false;
- `pcmCruise` is true;
- the Jeep's factory ACC owns moving acceleration and braking;
- `CustomStockLong` is off, so no synthetic hold or resume is sent;
- no Jeep long-shadow, diagnostic, or actuation safety flag is given to the
  embedded Panda; and
- the external White Panda remains in factory pass-through because the host
  never requests longitudinal ownership.

### Custom Stock Long on, Experimental Longitudinal Alpha off

- factory ACC retains moving throttle, braking, set-speed, and engagement
  ownership;
- the proven b6 standstill hold/resume bridge is enabled;
- the embedded Panda permits only the exact copied-stock `DAS_3` hold payload
  and its short-lived RESUME; and
- all private openpilot-long throttle/brake frames remain blocked.

Lateral response-5 steering remains available in all three modes.

### Experimental Longitudinal Alpha on (diagnostic mode)

- the current b6z openpilot-long controller is enabled;
- all three matched Jeep-long Panda safety flags are enabled together; and
- experimental mode can use openpilot longitudinal control.

The b6z ACC-fault and authority limitations remain. The toggle does not make
that path production-ready.

## Switching rule

The settings are sampled only while the car process fingerprints the Jeep.
Change them offroad and restart the comma before driving. Experimental
Longitudinal Alpha takes precedence if both toggles are on. There is
deliberately no live onroad switch and no partial Panda reconfiguration.

## Factory stop-and-go

The driver confirmed that the apparent route fault occurred while the comma
was being touched and a cable was jiggled. It must not be attributed to the
factory-based standstill hold. The proven factory stop-and-go bridge remains a
implemented as the separately selectable factory-ACC-plus-hold mode, with its
existing narrow hold-frame and resume guards preserved.

# b7o selectable Jeep longitudinal modes

Date: 2026-08-12

This design stays on the current b7o code, current CAN2 routing, and current
White Panda architecture. It does **not** revert to the old b6-shadow source.
Only the successful b6 behavior is retained as a concept: factory ACC controls
the moving vehicle, a narrowly filtered command maintains the stopped brake
hold after the factory timeout, and a RESUME request returns control to factory
ACC after the lead starts moving.

## Restart-gated selector

The existing sunnypilot offroad toggles map to three mutually exclusive modes:

| Experimental Longitudinal (Alpha) | Custom Stock Long | Selected mode | Moving owner | Standstill extension |
|---|---|---|---|---|
| Off | Off | Factory | Factory ACC | None |
| Off | On | Factory + stop/go | Factory ACC | Guarded copied-stock hold and bounded RESUME |
| On | Either | Experimental | openpilot, but only in a separately activated build | openpilot low-speed state machine |

The preferred UI is ultimately one three-position selector with those exact
mode names. It is harder to misconfigure and communicates that the choices are
mutually exclusive. This checkout contains sunnypilot's prebuilt UI executable,
not the editable settings source, so the first implementation retains the two
existing toggles and resolves them through the same single mode selector in the
car interface.

The settings are sampled during fingerprinting. Switching is offroad-only and
requires a comma restart so CarParams and both Panda safety configurations
change together. There is no live onroad ownership switch.

If both toggles are selected, Experimental has priority and the factory
stop/go augmentation is disabled. This prevents two longitudinal augmentation
paths from being armed together.

On b7o, longitudinal actuation remains compiled out. An Experimental request
therefore resolves to `experimental_shadow`: factory ACC remains the vehicle
owner, factory stop/go is not armed, and experimental behavior is evaluated
only offline/in shadow. The existing Alpha toggle must not be presented as an
active vehicle mode until a separate reviewed activation build exists.

## Current factory stop/go path

Factory + stop/go uses the newer implementation already descended from b6y,
not the original b6-shadow transmitter:

- Factory ACC retains moving throttle, braking, set speed, and engagement.
- The host arms only after factory ACC is observed decelerating below 3 m/s and
  subsequently reaches standstill without an ACC fault or stock AEB event.
- The stopped command copies the latest stock `DAS_3` and changes only the
  fixed hold fields; propulsion authority and `ACC_GO` remain clear.
- The frame counter stays synchronized to fresh stock traffic.
- The current CAN2/White Panda path remains in use; the external White Panda
  passes the factory/guarded direct traffic while the embedded Panda permits
  only the narrowly described stopped hold and RESUME envelope.
- Cancel, either pedal, motion, non-forward gear, unavailable cruise, ACC
  fault, or stock AEB releases the hold immediately.
- Automatic RESUME is bounded to three attempts. Each attempt is a six-counter
  physical-button-shaped pulse, cancelled by any real driver button press.
- RESUME requires sustained lead departure confirmed by vision plus raw radar,
  with a conservative high-confidence vision fallback if radar association
  drops during launch.

This preserves the useful b6 outcome without restoring its indefinite 100 ms
RESUME cadence or its pre-CAN2/White-Panda topology.

## Evidence status

The older b6 route contained seven automatic ACC resume/movement events and no
observed hold dropout. That demonstrates the vehicle can resume stock ACC from
the extended hold, but it does not validate the newer selector by itself.

In recent b7f route 54, the host emitted 5,131 guarded hold frames. The driver
pressed physical RESUME before lead-motion confirmation completed; software
issued its first guarded RESUME about 163 ms later. Consequently that route
proves the hold path but does not prove a hands-off automatic launch. A future
test must wait at least two seconds after the lead begins moving before manual
intervention, while remaining ready to brake.

## Required invariants

1. Exactly one moving longitudinal owner is selected at startup.
2. Factory stop/go and openpilot-long can never be armed together.
3. A shadow build cannot grant any openpilot longitudinal safety flag or
   vehicle actuation authority.
4. Factory + stop/go cannot request propulsion or set `ACC_GO`; departure is a
   bounded request for factory ACC to resume.
5. Experimental activation continues to require explicit physical isolation,
   healthy isolation diagnostics, a quiet stock-command path, and independent
   embedded/White Panda acceptance.
6. Any ownership conflict, fault, pedal input, or invalid topology fails back
   to no openpilot actuation; it never blends factory and openpilot commands.

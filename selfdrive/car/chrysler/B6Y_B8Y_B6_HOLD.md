# b6y: b8y moving control plus the proven b6 standstill bridge

`b6y` is a local merge candidate based on b8y host commit `91ed644c63`.
It preserves b8y's response-5 steering and moving-only openpilot
longitudinal control, then restores the b6 direct `DAS_3` standstill hold and
stock-ACC RESUME path.

The b8y history already descends from b6-shadow. The intervening b8x work
removed the old active brake-hold path, so this candidate restores only that
bounded behavior rather than trying to merge two unrelated histories.

This is not a general full-speed longitudinal expansion:

- b8y's private White Panda engine-torque and brake protocol remains
  moving-only and retains its existing minimum-speed gate;
- the b6 standstill path can request only a fixed `-2.0 m/s^2` brake hold;
- the standstill path explicitly clears engine-torque authority, `ACC_GO`,
  and the standstill propulsion bit;
- departure is handed back to stock ACC with RESUME;
- b8y's private moving control can become eligible only after the Jeep is
  moving above its existing minimum.

## Host state machine

The hold arms only on a supported Jeep with the White Panda flag when all of
the following are true:

- openpilot longitudinal control is compiled on;
- openpilot is enabled and longitudinally active;
- stock ACC is active;
- stock ACC is requesting meaningful deceleration;
- the Jeep is at standstill;
- there is no ACC fault or stock AEB event.

It releases on openpilot disable, cancel, gas, brake, non-drive gear, vehicle
movement, ACC fault, or stock AEB.

Once the stock stop-and-go timeout makes stock ACC inactive, the controller
copies the latest stock `DAS_3`, changes only the b6 hold fields, and sends the
fixed brake command. It then sends RESUME at the proven b6 cadence of once
every 10 control frames. RESUME asks stock ACC to re-enter control; it does not
request engine torque.

The internal b6y latch is intentionally not published as generic
`brakeHoldActive`. In this SunnyPilot generation, generic brake hold is a user
disable event when openpilot longitudinal control is enabled.

## Embedded Panda safety

The matching embedded Panda source is published as:

- branch: `panda-0971-b6y`;
- commit: `fc1905071b678387aaec4cc2fa3798e1ccf47933`.

Direct `DAS_3` hold is permitted only when:

- both b8y's Jeep long-shadow and actuation flags are set;
- the platform is the Pacifica/Jeep safety platform;
- ACC main is on and the Jeep is stopped;
- gas and brake are not pressed;
- stock collision/brake-prep state is clear;
- stock `DAS_3`, speed, gas, and brake sources were all seen within 100 ms;
- the command matches the fresh stock frame outside the narrow allowed mask;
- brake deceleration encodes exactly as raw `2866` (`-2.0 m/s^2`);
- available and active are set, maximum gear is 2, command type is brake,
  brake prep is clear, engine-torque request authority is disabled, and GO is
  clear;
- the counter is stock +2 or +3 and the checksum is valid.

The special standstill RESUME bypass exists only for 100 ms after an accepted
exact hold frame. Motion, gas, brake, collision state, cancel, stale inputs, or
any malformed direct hold clears it.

## White Panda boundary

No White Panda source change is required. The matching b8y White Panda source
commit is `5e755d76de`.

Its forwarding path already sends a Comma-side direct `0x1F4` frame to the
vehicle bus unchanged. Its private b8y protocol remains responsible only for
moving-control substitution of the stock frame. This keeps all new
standstill authorization inside the embedded Panda safety hook.

## Local verification

- 33 focused Panda tests covering b6y hold, b8y longitudinal safety, and
  response-5 steering: passed.
- Exact hold acceptance plus rejection of wrong decel, GO, propulsion,
  standstill flag, wrong gear request, brake prep, wrong command type, bad
  counter, bad checksum, motion, gas, brake, collision, stale source, and
  post-cancel RESUME: passed.
- Full Chrysler Panda suite on untouched b8y:
  98 passed, 17 failed, 13 skipped.
- Full Chrysler Panda suite on b6y:
  103 passed, the same 17 failed, 13 skipped.
- The 17 failures are pre-existing generic-test expectation mismatches in
  this older fork; b6y adds no new full-suite failure.
- Host Jeep longitudinal/radar/steering tests available without device-only
  compiled modules: 40 passed.
- Planner-shadow tests: 7 passed.
- Changed Python sources: byte-compiled successfully.
- F4 and H7 Panda firmware: compiled with `-Wall -Wextra
  -Wstrict-prototypes -Werror` and signed as local validation artifacts.

The signed artifacts embedded in `b6y` identify themselves as
`DEV-fc190507-DEBUG`:

- F4 SHA-256:
  `1B66BE27F5615039B07845073E4703ED1CB69F3C8B7E3A762B2B9C8FEB62AE64`
- H7 SHA-256:
  `8CDE6D863A32E322B8921528B3C3C4580D40F2D2071ED0E82E98D87EAC79CF48`

Both firmware targets were rebuilt from the published source commit before
being embedded in the host branch.

## Current disposition

The host branch packages the matched source changes and embedded comma Panda
firmware. Installation can update the embedded comma Panda through the normal
openpilot startup firmware-version check. The external White Panda firmware
remains the unchanged matched b8y build.

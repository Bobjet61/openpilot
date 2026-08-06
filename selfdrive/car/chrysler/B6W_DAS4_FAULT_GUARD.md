# b6w DAS_4 fault guard

`b6w` is a narrow safety and recovery revision of `b6v`.

- The host, embedded Panda, and external White Panda cap running engine-torque
  requests at 440 Nm. The separate low-speed launch ceiling remains 200 Nm.
- `DAS_4.ACC_FAULTED` is treated as an ACC fault in addition to the older
  `DAS_3.ACC_FAULTED` field.
- The embedded Panda requires fresh clear DAS_4 traffic before accepting a
  private longitudinal cycle. A DAS_4 fault immediately revokes longitudinal
  permission and requires a new clean physical SET/RES action after it clears.
- The White Panda requires fresh clear DAS_4 traffic before it can substitute
  openpilot commands into the stock DAS_3 stream. A dashboard fault invalidates
  its committed command and returns ownership to off.
- Steering response 5, steering torque limits, and the bump-steer filter are
  unchanged from `b6v`.

For the marked-road validation drive, select lane-line mode rather than the
always-laneless Dynamic Lane Profile and leave camera/path offsets at zero. The
branch deliberately does not rewrite the user's persistent lateral settings.

Both Panda firmwares changed. Installing the host branch updates the embedded
Panda firmware; the external White Panda must also be flashed with `wp-b6w`.

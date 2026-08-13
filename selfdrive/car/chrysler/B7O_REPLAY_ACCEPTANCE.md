# b7o replay acceptance baseline

Date: 2026-08-12

This is an offline, non-actuating acceptance baseline. The replay model always
returns `actuation_allowed=False`, and the b7o Jeep longitudinal transport is
compiled out. Nothing in this report authorizes vehicle installation.

## Topology correction

`cruiseState.enabled` is not an actuator-ownership signal. It cannot prove that
factory longitudinal commands have been isolated. The replay now has two
explicit modes:

- `recorded_unproven_isolation`: the topology recorded by existing routes. The
  openpilot shadow owner must never arm.
- `counterfactual_stock_isolated`: an offline-only assumption that a healthy
  exclusive isolation boundary exists and its stock-command output is quiet.
  This exercises handoff and planner-request behavior but does not establish
  that the physical Jeep has the assumed topology.

Handoff confirmation is based on one second of monotonic elapsed time, not a
sample count. A stock command reappearing after handoff revokes output on that
sample and latches authorization off until controls are explicitly reset.

## Routes replayed

| Route | Prior configuration | Duration (s) | Eligible candidate samples | Candidate brake | Candidate accel | Recorded owner entries | Counterfactual owner entries |
|---|---|---:|---:|---:|---:|---:|---:|
| `0000005d--69a092ece3` | b7f factory ACC | 1,922.144 | 10,066 | 4,444 | 4,085 | 0 | 41 |
| `0000005e--bfc03be27d` | b7f factory ACC | 2,761.855 | 10,742 | 5,838 | 3,513 | 0 | 38 |
| `0000003d--10856f3742` | b6z experimental OP Long | 318.924 | 900 | 390 | 208 | 0 | 9 |
| `0000003e--3f6f0a64f4` | b6z experimental OP Long | 257.368 | 812 | 296 | 449 | 0 | 4 |
| `0000003f--3351676c53` | b6z experimental OP Long | 253.503 | 1,149 | 251 | 512 | 0 | 4 |
| `00000040--7fb5d8a01a` | b6z experimental OP Long | 247.240 | 1,128 | 343 | 704 | 0 | 5 |
| **Total** | | **5,761.034** | **24,797** | **11,562** | **9,471** | **0** | **101** |

Every replay reported exactly zero CAN actuation frames. The recorded topology
reported zero shadow-owner entries across all six routes. The counterfactual
mode shows that past drives contain substantial braking and propulsion planner
coverage without conflating those candidate requests with actuator authority.

## Required invariants

The branch is accepted for continued offline development only when all of these
remain true:

1. Recorded/unproven isolation produces zero `OPENPILOT_SHADOW` entries.
2. Every replay produces zero CAN actuation frames.
3. Handoff requires explicit isolation health plus a complete one-second quiet
   interval measured with monotonic time.
4. Controls disable, stock-cruise unavailability, pedal input, non-forward gear,
   restraint/door state,
   collision, stock fault, isolation loss, or invalid data suppresses shadow
   output immediately.
5. A post-handoff stock-command conflict revokes immediately and cannot
   automatically re-arm; a controls reset is required.
6. Unit tests exhaust all 2,048 Boolean gate combinations and assert that no
   unsafe combination reaches the shadow owner.

Before any actuating branch can be considered, a separate recorded hardware
signal must prove the physical isolation boundary, and both embedded and White
Panda safety layers require independent counter/timing review.

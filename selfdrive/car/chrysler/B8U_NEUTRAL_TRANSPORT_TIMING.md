# b8u neutral transport timing

`b8u` remains a transport-only branch. It cannot request propulsion or
braking:

- `JEEP_LONG_ACTUATION_COMPILED` remains `False`;
- `openpilotLongitudinalControl` and experimental longitudinal remain off;
- `OP_LONG_ENABLE` remains zero;
- brake, stop/go, brake-prep, engine-request, and engine-torque fields remain
  strictly neutral.

## Why this branch exists

The first `b8t` road log recorded 10,495 neutral transport cycles. The
embedded Panda accepted 10,096 and rejected 399. The rejected bursts aligned
with three integration mismatches:

- host scheduling occasionally placed cycles at or below the Panda's 15 ms
  minimum;
- a delayed/skipped host cycle advanced the frame-derived counter by more than
  one even though less than 100 ms had elapsed;
- the host continued the moving-only probe at standstill while Panda required
  `vehicle_moving`.

## Changes

- A dedicated transport counter advances only when a cycle is appended to
  `can_sends`.
- An 18 ms minimum host interval leaves 3 ms margin above Panda's 15 ms
  minimum. A too-early cycle is skipped without consuming a counter.
- Transport eligibility is false at standstill or at `vEgo <= 0.1 m/s`.

The candidate longitudinal computation remains log-only and is never packed
into the transmitted frames.

## Acceptance criteria

On the next passive drive:

1. every sent private cycle is ordered `0x1F6`, `0x1F7`, `0x272`;
2. every payload is neutral and has a valid checksum;
3. the embedded Panda returns every cycle as accepted (source 128), with no
   private source-192 rejection;
4. `host_enabled` remains false;
5. no private cycle is sent at standstill;
6. the read-only external White Panda USB monitor confirms receipt without
   observing brake or propulsion output.

Passing these criteria does not authorize active longitudinal control.

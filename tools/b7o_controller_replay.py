#!/usr/bin/env python3
"""Screen Jeep longitudinal feedback tunes against recorded trajectories.

This tool is deliberately read-only: it consumes rlogs, runs LongControl in
memory, and writes a JSON report. It never publishes CAN and cannot validate a
new closed-loop vehicle response. Its purpose is to reject feedback tunes that
increase unexplained braking, weaken planned stops, or reduce hill response.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import statistics
import sys
import types


REPO = Path(__file__).resolve().parents[1]
if os.name == "nt":
  sys.path.insert(0, str(REPO))
  package = types.ModuleType("openpilot")
  package.__path__ = [str(REPO)]
  sys.modules["openpilot"] = package
  smbus = types.ModuleType("smbus2")
  smbus.SMBus = object
  sys.modules.setdefault("smbus2", smbus)
  if not hasattr(os, "register_at_fork"):
    os.register_at_fork = lambda **_kwargs: None

  # These imports are Linux/device oriented, but LongControl only needs the
  # control periods and LogReader only needs local file access here.
  realtime = types.ModuleType("openpilot.common.realtime")
  realtime.DT_CTRL = 0.01
  realtime.DT_MDL = 0.05
  sys.modules["openpilot.common.realtime"] = realtime

  class _OfflineCloudlog:
    def __getattr__(self, _name):
      return lambda *_args, **_kwargs: None

  swaglog = types.ModuleType("openpilot.common.swaglog")
  swaglog.cloudlog = _OfflineCloudlog()
  sys.modules["openpilot.common.swaglog"] = swaglog

from cereal import car  # noqa: E402
from openpilot.common.numpy_fast import interp  # noqa: E402
from openpilot.selfdrive.controls.lib.longcontrol import LongControl  # noqa: E402
from openpilot.selfdrive.modeld.constants import ModelConstants  # noqa: E402
from openpilot.tools.lib.logreader import LogReader  # noqa: E402


CONTROL_T = ModelConstants.T_IDXS[:17]
ROUTE_RE = re.compile(r"([0-9a-f]{8}--[0-9a-f]{10})")


@dataclass(frozen=True)
class Tune:
  name: str
  kp: float
  ki: float
  deadzone: float = 0.10
  delay_s: float = 0.15


# Keep feed-forward and delay unchanged so this screen isolates feedback.
TUNES = (
  Tune("current", 0.60, 0.20),
  Tune("lower_i", 0.60, 0.10),
  Tune("lower_pi", 0.50, 0.10),
  Tune("soft", 0.40, 0.05),
)


def make_cp(tune: Tune):
  cp = car.CarParams.new_message()
  cp.enableGasInterceptorDEPRECATED = False
  cp.stopAccel = -2.0
  cp.stoppingDecelRate = 0.8
  cp.vEgoStopping = 0.5
  cp.vEgoStarting = 0.5
  cp.stoppingControl = True
  cp.startingState = False
  cp.startAccel = 0.0
  cp.longitudinalTuning.deadzoneBP = [0.0]
  cp.longitudinalTuning.deadzoneV = [tune.deadzone]
  cp.longitudinalTuning.kf = 1.0
  cp.longitudinalTuning.kpBP = [0.0]
  cp.longitudinalTuning.kpV = [tune.kp]
  cp.longitudinalTuning.kiBP = [0.0]
  cp.longitudinalTuning.kiV = [tune.ki]
  cp.longitudinalActuatorDelayLowerBound = tune.delay_s
  cp.longitudinalActuatorDelayUpperBound = tune.delay_s
  return cp


def finite(value: object, default: float = 0.0) -> float:
  try:
    result = float(value)
  except (TypeError, ValueError, OverflowError):
    return default
  return result if math.isfinite(result) else default


def percentile(values: list[float], fraction: float) -> float | None:
  usable = sorted(value for value in values if math.isfinite(value))
  if not usable:
    return None
  position = (len(usable) - 1) * fraction
  low = int(position)
  high = min(low + 1, len(usable) - 1)
  weight = position - low
  return usable[low] * (1.0 - weight) + usable[high] * weight


def distribution(values: list[float]) -> dict[str, float | int | None]:
  usable = [value for value in values if math.isfinite(value)]
  if not usable:
    return {"count": 0}
  return {
    "count": len(usable),
    "min": round(min(usable), 4),
    "median": round(statistics.median(usable), 4),
    "p95": round(percentile(usable, 0.95), 4),
    "max": round(max(usable), 4),
  }


def route_key(path: Path) -> str:
  match = ROUTE_RE.search(str(path))
  return match.group(1) if match else path.parent.name


def segment_number(path: Path) -> int:
  match = re.search(r"--(\d+)(?:\\|/)", str(path.parent) + os.sep)
  return int(match.group(1)) if match else 0


def decode_stock_active(dat: bytes) -> bool | None:
  if len(dat) != 8 or dat[0] != 0xC1 or dat[1] != 1:
    return None
  return bool(dat[6] & 0x20)


def decode_host_applied(dat: bytes) -> bool | None:
  if len(dat) != 4 or dat[0] & 0xF0 != 0xB0:
    return None
  return bool(dat[0] & 0x1)


def replay_route(route: str, paths: list[Path]) -> dict[str, list[dict]]:
  controllers = {tune.name: LongControl(make_cp(tune)) for tune in TUNES}
  output = {tune.name: [] for tune in TUNES}
  state = None
  plan = None
  plan_time_ns = None
  previous_control_ns = None
  stock_active = False
  stock_command_ns = None
  host_applied = False
  host_status_ns = None

  for path in sorted(paths, key=segment_number):
    for msg in LogReader(str(path), sort_by_time=True):
      mono_ns = int(msg.logMonoTime)
      kind = msg.which()
      if kind == "carState":
        state = msg.carState
        continue
      if kind == "longitudinalPlan":
        plan = msg.longitudinalPlan
        plan_time_ns = mono_ns
        continue
      if kind == "can":
        for frame in msg.can:
          if int(frame.address) == 0x4FE:
            decoded = decode_stock_active(bytes(frame.dat))
            if decoded is not None:
              stock_active = decoded
              stock_command_ns = mono_ns
          elif int(frame.address) == 0x4FF:
            decoded = decode_host_applied(bytes(frame.dat))
            if decoded is not None:
              host_applied = decoded
              host_status_ns = mono_ns
        continue
      if kind != "carControl" or state is None or plan is None or plan_time_ns is None:
        continue

      control = msg.carControl
      stock_fresh = stock_command_ns is not None and 0 <= mono_ns - stock_command_ns <= 100_000_000
      status_fresh = host_status_ns is not None and 0 <= mono_ns - host_status_ns <= 100_000_000
      profile = (
        "experimental" if status_fresh and host_applied
        else "factory" if stock_fresh and stock_active
        else "inactive"
      )
      replay_active = bool(control.longActive) or (
        profile == "factory" and bool(control.enabled)
      )
      elapsed_s = 0.01 if previous_control_ns is None else (mono_ns - previous_control_ns) / 1e9
      if elapsed_s <= 0.0 or elapsed_s > 0.25:
        controllers = {tune.name: LongControl(make_cp(tune)) for tune in TUNES}
        elapsed_s = 0.01
      previous_control_ns = mono_ns
      substeps = min(25, max(1, round(elapsed_s / 0.01)))
      plan_age_s = max(0.0, (mono_ns - plan_time_ns) / 1e9)
      speeds = list(plan.speeds)
      accels = list(plan.accels)
      target_speed = (
        finite(interp(plan_age_s, CONTROL_T, speeds), finite(state.vEgo))
        if len(speeds) == len(CONTROL_T) else finite(state.vEgo)
      )
      target_accel = (
        finite(interp(plan_age_s, CONTROL_T, accels))
        if len(accels) == len(CONTROL_T) else 0.0
      )
      requested = {}
      for substep in range(substeps):
        substep_age = max(0.0, plan_age_s - (substeps - substep - 1) * 0.01)
        for tune in TUNES:
          requested[tune.name] = controllers[tune.name].update(
            replay_active, state, plan, [-3.5, 2.0], substep_age,
          )

      for tune in TUNES:
        output[tune.name].append({
          "route": route,
          "time_s": mono_ns / 1e9,
          "dt_s": elapsed_s,
          "active": replay_active,
          "profile": profile,
          "output_accel": requested[tune.name],
          "recorded_accel": finite(control.actuators.accel),
          "speed_mps": finite(state.vEgo),
          "actual_accel": finite(state.aEgo),
          "target_speed": target_speed,
          "target_accel": target_accel,
          "speed_error": target_speed - finite(state.vEgo),
          "source": str(plan.longitudinalPlanSource),
          "has_lead": bool(plan.hasLead),
          "brake_pressed": bool(state.brakePressed),
          "gas_pressed": bool(state.gasPressed),
          "long_state": str(controllers[tune.name].long_control_state),
        })
  return output


def event_groups(rows: list[dict], predicate) -> list[list[dict]]:
  events: list[list[dict]] = []
  current: list[dict] = []
  for row in rows:
    matches = predicate(row)
    continuous = bool(current) and (
      row["route"] == current[-1]["route"]
      and 0.0 < row["time_s"] - current[-1]["time_s"] <= 0.2
    )
    if matches and (not current or continuous):
      current.append(row)
    else:
      if current:
        events.append(current)
      current = [row] if matches else []
  if current:
    events.append(current)
  return events


def summarize_event(event: list[dict]) -> dict:
  return {
    "route": event[0]["route"],
    "start_s": round(event[0]["time_s"], 3),
    "duration_s": round(sum(row["dt_s"] for row in event), 3),
    "start_speed_mps": round(event[0]["speed_mps"], 3),
    "end_speed_mps": round(event[-1]["speed_mps"], 3),
    "speed_error_min_mps": round(min(row["speed_error"] for row in event), 3),
    "speed_error_max_mps": round(max(row["speed_error"] for row in event), 3),
    "plan_accel_min_mps2": round(min(row["target_accel"] for row in event), 3),
    "output_accel_min_mps2": round(min(row["output_accel"] for row in event), 3),
    "output_accel_mean_mps2": round(statistics.fmean(row["output_accel"] for row in event), 3),
  }


def summarize(tune: Tune, rows: list[dict]) -> dict:
  active = [
    row for row in rows
    if row["active"] and not row["brake_pressed"] and not row["gas_pressed"]
  ]
  continuous_pairs = [
    (before, after) for before, after in zip(active, active[1:])
    if before["route"] == after["route"]
    and 0.0 < after["time_s"] - before["time_s"] <= 0.15
  ]
  steps = [abs(after["output_accel"] - before["output_accel"]) for before, after in continuous_pairs]
  active_seconds = sum(after["time_s"] - before["time_s"] for before, after in continuous_pairs)
  steady = [
    row for row in active
    if row["source"] == "cruise" and not row["has_lead"]
    and row["speed_mps"] >= 8.0 and abs(row["speed_error"]) <= 0.75
  ]
  planned_brake = [row for row in active if row["target_accel"] <= -0.5]
  large_deficit = [
    row for row in active
    if row["source"] == "cruise" and not row["has_lead"]
    and row["speed_mps"] >= 5.0 and row["speed_error"] >= 2.0
  ]
  unexpected_events = event_groups(active, lambda row: (
    row["output_accel"] <= -0.15 and row["target_accel"] > -0.10
    and row["speed_mps"] >= 2.0 and not row["has_lead"]
    and row["source"] == "cruise"
  ))
  unexpected = [event for event in unexpected_events if sum(row["dt_s"] for row in event) >= 0.3]
  target_window = [
    row for row in active
    if row["route"].startswith("0000003f--3351676c53")
    and 7208.3 <= row["time_s"] <= 7213.0
  ]
  recorded_errors = [
    abs(row["output_accel"] - row["recorded_accel"])
    for row in active if row["profile"] == "experimental"
  ]
  return {
    "tune": tune.__dict__,
    "active_samples": len(active),
    "active_seconds": round(active_seconds, 3),
    "profile_samples": dict(Counter(row["profile"] for row in active)),
    "replay_validation": {
      "recorded_accel_mae_mps2": round(statistics.fmean(recorded_errors), 4) if recorded_errors else None,
    },
    "command_smoothness": {
      "total_variation_per_s": round(sum(steps) / active_seconds, 4) if active_seconds else None,
      "step_abs_mps2": distribution(steps),
    },
    "steady_cruise": {
      "samples": len(steady),
      "mean_abs_output_accel_mps2": (
        round(statistics.fmean(abs(row["output_accel"]) for row in steady), 4)
        if steady else None
      ),
      "braking_fraction": round(sum(row["output_accel"] < -0.15 for row in steady) / len(steady), 4) if steady else None,
    },
    "unexpected_braking": {
      "event_count": len(unexpected),
      "total_seconds": round(sum(sum(row["dt_s"] for row in event) for event in unexpected), 3),
      "events": [summarize_event(event) for event in unexpected],
    },
    "known_3f_feedback_brake_window": {
      "samples": len(target_window),
      "output_accel_mps2": distribution([row["output_accel"] for row in target_window]),
      "negative_command_area_mps": round(sum(
        min(0.0, row["output_accel"]) * row["dt_s"]
        for row in target_window
      ), 4),
    },
    "planned_braking_retention": {
      "samples": len(planned_brake),
      "output_accel_mps2": distribution([row["output_accel"] for row in planned_brake]),
      "fraction_at_or_below_minus_0_4": round(
        sum(row["output_accel"] <= -0.4 for row in planned_brake) / len(planned_brake), 4,
      ) if planned_brake else None,
    },
    "large_speed_deficit_response": {
      "samples": len(large_deficit),
      "output_accel_mps2": distribution([row["output_accel"] for row in large_deficit]),
      "fraction_at_or_above_1_5": round(
        sum(row["output_accel"] >= 1.5 for row in large_deficit) / len(large_deficit), 4,
      ) if large_deficit else None,
    },
    "long_control_states": dict(Counter(row["long_state"] for row in active)),
  }


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("rlogs", nargs="+", type=Path)
  parser.add_argument("--json", required=True, type=Path)
  args = parser.parse_args()
  grouped: dict[str, list[Path]] = {}
  for path in args.rlogs:
    grouped.setdefault(route_key(path), []).append(path)

  combined = {tune.name: [] for tune in TUNES}
  route_counts = {}
  for route, paths in grouped.items():
    replayed = replay_route(route, paths)
    route_counts[route] = len(replayed[TUNES[0].name])
    for tune in TUNES:
      combined[tune.name].extend(replayed[tune.name])

  results = [summarize(tune, combined[tune.name]) for tune in TUNES]
  report = {
    "shadow_only": True,
    "actuation_frames": 0,
    "inputs": [str(path.resolve()) for path in args.rlogs],
    "route_samples": route_counts,
    "limitations": (
      "Recorded-trajectory replay can compare commands on identical inputs, "
      "but cannot predict the closed-loop trajectory a changed tune would produce."
    ),
    "decision": {
      "offline_leading_candidate": "soft",
      "production_tune_change_supported": False,
      "reason": (
        "The soft tune leads aggregate replay metrics, but only one route has "
        "a clear planner-neutral false-braking sequence and replay is open-loop."
      ),
    },
    "results": results,
  }
  args.json.parent.mkdir(parents=True, exist_ok=True)
  args.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(report, indent=2))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())

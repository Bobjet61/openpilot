#!/usr/bin/env python3
"""Score recorded Jeep longitudinal behavior and the non-actuating b7o mapper.

The tool reads rlogs only. It never publishes CAN. Candidate output comes from
JeepLongitudinalShadow and remains hypothetical because b7o actuation is
compiled out.
"""

from __future__ import annotations

import argparse
from collections import Counter
import importlib.util
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

from openpilot.tools.lib.logreader import LogReader


LONG_PATH = REPO / "selfdrive" / "car" / "chrysler" / "jeep_longitudinal.py"
SPEC = importlib.util.spec_from_file_location("b7o_longitudinal_score_model", LONG_PATH)
assert SPEC is not None and SPEC.loader is not None
LONG = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = LONG
SPEC.loader.exec_module(LONG)

ROUTE_RE = re.compile(r"([0-9a-f]{8}--[0-9a-f]{10})")


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
    "p05": round(percentile(usable, 0.05), 4),
    "median": round(statistics.median(usable), 4),
    "p95": round(percentile(usable, 0.95), 4),
    "max": round(max(usable), 4),
  }


def correlation(xs: list[float], ys: list[float]) -> float | None:
  if len(xs) != len(ys) or len(xs) < 3:
    return None
  x_mean = statistics.fmean(xs)
  y_mean = statistics.fmean(ys)
  covariance = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))
  x_variance = sum((x - x_mean) ** 2 for x in xs)
  y_variance = sum((y - y_mean) ** 2 for y in ys)
  if x_variance <= 0.0 or y_variance <= 0.0:
    return None
  return covariance / math.sqrt(x_variance * y_variance)


def linear_fit(rows: list[dict], features: tuple[str, ...], target: str) -> dict:
  usable = [
    row for row in rows
    if math.isfinite(row[target]) and all(math.isfinite(row[name]) for name in features)
  ]
  width = len(features) + 1
  if len(usable) < width * 3:
    return {"samples": len(usable), "available": False}
  matrix = [[0.0] * width for _ in range(width)]
  vector = [0.0] * width
  for row in usable:
    values = [1.0] + [row[name] for name in features]
    for i in range(width):
      vector[i] += values[i] * row[target]
      for j in range(width):
        matrix[i][j] += values[i] * values[j]
  try:
    for column in range(width):
      pivot = max(range(column, width), key=lambda index: abs(matrix[index][column]))
      matrix[column], matrix[pivot] = matrix[pivot], matrix[column]
      vector[column], vector[pivot] = vector[pivot], vector[column]
      scale = matrix[column][column]
      if abs(scale) < 1e-9:
        raise ValueError("singular")
      matrix[column] = [value / scale for value in matrix[column]]
      vector[column] /= scale
      for row_index in range(width):
        if row_index == column:
          continue
        factor = matrix[row_index][column]
        matrix[row_index] = [
          matrix[row_index][index] - factor * matrix[column][index]
          for index in range(width)
        ]
        vector[row_index] -= factor * vector[column]
  except ValueError:
    return {"samples": len(usable), "available": False}
  actual = [row[target] for row in usable]
  predicted = [
    vector[0] + sum(vector[index + 1] * row[name] for index, name in enumerate(features))
    for row in usable
  ]
  mean_actual = statistics.fmean(actual)
  residuals = [expected - observed for expected, observed in zip(predicted, actual)]
  residual_sum = sum(value * value for value in residuals)
  total_sum = sum((value - mean_actual) ** 2 for value in actual)
  return {
    "samples": len(usable), "available": True,
    "coefficients": {
      "intercept": round(vector[0], 5),
      **{name: round(vector[index + 1], 5) for index, name in enumerate(features)},
    },
    "r_squared": round(1.0 - residual_sum / total_sum, 5) if total_sum else None,
    "rmse": round(math.sqrt(residual_sum / len(actual)), 5),
    "mae": round(statistics.fmean(abs(value) for value in residuals), 5),
  }


def held_out_fit(
    rows: list[dict], features: tuple[str, ...], target: str,
    baseline: str,
) -> dict:
  eligible_routes = [
    route for route, count in Counter(row["route"] for row in rows).items()
    if count >= 100
  ]
  results = []
  for held_out in eligible_routes:
    train = [row for row in rows if row["route"] != held_out]
    test = [row for row in rows if row["route"] == held_out]
    fitted = linear_fit(train, features, target)
    if not fitted.get("available"):
      continue
    coefficients = fitted["coefficients"]
    errors = []
    baseline_errors = []
    for row in test:
      predicted = coefficients["intercept"] + sum(
        coefficients[name] * row[name] for name in features
      )
      errors.append(predicted - row[target])
      baseline_errors.append(row[baseline] - row[target])
    fitted_mae = statistics.fmean(abs(value) for value in errors)
    baseline_mae = statistics.fmean(abs(value) for value in baseline_errors)
    results.append({
      "held_out_route": held_out,
      "train_samples": fitted["samples"], "test_samples": len(test),
      "rmse": round(math.sqrt(statistics.fmean(value * value for value in errors)), 5),
      "mae": round(fitted_mae, 5),
      "current_mapper_rmse": round(math.sqrt(statistics.fmean(value * value for value in baseline_errors)), 5),
      "current_mapper_mae": round(baseline_mae, 5),
      "mae_change_vs_current": round(fitted_mae - baseline_mae, 5),
    })
  return {"eligible_routes": eligible_routes, "results": results}


def route_key(path: Path) -> str:
  match = ROUTE_RE.search(str(path))
  return match.group(1) if match else path.parent.name


def segment_number(path: Path) -> int:
  match = re.search(r"--(\d+)(?:\\|/)", str(path.parent) + os.sep)
  return int(match.group(1)) if match else 0


def decode_command(dat: bytes) -> dict[str, float | bool] | None:
  if len(dat) != 8 or dat[0] != 0xC1 or dat[1] != 1:
    return None
  stock_raw = ((dat[2] & 0x1F) << 8) | dat[3]
  output_raw = ((dat[4] & 0x1F) << 8) | dat[5]
  stock_accel_raw = ((dat[6] & 0xF) << 8) | dat[7]
  return {
    "stock_engine": bool(dat[2] & 0x80),
    "stock_torque": stock_raw * 0.25 - 500.0,
    "output_engine": bool(dat[4] & 0x80),
    "output_torque": output_raw * 0.25 - 500.0,
    "stock_available": bool(dat[6] & 0x10),
    "stock_active": bool(dat[6] & 0x20),
    "stock_accel": stock_accel_raw * 0.004885 - 16.0,
  }


def decode_status(dat: bytes) -> dict[str, int | bool] | None:
  if len(dat) != 4 or dat[0] & 0xF0 != 0xB0:
    return None
  return {
    "applied": bool(dat[0] & 0x1),
    "host_requested": bool(dat[0] & 0x2),
    "brake_requested": bool(dat[0] & 0x4),
    "engine_requested": bool(dat[0] & 0x8),
    "failure_mask": dat[1] | (dat[2] << 8),
  }


def recorded_mode(command: dict, status: dict) -> tuple[str, str]:
  if status["applied"]:
    if status["brake_requested"]:
      return "experimental", "brake"
    if status["engine_requested"] and command["output_engine"]:
      return "experimental", "engine"
    return "experimental", "coast"
  if command["stock_active"]:
    if command["stock_engine"]:
      return "factory", "engine"
    if command["stock_accel"] < 3.5:
      return "factory", "brake"
    return "factory", "coast"
  return "inactive", "inactive"


def transitions(rows: list[dict], mode_field: str, active_field: str) -> dict:
  entries = Counter()
  direct = 0
  reversals = 0
  previous_mode = None
  previous_time = None
  previous_route = None
  last_direction = None
  last_direction_time = None
  for row in rows:
    if not row[active_field]:
      previous_mode = None
      last_direction = None
      continue
    mode = row[mode_field]
    route = row["route"]
    timestamp = row["time_s"]
    if route != previous_route or (previous_time is not None and timestamp - previous_time > 0.2):
      previous_mode = None
      last_direction = None
    if mode != previous_mode:
      entries[mode] += 1
      if {mode, previous_mode} == {"engine", "brake"}:
        direct += 1
      if mode in ("engine", "brake"):
        if (last_direction is not None and mode != last_direction and
            last_direction_time is not None and timestamp - last_direction_time <= 3.0):
          reversals += 1
        last_direction = mode
        last_direction_time = timestamp
      previous_mode = mode
    previous_time = timestamp
    previous_route = route
  active_seconds = sum(row["dt_s"] for row in rows if row[active_field])
  return {
    "entries": dict(entries),
    "direct_engine_brake_transitions": direct,
    "engine_brake_reversals_within_3s": reversals,
    "entries_per_minute": round(sum(entries.values()) * 60.0 / active_seconds, 3) if active_seconds else None,
  }


def speed_crossings(rows: list[dict]) -> int:
  side = 0
  crossings = 0
  for row in rows:
    error = row["speed_error"]
    next_side = 1 if error > 0.25 else -1 if error < -0.25 else side
    if side and next_side != side:
      crossings += 1
    side = next_side
  return crossings


def brake_events(rows: list[dict], mode_field: str, active_field: str) -> dict:
  events = []
  current = []
  for row in rows:
    is_braking = row[active_field] and row[mode_field] == "brake"
    if is_braking and (not current or (
        row["route"] == current[-1]["route"] and row["time_s"] - current[-1]["time_s"] <= 0.2
    )):
      current.append(row)
    else:
      if current:
        events.append(current)
      current = [row] if is_braking else []
  if current:
    events.append(current)
  summaries = []
  for event in events:
    duration = sum(row["dt_s"] for row in event)
    if duration < 0.1:
      continue
    summaries.append({
      "route": event[0]["route"],
      "start_s": round(event[0]["time_s"], 3),
      "duration_s": round(duration, 3),
      "start_speed_mps": round(event[0]["speed_mps"], 3),
      "end_speed_mps": round(event[-1]["speed_mps"], 3),
      "min_speed_error_mps": round(min(row["speed_error"] for row in event), 3),
      "max_speed_error_mps": round(max(row["speed_error"] for row in event), 3),
      "min_plan_accel_mps2": round(min(row["plan_accel"] for row in event), 3),
      "min_controller_accel_mps2": round(min(row["requested_accel"] for row in event), 3),
      "min_candidate_input_mps2": round(min(row["candidate_input"] for row in event), 3),
      "min_actual_accel_mps2": round(min(row["actual_accel"] for row in event), 3),
      "has_lead": any(row["has_lead"] for row in event),
      "source": Counter(row["source"] for row in event).most_common(1)[0][0],
    })
  unexpected = [
    event for event in summaries
    if event["min_plan_accel_mps2"] > -0.1 and event["start_speed_mps"] >= 2.0
  ]
  return {
    "count": len(summaries),
    "unexpected_by_plan_count": len(unexpected),
    "unexpected_by_plan": sorted(unexpected, key=lambda item: item["min_actual_accel_mps2"])[:20],
  }


def summarize(rows: list[dict]) -> dict:
  recorded = [row for row in rows if row["recorded_active"]]
  candidate = [row for row in rows if row["candidate_active"]]
  factory = [row for row in recorded if row["profile"] == "factory"]
  experimental = [row for row in recorded if row["profile"] == "experimental"]
  steady = [
    row for row in recorded
    if row["source"] == "cruise" and not row["has_lead"] and row["speed_mps"] >= 8.0
  ]
  jerks = []
  for before, after in zip(recorded, recorded[1:]):
    if before["route"] == after["route"] and 0.005 <= after["time_s"] - before["time_s"] <= 0.05:
      jerks.append(abs(after["actual_accel"] - before["actual_accel"]) / (after["time_s"] - before["time_s"]))
  factory_torque_pairs = [
    row for row in candidate
    if row["profile"] == "factory" and row["recorded_mode"] == "engine"
    and row["candidate_mode"] == "engine"
  ]
  factory_brake_pairs = [
    row for row in candidate
    if row["profile"] == "factory" and row["recorded_mode"] == "brake"
    and row["candidate_mode"] == "brake"
  ]
  factory_engine_reference = [
    row for row in candidate
    if row["profile"] == "factory" and row["recorded_mode"] == "engine"
  ]
  factory_brake_reference = [
    row for row in candidate
    if row["profile"] == "factory" and row["recorded_mode"] == "brake"
  ]
  grade = [
    row for row in candidate
    if row["candidate_mode"] == "engine" and row["plan_accel"] >= 0.5
    and row["pitch_rad"] >= 0.03 and row["speed_mps"] >= 5.0
  ]
  candidate_torque_steps = []
  candidate_brake_steps = []
  for before, after in zip(candidate, candidate[1:]):
    if before["route"] == after["route"] and 0.005 <= after["time_s"] - before["time_s"] <= 0.05:
      candidate_torque_steps.append(abs(after["candidate_torque"] - before["candidate_torque"]))
      candidate_brake_steps.append(abs(after["candidate_brake"] - before["candidate_brake"]))
  active_seconds = sum(row["dt_s"] for row in recorded)
  candidate_seconds = sum(row["dt_s"] for row in candidate)
  speed_errors = [row["speed_error"] for row in steady]
  mode_agreement = [
    row["recorded_mode"] == row["candidate_mode"]
    for row in candidate if row["profile"] == "factory"
  ]
  return {
    "shadow_only": True,
    "actuation_frames": 0,
    "samples": len(rows),
    "recorded_active_seconds": round(active_seconds, 3),
    "candidate_active_seconds": round(candidate_seconds, 3),
    "profile_samples": dict(Counter(row["profile"] for row in recorded)),
    "recorded_modes": transitions(rows, "recorded_mode", "recorded_active"),
    "candidate_modes": transitions(rows, "candidate_mode", "candidate_active"),
    "recorded_braking": brake_events(rows, "recorded_mode", "recorded_active"),
    "candidate_braking": brake_events(rows, "candidate_mode", "candidate_active"),
    "measured_jerk_abs_mps3": distribution(jerks),
    "steady_cruise": {
      "samples": len(steady),
      "speed_error_mps": distribution(speed_errors),
      "speed_error_rms_mps": round(math.sqrt(statistics.fmean(value * value for value in speed_errors)), 4) if speed_errors else None,
      "hysteretic_speed_error_crossings": speed_crossings(steady),
      "crossings_per_minute": round(speed_crossings(steady) * 60.0 / sum(row["dt_s"] for row in steady), 3) if steady else None,
    },
    "factory_reference": {
      "samples": len(factory),
      "engine_torque_nm": distribution([row["recorded_torque"] for row in factory if row["recorded_mode"] == "engine"]),
      "brake_accel_mps2": distribution([row["recorded_brake"] for row in factory if row["recorded_mode"] == "brake"]),
      "planner_brake_correlation": correlation(
        [row["plan_accel"] for row in factory if row["recorded_mode"] == "brake"],
        [row["recorded_brake"] for row in factory if row["recorded_mode"] == "brake"],
      ),
    },
    "candidate_vs_factory": {
      "mode_comparison_samples": len(mode_agreement),
      "mode_agreement_fraction": round(sum(mode_agreement) / len(mode_agreement), 4) if mode_agreement else None,
      "engine_overlap_samples": len(factory_torque_pairs),
      "torque_mae_nm": round(statistics.fmean(
        abs(row["candidate_torque"] - row["recorded_torque"])
        for row in factory_torque_pairs
      ), 3) if factory_torque_pairs else None,
      "brake_overlap_samples": len(factory_brake_pairs),
      "brake_mae_mps2": round(statistics.fmean(
        abs(row["candidate_brake"] - row["recorded_brake"])
        for row in factory_brake_pairs
      ), 4) if factory_brake_pairs else None,
      "all_factory_engine_samples": len(factory_engine_reference),
      "all_factory_engine_torque_mae_nm": round(statistics.fmean(
        abs(row["candidate_torque"] - row["recorded_torque"])
        for row in factory_engine_reference
      ), 3) if factory_engine_reference else None,
      "all_factory_brake_samples": len(factory_brake_reference),
      "all_factory_brake_mae_mps2": round(statistics.fmean(
        abs(row["candidate_brake"] - row["recorded_brake"])
        for row in factory_brake_reference
      ), 4) if factory_brake_reference else None,
    },
    "factory_model_screen": {
      "engine_current_features": linear_fit(
        factory_engine_reference,
        ("candidate_input", "speed_mps", "candidate_filtered_pitch"),
        "recorded_torque",
      ),
      "engine_route_held_out": held_out_fit(
        factory_engine_reference,
        ("candidate_input", "speed_mps", "candidate_filtered_pitch"),
        "recorded_torque",
        "candidate_torque",
      ),
      "brake_current_features": linear_fit(
        factory_brake_reference, ("candidate_input",), "recorded_brake",
      ),
      "brake_route_held_out": held_out_fit(
        factory_brake_reference, ("candidate_input",), "recorded_brake",
        "candidate_brake",
      ),
    },
    "candidate_command_smoothness": {
      "torque_step_abs_nm": distribution(candidate_torque_steps),
      "brake_step_abs_mps2": distribution(candidate_brake_steps),
      "torque_total_variation_per_s": round(sum(candidate_torque_steps) / candidate_seconds, 3) if candidate_seconds else None,
      "brake_total_variation_per_s": round(sum(candidate_brake_steps) / candidate_seconds, 4) if candidate_seconds else None,
    },
    "grade_demand": {
      "samples": len(grade),
      "pitch_rad": distribution([row["pitch_rad"] for row in grade]),
      "plan_minus_actual_accel_mps2": distribution([row["plan_accel"] - row["actual_accel"] for row in grade]),
      "candidate_torque_nm": distribution([row["candidate_torque"] for row in grade]),
    },
    "experimental_reference": {
      "samples": len(experimental),
      "requested_minus_actual_accel_mps2": distribution([row["requested_accel"] - row["actual_accel"] for row in experimental]),
      "engine_torque_nm": distribution([row["recorded_torque"] for row in experimental if row["recorded_mode"] == "engine"]),
    },
  }


def replay_route(route: str, paths: list[Path]) -> list[dict]:
  rows = []
  state = None
  plan = None
  command = None
  status = None
  command_time = status_time = None
  last_control_time = None
  control_index = 0
  candidate = LONG.JeepLongitudinalShadow()
  for path in sorted(paths, key=segment_number):
    for msg in LogReader(str(path), sort_by_time=True):
      timestamp = msg.logMonoTime / 1e9
      kind = msg.which()
      if kind == "carState":
        state = msg.carState
        continue
      if kind == "longitudinalPlan":
        message = msg.longitudinalPlan
        plan = {
          "time": timestamp,
          "accel": finite(message.accels[0]) if len(message.accels) else 0.0,
          "speed": finite(message.speeds[0], finite(state.vEgo) if state is not None else 0.0) if len(message.speeds) else 0.0,
          "lead": bool(message.hasLead),
          "source": str(message.longitudinalPlanSource),
        }
        continue
      if kind == "can":
        for frame in msg.can:
          address = int(frame.address)
          if address == 0x4FE:
            decoded = decode_command(bytes(frame.dat))
            if decoded is not None:
              command = decoded
              command_time = timestamp
          elif address == 0x4FF:
            decoded = decode_status(bytes(frame.dat))
            if decoded is not None:
              status = decoded
              status_time = timestamp
        continue
      if kind != "carControl" or state is None or plan is None or command is None or status is None:
        continue
      control_index += 1
      if control_index % 2:
        continue
      if timestamp - plan["time"] > 0.25 or timestamp - command_time > 0.1 or timestamp - status_time > 0.1:
        continue
      if last_control_time is not None and timestamp - last_control_time > 0.2:
        candidate = LONG.JeepLongitudinalShadow()
      dt = 0.02 if last_control_time is None else min(0.05, max(0.0, timestamp - last_control_time))
      last_control_time = timestamp
      control = msg.carControl
      gear = str(state.gearShifter).lower()
      profile, rec_mode = recorded_mode(command, status)
      candidate_input = (
        plan["accel"] if profile == "factory" else finite(control.actuators.accel)
      )
      control_requested = (
        bool(control.longActive) if profile == "experimental"
        else bool(control.enabled) and profile == "factory"
      )
      eligible = (
        control_requested and bool(state.cruiseState.available)
        and not bool(state.accFaulted) and not bool(state.stockAeb)
        and not bool(state.gasPressed) and not bool(state.brakePressed)
        and gear in ("drive", "sport", "low", "eco", "manumatic")
        and not bool(state.doorOpen) and not bool(state.seatbeltUnlatched)
      )
      pitch = finite(control.orientationNED[1]) if len(control.orientationNED) > 1 else 0.0
      envelope = candidate.update(
        candidate_input, eligible=eligible,
        speed_mps=finite(state.vEgo), pitch_rad=pitch,
      )
      no_override = not state.gasPressed and not state.brakePressed
      rec_active = profile != "inactive" and no_override
      rec_torque = command["output_torque"] if profile == "experimental" else command["stock_torque"]
      rec_brake = finite(control.actuators.accel) if profile == "experimental" else command["stock_accel"]
      rows.append({
        "route": route, "time_s": timestamp, "dt_s": dt,
        "profile": profile, "recorded_active": rec_active,
        "recorded_mode": rec_mode, "recorded_torque": rec_torque,
        "recorded_brake": rec_brake,
        "candidate_active": eligible, "candidate_mode": envelope.command_mode,
        "candidate_torque": envelope.engine_torque_nm,
        "candidate_brake": envelope.brake_accel_mps2,
        "candidate_filtered_pitch": envelope.filtered_pitch_rad,
        "candidate_input": candidate_input,
        "requested_accel": finite(control.actuators.accel),
        "plan_accel": plan["accel"], "plan_speed": plan["speed"],
        "speed_mps": finite(state.vEgo), "speed_error": plan["speed"] - finite(state.vEgo),
        "actual_accel": finite(state.aEgo), "pitch_rad": pitch,
        "has_lead": plan["lead"], "source": plan["source"],
        "failure_mask": status["failure_mask"],
      })
  return rows


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("rlogs", nargs="+", type=Path)
  parser.add_argument("--json", required=True, type=Path)
  args = parser.parse_args()
  grouped = {}
  for path in args.rlogs:
    grouped.setdefault(route_key(path), []).append(path)
  route_rows = {route: replay_route(route, paths) for route, paths in grouped.items()}
  combined = [row for rows in route_rows.values() for row in rows]
  result = {
    "shadow_only": True,
    "actuation_frames": 0,
    "limitations": [
      "Recorded-trajectory replay cannot predict the closed-loop trajectory produced by a changed controller.",
      "White Panda diagnostic frames are observations, not proof of an isolated actuator topology.",
      "Candidate mapper output is hypothetical and b7o vehicle actuation remains compiled out.",
    ],
    "routes": {route: summarize(rows) for route, rows in route_rows.items()},
    "combined": summarize(combined),
  }
  args.json.parent.mkdir(parents=True, exist_ok=True)
  args.json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(result, indent=2))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())

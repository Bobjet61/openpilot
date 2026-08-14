#!/usr/bin/env python3
"""Read-only triage of b7p qlogs for steering and longitudinal events."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
import os
from pathlib import Path
import re
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


SEGMENT_RE = re.compile(r"(?P<route>[0-9a-f]{8}--[0-9a-f]{10})--(?P<segment>\d+)")


def finite(value, default=0.0):
  try:
    result = float(value)
  except (TypeError, ValueError, OverflowError):
    return default
  return result if math.isfinite(result) else default


def enum_name(value):
  return str(value).split(".")[-1]


def analyze(paths):
  route_samples = defaultdict(list)
  route_meta = defaultdict(lambda: {
    "segments": [], "calibrations": [], "events": Counter(),
    "alerts": Counter(), "fault_samples": 0,
  })

  for path in paths:
    match = SEGMENT_RE.search(str(path.parent))
    if match is None:
      continue
    route = match.group("route")
    segment = int(match.group("segment"))
    route_meta[route]["segments"].append(segment)
    first_ns = None
    latest_state = None
    latest_controls = None
    latest_output = None
    latest_calibration = None
    latest_model_curvature = 0.0
    latest_lane_probs = []

    for msg in LogReader(str(path), sort_by_time=True):
      if first_ns is None:
        first_ns = int(msg.logMonoTime)
      # Segment logs retain a route-start event, so logMonoTime is already
      # route-relative after subtracting the first timestamp. Adding the
      # segment index again double-counts elapsed time for segments > 0.
      time_s = (int(msg.logMonoTime) - first_ns) / 1e9
      which = msg.which()
      if which == "carState":
        latest_state = msg.carState
        for event in latest_state.events:
          route_meta[route]["events"][enum_name(event.name)] += 1
        if latest_state.steerFaultTemporary or latest_state.steerFaultPermanent:
          route_meta[route]["fault_samples"] += 1
      elif which == "controlsState":
        latest_controls = msg.controlsState
        if latest_controls.alertType:
          route_meta[route]["alerts"][str(latest_controls.alertType)] += 1
      elif which == "carOutput":
        latest_output = msg.carOutput
      elif which == "liveCalibration":
        latest_calibration = msg.liveCalibration
        rpy = tuple(round(finite(value), 6) for value in latest_calibration.rpyCalib)
        record = {
          "time_s": round(time_s, 3),
          "status": enum_name(latest_calibration.calStatus),
          "rpy": rpy,
          "valid_blocks": int(latest_calibration.validBlocks),
        }
        calibrations = route_meta[route]["calibrations"]
        if not calibrations or (
            record["status"] != calibrations[-1]["status"]
            or max(abs(a - b) for a, b in zip(record["rpy"], calibrations[-1]["rpy"])) > 0.002
        ):
          calibrations.append(record)
      elif which == "modelV2":
        model = msg.modelV2
        latest_model_curvature = finite(model.action.desiredCurvature)
        latest_lane_probs = [finite(value) for value in model.laneLineProbs]
      elif which == "carControl" and latest_state is not None:
        control = msg.carControl
        torque_state = None
        if latest_controls is not None:
          try:
            torque_state = latest_controls.lateralControlState.torqueState
          except Exception:
            torque_state = None
        orientation = list(control.orientationNED)
        sample = {
          "time_s": time_s,
          "segment": segment,
          "enabled": bool(control.enabled),
          "lat_active": bool(control.latActive),
          "long_active": bool(control.longActive),
          "v_ego": finite(latest_state.vEgo),
          "a_ego": finite(latest_state.aEgo),
          "angle": finite(latest_state.steeringAngleDeg),
          "angle_rate": finite(latest_state.steeringRateDeg),
          "yaw_rate": finite(latest_state.yawRate),
          "driver_torque": finite(latest_state.steeringTorque),
          "eps_torque": finite(latest_state.steeringTorqueEps),
          "steering_pressed": bool(latest_state.steeringPressed),
          "steer_fault": bool(latest_state.steerFaultTemporary or latest_state.steerFaultPermanent),
          "steer": finite(control.actuators.steer),
          "steer_can": finite(latest_output.actuatorsOutput.steerOutputCan) if latest_output else 0.0,
          "accel": finite(control.actuators.accel),
          "long_state": enum_name(control.actuators.longControlState),
          "gas_pressed": bool(latest_state.gasPressed),
          "brake_pressed": bool(latest_state.brakePressed),
          "buttons": [enum_name(event.type) for event in latest_state.buttonEvents],
          "cruise_enabled": bool(latest_state.cruiseState.enabled),
          "cruise_available": bool(latest_state.cruiseState.available),
          "standstill": bool(latest_state.standstill),
          "acc_fault": bool(latest_state.accFaulted),
          "desired_curvature": finite(latest_controls.desiredCurvature) if latest_controls else 0.0,
          "actual_curvature": finite(latest_controls.curvature) if latest_controls else 0.0,
          "model_curvature": latest_model_curvature,
          "lat_output": finite(torque_state.output) if torque_state else 0.0,
          "desired_lat_accel": finite(torque_state.desiredLateralAccel) if torque_state else 0.0,
          "actual_lat_accel": finite(torque_state.actualLateralAccel) if torque_state else 0.0,
          "lane_probs": latest_lane_probs,
          "calibration": tuple(finite(value) for value in latest_calibration.rpyCalib) if latest_calibration else (),
          "pitch": finite(orientation[1]) if len(orientation) > 1 else 0.0,
          "v_cruise": finite(control.vCruise, 255.0),
        }
        route_samples[route].append(sample)

  report = {"routes": {}, "steering_candidates": [], "stop_episodes": [], "weak_response_candidates": []}
  for route, samples in sorted(route_samples.items()):
    samples.sort(key=lambda sample: sample["time_s"])
    meta = route_meta[route]
    active = [sample for sample in samples if sample["enabled"]]
    lateral = [sample for sample in samples if sample["lat_active"]]
    longitudinal = [sample for sample in samples if sample["long_active"]]
    report["routes"][route] = {
      "segments": sorted(set(meta["segments"])),
      "duration_s": round(samples[-1]["time_s"], 3) if samples else 0.0,
      "samples": len(samples),
      "enabled_samples": len(active),
      "lateral_samples": len(lateral),
      "longitudinal_samples": len(longitudinal),
      "max_speed_mps": round(max((sample["v_ego"] for sample in samples), default=0.0), 3),
      "steer_fault_samples": meta["fault_samples"],
      "events": dict(meta["events"].most_common()),
      "alerts": dict(meta["alerts"].most_common()),
      "calibrations": meta["calibrations"],
    }

    prior = None
    for sample in samples:
      if prior is not None:
        dt = sample["time_s"] - prior["time_s"]
        if (
            sample["lat_active"] and prior["lat_active"]
            and sample["v_ego"] > 4.0 and 0.02 <= dt <= 0.25
        ):
          steer_rate = (sample["steer"] - prior["steer"]) / dt
          curvature_rate = (sample["desired_curvature"] - prior["desired_curvature"]) / dt
          score = abs(steer_rate) + abs(sample["angle_rate"]) / 100.0
          if abs(steer_rate) >= 0.6 or abs(sample["angle_rate"]) >= 40.0:
            report["steering_candidates"].append({
              "route": route,
              "time_s": round(sample["time_s"], 3),
              "segment": sample["segment"],
              "speed_mps": round(sample["v_ego"], 3),
              "steer": round(sample["steer"], 4),
              "steer_step": round(sample["steer"] - prior["steer"], 4),
              "steer_rate_s": round(steer_rate, 3),
              "angle_deg": round(sample["angle"], 3),
              "angle_rate_deg_s": round(sample["angle_rate"], 3),
              "desired_curvature": round(sample["desired_curvature"], 6),
              "curvature_rate_s": round(curvature_rate, 6),
              "model_curvature": round(sample["model_curvature"], 6),
              "driver_torque": round(sample["driver_torque"], 3),
              "steering_pressed": sample["steering_pressed"],
              "lane_probs": [round(value, 3) for value in sample["lane_probs"]],
              "calibration": [round(value, 6) for value in sample["calibration"]],
              "score": round(score, 3),
            })
      prior = sample

    stopped = []
    for sample in samples + [None]:
      qualifies = sample is not None and sample["v_ego"] < 0.12
      if qualifies:
        if not stopped or sample["time_s"] - stopped[-1]["time_s"] <= 0.5:
          stopped.append(sample)
        else:
          stopped = [sample]
      elif stopped:
        duration = stopped[-1]["time_s"] - stopped[0]["time_s"]
        if duration >= 1.0:
          after = [s for s in samples if stopped[-1]["time_s"] < s["time_s"] <= stopped[-1]["time_s"] + 10.0]
          moving = next((s for s in after if s["v_ego"] > 0.5), None)
          report["stop_episodes"].append({
            "route": route,
            "start_s": round(stopped[0]["time_s"], 3),
            "end_s": round(stopped[-1]["time_s"], 3),
            "duration_s": round(duration, 3),
            "max_requested_accel": round(max(s["accel"] for s in stopped), 3),
            "long_states": dict(Counter(s["long_state"] for s in stopped)),
            "enabled_samples": sum(s["enabled"] for s in stopped),
            "long_active_samples": sum(s["long_active"] for s in stopped),
            "cruise_enabled_samples": sum(s["cruise_enabled"] for s in stopped),
            "acc_fault_samples": sum(s["acc_fault"] for s in stopped),
            "brake_pressed_samples": sum(s["brake_pressed"] for s in stopped),
            "gas_pressed_samples": sum(s["gas_pressed"] for s in stopped),
            "button_events": dict(Counter(
              button for s in stopped for button in s["buttons"]
            )),
            "button_events_after_stop": dict(Counter(
              button for s in after for button in s["buttons"]
            )),
            "launched_within_10s": moving is not None,
            "launch_delay_s": round(moving["time_s"] - stopped[-1]["time_s"], 3) if moving else None,
          })
        stopped = []

    weak = []
    for sample in samples:
      if (
          sample["long_active"] and sample["v_ego"] > 2.0
          and sample["accel"] >= 1.2 and sample["a_ego"] < 0.2
          and not sample["gas_pressed"] and not sample["brake_pressed"]
      ):
        weak.append(sample)
    for sample in weak:
      report["weak_response_candidates"].append({
        "route": route,
        "time_s": round(sample["time_s"], 3),
        "segment": sample["segment"],
        "speed_mps": round(sample["v_ego"], 3),
        "actual_accel": round(sample["a_ego"], 3),
        "requested_accel": round(sample["accel"], 3),
        "pitch": round(sample["pitch"], 5),
        "v_cruise": round(sample["v_cruise"], 2),
      })

  report["steering_candidates"].sort(key=lambda event: event["score"], reverse=True)
  report["steering_candidates"] = report["steering_candidates"][:80]
  report["weak_response_candidates"] = report["weak_response_candidates"][:200]
  return report


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("logs", nargs="+", type=Path)
  parser.add_argument("--json", type=Path, required=True)
  args = parser.parse_args()
  report = analyze(args.logs)
  args.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(report, indent=2))


if __name__ == "__main__":
  main()

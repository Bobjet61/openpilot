#!/usr/bin/env python3
"""Read-only event timeline for a narrow b7p rlog window."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
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


def enum_name(value):
  return str(value).split(".")[-1]


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("rlog", type=Path)
  parser.add_argument("--start", type=float, required=True)
  parser.add_argument("--end", type=float, required=True)
  parser.add_argument("--json", type=Path, required=True)
  args = parser.parse_args()

  first_ns = None
  last_state = None
  last_control = None
  last_controls = None
  last_output = None
  last_model = None
  last_location = None
  rows = []
  messages = []
  last_row_t = -1e9
  last_signature = None

  for msg in LogReader(str(args.rlog), sort_by_time=True):
    if first_ns is None:
      first_ns = int(msg.logMonoTime)
    t = (int(msg.logMonoTime) - first_ns) / 1e9
    if t < args.start - 2.0 or t > args.end + 2.0:
      continue
    which = msg.which()
    if which == "carState":
      last_state = msg.carState
    elif which == "carControl":
      last_control = msg.carControl
    elif which == "controlsState":
      last_controls = msg.controlsState
    elif which == "carOutput":
      last_output = msg.carOutput
    elif which == "modelV2":
      last_model = msg.modelV2
    elif which == "liveLocationKalman":
      last_location = msg.liveLocationKalman
    elif which == "logMessage":
      try:
        text = bytes(msg.logMessage).decode("utf-8", errors="replace")
      except TypeError:
        text = str(msg.logMessage)
      if any(token in text for token in ("B6Y hold", "Jeep long", "ACC", "fault")):
        try:
          decoded = json.loads(text)
          text = decoded.get("msg", text)
        except (json.JSONDecodeError, TypeError):
          pass
        messages.append({"time_s": round(t, 6), "text": text})

    if which != "carState" or last_state is None:
      continue
    buttons = [
      {"type": enum_name(button.type), "pressed": bool(button.pressed)}
      for button in last_state.buttonEvents
    ]
    events = [enum_name(event.name) for event in last_state.events]
    signature = (
      bool(last_state.standstill), bool(last_state.cruiseState.enabled),
      bool(last_state.cruiseState.standstill), bool(last_state.gasPressed),
      bool(last_state.brakePressed), tuple((b["type"], b["pressed"]) for b in buttons),
      tuple(events),
      bool(last_control.enabled) if last_control is not None else False,
      bool(last_control.longActive) if last_control is not None else False,
      bool(last_control.cruiseControl.resume) if last_control is not None else False,
      bool(last_control.cruiseControl.cancel) if last_control is not None else False,
      str(last_controls.alertType) if last_controls is not None else "",
    )
    if t - last_row_t < 0.1 and signature == last_signature:
      continue
    if args.start <= t <= args.end:
      rows.append({
        "time_s": round(t, 6),
        "v_ego_mps": round(float(last_state.vEgo), 4),
        "a_ego_mps2": round(float(last_state.aEgo), 4),
        "standstill": bool(last_state.standstill),
        "cruise_enabled": bool(last_state.cruiseState.enabled),
        "cruise_standstill": bool(last_state.cruiseState.standstill),
        "gas": bool(last_state.gasPressed),
        "brake": bool(last_state.brakePressed),
        "buttons": buttons,
        "events": events,
        "enabled": bool(last_control.enabled) if last_control is not None else None,
        "long_active": bool(last_control.longActive) if last_control is not None else None,
        "resume_request": bool(last_control.cruiseControl.resume) if last_control is not None else None,
        "cancel_request": bool(last_control.cruiseControl.cancel) if last_control is not None else None,
        "requested_accel": round(float(last_control.actuators.accel), 4) if last_control is not None else None,
        "requested_steer": round(float(last_control.actuators.steer), 5) if last_control is not None else None,
        "requested_curvature": round(float(last_control.actuators.curvature), 7) if last_control is not None else None,
        "output_steer_can": int(last_output.actuatorsOutput.steerOutputCan) if last_output is not None else None,
        "controls_desired_curvature": round(float(last_controls.desiredCurvature), 7) if last_controls is not None else None,
        "model_desired_curvature": round(float(last_model.action.desiredCurvature), 7) if last_model is not None else None,
        "lane_probs": [round(float(value), 4) for value in last_model.laneLineProbs] if last_model is not None else [],
        "lane_stds": [round(float(value), 4) for value in last_model.laneLineStds] if last_model is not None else [],
        "road_edge_stds": [round(float(value), 4) for value in last_model.roadEdgeStds] if last_model is not None else [],
        "path_std_near": (
          round(float(last_model.position.yStd[5]), 4)
          if last_model is not None and len(last_model.position.yStd) > 5 else None
        ),
        "path_std_far": (
          round(float(last_model.position.yStd[15]), 4)
          if last_model is not None and len(last_model.position.yStd) > 15 else None
        ),
        "calibrated_yaw_rate": (
          round(float(last_location.angularVelocityCalibrated.value[2]), 6)
          if last_location is not None and len(last_location.angularVelocityCalibrated.value) >= 3 else None
        ),
        "alert": str(last_controls.alertType) if last_controls is not None else "",
      })
      last_row_t = t
      last_signature = signature

  result = {"rows": rows, "messages": messages}
  args.json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(result, indent=2))


if __name__ == "__main__":
  main()

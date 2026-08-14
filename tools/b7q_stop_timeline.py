#!/usr/bin/env python3
"""Build a compact timeline around a b7q experimental stop/go event."""

from __future__ import annotations

import argparse
import json
import math
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


def finite(value, default=0.0):
  try:
    result = float(value)
  except (TypeError, ValueError, OverflowError):
    return default
  return result if math.isfinite(result) else default


def enum_name(value):
  return str(value).split(".")[-1]


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("log", type=Path)
  parser.add_argument("--start", type=float, default=105.0)
  parser.add_argument("--end", type=float, default=135.0)
  parser.add_argument("--step", type=float, default=0.25)
  parser.add_argument("--json", type=Path, required=True)
  args = parser.parse_args()

  first_ns = None
  state = None
  control = None
  plan = None
  diag = {"status": None, "failure": None, "counters": None,
          "stock_engine_word": None, "output_engine_word": None,
          "owner": None, "stock": None, "stock_fault": None}
  next_sample = args.start
  rows = []

  for msg in LogReader(str(args.log), sort_by_time=True):
    if first_ns is None:
      first_ns = int(msg.logMonoTime)
    t = (int(msg.logMonoTime) - first_ns) / 1e9
    which = msg.which()

    if which == "carState":
      state = msg.carState
    elif which == "carControl":
      control = msg.carControl
    elif which == "longitudinalPlan":
      plan = msg.longitudinalPlan
    elif which == "can":
      for frame in msg.can:
        if int(frame.src) != 0:
          continue
        dat = bytes(frame.dat)
        if int(frame.address) == 0x4FF and len(dat) >= 4:
          diag["status"] = dat[0]
          diag["failure"] = dat[1] | (dat[2] << 8)
          diag["counters"] = dat[3]
        elif int(frame.address) == 0x4FE and len(dat) >= 6 and dat[0] == 0xC1 and dat[1] == 1:
          diag["stock_engine_word"] = (dat[2] << 8) | dat[3]
          diag["output_engine_word"] = (dat[4] << 8) | dat[5]
        elif int(frame.address) == 0x4FD and len(dat) >= 3:
          diag["owner"], diag["stock"], diag["stock_fault"] = dat[0], dat[1], dat[2]

    if t < args.start or state is None or control is None:
      continue
    if t > args.end:
      break
    if t + 1e-9 < next_sample:
      continue

    accels = list(plan.accels) if plan is not None else []
    speeds = list(plan.speeds) if plan is not None else []
    status = diag["status"]
    owner = diag["owner"]
    rows.append({
      "time_s": round(t, 3),
      "v_ego": round(finite(state.vEgo), 4),
      "a_ego": round(finite(state.aEgo), 4),
      "physical_standstill": bool(state.standstill),
      "cruise_standstill": bool(state.cruiseState.standstill),
      "cruise_enabled": bool(state.cruiseState.enabled),
      "acc_faulted": bool(state.accFaulted),
      "gas_pressed": bool(state.gasPressed),
      "brake_pressed": bool(state.brakePressed),
      "long_active": bool(control.longActive),
      "long_state": enum_name(control.actuators.longControlState),
      "requested_accel": round(finite(control.actuators.accel), 4),
      "plan_accel_0": round(finite(accels[0]), 4) if accels else None,
      "plan_speed_0": round(finite(speeds[0]), 4) if speeds else None,
      # ModelConstants.T_IDXS[10] is 0.9766 seconds, close enough to the
      # v_target_1sec signal used by LongControl's launch transition.
      "plan_speed_1s": round(finite(speeds[10]), 4) if len(speeds) > 10 else None,
      "wp_status_hex": f"0x{status:02x}" if status is not None else None,
      "wp_applied": bool(status & 0x1) if status is not None else None,
      "wp_host_requested": bool(status & 0x2) if status is not None else None,
      "wp_brake_requested": bool(status & 0x4) if status is not None else None,
      "wp_engine_requested": bool(status & 0x8) if status is not None else None,
      "wp_failure_hex": f"0x{diag['failure']:04x}" if diag["failure"] is not None else None,
      "wp_private_counter": (diag["counters"] >> 4) if diag["counters"] is not None else None,
      "wp_stock_counter": (diag["counters"] & 0xF) if diag["counters"] is not None else None,
      "wp_stock_engine_active": bool(diag["stock_engine_word"] & 0x8000) if diag["stock_engine_word"] is not None else None,
      "wp_stock_engine_torque_nm": round((diag["stock_engine_word"] & 0x1FFF) * 0.25 - 500.0, 2) if diag["stock_engine_word"] is not None else None,
      "wp_output_engine_active": bool(diag["output_engine_word"] & 0x8000) if diag["output_engine_word"] is not None else None,
      "wp_output_engine_torque_nm": round((diag["output_engine_word"] & 0x1FFF) * 0.25 - 500.0, 2) if diag["output_engine_word"] is not None else None,
      "wp_owner": (owner & 0xF) if owner is not None and (owner & 0xF0) == 0xD0 else None,
      "wp_stock_hex": f"0x{diag['stock']:02x}" if diag["stock"] is not None else None,
      "wp_stock_fault_hex": f"0x{diag['stock_fault']:02x}" if diag["stock_fault"] is not None else None,
    })
    while next_sample <= t:
      next_sample += args.step

  args.json.write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
  transitions = []
  previous = None
  keys = ("physical_standstill", "cruise_standstill", "acc_faulted", "long_state",
          "wp_status_hex", "wp_failure_hex", "wp_owner", "gas_pressed", "brake_pressed")
  for row in rows:
    signature = tuple(row[key] for key in keys)
    if signature != previous:
      transitions.append(row)
      previous = signature
  print(json.dumps({"rows": len(rows), "transitions": transitions}, indent=2))


if __name__ == "__main__":
  main()

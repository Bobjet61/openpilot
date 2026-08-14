#!/usr/bin/env python3
"""Trace Jeep stop/go messages across host sendcan and observed CAN buses."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
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


DAS_3 = 0x1F4
CRUISE_BUTTONS = 0x23B
RELEVANT = {DAS_3, CRUISE_BUTTONS, 0x1F6, 0x1F7, 0x272}


def analyze(paths):
  report = {}
  for path in paths:
    first_ns = None
    car_state = None
    car_control = None
    counts = Counter()
    payloads = defaultdict(Counter)
    stop_frames = []
    panda_snapshots = []
    for msg in LogReader(str(path), sort_by_time=True):
      if first_ns is None:
        first_ns = int(msg.logMonoTime)
      t = (int(msg.logMonoTime) - first_ns) / 1e9
      which = msg.which()
      if which == "carState":
        car_state = msg.carState
      elif which == "carControl":
        car_control = msg.carControl
      elif which == "pandaStates":
        panda_snapshots.append({
          "time_s": round(t, 3),
          "states": [{
            "type": str(p.pandaType),
            "safety_model": str(p.safetyModel),
            "safety_param": int(p.safetyParam),
            "tx_blocked": int(p.safetyTxBlocked),
            "rx_invalid": int(p.safetyRxInvalid),
            "rx_checks_invalid": bool(p.safetyRxChecksInvalid),
          } for p in msg.pandaStates],
        })
      elif which in ("can", "sendcan"):
        frames = getattr(msg, which)
        for frame in frames:
          address = int(frame.address)
          if address not in RELEVANT:
            continue
          bus = int(frame.src)
          dat = bytes(frame.dat).hex()
          counts[(which, address, bus)] += 1
          payloads[(which, address, bus)][dat] += 1
          stopped = car_state is not None and float(car_state.vEgo) < 0.12
          engaged = car_control is not None and bool(car_control.enabled)
          if stopped and engaged:
            stop_frames.append({
              "time_s": round(t, 3),
              "stream": which,
              "address": hex(address),
              "bus": bus,
              "data": dat,
              "v_ego": round(float(car_state.vEgo), 5),
              "standstill": bool(car_state.standstill),
              "cruise_available": bool(car_state.cruiseState.available),
              "cruise_enabled": bool(car_state.cruiseState.enabled),
              "long_active": bool(car_control.longActive),
            })
    report[str(path)] = {
      "counts": [
        {"stream": key[0], "address": hex(key[1]), "bus": key[2], "count": value}
        for key, value in sorted(counts.items())
      ],
      "top_payloads": [
        {
          "stream": key[0], "address": hex(key[1]), "bus": key[2],
          "payloads": [{"data": dat, "count": count} for dat, count in counter.most_common(8)],
        }
        for key, counter in sorted(payloads.items())
      ],
      "panda_snapshots": panda_snapshots[::max(1, len(panda_snapshots) // 10)],
      "stop_frames": stop_frames,
    }
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

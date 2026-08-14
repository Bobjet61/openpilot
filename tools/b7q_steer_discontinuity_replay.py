#!/usr/bin/env python3
"""Offline screening of a first-event Jeep steering discontinuity guard."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
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


STEER_MAX = 261
STEER_ERROR_MAX = 80
NORMAL_DELTA = 5
GUARD_DELTA = 3
LOW_CONFIDENCE = 0.40
ENTER_CURVATURE = 0.0008
ESTABLISHED_CURVATURE = 0.0015
ESTABLISHED_CYCLES = 5
GUARD_CYCLES = 50
GUARD_MAX_RAW = 100
MIN_SPEED_MPS = 8.0


def clip(value, lower, upper):
  return min(max(value, lower), upper)


def sign(value):
  return (value > 0) - (value < 0)


def limited_target(requested, previous, eps, delta):
  max_allowed = min(max(eps + STEER_ERROR_MAX, STEER_ERROR_MAX), STEER_MAX)
  min_allowed = max(min(eps - STEER_ERROR_MAX, -STEER_ERROR_MAX), -STEER_MAX)
  target = clip(requested, min_allowed, max_allowed)
  if previous > 0:
    lower = max(previous - delta, -delta)
    upper = previous + delta
  else:
    lower = previous - delta
    upper = min(previous + delta, delta)
  return round(clip(target, lower, upper))


@dataclass
class Guard:
  established_sign: int = 0
  establishing_sign: int = 0
  establishing_cycles: int = 0
  remaining: int = 0

  def update(self, curvature, lane_confidence):
    current_sign = sign(curvature) if abs(curvature) >= ENTER_CURVATURE else 0
    low_confidence = lane_confidence < LOW_CONFIDENCE
    activated = False
    if (
        self.established_sign != 0 and current_sign != 0
        and current_sign != self.established_sign and low_confidence
    ):
      self.remaining = GUARD_CYCLES
      self.established_sign = current_sign
      self.establishing_sign = current_sign
      self.establishing_cycles = 0
      activated = True
    elif self.remaining > 0:
      self.remaining -= 1

    strong_sign = sign(curvature) if abs(curvature) >= ESTABLISHED_CURVATURE else 0
    if strong_sign == 0:
      self.establishing_sign = 0
      self.establishing_cycles = 0
    elif strong_sign == self.establishing_sign:
      self.establishing_cycles += 1
    else:
      self.establishing_sign = strong_sign
      self.establishing_cycles = 1
    if self.establishing_cycles >= ESTABLISHED_CYCLES and self.remaining == 0:
      self.established_sign = self.establishing_sign
    return self.remaining > 0, activated


def analyze(path):
  first_ns = None
  state = None
  model = None
  output = None
  previous_sample_ns = -10**30
  previous_candidate = 0
  guard = Guard()
  activations = []
  affected = 0
  max_reduction = 0
  max_baseline = 0
  max_candidate = 0

  for msg in LogReader(str(path), sort_by_time=True):
    if first_ns is None:
      first_ns = int(msg.logMonoTime)
    which = msg.which()
    if which == "carState":
      state = msg.carState
    elif which == "modelV2":
      model = msg.modelV2
    elif which == "carOutput":
      output = msg.carOutput
    if which != "carControl":
      continue

    if state is None or model is None:
      continue

    # Steering commands are emitted at 50 Hz. Downsample carControl's 100 Hz
    # stream to the same period for an identical-input candidate comparison.
    now_ns = int(msg.logMonoTime)
    if now_ns - previous_sample_ns < 15_000_000:
      continue
    previous_sample_ns = now_ns
    control = msg.carControl
    active = bool(control.latActive) and not bool(state.steeringPressed) and float(state.vEgo) >= MIN_SPEED_MPS
    if not active:
      previous_candidate = 0
      guard = Guard()
      continue

    requested = round(float(control.actuators.steer) * STEER_MAX)
    curvature = float(control.actuators.curvature)
    lanes = list(model.laneLineProbs)
    left_lane_probability = float(lanes[1]) if len(lanes) >= 3 else 0.0
    right_lane_probability = float(lanes[2]) if len(lanes) >= 3 else 0.0
    lane_confidence = max(left_lane_probability, right_lane_probability)
    guarded, activated = guard.update(curvature, lane_confidence)
    guarded_request = int(clip(requested, -GUARD_MAX_RAW, GUARD_MAX_RAW)) if guarded else requested
    candidate = limited_target(
      guarded_request,
      previous_candidate,
      float(state.steeringTorqueEps),
      GUARD_DELTA if guarded else NORMAL_DELTA,
    )
    baseline = int(output.actuatorsOutput.steerOutputCan) if output is not None else 0
    if guarded:
      affected += 1
      max_reduction = max(max_reduction, abs(baseline) - abs(candidate))
    if activated:
      activations.append({
        "time_s": round((now_ns - first_ns) / 1e9, 3),
        "speed_mps": round(float(state.vEgo), 3),
        "curvature": round(curvature, 7),
        "left_lane_probability": round(left_lane_probability, 3),
        "right_lane_probability": round(right_lane_probability, 3),
        "lane_confidence": round(lane_confidence, 3),
        "requested_raw": requested,
        "baseline_raw": baseline,
        "candidate_raw": candidate,
      })
    max_baseline = max(max_baseline, abs(baseline))
    max_candidate = max(max_candidate, abs(candidate))
    previous_candidate = candidate

  return {
    "path": str(path),
    "activation_count": len(activations),
    "activations": activations,
    "affected_samples": affected,
    "affected_s": round(affected / 50.0, 3),
    "max_baseline_raw": max_baseline,
    "max_candidate_raw": max_candidate,
    "max_abs_reduction_raw": max_reduction,
  }


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("rlogs", nargs="+", type=Path)
  parser.add_argument("--json", type=Path, required=True)
  args = parser.parse_args()
  report = {
    "non_actuating": True,
    "parameters": {
      "low_confidence": LOW_CONFIDENCE,
      "enter_curvature": ENTER_CURVATURE,
      "established_curvature": ESTABLISHED_CURVATURE,
      "guard_duration_s": GUARD_CYCLES / 50.0,
      "guard_max_raw": GUARD_MAX_RAW,
      "guard_delta": GUARD_DELTA,
    },
    "files": [analyze(path) for path in args.rlogs],
  }
  args.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(report, indent=2))


if __name__ == "__main__":
  main()

#!/usr/bin/env python3
"""Replay logs through b7o's non-actuating single-owner handoff model."""

import argparse
from collections import Counter
import importlib.util
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


MODULE_PATH = (
  Path(__file__).resolve().parents[1] / "selfdrive" / "car" / "chrysler"
  / "jeep_single_owner_shadow.py"
)
SPEC = importlib.util.spec_from_file_location("b7o_owner_replay_model", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODEL)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument(
    "--assume-stock-isolated", action="store_true",
    help="counterfactual replay with a healthy exclusive stock-command isolation boundary",
  )
  parser.add_argument("logs", nargs="+", type=Path)
  args = parser.parse_args()

  messages = []
  for path in args.logs:
    messages.extend(LogReader(str(path)))
  messages.sort(key=lambda msg: msg.logMonoTime)
  if not messages:
    raise SystemExit("no messages")

  first = messages[0].logMonoTime
  latest_control = None
  latest_plan_accel = 0.0
  shadow = MODEL.JeepSingleOwnerShadow()
  owners = Counter()
  reasons = Counter()
  owner_transition_counts = Counter()
  transitions = []
  previous_owner = None
  shadow_brake_samples = 0
  shadow_accel_samples = 0
  candidate_brake_samples = 0
  candidate_accel_samples = 0
  eligible_candidate_samples = 0

  for msg in messages:
    kind = msg.which()
    if kind == "carControl":
      latest_control = msg.carControl
      continue
    if kind == "longitudinalPlan":
      accels = msg.longitudinalPlan.accels
      if len(accels):
        latest_plan_accel = float(accels[0])
      continue
    if kind != "carState" or latest_control is None:
      continue

    state = msg.carState
    gear = str(state.gearShifter).lower()
    sample = MODEL.SingleOwnerInput(
      monotonic_time_s=(msg.logMonoTime - first) / 1e9,
      requested=True,
      controls_enabled=bool(latest_control.enabled),
      stock_available=bool(state.cruiseState.available),
      # cruiseState.enabled is an engagement indication, not proof that stock
      # actuator commands are absent. Recorded b7f routes therefore cannot
      # establish isolation. Counterfactual isolated replay assumes the stock
      # command path is physically quiet behind a healthy isolation boundary.
      stock_isolated=bool(args.assume_stock_isolated),
      isolation_healthy=True,
      stock_command_observed=False,
      stock_fault=bool(state.accFaulted),
      collision=bool(state.stockAeb),
      gas_pressed=bool(state.gasPressed),
      brake_pressed=bool(state.brakePressed),
      forward_gear=gear in ("drive", "sport", "low", "eco", "manumatic"),
      door_open=bool(state.doorOpen),
      seatbelt_unlatched=bool(state.seatbeltUnlatched),
      speed_mps=float(state.vEgo),
      # Factory-long carControl has no longitudinal actuator output. Replay
      # the production planner's immediate acceleration target instead.
      requested_accel_mps2=latest_plan_accel,
    )
    result = shadow.update(sample)
    owners[result.owner.name] += 1
    reasons[result.reason] += 1
    shadow_brake_samples += int(result.would_brake)
    shadow_accel_samples += int(result.would_accelerate)
    candidate_eligible = (
      sample.controls_enabled and sample.stock_available
      and not sample.stock_fault and not sample.collision
      and not sample.gas_pressed and not sample.brake_pressed
      and sample.forward_gear and not sample.door_open
      and not sample.seatbelt_unlatched
      and math.isfinite(sample.speed_mps)
      and math.isfinite(sample.requested_accel_mps2)
    )
    eligible_candidate_samples += int(candidate_eligible)
    candidate_brake_samples += int(candidate_eligible and sample.requested_accel_mps2 < -0.05)
    candidate_accel_samples += int(candidate_eligible and sample.requested_accel_mps2 > 0.05)
    if result.owner != previous_owner:
      if previous_owner is not None:
        owner_transition_counts[f"{previous_owner.name}->{result.owner.name}"] += 1
      transitions.append({
        "time_s": round((msg.logMonoTime - first) / 1e9, 3),
        "owner": result.owner.name,
        "reason": result.reason,
        "speed_mps": round(sample.speed_mps, 3),
        "cruise_engaged": bool(state.cruiseState.enabled),
      })
      previous_owner = result.owner

  print(json.dumps({
    "shadow_only": True,
    "topology": "counterfactual_stock_isolated" if args.assume_stock_isolated else "recorded_unproven_isolation",
    "actuation_frames": 0,
    "duration_s": round((messages[-1].logMonoTime - first) / 1e9, 3),
    "owner_samples": dict(owners),
    "reason_samples": dict(reasons),
    "owner_transition_counts": dict(owner_transition_counts),
    "shadow_entry_count": sum(
      count for transition, count in owner_transition_counts.items()
      if transition.endswith("->OPENPILOT_SHADOW")
    ),
    "would_brake_samples": shadow_brake_samples,
    "would_accelerate_samples": shadow_accel_samples,
    "eligible_candidate_samples": eligible_candidate_samples,
    "candidate_would_brake_samples": candidate_brake_samples,
    "candidate_would_accelerate_samples": candidate_accel_samples,
    "transitions": transitions,
  }, indent=2))


if __name__ == "__main__":
  main()

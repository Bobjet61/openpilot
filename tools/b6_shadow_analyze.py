#!/usr/bin/env python3
"""Offline, non-actuating analysis of Jeep stock ACC and openpilot longitudinal plans.

This tool reads existing openpilot rlog/qlog files. It does not run on the
vehicle control path and cannot transmit CAN messages or command the vehicle.
"""

from __future__ import annotations

import argparse
import csv
import importlib
import json
import math
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Iterable


CSV_FIELDS = [
  "t_rel_s",
  "log_mono_time_ns",
  "v_ego_mps",
  "a_ego_mps2",
  "standstill",
  "gas_pressed",
  "brake_pressed",
  "cruise_available",
  "cruise_enabled",
  "cruise_standstill",
  "cruise_set_speed_mps",
  "brake_hold_active",
  "plan_age_s",
  "plan_has_lead",
  "plan_v0_mps",
  "plan_a0_mps2",
  "radar_age_s",
  "lead_status",
  "lead_d_rel_m",
  "lead_v_rel_mps",
  "lead_a_rel_mps2",
  "lead_v_lead_mps",
]


def _safe_float(value: Any, default: float = math.nan) -> float:
  try:
    return float(value)
  except (TypeError, ValueError, OverflowError):
    return default


def _safe_bool(value: Any, default: bool = False) -> bool:
  try:
    return bool(value)
  except Exception:
    return default


def _first(sequence: Any, default: float = math.nan) -> float:
  try:
    return _safe_float(sequence[0], default) if len(sequence) else default
  except Exception:
    return default


def _attr(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


@dataclass
class HoldEpisode:
  start_s: float
  end_s: float | None = None
  duration_s: float | None = None
  lead_depart_s: float | None = None
  plan_positive_accel_s: float | None = None
  vehicle_move_s: float | None = None
  start_lead_distance_m: float | None = None
  max_plan_accel_mps2: float | None = None
  max_actual_accel_mps2: float | None = None
  min_actual_accel_mps2: float | None = None
  ended_while_active: bool = False


class EpisodeTracker:
  def __init__(self) -> None:
    self.episodes: list[HoldEpisode] = []
    self.current: HoldEpisode | None = None
    self.pending_move: HoldEpisode | None = None
    self.previous_hold = False

  @staticmethod
  def _finite(value: float) -> bool:
    return math.isfinite(value)

  def update(self, row: dict[str, Any]) -> None:
    t = float(row["t_rel_s"])
    hold = bool(row["brake_hold_active"])
    plan_a = _safe_float(row["plan_a0_mps2"])
    actual_a = _safe_float(row["a_ego_mps2"])
    v_ego = _safe_float(row["v_ego_mps"])
    lead_v_rel = _safe_float(row["lead_v_rel_mps"])
    lead_distance = _safe_float(row["lead_d_rel_m"])

    if hold and not self.previous_hold:
      self.current = HoldEpisode(
        start_s=t,
        start_lead_distance_m=lead_distance if self._finite(lead_distance) else None,
      )
      self.pending_move = None

    if self.current is not None:
      ep = self.current
      if self._finite(plan_a):
        ep.max_plan_accel_mps2 = plan_a if ep.max_plan_accel_mps2 is None else max(ep.max_plan_accel_mps2, plan_a)
        if ep.plan_positive_accel_s is None and plan_a > 0.15:
          ep.plan_positive_accel_s = t
      if self._finite(actual_a):
        ep.max_actual_accel_mps2 = actual_a if ep.max_actual_accel_mps2 is None else max(ep.max_actual_accel_mps2, actual_a)
        ep.min_actual_accel_mps2 = actual_a if ep.min_actual_accel_mps2 is None else min(ep.min_actual_accel_mps2, actual_a)
      if ep.lead_depart_s is None and bool(row["lead_status"]) and self._finite(lead_v_rel) and lead_v_rel > 0.30:
        ep.lead_depart_s = t
      if ep.vehicle_move_s is None and self._finite(v_ego) and v_ego > 0.30:
        ep.vehicle_move_s = t

    if not hold and self.previous_hold and self.current is not None:
      self.current.end_s = t
      self.current.duration_s = max(0.0, t - self.current.start_s)
      self.episodes.append(self.current)
      self.pending_move = self.current
      self.current = None

    if self.pending_move is not None:
      if self.pending_move.vehicle_move_s is None and self._finite(v_ego) and v_ego > 0.30:
        self.pending_move.vehicle_move_s = t
      if t - (self.pending_move.end_s or t) > 5.0:
        self.pending_move = None

    self.previous_hold = hold

  def finish(self, final_t: float | None) -> None:
    if self.current is not None:
      end_t = self.current.start_s if final_t is None else final_t
      self.current.end_s = end_t
      self.current.duration_s = max(0.0, end_t - self.current.start_s)
      self.current.ended_while_active = True
      self.episodes.append(self.current)
      self.current = None


class ShadowAnalyzer:
  def __init__(self) -> None:
    self.t0: float | None = None
    self.last_t: float | None = None
    self.latest_plan: dict[str, Any] = {}
    self.latest_plan_t: float | None = None
    self.latest_lead: dict[str, Any] = {}
    self.latest_radar_t: float | None = None
    self.samples = 0
    self.tracker = EpisodeTracker()

  def _relative_time(self, mono_ns: int) -> tuple[float, float]:
    t = mono_ns * 1e-9
    if self.t0 is None:
      self.t0 = t
    self.last_t = t
    return t, t - self.t0

  def handle(self, msg: Any, writer: csv.DictWriter) -> None:
    which = msg.which()
    mono_ns = int(_attr(msg, "logMonoTime", 0))
    t, t_rel = self._relative_time(mono_ns)

    if which == "longitudinalPlan":
      plan = msg.longitudinalPlan
      self.latest_plan = {
        "has_lead": _safe_bool(_attr(plan, "hasLead", False)),
        "v0": _first(_attr(plan, "speeds", [])),
        "a0": _first(_attr(plan, "accels", [])),
      }
      self.latest_plan_t = t
      return

    if which == "radarState":
      lead = msg.radarState.leadOne
      self.latest_lead = {
        "status": _safe_bool(_attr(lead, "status", False)),
        "d_rel": _safe_float(_attr(lead, "dRel", math.nan)),
        "v_rel": _safe_float(_attr(lead, "vRel", math.nan)),
        "a_rel": _safe_float(_attr(lead, "aRel", math.nan)),
        "v_lead": _safe_float(_attr(lead, "vLead", math.nan)),
      }
      self.latest_radar_t = t
      return

    if which != "carState":
      return

    cs = msg.carState
    cruise = cs.cruiseState
    plan_age = math.nan if self.latest_plan_t is None else max(0.0, t - self.latest_plan_t)
    radar_age = math.nan if self.latest_radar_t is None else max(0.0, t - self.latest_radar_t)

    row = {
      "t_rel_s": round(t_rel, 6),
      "log_mono_time_ns": mono_ns,
      "v_ego_mps": _safe_float(_attr(cs, "vEgo", math.nan)),
      "a_ego_mps2": _safe_float(_attr(cs, "aEgo", math.nan)),
      "standstill": _safe_bool(_attr(cs, "standstill", False)),
      "gas_pressed": _safe_bool(_attr(cs, "gasPressed", False)),
      "brake_pressed": _safe_bool(_attr(cs, "brakePressed", False)),
      "cruise_available": _safe_bool(_attr(cruise, "available", False)),
      "cruise_enabled": _safe_bool(_attr(cruise, "enabled", False)),
      "cruise_standstill": _safe_bool(_attr(cruise, "standstill", False)),
      "cruise_set_speed_mps": _safe_float(_attr(cruise, "speed", math.nan)),
      "brake_hold_active": _safe_bool(_attr(cs, "brakeHoldActive", False)),
      "plan_age_s": plan_age,
      "plan_has_lead": self.latest_plan.get("has_lead", False),
      "plan_v0_mps": self.latest_plan.get("v0", math.nan),
      "plan_a0_mps2": self.latest_plan.get("a0", math.nan),
      "radar_age_s": radar_age,
      "lead_status": self.latest_lead.get("status", False),
      "lead_d_rel_m": self.latest_lead.get("d_rel", math.nan),
      "lead_v_rel_mps": self.latest_lead.get("v_rel", math.nan),
      "lead_a_rel_mps2": self.latest_lead.get("a_rel", math.nan),
      "lead_v_lead_mps": self.latest_lead.get("v_lead", math.nan),
    }
    writer.writerow(row)
    self.samples += 1
    self.tracker.update(row)

  def summary(self, source_files: list[str]) -> dict[str, Any]:
    final_rel = None if self.t0 is None or self.last_t is None else self.last_t - self.t0
    self.tracker.finish(final_rel)
    return {
      "non_actuating": True,
      "source_files": source_files,
      "sample_count": self.samples,
      "duration_s": final_rel,
      "brake_hold_episode_count": len(self.tracker.episodes),
      "brake_hold_episodes": [asdict(ep) for ep in self.tracker.episodes],
      "interpretation_notes": [
        "plan_a0_mps2 is the first acceleration point published by openpilot's longitudinal planner.",
        "a_ego_mps2 is the Jeep's measured/filtered actual acceleration.",
        "lead_depart_s uses lead_v_rel_mps > 0.30 while a lead is valid.",
        "This report does not prove that a command would be safe or accepted by the Jeep.",
      ],
    }


def load_log_reader() -> Any:
  errors: list[str] = []
  for module_name in ("openpilot.tools.lib.logreader", "tools.lib.logreader"):
    try:
      module = importlib.import_module(module_name)
      return module.LogReader
    except Exception as exc:
      errors.append(f"{module_name}: {exc}")
  raise RuntimeError(
    "Could not import LogReader. Run this from the openpilot repository or on a comma device.\n" +
    "\n".join(errors)
  )


def iter_messages(paths: Iterable[str]) -> Iterable[Any]:
  LogReader = load_log_reader()
  for source in paths:
    for msg in LogReader(source):
      yield msg


def analyze(paths: list[str], csv_output: Path, json_output: Path) -> dict[str, Any]:
  analyzer = ShadowAnalyzer()
  csv_output.parent.mkdir(parents=True, exist_ok=True)
  with csv_output.open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    writer.writeheader()
    for msg in iter_messages(paths):
      analyzer.handle(msg, writer)

  summary = analyzer.summary(paths)
  json_output.parent.mkdir(parents=True, exist_ok=True)
  json_output.write_text(json.dumps(summary, indent=2, allow_nan=True) + "\n", encoding="utf-8")
  return summary


def run_self_test() -> None:
  tracker = EpisodeTracker()
  synthetic = [
    {"t_rel_s": 0.0, "brake_hold_active": False, "plan_a0_mps2": 0.0, "a_ego_mps2": 0.0,
     "v_ego_mps": 0.0, "lead_status": True, "lead_v_rel_mps": 0.0, "lead_d_rel_m": 5.0},
    {"t_rel_s": 1.0, "brake_hold_active": True, "plan_a0_mps2": -0.2, "a_ego_mps2": 0.0,
     "v_ego_mps": 0.0, "lead_status": True, "lead_v_rel_mps": 0.0, "lead_d_rel_m": 5.0},
    {"t_rel_s": 2.0, "brake_hold_active": True, "plan_a0_mps2": 0.4, "a_ego_mps2": 0.0,
     "v_ego_mps": 0.0, "lead_status": True, "lead_v_rel_mps": 1.0, "lead_d_rel_m": 5.5},
    {"t_rel_s": 2.5, "brake_hold_active": False, "plan_a0_mps2": 0.5, "a_ego_mps2": 0.2,
     "v_ego_mps": 0.0, "lead_status": True, "lead_v_rel_mps": 1.2, "lead_d_rel_m": 6.0},
    {"t_rel_s": 3.0, "brake_hold_active": False, "plan_a0_mps2": 0.5, "a_ego_mps2": 0.4,
     "v_ego_mps": 0.5, "lead_status": True, "lead_v_rel_mps": 0.8, "lead_d_rel_m": 6.5},
  ]
  for row in synthetic:
    tracker.update(row)
  tracker.finish(3.0)
  assert len(tracker.episodes) == 1
  episode = tracker.episodes[0]
  assert episode.start_s == 1.0
  assert episode.end_s == 2.5
  assert episode.lead_depart_s == 2.0
  assert episode.plan_positive_accel_s == 2.0
  assert episode.vehicle_move_s == 3.0
  print("SELF_TEST_OK")


def parse_args(argv: list[str]) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Create a non-actuating CSV/JSON comparison of Jeep stock ACC behavior and openpilot's longitudinal plan."
  )
  parser.add_argument("logs", nargs="*", help="Local rlog/qlog files or LogReader-supported sources, in time order.")
  parser.add_argument("--csv", default="b6_shadow.csv", help="CSV output path (default: b6_shadow.csv).")
  parser.add_argument("--json", default="b6_shadow_summary.json", help="JSON summary output path.")
  parser.add_argument("--self-test", action="store_true", help="Run the built-in dependency-free test and exit.")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(sys.argv[1:] if argv is None else argv)
  if args.self_test:
    run_self_test()
    return 0
  if not args.logs:
    print("No logs supplied. Use --help for examples.", file=sys.stderr)
    return 2

  try:
    summary = analyze(args.logs, Path(args.csv), Path(args.json))
  except Exception as exc:
    print(f"ERROR: {exc}", file=sys.stderr)
    return 1

  print(json.dumps(summary, indent=2, allow_nan=True))
  print(f"\nCSV: {Path(args.csv).resolve()}")
  print(f"JSON: {Path(args.json).resolve()}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())

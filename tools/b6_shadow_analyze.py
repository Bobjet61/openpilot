#!/usr/bin/env python3
"""Offline Jeep longitudinal and lateral shadow analysis.

This tool reads existing openpilot rlog/qlog files. It is not imported by the
vehicle-control path and cannot transmit CAN messages or command the vehicle.
"""

from __future__ import annotations

import argparse
import csv
import importlib
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable


CSV_FIELDS = [
  # Timing and vehicle state
  "t_rel_s",
  "log_mono_time_ns",
  "v_ego_mps",
  "a_ego_mps2",
  "standstill",
  "gas_pressed",
  "brake_pressed",

  # Stock ACC and brake hold
  "cruise_available",
  "cruise_enabled",
  "cruise_standstill",
  "cruise_set_speed_mps",
  "brake_hold_active",

  # Longitudinal planner and radar
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

  # Lateral engagement and measured steering
  "lat_active",
  "steering_angle_deg",
  "steering_rate_deg",
  "steering_torque_driver",
  "steering_torque_eps",
  "steering_pressed",
  "steer_fault_temporary",
  "steer_fault_permanent",

  # Requested/output steering and curvature
  "cc_steer_normalized",
  "cc_steer_output_can",
  "cc_curvature",
  "co_steer_normalized",
  "co_steer_output_can",
  "co_curvature",
  "controls_curvature",
  "controls_desired_curvature",
  "lateral_plan_curvature0",
  "lateral_plan_curvature_rate0",

  # Derived lateral metrics
  "curvature_error",
  "desired_lateral_accel_mps2",
  "actual_lateral_accel_mps2",
  "lateral_accel_error_mps2",

  # Lateral controller state
  "lateral_controller_type",
  "lateral_controller_output",
  "lateral_controller_saturated",
  "lateral_controller_error",

  # Learned/live vehicle parameters
  "live_steer_ratio",
  "live_stiffness_factor",
  "live_angle_offset_deg",
  "torque_lat_accel_factor",
  "torque_lat_accel_offset",
  "torque_friction",
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


def _first_attr(obj: Any, names: tuple[str, ...], default: Any = None) -> Any:
  for name in names:
    value = _attr(obj, name, None)
    if value is not None:
      return value
  return default


def _finite(value: Any) -> bool:
  try:
    return math.isfinite(float(value))
  except (TypeError, ValueError):
    return False


def _optional_float(value: Any) -> float | None:
  return float(value) if _finite(value) else None


def _actuator_snapshot(message_part: Any, field_names: tuple[str, ...]) -> dict[str, float]:
  actuators = None
  for field_name in field_names:
    actuators = _attr(message_part, field_name, None)
    if actuators is not None:
      break

  if actuators is None:
    return {}

  return {
    "steer": _safe_float(_attr(actuators, "steer", math.nan)),
    "steer_output_can": _safe_float(_attr(actuators, "steerOutputCan", math.nan)),
    "curvature": _safe_float(_attr(actuators, "curvature", math.nan)),
  }


def _lateral_controller_snapshot(controls_state: Any) -> dict[str, Any]:
  result: dict[str, Any] = {
    "type": "",
    "output": math.nan,
    "saturated": False,
    "error": math.nan,
    "desired_lateral_accel": math.nan,
    "actual_lateral_accel": math.nan,
  }

  lateral_state = _attr(controls_state, "lateralControlState", None)
  if lateral_state is None:
    return result

  try:
    state_type = lateral_state.which()
  except Exception:
    state_type = ""

  result["type"] = state_type or ""
  state = _attr(lateral_state, state_type, None) if state_type else lateral_state
  if state is None:
    return result

  result["output"] = _safe_float(_first_attr(state, ("output", "steeringAngleDeg"), math.nan))
  result["saturated"] = _safe_bool(_attr(state, "saturated", False))
  result["error"] = _safe_float(_first_attr(state, ("error", "lateralAccelError"), math.nan))
  result["desired_lateral_accel"] = _safe_float(
    _first_attr(state, ("desiredLateralAccel", "desiredLateralAcceleration"), math.nan)
  )
  result["actual_lateral_accel"] = _safe_float(
    _first_attr(state, ("actualLateralAccel", "actualLateralAcceleration"), math.nan)
  )
  return result


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
        start_lead_distance_m=_optional_float(lead_distance),
      )
      self.pending_move = None

    if self.current is not None:
      episode = self.current
      if _finite(plan_a):
        episode.max_plan_accel_mps2 = (
          plan_a if episode.max_plan_accel_mps2 is None
          else max(episode.max_plan_accel_mps2, plan_a)
        )
        if episode.plan_positive_accel_s is None and plan_a > 0.15:
          episode.plan_positive_accel_s = t

      if _finite(actual_a):
        episode.max_actual_accel_mps2 = (
          actual_a if episode.max_actual_accel_mps2 is None
          else max(episode.max_actual_accel_mps2, actual_a)
        )
        episode.min_actual_accel_mps2 = (
          actual_a if episode.min_actual_accel_mps2 is None
          else min(episode.min_actual_accel_mps2, actual_a)
        )

      if (episode.lead_depart_s is None and bool(row["lead_status"]) and
          _finite(lead_v_rel) and lead_v_rel > 0.30):
        episode.lead_depart_s = t

      if episode.vehicle_move_s is None and _finite(v_ego) and v_ego > 0.30:
        episode.vehicle_move_s = t

    if not hold and self.previous_hold and self.current is not None:
      self.current.end_s = t
      self.current.duration_s = max(0.0, t - self.current.start_s)
      self.episodes.append(self.current)
      self.pending_move = self.current
      self.current = None

    if self.pending_move is not None:
      if self.pending_move.vehicle_move_s is None and _finite(v_ego) and v_ego > 0.30:
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


class RunningStats:
  def __init__(self) -> None:
    self.count = 0
    self.total = 0.0
    self.abs_total = 0.0
    self.square_total = 0.0
    self.minimum = math.inf
    self.maximum = -math.inf

  def add(self, value: Any) -> None:
    if not _finite(value):
      return
    number = float(value)
    self.count += 1
    self.total += number
    self.abs_total += abs(number)
    self.square_total += number * number
    self.minimum = min(self.minimum, number)
    self.maximum = max(self.maximum, number)

  def summary(self) -> dict[str, Any]:
    if self.count == 0:
      return {
        "count": 0,
        "mean": None,
        "mean_abs": None,
        "rms": None,
        "min": None,
        "max": None,
      }
    return {
      "count": self.count,
      "mean": self.total / self.count,
      "mean_abs": self.abs_total / self.count,
      "rms": math.sqrt(self.square_total / self.count),
      "min": self.minimum,
      "max": self.maximum,
    }


class LateralBucket:
  def __init__(self) -> None:
    self.rows = 0
    self.active_rows = 0
    self.saturated_rows = 0
    self.curvature_error = RunningStats()
    self.lateral_accel_error = RunningStats()

  def add(self, row: dict[str, Any]) -> None:
    self.rows += 1
    if bool(row["lat_active"]):
      self.active_rows += 1
      if bool(row["lateral_controller_saturated"]):
        self.saturated_rows += 1
      self.curvature_error.add(row["curvature_error"])
      self.lateral_accel_error.add(row["lateral_accel_error_mps2"])

  def summary(self) -> dict[str, Any]:
    return {
      "rows": self.rows,
      "active_rows": self.active_rows,
      "saturation_fraction_active": (
        self.saturated_rows / self.active_rows if self.active_rows else None
      ),
      "curvature_error": self.curvature_error.summary(),
      "lateral_accel_error_mps2": self.lateral_accel_error.summary(),
    }


class LateralTracker:
  """Streaming summary metrics. These are diagnostics, not safety validation."""

  SPEED_BINS = (
    ("under_10_mph", 0.0, 4.4704),
    ("10_to_30_mph", 4.4704, 13.4112),
    ("30_to_50_mph", 13.4112, 22.352),
    ("50_mph_and_up", 22.352, math.inf),
  )

  CURVE_BINS = (
    ("under_0_5_mps2", 0.0, 0.5),
    ("0_5_to_1_0_mps2", 0.5, 1.0),
    ("1_0_to_2_0_mps2", 1.0, 2.0),
    ("2_0_mps2_and_up", 2.0, math.inf),
  )

  def __init__(self) -> None:
    self.rows = 0
    self.active_rows = 0
    self.driver_override_rows = 0
    self.saturated_rows = 0
    self.temporary_fault_edges = 0
    self.permanent_fault_edges = 0
    self.active_duration_s = 0.0
    self.command_reversals = 0

    self.curvature_error = RunningStats()
    self.lateral_accel_error = RunningStats()
    self.commanded_steer = RunningStats()
    self.eps_torque = RunningStats()
    self.driver_torque = RunningStats()
    self.steering_rate = RunningStats()

    self.speed_buckets = {name: LateralBucket() for name, _, _ in self.SPEED_BINS}
    self.curve_buckets = {name: LateralBucket() for name, _, _ in self.CURVE_BINS}

    self.last_t: float | None = None
    self.last_command_sign = 0
    self.last_temp_fault = False
    self.last_perm_fault = False

  @staticmethod
  def _bin_name(value: float, bins: tuple[tuple[str, float, float], ...]) -> str | None:
    if not _finite(value):
      return None
    number = float(value)
    for name, lower, upper in bins:
      if lower <= number < upper:
        return name
    return None

  @staticmethod
  def _normalized_steer_command(row: dict[str, Any]) -> float:
    for field in ("co_steer_normalized", "cc_steer_normalized"):
      value = _safe_float(row[field])
      if _finite(value):
        return value
    return math.nan

  def update(self, row: dict[str, Any]) -> None:
    self.rows += 1
    t = float(row["t_rel_s"])
    active = bool(row["lat_active"])
    speed = _safe_float(row["v_ego_mps"])

    if self.last_t is not None and active and _finite(speed) and speed > 4.0:
      self.active_duration_s += max(0.0, min(0.2, t - self.last_t))
    self.last_t = t

    temporary_fault = bool(row["steer_fault_temporary"])
    permanent_fault = bool(row["steer_fault_permanent"])
    if temporary_fault and not self.last_temp_fault:
      self.temporary_fault_edges += 1
    if permanent_fault and not self.last_perm_fault:
      self.permanent_fault_edges += 1
    self.last_temp_fault = temporary_fault
    self.last_perm_fault = permanent_fault

    speed_bucket = self._bin_name(speed, self.SPEED_BINS)
    if speed_bucket is not None:
      self.speed_buckets[speed_bucket].add(row)

    desired_lateral_accel = abs(_safe_float(row["desired_lateral_accel_mps2"]))
    curve_bucket = self._bin_name(desired_lateral_accel, self.CURVE_BINS)
    if curve_bucket is not None:
      self.curve_buckets[curve_bucket].add(row)

    if not active:
      self.last_command_sign = 0
      return

    self.active_rows += 1
    if bool(row["steering_pressed"]):
      self.driver_override_rows += 1
    if bool(row["lateral_controller_saturated"]):
      self.saturated_rows += 1

    self.curvature_error.add(row["curvature_error"])
    self.lateral_accel_error.add(row["lateral_accel_error_mps2"])
    self.eps_torque.add(row["steering_torque_eps"])
    self.driver_torque.add(row["steering_torque_driver"])
    self.steering_rate.add(row["steering_rate_deg"])

    command = self._normalized_steer_command(row)
    self.commanded_steer.add(command)

    # Reversals through a deadband are a useful oscillation-screening proxy.
    # They are not, by themselves, proof of poor tuning.
    if _finite(command) and _finite(speed) and speed > 4.0:
      sign = 1 if command > 0.03 else -1 if command < -0.03 else 0
      if sign and self.last_command_sign and sign != self.last_command_sign:
        self.command_reversals += 1
      if sign:
        self.last_command_sign = sign

  def summary(self) -> dict[str, Any]:
    return {
      "row_count": self.rows,
      "active_row_count": self.active_rows,
      "active_duration_s_above_4_mps": self.active_duration_s,
      "driver_override_fraction_active": (
        self.driver_override_rows / self.active_rows if self.active_rows else None
      ),
      "controller_saturation_fraction_active": (
        self.saturated_rows / self.active_rows if self.active_rows else None
      ),
      "temporary_steer_fault_events": self.temporary_fault_edges,
      "permanent_steer_fault_events": self.permanent_fault_edges,
      "steer_command_reversals": self.command_reversals,
      "steer_command_reversals_per_minute_active": (
        self.command_reversals * 60.0 / self.active_duration_s
        if self.active_duration_s > 0.0 else None
      ),
      "curvature_error": self.curvature_error.summary(),
      "lateral_accel_error_mps2": self.lateral_accel_error.summary(),
      "normalized_steer_command": self.commanded_steer.summary(),
      "eps_torque": self.eps_torque.summary(),
      "driver_torque": self.driver_torque.summary(),
      "steering_rate_deg_per_s": self.steering_rate.summary(),
      "by_speed": {name: bucket.summary() for name, bucket in self.speed_buckets.items()},
      "by_desired_lateral_accel": {
        name: bucket.summary() for name, bucket in self.curve_buckets.items()
      },
      "interpretation_notes": [
        "Curvature and lateral-acceleration errors are calculated only when both desired and actual values are available.",
        "Steer-command reversal rate is a screening metric for possible ping-pong behavior; normal curves and lane changes also create reversals.",
        "Controller saturation can indicate insufficient authority, but it must be interpreted with road curvature, speed, driver input, and EPS limits.",
        "These metrics do not establish that a tuning change is safe.",
      ],
    }


class ShadowAnalyzer:
  def __init__(self) -> None:
    self.t0: float | None = None
    self.last_t: float | None = None

    self.latest_plan: dict[str, Any] = {}
    self.latest_plan_t: float | None = None
    self.latest_lead: dict[str, Any] = {}
    self.latest_radar_t: float | None = None

    self.latest_car_control: dict[str, Any] = {}
    self.latest_car_output: dict[str, Any] = {}
    self.latest_controls_state: dict[str, Any] = {}
    self.latest_lateral_plan: dict[str, Any] = {}
    self.latest_live_parameters: dict[str, Any] = {}
    self.latest_torque_parameters: dict[str, Any] = {}

    self.samples = 0
    self.episode_tracker = EpisodeTracker()
    self.lateral_tracker = LateralTracker()

  def _relative_time(self, mono_ns: int) -> tuple[float, float]:
    t = mono_ns * 1e-9
    if self.t0 is None:
      self.t0 = t
    self.last_t = t
    return t, t - self.t0

  def _handle_non_car_state(self, which: str, msg: Any, t: float) -> bool:
    if which == "longitudinalPlan":
      plan = msg.longitudinalPlan
      self.latest_plan = {
        "has_lead": _safe_bool(_attr(plan, "hasLead", False)),
        "v0": _first(_attr(plan, "speeds", [])),
        "a0": _first(_attr(plan, "accels", [])),
      }
      self.latest_plan_t = t
      return True

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
      return True

    if which == "carControl":
      car_control = msg.carControl
      self.latest_car_control = {
        "lat_active": _safe_bool(_attr(car_control, "latActive", False)),
        **_actuator_snapshot(car_control, ("actuators",)),
      }
      return True

    if which == "carOutput":
      car_output = msg.carOutput
      self.latest_car_output = _actuator_snapshot(
        car_output, ("actuatorsOutput", "actuators")
      )
      return True

    if which == "controlsState":
      controls = msg.controlsState
      controller = _lateral_controller_snapshot(controls)
      self.latest_controls_state = {
        "curvature": _safe_float(_attr(controls, "curvature", math.nan)),
        "desired_curvature": _safe_float(_attr(controls, "desiredCurvature", math.nan)),
        **controller,
      }
      return True

    if which in ("lateralPlan", "lateralPlanSP"):
      plan = _attr(msg, which, None)
      if plan is not None:
        self.latest_lateral_plan = {
          "curvature0": _first(_attr(plan, "curvatures", [])),
          "curvature_rate0": _first(_attr(plan, "curvatureRates", [])),
        }
      return True

    if which == "liveParameters":
      params = msg.liveParameters
      self.latest_live_parameters = {
        "steer_ratio": _safe_float(_attr(params, "steerRatio", math.nan)),
        "stiffness_factor": _safe_float(_attr(params, "stiffnessFactor", math.nan)),
        "angle_offset_deg": _safe_float(
          _first_attr(params, ("angleOffsetDeg", "angleOffsetAverageDeg"), math.nan)
        ),
      }
      return True

    if which == "liveTorqueParameters":
      params = msg.liveTorqueParameters
      self.latest_torque_parameters = {
        "lat_accel_factor": _safe_float(_attr(params, "latAccelFactor", math.nan)),
        "lat_accel_offset": _safe_float(_attr(params, "latAccelOffset", math.nan)),
        "friction": _safe_float(
          _first_attr(params, ("frictionCoefficient", "friction"), math.nan)
        ),
      }
      return True

    return False

  def handle(self, msg: Any, writer: csv.DictWriter) -> None:
    which = msg.which()
    mono_ns = int(_attr(msg, "logMonoTime", 0))
    t, t_rel = self._relative_time(mono_ns)

    if self._handle_non_car_state(which, msg, t):
      return

    if which != "carState":
      return

    cs = msg.carState
    cruise = cs.cruiseState
    plan_age = math.nan if self.latest_plan_t is None else max(0.0, t - self.latest_plan_t)
    radar_age = math.nan if self.latest_radar_t is None else max(0.0, t - self.latest_radar_t)

    speed = _safe_float(_attr(cs, "vEgo", math.nan))

    controls_curvature = _safe_float(
      self.latest_controls_state.get("curvature", math.nan)
    )
    controls_desired_curvature = _safe_float(
      self.latest_controls_state.get("desired_curvature", math.nan)
    )

    actual_curvature = controls_curvature
    if not _finite(actual_curvature):
      actual_curvature = _safe_float(self.latest_car_output.get("curvature", math.nan))

    desired_curvature = controls_desired_curvature
    if not _finite(desired_curvature):
      desired_curvature = _safe_float(self.latest_car_control.get("curvature", math.nan))
    if not _finite(desired_curvature):
      desired_curvature = _safe_float(self.latest_lateral_plan.get("curvature0", math.nan))

    curvature_error = (
      desired_curvature - actual_curvature
      if _finite(desired_curvature) and _finite(actual_curvature)
      else math.nan
    )

    desired_lateral_accel = _safe_float(
      self.latest_controls_state.get("desired_lateral_accel", math.nan)
    )
    if not _finite(desired_lateral_accel) and _finite(desired_curvature) and _finite(speed):
      desired_lateral_accel = desired_curvature * speed * speed

    actual_lateral_accel = _safe_float(
      self.latest_controls_state.get("actual_lateral_accel", math.nan)
    )
    if not _finite(actual_lateral_accel) and _finite(actual_curvature) and _finite(speed):
      actual_lateral_accel = actual_curvature * speed * speed

    lateral_accel_error = (
      desired_lateral_accel - actual_lateral_accel
      if _finite(desired_lateral_accel) and _finite(actual_lateral_accel)
      else math.nan
    )

    row = {
      "t_rel_s": round(t_rel, 6),
      "log_mono_time_ns": mono_ns,
      "v_ego_mps": speed,
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

      "lat_active": self.latest_car_control.get("lat_active", False),
      "steering_angle_deg": _safe_float(_attr(cs, "steeringAngleDeg", math.nan)),
      "steering_rate_deg": _safe_float(_attr(cs, "steeringRateDeg", math.nan)),
      "steering_torque_driver": _safe_float(_attr(cs, "steeringTorque", math.nan)),
      "steering_torque_eps": _safe_float(_attr(cs, "steeringTorqueEps", math.nan)),
      "steering_pressed": _safe_bool(_attr(cs, "steeringPressed", False)),
      "steer_fault_temporary": _safe_bool(_attr(cs, "steerFaultTemporary", False)),
      "steer_fault_permanent": _safe_bool(_attr(cs, "steerFaultPermanent", False)),

      "cc_steer_normalized": self.latest_car_control.get("steer", math.nan),
      "cc_steer_output_can": self.latest_car_control.get("steer_output_can", math.nan),
      "cc_curvature": self.latest_car_control.get("curvature", math.nan),
      "co_steer_normalized": self.latest_car_output.get("steer", math.nan),
      "co_steer_output_can": self.latest_car_output.get("steer_output_can", math.nan),
      "co_curvature": self.latest_car_output.get("curvature", math.nan),
      "controls_curvature": controls_curvature,
      "controls_desired_curvature": controls_desired_curvature,
      "lateral_plan_curvature0": self.latest_lateral_plan.get("curvature0", math.nan),
      "lateral_plan_curvature_rate0": self.latest_lateral_plan.get(
        "curvature_rate0", math.nan
      ),

      "curvature_error": curvature_error,
      "desired_lateral_accel_mps2": desired_lateral_accel,
      "actual_lateral_accel_mps2": actual_lateral_accel,
      "lateral_accel_error_mps2": lateral_accel_error,

      "lateral_controller_type": self.latest_controls_state.get("type", ""),
      "lateral_controller_output": self.latest_controls_state.get("output", math.nan),
      "lateral_controller_saturated": self.latest_controls_state.get("saturated", False),
      "lateral_controller_error": self.latest_controls_state.get("error", math.nan),

      "live_steer_ratio": self.latest_live_parameters.get("steer_ratio", math.nan),
      "live_stiffness_factor": self.latest_live_parameters.get(
        "stiffness_factor", math.nan
      ),
      "live_angle_offset_deg": self.latest_live_parameters.get(
        "angle_offset_deg", math.nan
      ),
      "torque_lat_accel_factor": self.latest_torque_parameters.get(
        "lat_accel_factor", math.nan
      ),
      "torque_lat_accel_offset": self.latest_torque_parameters.get(
        "lat_accel_offset", math.nan
      ),
      "torque_friction": self.latest_torque_parameters.get("friction", math.nan),
    }

    writer.writerow(row)
    self.samples += 1
    self.episode_tracker.update(row)
    self.lateral_tracker.update(row)

  def summary(self, source_files: list[str]) -> dict[str, Any]:
    final_rel = None if self.t0 is None or self.last_t is None else self.last_t - self.t0
    self.episode_tracker.finish(final_rel)
    return {
      "non_actuating": True,
      "source_files": source_files,
      "sample_count": self.samples,
      "duration_s": final_rel,
      "longitudinal": {
        "brake_hold_episode_count": len(self.episode_tracker.episodes),
        "brake_hold_episodes": [
          asdict(episode) for episode in self.episode_tracker.episodes
        ],
        "interpretation_notes": [
          "plan_a0_mps2 is the first acceleration point published by openpilot's longitudinal planner.",
          "a_ego_mps2 is the Jeep's measured/filtered actual acceleration.",
          "lead_depart_s uses lead_v_rel_mps > 0.30 while a lead is valid.",
          "This report does not prove that a command would be safe or accepted by the Jeep.",
        ],
      },
      "lateral": self.lateral_tracker.summary(),
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
    "Could not import LogReader. Run this from the openpilot repository or on a comma device.\n"
    + "\n".join(errors)
  )


def iter_messages(paths: Iterable[str]) -> Iterable[Any]:
  LogReader = load_log_reader()
  for source in paths:
    for msg in LogReader(source):
      yield msg


def render_markdown(summary: dict[str, Any]) -> str:
  longitudinal = summary["longitudinal"]
  lateral = summary["lateral"]

  def display(value: Any, digits: int = 4) -> str:
    if value is None:
      return "not available"
    if isinstance(value, float):
      return f"{value:.{digits}f}"
    return str(value)

  lines = [
    "# Jeep Shadow Analysis",
    "",
    f"- Samples: {summary['sample_count']}",
    f"- Duration: {display(summary['duration_s'], 1)} seconds",
    f"- Brake-hold episodes: {longitudinal['brake_hold_episode_count']}",
    "",
    "## Lateral summary",
    "",
    f"- Active samples: {lateral['active_row_count']}",
    f"- Active duration above 4 m/s: {display(lateral['active_duration_s_above_4_mps'], 1)} seconds",
    f"- Driver override fraction: {display(lateral['driver_override_fraction_active'])}",
    f"- Controller saturation fraction: {display(lateral['controller_saturation_fraction_active'])}",
    f"- Temporary steering-fault events: {lateral['temporary_steer_fault_events']}",
    f"- Permanent steering-fault events: {lateral['permanent_steer_fault_events']}",
    f"- Steering-command reversals: {lateral['steer_command_reversals']}",
    f"- Reversals per active minute: {display(lateral['steer_command_reversals_per_minute_active'], 2)}",
    f"- Curvature-error RMS: {display(lateral['curvature_error']['rms'], 6)}",
    f"- Lateral-acceleration-error RMS: {display(lateral['lateral_accel_error_mps2']['rms'], 4)} m/s²",
    "",
    "## Brake-hold episodes",
    "",
  ]

  episodes = longitudinal["brake_hold_episodes"]
  if not episodes:
    lines.append("No brake-hold episodes were found in the supplied logs.")
  else:
    for index, episode in enumerate(episodes, start=1):
      lines.extend([
        f"### Episode {index}",
        "",
        f"- Start: {display(episode['start_s'], 2)} s",
        f"- End: {display(episode['end_s'], 2)} s",
        f"- Duration: {display(episode['duration_s'], 2)} s",
        f"- Lead departure: {display(episode['lead_depart_s'], 2)} s",
        f"- Planner first requested positive acceleration: {display(episode['plan_positive_accel_s'], 2)} s",
        f"- Jeep began moving: {display(episode['vehicle_move_s'], 2)} s",
        "",
      ])

  lines.extend([
    "## Important limitations",
    "",
    "- This is an offline analysis of recorded data.",
    "- Steering-command reversals are only a screening proxy; lane changes and normal curves also cause reversals.",
    "- The report does not validate a tuning change or authorize additional vehicle commands.",
    "",
  ])
  return "\n".join(lines)


def analyze(
  paths: list[str],
  csv_output: Path,
  json_output: Path,
  markdown_output: Path,
) -> dict[str, Any]:
  analyzer = ShadowAnalyzer()
  csv_output.parent.mkdir(parents=True, exist_ok=True)

  with csv_output.open("w", newline="", encoding="utf-8") as file:
    writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
    writer.writeheader()
    for message in iter_messages(paths):
      analyzer.handle(message, writer)

  summary = analyzer.summary(paths)

  json_output.parent.mkdir(parents=True, exist_ok=True)
  json_output.write_text(
    json.dumps(summary, indent=2, allow_nan=True) + "\n",
    encoding="utf-8",
  )

  markdown_output.parent.mkdir(parents=True, exist_ok=True)
  markdown_output.write_text(render_markdown(summary), encoding="utf-8")
  return summary


def _synthetic_lateral_row(
  t: float,
  command: float,
  desired_curvature: float,
  actual_curvature: float,
  *,
  saturated: bool = False,
  steering_pressed: bool = False,
  temporary_fault: bool = False,
) -> dict[str, Any]:
  speed = 15.0
  return {
    "t_rel_s": t,
    "lat_active": True,
    "v_ego_mps": speed,
    "steering_pressed": steering_pressed,
    "lateral_controller_saturated": saturated,
    "curvature_error": desired_curvature - actual_curvature,
    "desired_lateral_accel_mps2": desired_curvature * speed * speed,
    "actual_lateral_accel_mps2": actual_curvature * speed * speed,
    "lateral_accel_error_mps2": (desired_curvature - actual_curvature) * speed * speed,
    "co_steer_normalized": command,
    "cc_steer_normalized": command,
    "steering_torque_eps": command * 100.0,
    "steering_torque_driver": 1.0 if steering_pressed else 0.0,
    "steering_rate_deg": command * 10.0,
    "steer_fault_temporary": temporary_fault,
    "steer_fault_permanent": False,
  }


def run_self_test() -> None:
  episode_tracker = EpisodeTracker()
  synthetic_longitudinal = [
    {
      "t_rel_s": 0.0,
      "brake_hold_active": False,
      "plan_a0_mps2": 0.0,
      "a_ego_mps2": 0.0,
      "v_ego_mps": 0.0,
      "lead_status": True,
      "lead_v_rel_mps": 0.0,
      "lead_d_rel_m": 5.0,
    },
    {
      "t_rel_s": 1.0,
      "brake_hold_active": True,
      "plan_a0_mps2": -0.2,
      "a_ego_mps2": 0.0,
      "v_ego_mps": 0.0,
      "lead_status": True,
      "lead_v_rel_mps": 0.0,
      "lead_d_rel_m": 5.0,
    },
    {
      "t_rel_s": 2.0,
      "brake_hold_active": True,
      "plan_a0_mps2": 0.4,
      "a_ego_mps2": 0.0,
      "v_ego_mps": 0.0,
      "lead_status": True,
      "lead_v_rel_mps": 1.0,
      "lead_d_rel_m": 5.5,
    },
    {
      "t_rel_s": 2.5,
      "brake_hold_active": False,
      "plan_a0_mps2": 0.5,
      "a_ego_mps2": 0.2,
      "v_ego_mps": 0.0,
      "lead_status": True,
      "lead_v_rel_mps": 1.2,
      "lead_d_rel_m": 6.0,
    },
    {
      "t_rel_s": 3.0,
      "brake_hold_active": False,
      "plan_a0_mps2": 0.5,
      "a_ego_mps2": 0.4,
      "v_ego_mps": 0.5,
      "lead_status": True,
      "lead_v_rel_mps": 0.8,
      "lead_d_rel_m": 6.5,
    },
  ]

  for row in synthetic_longitudinal:
    episode_tracker.update(row)
  episode_tracker.finish(3.0)

  assert len(episode_tracker.episodes) == 1
  episode = episode_tracker.episodes[0]
  assert episode.start_s == 1.0
  assert episode.end_s == 2.5
  assert episode.lead_depart_s == 2.0
  assert episode.plan_positive_accel_s == 2.0
  assert episode.vehicle_move_s == 3.0

  lateral_tracker = LateralTracker()
  synthetic_lateral = [
    _synthetic_lateral_row(0.0, 0.20, 0.010, 0.008),
    _synthetic_lateral_row(0.1, 0.25, 0.011, 0.009, saturated=True),
    _synthetic_lateral_row(0.2, -0.20, -0.010, -0.008, steering_pressed=True),
    _synthetic_lateral_row(0.3, -0.25, -0.011, -0.009, temporary_fault=True),
  ]

  for row in synthetic_lateral:
    lateral_tracker.update(row)

  lateral_summary = lateral_tracker.summary()
  assert lateral_summary["active_row_count"] == 4
  assert lateral_summary["steer_command_reversals"] == 1
  assert lateral_summary["temporary_steer_fault_events"] == 1
  assert abs(lateral_summary["controller_saturation_fraction_active"] - 0.25) < 1e-9
  assert abs(lateral_summary["driver_override_fraction_active"] - 0.25) < 1e-9
  assert lateral_summary["curvature_error"]["count"] == 4

  print("SELF_TEST_OK")


def parse_args(argv: list[str]) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description=(
      "Create non-actuating CSV/JSON/Markdown comparisons of Jeep stock ACC, "
      "openpilot longitudinal planning, and lateral steering behavior."
    )
  )
  parser.add_argument(
    "logs",
    nargs="*",
    help="Local rlog/qlog files or LogReader-supported sources, in time order.",
  )
  parser.add_argument(
    "--csv",
    default="b6_shadow.csv",
    help="CSV output path (default: b6_shadow.csv).",
  )
  parser.add_argument(
    "--json",
    default="b6_shadow_summary.json",
    help="JSON summary output path.",
  )
  parser.add_argument(
    "--report",
    default="b6_shadow_report.md",
    help="Markdown report output path.",
  )
  parser.add_argument(
    "--self-test",
    action="store_true",
    help="Run dependency-free longitudinal and lateral tests and exit.",
  )
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
    summary = analyze(
      args.logs,
      Path(args.csv),
      Path(args.json),
      Path(args.report),
    )
  except Exception as exc:
    print(f"ERROR: {exc}", file=sys.stderr)
    return 1

  print(json.dumps(summary, indent=2, allow_nan=True))
  print(f"\nCSV: {Path(args.csv).resolve()}")
  print(f"JSON: {Path(args.json).resolve()}")
  print(f"Report: {Path(args.report).resolve()}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())

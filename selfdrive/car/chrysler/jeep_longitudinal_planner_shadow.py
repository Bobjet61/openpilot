"""Passive openpilot longitudinal-plan follower for Jeep diagnostics only.

This module consumes an already-published LongitudinalPlan and runs the normal
LongControl state machine in memory. It has no messaging publisher, CAN packer,
sendcan access, or vehicle output.
"""

from dataclasses import dataclass
import math
from typing import Any

from cereal import car
from openpilot.common.numpy_fast import interp
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.longcontrol import (
  CONTROL_N_T_IDX,
  LongControl,
)

from openpilot.selfdrive.car.chrysler.jeep_longitudinal import (
  ACCEL_DEADBAND,
  ACCEL_MAX,
  ACCEL_MIN,
)


PLAN_MAX_AGE_S = 0.5
LongCtrlState = car.CarControl.Actuators.LongControlState


@dataclass(frozen=True)
class JeepLongitudinalPlanResult:
  plan_valid: bool
  eligible: bool
  reason: str
  plan_age_s: float
  source: str
  has_lead: bool
  fcw: bool
  lead_state: str
  radar_supported: bool
  radar_d_rel: float
  radar_v_rel: float
  target_speed_mps: float
  target_accel_mps2: float
  controller_accel_mps2: float
  control_state: str


@dataclass(frozen=True)
class JeepLongitudinalPlanWindow:
  samples: int
  plan_valid_samples: int
  eligible_samples: int
  lead_samples: int
  radar_supported_samples: int
  lead_confirmed_samples: int
  lead_unconfirmed_samples: int
  radar_only_samples: int
  fcw_samples: int
  brake_request_samples: int
  engine_request_samples: int
  max_brake_mps2: float
  max_accel_mps2: float


class _InactivePlan:
  speeds: tuple[float, ...] = ()
  accels: tuple[float, ...] = ()


def finite_trajectory(values: Any) -> bool:
  try:
    return (
      len(values) == CONTROL_N
      and all(math.isfinite(float(value)) for value in values)
    )
  except (TypeError, ValueError, OverflowError):
    return False


def plan_is_valid(
    plan: Any,
    *,
    seen: bool,
    service_valid: bool,
    plan_age_s: float,
) -> tuple[bool, str]:
  if not seen:
    return False, "plan_unseen"
  if not service_valid:
    return False, "plan_service_invalid"
  if not math.isfinite(plan_age_s) or plan_age_s < 0.0:
    return False, "plan_age_invalid"
  if plan_age_s > PLAN_MAX_AGE_S:
    return False, "plan_stale"
  if not finite_trajectory(plan.speeds):
    return False, "plan_speeds_invalid"
  if not finite_trajectory(plan.accels):
    return False, "plan_accels_invalid"
  return True, "eligible"


class JeepLongitudinalPlanShadow:
  """Run the production longitudinal controller with no output connection."""

  def __init__(self, CP):
    self.long_control = LongControl(CP)
    self._reset_window()

  def _reset_window(self):
    self.samples = 0
    self.plan_valid_samples = 0
    self.eligible_samples = 0
    self.lead_samples = 0
    self.radar_supported_samples = 0
    self.lead_confirmed_samples = 0
    self.lead_unconfirmed_samples = 0
    self.radar_only_samples = 0
    self.fcw_samples = 0
    self.brake_request_samples = 0
    self.engine_request_samples = 0
    self.max_brake_mps2 = 0.0
    self.max_accel_mps2 = 0.0

  def update(
      self,
      *,
      plan: Any,
      car_state: Any,
      seen: bool,
      service_valid: bool,
      plan_age_s: float,
      vehicle_eligible: bool,
      vehicle_reason: str,
      radar_selection: Any | None,
  ) -> JeepLongitudinalPlanResult:
    valid, plan_reason = plan_is_valid(
      plan,
      seen=seen,
      service_valid=service_valid,
      plan_age_s=plan_age_s,
    )
    eligible = vehicle_eligible and valid
    reason = (
      "eligible" if eligible
      else vehicle_reason if not vehicle_eligible
      else plan_reason
    )

    radar_track = (
      radar_selection.track
      if radar_selection is not None else None
    )
    radar_supported = radar_track is not None
    has_lead = bool(plan.hasLead) if valid else False
    if has_lead and radar_supported:
      lead_state = "confirmed"
    elif has_lead:
      lead_state = "unconfirmed"
    elif radar_supported:
      lead_state = "radar_only"
    else:
      lead_state = "no_lead"

    active_plan = plan if valid else _InactivePlan()
    controller_accel = float(self.long_control.update(
      eligible,
      car_state,
      active_plan,
      [ACCEL_MIN, ACCEL_MAX],
      plan_age_s if valid else 0.0,
    ))

    if valid:
      target_speed = float(interp(
        plan_age_s,
        CONTROL_N_T_IDX,
        plan.speeds,
      ))
      target_accel = float(interp(
        plan_age_s,
        CONTROL_N_T_IDX,
        plan.accels,
      ))
      source = str(plan.longitudinalPlanSource)
      fcw = bool(plan.fcw)
    else:
      target_speed = 0.0
      target_accel = 0.0
      source = "invalid"
      fcw = False

    self.samples += 1
    self.plan_valid_samples += int(valid)
    self.eligible_samples += int(eligible)
    self.lead_samples += int(has_lead)
    self.radar_supported_samples += int(radar_supported)
    self.lead_confirmed_samples += int(lead_state == "confirmed")
    self.lead_unconfirmed_samples += int(lead_state == "unconfirmed")
    self.radar_only_samples += int(lead_state == "radar_only")
    self.fcw_samples += int(fcw)
    self.brake_request_samples += int(
      eligible and controller_accel < -ACCEL_DEADBAND,
    )
    self.engine_request_samples += int(
      eligible and controller_accel > ACCEL_DEADBAND,
    )
    self.max_brake_mps2 = min(
      self.max_brake_mps2,
      controller_accel,
    )
    self.max_accel_mps2 = max(
      self.max_accel_mps2,
      controller_accel,
    )

    return JeepLongitudinalPlanResult(
      plan_valid=valid,
      eligible=eligible,
      reason=reason,
      plan_age_s=plan_age_s,
      source=source,
      has_lead=has_lead,
      fcw=fcw,
      lead_state=lead_state,
      radar_supported=radar_supported,
      radar_d_rel=(
        float(radar_track.d_rel) if radar_supported else 0.0
      ),
      radar_v_rel=(
        float(radar_track.v_rel) if radar_supported else 0.0
      ),
      target_speed_mps=target_speed,
      target_accel_mps2=target_accel,
      controller_accel_mps2=controller_accel,
      control_state=str(self.long_control.long_control_state),
    )

  def snapshot(self) -> JeepLongitudinalPlanWindow:
    window = JeepLongitudinalPlanWindow(
      samples=self.samples,
      plan_valid_samples=self.plan_valid_samples,
      eligible_samples=self.eligible_samples,
      lead_samples=self.lead_samples,
      radar_supported_samples=self.radar_supported_samples,
      lead_confirmed_samples=self.lead_confirmed_samples,
      lead_unconfirmed_samples=self.lead_unconfirmed_samples,
      radar_only_samples=self.radar_only_samples,
      fcw_samples=self.fcw_samples,
      brake_request_samples=self.brake_request_samples,
      engine_request_samples=self.engine_request_samples,
      max_brake_mps2=self.max_brake_mps2,
      max_accel_mps2=self.max_accel_mps2,
    )
    self._reset_window()
    return window

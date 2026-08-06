"""Fail-closed active longitudinal assistance from a vision-matched Jeep radar lead."""

from dataclasses import dataclass
import math

from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.car.chrysler.jeep_radar_shadow import (
  JeepRadarSelection,
  JeepVisionLead,
)


MATCH_CONFIRM_CYCLES = 3
MATCH_STALE_NS = 200_000_000
ACTIVE_MIN_MODEL_PROB = 0.80
MAX_ACTIVE_DISTANCE_M = 130.0

BRAKE_MIN_SPEED_MPS = 2.0
BRAKE_MIN_CLOSING_MPS = 0.75
BRAKE_TIME_GAP_S = 2.0
BRAKE_STANDSTILL_GAP_M = 5.0
BRAKE_MIN_ACCEL_MPS2 = -2.5
BRAKE_MIN_EFFECT_MPS2 = 0.15
RADAR_EXTRA_CLOSING_LIMIT_MPS = 3.0

LAUNCH_ENTRY_SPEED_MPS = 0.15
LAUNCH_EXIT_SPEED_MPS = 1.0
LAUNCH_MAX_DISTANCE_M = 60.0
LAUNCH_RADAR_MIN_VREL_MPS = 0.75
LAUNCH_VISION_MIN_VREL_MPS = 0.25
LAUNCH_MIN_TARGET_SPEED_MPS = 0.50
LAUNCH_CONFIRM_CYCLES = 4
LAUNCH_ACCEL_MPS2 = 0.45
LAUNCH_MAX_DURATION_NS = 5_000_000_000


@dataclass(frozen=True)
class JeepRadarAssistResult:
  requested_accel_mps2: float
  planner_accel_mps2: float
  target_accel_mps2: float
  active: bool
  mode: str
  match_confirmed: bool
  radar_d_rel: float
  radar_v_rel: float


class JeepRadarLongitudinalAssist:
  """Apply only a stable radar observation independently supported by vision.

  The assist cannot act on a radar-only target. Missing, ambiguous, weak, or
  stale associations immediately return control to the production planner.
  """

  def __init__(self):
    self.last_cycle = -1
    self.last_cycle_nanos = 0
    self.selected_track_index: int | None = None
    self.match_confirm_cycles = 0
    self.launch_confirm_cycles = 0
    self.launch_active = False
    self.launch_started_nanos = 0

  def _reset_match(self) -> None:
    self.selected_track_index = None
    self.match_confirm_cycles = 0
    self.launch_confirm_cycles = 0

  def _reset_all(self) -> None:
    self._reset_match()
    self.launch_active = False
    self.launch_started_nanos = 0

  @staticmethod
  def _valid_match(
      selection: JeepRadarSelection | None,
      vision: JeepVisionLead | None,
  ) -> bool:
    return bool(
      selection is not None
      and selection.track is not None
      and vision is not None
      and vision.eligible
      and vision.model_prob >= ACTIVE_MIN_MODEL_PROB
      and selection.track.d_rel <= MAX_ACTIVE_DISTANCE_M
    )

  @staticmethod
  def _inactive_result(planner_accel: float, mode: str) -> JeepRadarAssistResult:
    return JeepRadarAssistResult(
      requested_accel_mps2=planner_accel,
      planner_accel_mps2=planner_accel,
      target_accel_mps2=planner_accel,
      active=False,
      mode=mode,
      match_confirmed=False,
      radar_d_rel=0.0,
      radar_v_rel=0.0,
    )

  def update(
      self,
      *,
      planner_accel_mps2: float,
      speed_mps: float,
      eligible: bool,
      selection: JeepRadarSelection | None,
      vision: JeepVisionLead | None,
      radar_cycle: int,
      now_nanos: int,
  ) -> JeepRadarAssistResult:
    planner_accel = float(planner_accel_mps2)
    if (
        not eligible
        or not math.isfinite(planner_accel)
        or not math.isfinite(speed_mps)
    ):
      self._reset_all()
      return self._inactive_result(planner_accel, "ineligible")

    new_cycle = radar_cycle != self.last_cycle
    if new_cycle:
      self.last_cycle = radar_cycle
      self.last_cycle_nanos = now_nanos
      if self._valid_match(selection, vision):
        track = selection.track
        assert track is not None
        if track.index == self.selected_track_index:
          self.match_confirm_cycles += 1
        else:
          self.selected_track_index = track.index
          self.match_confirm_cycles = 1
          self.launch_confirm_cycles = 0

        launch_observed = bool(
          speed_mps <= LAUNCH_ENTRY_SPEED_MPS
          and track.d_rel <= LAUNCH_MAX_DISTANCE_M
          and track.v_rel >= LAUNCH_RADAR_MIN_VREL_MPS
          and vision is not None
          and vision.v_rel >= LAUNCH_VISION_MIN_VREL_MPS
        )
        if launch_observed:
          self.launch_confirm_cycles += 1
        else:
          self.launch_confirm_cycles = 0
      else:
        self._reset_match()

    fresh = (
      self.last_cycle_nanos > 0
      and now_nanos - self.last_cycle_nanos <= MATCH_STALE_NS
    )
    valid_match = self._valid_match(selection, vision)
    match_confirmed = bool(
      fresh
      and valid_match
      and self.match_confirm_cycles >= MATCH_CONFIRM_CYCLES
    )
    if not fresh or not valid_match:
      self._reset_all()
      return self._inactive_result(planner_accel, "no_fresh_match")

    track = selection.track
    assert track is not None
    assert vision is not None

    if (
        match_confirmed
        and self.launch_confirm_cycles >= LAUNCH_CONFIRM_CYCLES
        and not self.launch_active
    ):
      self.launch_active = True
      self.launch_started_nanos = now_nanos

    target_speed_mps = speed_mps + track.v_rel
    if self.launch_active and (
        speed_mps >= LAUNCH_EXIT_SPEED_MPS
        or target_speed_mps < LAUNCH_MIN_TARGET_SPEED_MPS
        or now_nanos - self.launch_started_nanos > LAUNCH_MAX_DURATION_NS
    ):
      self.launch_active = False

    if self.launch_active:
      requested = max(planner_accel, LAUNCH_ACCEL_MPS2)
      return JeepRadarAssistResult(
        requested_accel_mps2=requested,
        planner_accel_mps2=planner_accel,
        target_accel_mps2=LAUNCH_ACCEL_MPS2,
        active=True,
        mode="matched_lead_launch",
        match_confirmed=True,
        radar_d_rel=track.d_rel,
        radar_v_rel=track.v_rel,
      )

    if not match_confirmed:
      return JeepRadarAssistResult(
        requested_accel_mps2=planner_accel,
        planner_accel_mps2=planner_accel,
        target_accel_mps2=planner_accel,
        active=False,
        mode="confirming_match",
        match_confirmed=False,
        radar_d_rel=track.d_rel,
        radar_v_rel=track.v_rel,
      )

    # Use the more conservative closing-speed estimate, but limit how much a
    # single radar observation may exceed the independently measured vision
    # closing speed. Distance also remains bounded by the two matched sources.
    fused_v_rel = min(
      vision.v_rel,
      max(track.v_rel, vision.v_rel - RADAR_EXTRA_CLOSING_LIMIT_MPS),
    )
    closing_speed = max(0.0, -fused_v_rel)
    lead_distance = min(track.d_rel, vision.d_rel)
    if (
        speed_mps < BRAKE_MIN_SPEED_MPS
        or closing_speed < BRAKE_MIN_CLOSING_MPS
    ):
      return JeepRadarAssistResult(
        requested_accel_mps2=planner_accel,
        planner_accel_mps2=planner_accel,
        target_accel_mps2=planner_accel,
        active=False,
        mode="matched_no_brake",
        match_confirmed=True,
        radar_d_rel=track.d_rel,
        radar_v_rel=track.v_rel,
      )

    desired_distance = max(
      BRAKE_STANDSTILL_GAP_M,
      BRAKE_TIME_GAP_S * speed_mps,
    )
    remaining_distance = lead_distance - desired_distance
    if remaining_distance > 3.0:
      required_decel = closing_speed ** 2 / (2.0 * remaining_distance)
    else:
      required_decel = (
        0.40
        + 0.25 * closing_speed
        + 0.05 * max(0.0, -remaining_distance)
      )
    target_accel = -clip(
      required_decel,
      BRAKE_MIN_EFFECT_MPS2,
      -BRAKE_MIN_ACCEL_MPS2,
    )
    requested = min(planner_accel, target_accel)
    active = requested < planner_accel - 0.01
    return JeepRadarAssistResult(
      requested_accel_mps2=requested,
      planner_accel_mps2=planner_accel,
      target_accel_mps2=target_accel,
      active=active,
      mode="matched_lead_brake" if active else "planner_more_conservative",
      match_confirmed=True,
      radar_d_rel=track.d_rel,
      radar_v_rel=track.v_rel,
    )

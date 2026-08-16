"""Brake-only continuity for a previously fused Jeep closing lead.

The production Jeep radar path remains vision-only.  This guard never
publishes a lead, selects a cruise target, or requests propulsion.  It may only
make an already-running openpilot-long acceleration request more negative for
a bounded period after a strongly vision-confirmed raw-radar target disappears
or the camera lead falls below the guard's confidence threshold.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import statistics


# Exact controller-event replay of route 8 provides three consecutive >=0.90
# vision/radar matches before the relevant dropout.  A fourth cycle misses the
# event because the production lead and raw-radar subscriber updates are not
# paired on the same controller frame.
CONFIRM_CYCLES = 3
MIN_MODEL_PROB = 0.90
VISION_DROPOUT_MODEL_PROB = 0.70
MIN_VEGO_MPS = 3.0
MIN_ACQUIRE_CLOSING_MPS = 1.0
MIN_CONTINUITY_CLOSING_MPS = 2.0
MIN_ABSOLUTE_TARGET_SPEED_MPS = -1.0
MAX_FUSION_DISTANCE_ERROR_M = 3.0
MAX_FUSION_DISTANCE_ERROR_RATIO = 0.15
MAX_FUSION_SPEED_ERROR_MPS = 2.0
MAX_RANGE_BIAS_M = 5.0
MIN_SUPPORT_MODEL_PROB = 0.50
# Weak support may preserve an already strongly confirmed raw target, but can
# never acquire one or initiate braking while the production lead is healthy.
# Route 8's subscriber pairing needs 4.5 m; the dropout-only transition below
# prevents this wider preservation tolerance from becoming a brake trigger.
MAX_SUPPORT_DISTANCE_ERROR_M = 4.5
MAX_SUPPORT_DISTANCE_ERROR_RATIO = 0.10
MAX_SUPPORT_SPEED_ERROR_MPS = 2.5
MAX_TRACK_DISTANCE_ERROR_M = 1.25
MAX_TRACK_DISTANCE_ERROR_RATIO = 0.05
MAX_TRACK_SPEED_STEP_MPS = 2.5
MAX_CLOSING_DISTANCE_RISE_M = 0.75
TRACK_AMBIGUITY_SCORE_MARGIN = 0.25
RAW_STALE_NS = 100_000_000
DROPOUT_TIMEOUT_NS = 3_350_000_000
# Independently cap supplemental authority even when weak-but-consistent
# vision/radar support keeps refreshing the support-age timeout.  This is
# longer than the 4.869 s route-8 episode, while still requiring a fresh
# strong-fusion confirmation for any later continuity event.
ABSOLUTE_ACTIVE_TIMEOUT_NS = 5_250_000_000
STOP_BUFFER_M = 5.4
TIME_GAP_S = 3.0
ASSUMED_LEAD_DECEL_MPS2 = 3.0
CONTINUITY_ACCEL_MIN_MPS2 = -2.5
STALE_HOLD_MAX_DISTANCE_M = 20.0
STALE_HOLD_MAX_TARGET_SPEED_MPS = 2.0
STALE_HOLD_MIN_CLOSING_MPS = 3.0
STALE_HOLD_MIN_BRAKE_MPS2 = 0.5


@dataclass(frozen=True)
class JeepRadarBrakeGuardResult:
  brake_floor_mps2: float | None
  active: bool
  reason: str
  confirmed: bool
  track_index: int | None
  d_rel: float | None
  v_rel: float | None
  dropout_age_s: float
  range_bias_m: float


def _clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


def _finite(*values: float) -> bool:
  return all(math.isfinite(float(value)) for value in values)


def _continuity_brake_floor(v_ego: float, d_rel: float, v_rel: float):
  """Return a radar-continuity floor, never a positive acceleration."""
  if not _finite(v_ego, d_rel, v_rel):
    return None
  v_ego = float(v_ego)
  d_rel = float(d_rel)
  v_rel = float(v_rel)
  closing_speed = max(0.0, -v_rel)
  if (
      v_ego < MIN_VEGO_MPS
      or closing_speed < MIN_CONTINUITY_CLOSING_MPS
      or d_rel <= 0.0
      or d_rel > STOP_BUFFER_M + TIME_GAP_S * v_ego
  ):
    return None

  lead_speed = max(0.0, v_ego + v_rel)
  lead_stopping_distance = (
    lead_speed * lead_speed / (2.0 * ASSUMED_LEAD_DECEL_MPS2)
  )
  available_distance = d_rel - STOP_BUFFER_M + lead_stopping_distance
  if available_distance <= 0.0:
    return CONTINUITY_ACCEL_MIN_MPS2
  required_decel = -(v_ego * v_ego) / (2.0 * available_distance)
  return _clip(required_decel, CONTINUITY_ACCEL_MIN_MPS2, 0.0)


class JeepRadarBrakeContinuityGuard:
  """Fail-closed braking continuity for an absent/degraded camera lead."""

  def __init__(self):
    self._last_cycle = None
    self._last_radar_cycle_time_ns = None
    self.reset()

  def reset(self):
    self._candidate_index = None
    self._confirmation_cycles = 0
    self._confirmed_index = None
    self._bias_samples = []
    self._range_bias_m = 0.0
    self._last_track = None
    self._last_track_time_ns = None
    self._last_target_speed_mps = None
    self._last_supported_ns = None
    self._dropout_start_ns = None
    self._active = False
    self._holding_stale_floor = False
    self._last_floor = None

  def _result(self, reason, brake_floor=None, now_nanos=None):
    age_s = 0.0
    if self._dropout_start_ns is not None and now_nanos is not None:
      age_s = max(0.0, (now_nanos - self._dropout_start_ns) / 1e9)
    d_rel = None
    v_rel = None
    if self._last_track is not None:
      d_rel = float(self._last_track.d_rel) - self._range_bias_m
      v_rel = float(self._last_track.v_rel)
    return JeepRadarBrakeGuardResult(
      brake_floor_mps2=brake_floor,
      active=self._active,
      reason=reason,
      confirmed=self._confirmed_index is not None,
      track_index=self._confirmed_index,
      d_rel=d_rel,
      v_rel=v_rel,
      dropout_age_s=age_s,
      range_bias_m=self._range_bias_m,
    )

  @staticmethod
  def _strong_fusion(v_ego, vision, selection):
    track = getattr(selection, "track", None)
    if (
        vision is None or track is None
        or getattr(selection, "reason", None) != "selected"
        or not bool(getattr(vision, "status", False))
        or bool(getattr(vision, "radar", False))
    ):
      return False
    values = (
      v_ego,
      getattr(vision, "d_rel", float("nan")),
      getattr(vision, "v_rel", float("nan")),
      getattr(vision, "model_prob", float("nan")),
      getattr(track, "d_rel", float("nan")),
      getattr(track, "v_rel", float("nan")),
    )
    if not _finite(*values):
      return False
    vision_d = float(vision.d_rel)
    vision_v = float(vision.v_rel)
    radar_d = float(track.d_rel)
    radar_v = float(track.v_rel)
    return (
      float(vision.model_prob) >= MIN_MODEL_PROB
      and float(v_ego) >= MIN_VEGO_MPS
      and radar_d > 0.0
      and radar_v <= -MIN_ACQUIRE_CLOSING_MPS
      and float(v_ego) + radar_v >= MIN_ABSOLUTE_TARGET_SPEED_MPS
      and abs(radar_d - vision_d) <= max(
        MAX_FUSION_DISTANCE_ERROR_M,
        MAX_FUSION_DISTANCE_ERROR_RATIO * vision_d,
      )
      and abs(radar_v - vision_v) <= MAX_FUSION_SPEED_ERROR_MPS
    )

  @staticmethod
  def _track_is_continuous(previous, current, v_ego, dt_s):
    previous_d = float(previous.d_rel)
    previous_v = float(previous.v_rel)
    current_d = float(current.d_rel)
    current_v = float(current.v_rel)
    predicted_d = previous_d + 0.5 * (previous_v + current_v) * dt_s
    distance_tolerance = max(
      MAX_TRACK_DISTANCE_ERROR_M,
      MAX_TRACK_DISTANCE_ERROR_RATIO * previous_d,
    )
    return (
      current_d > 0.0
      and float(v_ego) + current_v >= MIN_ABSOLUTE_TARGET_SPEED_MPS
      and abs(current_d - predicted_d) <= distance_tolerance
      and abs(current_v - previous_v) <= MAX_TRACK_SPEED_STEP_MPS
      and not (
        current_v < -MIN_ACQUIRE_CLOSING_MPS
        and current_d - previous_d > MAX_CLOSING_DISTANCE_RISE_M
      )
    )

  @staticmethod
  def _vision_supports_track(vision, track):
    if (
        vision is None or track is None
        or not bool(getattr(vision, "status", False))
        or bool(getattr(vision, "radar", False))
    ):
      return False
    values = (
      getattr(vision, "d_rel", float("nan")),
      getattr(vision, "v_rel", float("nan")),
      getattr(vision, "model_prob", float("nan")),
      getattr(track, "d_rel", float("nan")),
      getattr(track, "v_rel", float("nan")),
    )
    if not _finite(*values):
      return False
    return (
      float(vision.model_prob) >= MIN_SUPPORT_MODEL_PROB
      and float(vision.d_rel) > 0.0
      and abs(float(track.d_rel) - float(vision.d_rel)) <= max(
        MAX_SUPPORT_DISTANCE_ERROR_M,
        MAX_SUPPORT_DISTANCE_ERROR_RATIO * float(vision.d_rel),
      )
      and abs(float(track.v_rel) - float(vision.v_rel))
        <= MAX_SUPPORT_SPEED_ERROR_MPS
    )

  def _can_hold_stale_floor(self):
    if (
        not self._active
        or self._last_track is None
        or self._last_floor is None
    ):
      return False
    corrected_d = max(
      0.1, float(self._last_track.d_rel) - self._range_bias_m,
    )
    return (
      corrected_d <= STALE_HOLD_MAX_DISTANCE_M
      and self._last_target_speed_mps is not None
      and abs(self._last_target_speed_mps)
        <= STALE_HOLD_MAX_TARGET_SPEED_MPS
      and -float(self._last_track.v_rel) >= STALE_HOLD_MIN_CLOSING_MPS
      and self._last_floor <= -STALE_HOLD_MIN_BRAKE_MPS2
    )

  def _reset_result(self, reason, now_nanos):
    self.reset()
    return self._result(reason, now_nanos=now_nanos)

  def update(
      self,
      *,
      eligible,
      v_ego,
      now_nanos,
      radar_cycle,
      cycle_complete,
      tracks,
      vision,
      selection,
      planner_decelerating,
  ):
    """Update guard state and return an optional nonpositive brake floor."""
    now_nanos = int(now_nanos)
    if (
        self._last_track_time_ns is not None
        and now_nanos < self._last_track_time_ns
    ):
      return self._reset_result("nonmonotonic_time", now_nanos)

    # Cycle count is durable across controller frames; the transient
    # `updated` flag can otherwise be consumed on an odd 100 Hz frame before
    # the 50 Hz actuator block sees it.
    new_cycle = radar_cycle != self._last_cycle
    previous_radar_cycle_time_ns = self._last_radar_cycle_time_ns
    if new_cycle:
      self._last_cycle = radar_cycle
      self._last_radar_cycle_time_ns = now_nanos

    # Observe the bus cycle even while ineligible so an old snapshot cannot
    # become a fresh confirmation when openpilot-long is engaged later.
    if not eligible or not _finite(v_ego) or float(v_ego) < MIN_VEGO_MPS:
      return self._reset_result("ineligible", now_nanos)

    if (
        self._active
        and self._dropout_start_ns is not None
        and now_nanos - self._dropout_start_ns
          >= ABSOLUTE_ACTIVE_TIMEOUT_NS
    ):
      return self._reset_result("absolute_active_timeout", now_nanos)

    tracking_state_present = (
      self._active
      or self._confirmed_index is not None
      or self._confirmation_cycles > 0
    )
    if (
        new_cycle
        and tracking_state_present
        and previous_radar_cycle_time_ns is not None
        and now_nanos - previous_radar_cycle_time_ns > RAW_STALE_NS
    ):
      return self._reset_result("radar_cycle_gap", now_nanos)

    if not new_cycle:
      if (
          self._last_radar_cycle_time_ns is None
          or now_nanos - self._last_radar_cycle_time_ns > RAW_STALE_NS
      ):
        return self._reset_result("radar_stale", now_nanos)
      if self._active:
        if (
            self._last_supported_ns is None
            or now_nanos - self._last_supported_ns > DROPOUT_TIMEOUT_NS
        ):
          return self._reset_result("dropout_timeout", now_nanos)
        if self._holding_stale_floor:
          if not self._can_hold_stale_floor():
            return self._reset_result("stale_hold_released", now_nanos)
          return self._result(
            "bounded_stale_brake_hold", self._last_floor, now_nanos,
          )
        corrected_d = max(
          0.1, float(self._last_track.d_rel) - self._range_bias_m,
        )
        floor = _continuity_brake_floor(
          float(v_ego), corrected_d, float(self._last_track.v_rel),
        )
        return self._result(
          "dropout_continuity", floor, now_nanos,
        )
      return self._result("awaiting_radar_cycle", now_nanos=now_nanos)

    if not cycle_complete:
      return self._reset_result("incomplete_radar_cycle", now_nanos)
    if (
        self._active
        and (
          self._last_supported_ns is None
          or now_nanos - self._last_supported_ns > DROPOUT_TIMEOUT_NS
        )
    ):
      return self._reset_result("dropout_timeout", now_nanos)

    if self._strong_fusion(v_ego, vision, selection):
      track = selection.track
      track_index = int(track.index)
      # A recovered production lead ends radar-only continuity immediately.
      # Require the complete clean confirmation sequence again before a later
      # dropout can reuse it.
      if self._active:
        self._candidate_index = None
        self._confirmation_cycles = 0
        self._confirmed_index = None
        self._bias_samples = []
        self._range_bias_m = 0.0
      if track_index != self._candidate_index:
        self._candidate_index = track_index
        self._confirmation_cycles = 0
        self._bias_samples = []
        self._range_bias_m = 0.0
      elif self._last_track is not None:
        dt_s = max(0.0, (now_nanos - self._last_track_time_ns) / 1e9)
        if not self._track_is_continuous(
            self._last_track, track, v_ego, dt_s,
        ):
          self._confirmation_cycles = 0
          self._bias_samples = []
          self._range_bias_m = 0.0
      self._confirmation_cycles += 1
      observed_bias = _clip(
        float(track.d_rel) - float(vision.d_rel), 0.0, MAX_RANGE_BIAS_M,
      )
      self._bias_samples.append(observed_bias)
      self._bias_samples = self._bias_samples[-15:]
      self._range_bias_m = statistics.median(self._bias_samples)
      self._last_track = track
      self._last_track_time_ns = now_nanos
      self._last_target_speed_mps = float(v_ego) + float(track.v_rel)
      self._last_supported_ns = now_nanos
      self._active = False
      self._holding_stale_floor = False
      self._last_floor = None
      self._dropout_start_ns = None
      if self._confirmation_cycles >= CONFIRM_CYCLES:
        self._confirmed_index = track_index
        return self._result("fused_confirmed", now_nanos=now_nanos)
      self._confirmed_index = None
      return self._result("confirming_fusion", now_nanos=now_nanos)

    if self._confirmed_index is None or self._last_track is None:
      return self._reset_result("fusion_not_confirmed", now_nanos)

    dt_s = max(0.0, (now_nanos - self._last_track_time_ns) / 1e9)
    previous_d = float(self._last_track.d_rel)
    previous_v = float(self._last_track.v_rel)
    predicted_d = previous_d + previous_v * dt_s
    distance_gate = max(
      MAX_TRACK_DISTANCE_ERROR_M,
      MAX_TRACK_DISTANCE_ERROR_RATIO * max(predicted_d, 0.0),
    )
    candidates = []
    for track in tracks:
      values = (
        getattr(track, "d_rel", float("nan")),
        getattr(track, "v_rel", float("nan")),
      )
      if not _finite(*values):
        continue
      current_d = float(track.d_rel)
      current_v = float(track.v_rel)
      distance_error = abs(current_d - predicted_d)
      speed_error = abs(current_v - previous_v)
      if (
          current_d <= 0.0
          or float(v_ego) + current_v < MIN_ABSOLUTE_TARGET_SPEED_MPS
          or distance_error > distance_gate
          or speed_error > MAX_TRACK_SPEED_STEP_MPS
          or (
            current_v < -MIN_ACQUIRE_CLOSING_MPS
            and current_d - previous_d > MAX_CLOSING_DISTANCE_RISE_M
          )
      ):
        continue
      score = (
        distance_error / distance_gate
        + speed_error / MAX_TRACK_SPEED_STEP_MPS
      )
      candidates.append((score, current_d, track))

    candidates.sort(key=lambda candidate: (candidate[0], candidate[1]))
    if not candidates:
      vision_values = (
        getattr(vision, "d_rel", float("nan")),
        getattr(vision, "v_rel", float("nan")),
        getattr(vision, "model_prob", float("nan")),
      )
      if (
          vision is not None
          and bool(getattr(vision, "status", False))
          and not bool(getattr(vision, "radar", False))
          and _finite(*vision_values)
          and float(vision.d_rel) > 0.0
          and float(vision.model_prob) >= VISION_DROPOUT_MODEL_PROB
      ):
        return self._reset_result("vision_reacquired_without_radar", now_nanos)
      if self._can_hold_stale_floor():
        self._holding_stale_floor = True
        return self._result(
          "bounded_stale_brake_hold", self._last_floor, now_nanos,
        )
      return self._reset_result("track_discontinuity", now_nanos)
    if (
        len(candidates) > 1
        and candidates[1][0] - candidates[0][0]
          < TRACK_AMBIGUITY_SCORE_MARGIN
    ):
      return self._reset_result("ambiguous_track_continuity", now_nanos)
    track = candidates[0][2]
    self._confirmed_index = int(track.index)
    current_d = float(track.d_rel)
    current_v = float(track.v_rel)

    # A credible but disagreeing vision lead may be a real new object. Lower
    # confidence vision may extend support only when it tightly agrees with
    # the already kinematically tracked raw object; it can never acquire one.
    vision_values = (
      getattr(vision, "d_rel", float("nan")),
      getattr(vision, "v_rel", float("nan")),
      getattr(vision, "model_prob", float("nan")),
    )
    production_lead_eligible = (
      vision is not None
      and bool(getattr(vision, "status", False))
      and not bool(getattr(vision, "radar", False))
      and _finite(*vision_values)
      and float(vision.d_rel) > 0.0
      and float(vision.model_prob) >= VISION_DROPOUT_MODEL_PROB
    )
    vision_supports_track = self._vision_supports_track(vision, track)
    if vision_supports_track:
      self._last_supported_ns = now_nanos
    elif production_lead_eligible:
      return self._reset_result("vision_disagreement", now_nanos)

    # The raw target can remain passively remembered through a subscriber-
    # pairing mismatch, but it is not a dropout guard while the production
    # lead is still eligible.  This specifically prevents a one-cycle brake
    # floor when strong fusion misses only the tighter speed/range tolerance.
    if production_lead_eligible:
      self._last_track = track
      self._last_track_time_ns = now_nanos
      self._last_target_speed_mps = float(v_ego) + current_v
      self._active = False
      self._holding_stale_floor = False
      self._last_floor = None
      self._dropout_start_ns = None
      return self._result("vision_supported_tracking", now_nanos=now_nanos)

    if not self._active:
      if (
          not planner_decelerating
          or -current_v < MIN_CONTINUITY_CLOSING_MPS
      ):
        return self._reset_result("dropout_not_braking", now_nanos)
      self._active = True
      self._dropout_start_ns = now_nanos
    elif (
        self._last_supported_ns is None
        or now_nanos - self._last_supported_ns > DROPOUT_TIMEOUT_NS
    ):
      return self._reset_result("dropout_timeout", now_nanos)

    self._last_track = track
    self._last_track_time_ns = now_nanos
    self._last_target_speed_mps = float(v_ego) + current_v
    self._holding_stale_floor = False
    corrected_d = max(0.1, current_d - self._range_bias_m)
    floor = _continuity_brake_floor(float(v_ego), corrected_d, current_v)
    self._last_floor = floor
    return self._result("dropout_continuity", floor, now_nanos)

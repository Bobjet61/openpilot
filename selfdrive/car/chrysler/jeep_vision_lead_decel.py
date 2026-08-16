"""Bounded camera-lead deceleration estimate for the White-Panda Jeep.

The generic vision-only radar path intentionally reports zero lead
acceleration.  On this Jeep that delayed two recorded stops even though the
model had already predicted sustained lead deceleration.  This helper accepts
only a short run of high-confidence, kinematically continuous model frames and
can only return a non-positive acceleration.  It does not command braking;
the normal longitudinal MPC remains responsible for the trajectory.
"""

from collections.abc import Sequence
from dataclasses import dataclass
import math


CONFIRM_FRAMES = 3
MAX_MODEL_FRAME_GAP_NS = 125_000_000
MAX_MODEL_AGE_NS = 150_000_000
MIN_MODEL_PROBABILITY = 0.90
MIN_CLOSING_SPEED_MPS = 1.0
MIN_SUSTAINED_DECEL_MPS2 = 0.5
MAX_ACCEL_STD_MPS2 = 1.0
MIN_EGO_SPEED_MPS = 3.0
MIN_LEAD_ACCEL_MPS2 = -2.5
MAX_DISTANCE_ERROR_M = 3.0
MAX_DISTANCE_ERROR_FRACTION = 0.10
MAX_VREL_CHANGE_MPS = 3.0
VISION_LEAD_ACCEL_TAU = 0.3


def jeep_vision_lead_decel_runtime_eligible(
    *,
    scoped_to_jeep_wp_op_long: bool,
    inputs_valid: bool,
    model_age_ns: int,
    cruise_enabled: bool,
    forward_gear: bool,
    brake_pressed: bool,
    stock_aeb: bool,
    acc_faulted: bool,
) -> bool:
  """Fail-closed runtime gate; gas override intentionally preserves perception."""
  return bool(
    scoped_to_jeep_wp_op_long
    and inputs_valid
    and 0 <= int(model_age_ns) <= MAX_MODEL_AGE_NS
    and cruise_enabled
    and forward_gear
    and not brake_pressed
    and not stock_aeb
    and not acc_faulted
  )


@dataclass(frozen=True)
class JeepVisionLeadDecelResult:
  a_lead_mps2: float
  confirmed: bool
  reason: str
  consecutive_frames: int


class JeepVisionLeadDecelEstimator:
  """Confirm a sustained, camera-only lead deceleration prediction."""

  def __init__(self):
    self._last_seen_model_time_ns: int | None = None
    self._last_candidate_time_ns: int | None = None
    self._last_distance_m: float | None = None
    self._last_v_rel_mps: float | None = None
    self._consecutive_frames = 0
    self._result = self._make_result(0.0, False, "uninitialized")

  @property
  def result(self) -> JeepVisionLeadDecelResult:
    return self._result

  def _make_result(self, a_lead_mps2: float, confirmed: bool, reason: str) -> JeepVisionLeadDecelResult:
    return JeepVisionLeadDecelResult(
      a_lead_mps2=min(0.0, float(a_lead_mps2)),
      confirmed=bool(confirmed),
      reason=reason,
      consecutive_frames=self._consecutive_frames,
    )

  def _clear_candidate(self):
    self._last_candidate_time_ns = None
    self._last_distance_m = None
    self._last_v_rel_mps = None
    self._consecutive_frames = 0

  def reset(self, model_mono_time_ns: int | None = None, reason: str = "reset") -> JeepVisionLeadDecelResult:
    self._clear_candidate()
    if model_mono_time_ns is not None:
      self._last_seen_model_time_ns = int(model_mono_time_ns)
    self._result = self._make_result(0.0, False, reason)
    return self._result

  @staticmethod
  def _read_model_values(
      x: Sequence[float],
      v: Sequence[float],
      a: Sequence[float],
      a_std: Sequence[float],
  ) -> tuple[float, float, float, float, float, float] | None:
    try:
      if len(x) < 1 or len(v) < 1 or len(a) < 2 or len(a_std) < 2:
        return None
      values = (
        float(x[0]),
        float(v[0]),
        float(a[0]),
        float(a[1]),
        float(a_std[0]),
        float(a_std[1]),
      )
    except (IndexError, TypeError, ValueError):
      return None
    return values if all(math.isfinite(value) for value in values) else None

  def update(
      self,
      *,
      model_mono_time_ns: int,
      v_ego_mps: float,
      model_v_ego_mps: float,
      probability: float,
      x: Sequence[float],
      v: Sequence[float],
      a: Sequence[float],
      a_std: Sequence[float],
  ) -> JeepVisionLeadDecelResult:
    try:
      model_mono_time_ns = int(model_mono_time_ns)
    except (TypeError, ValueError):
      return self.reset(reason="invalid_model_time")
    if model_mono_time_ns <= 0:
      return self.reset(reason="invalid_model_time")

    if self._last_seen_model_time_ns == model_mono_time_ns:
      self._result = self._make_result(
        self._result.a_lead_mps2,
        self._result.confirmed,
        "duplicate_model_frame",
      )
      return self._result

    if self._last_seen_model_time_ns is not None and model_mono_time_ns < self._last_seen_model_time_ns:
      self._last_seen_model_time_ns = model_mono_time_ns
      return self.reset(model_mono_time_ns, "out_of_order_model_frame")

    previous_seen_time_ns = self._last_seen_model_time_ns
    self._last_seen_model_time_ns = model_mono_time_ns

    try:
      probability = float(probability)
      v_ego_mps = float(v_ego_mps)
      model_v_ego_mps = float(model_v_ego_mps)
    except (TypeError, ValueError):
      return self.reset(model_mono_time_ns, "invalid_model_values")

    values = self._read_model_values(x, v, a, a_std)
    if values is None or not all(math.isfinite(value) for value in (probability, v_ego_mps, model_v_ego_mps)):
      return self.reset(model_mono_time_ns, "invalid_model_values")

    distance_m, v_lead_mps, a_now_mps2, a_2s_mps2, a_now_std_mps2, a_2s_std_mps2 = values
    v_rel_mps = v_lead_mps - model_v_ego_mps

    gate_reason = None
    if distance_m <= 0.0:
      gate_reason = "invalid_distance"
    elif v_ego_mps < MIN_EGO_SPEED_MPS:
      gate_reason = "low_ego_speed"
    elif probability < MIN_MODEL_PROBABILITY:
      gate_reason = "low_probability"
    elif v_rel_mps > -MIN_CLOSING_SPEED_MPS:
      gate_reason = "not_closing"
    elif a_now_mps2 > -MIN_SUSTAINED_DECEL_MPS2 or a_2s_mps2 > -MIN_SUSTAINED_DECEL_MPS2:
      gate_reason = "decel_not_sustained"
    elif not (0.0 <= a_now_std_mps2 <= MAX_ACCEL_STD_MPS2 and 0.0 <= a_2s_std_mps2 <= MAX_ACCEL_STD_MPS2):
      gate_reason = "accel_uncertain"

    if gate_reason is not None:
      return self.reset(model_mono_time_ns, gate_reason)

    frame_gap_too_large = (
      previous_seen_time_ns is not None
      and model_mono_time_ns - previous_seen_time_ns > MAX_MODEL_FRAME_GAP_NS
    )
    continuous_identity = False
    if (
      not frame_gap_too_large
      and self._last_candidate_time_ns is not None
      and self._last_distance_m is not None
      and self._last_v_rel_mps is not None
    ):
      dt = (model_mono_time_ns - self._last_candidate_time_ns) / 1e9
      if 0.0 < dt <= MAX_MODEL_FRAME_GAP_NS / 1e9:
        predicted_distance_m = self._last_distance_m + 0.5 * (self._last_v_rel_mps + v_rel_mps) * dt
        distance_tolerance_m = max(
          MAX_DISTANCE_ERROR_M,
          MAX_DISTANCE_ERROR_FRACTION * self._last_distance_m,
        )
        continuous_identity = (
          abs(distance_m - predicted_distance_m) <= distance_tolerance_m
          and abs(v_rel_mps - self._last_v_rel_mps) <= MAX_VREL_CHANGE_MPS
        )

    self._consecutive_frames = self._consecutive_frames + 1 if continuous_identity else 1
    self._last_candidate_time_ns = model_mono_time_ns
    self._last_distance_m = distance_m
    self._last_v_rel_mps = v_rel_mps

    if self._consecutive_frames < CONFIRM_FRAMES:
      reason = "confirming" if not frame_gap_too_large else "model_frame_gap"
      self._result = self._make_result(0.0, False, reason)
      return self._result

    # Both predictions must show braking. Use the less-negative value so the
    # estimate cannot exaggerate the model's sustained deceleration, then
    # retain a hard lower bound before the MPC consumes it.
    a_lead_mps2 = max(MIN_LEAD_ACCEL_MPS2, max(a_now_mps2, a_2s_mps2))
    self._result = self._make_result(a_lead_mps2, True, "confirmed")
    return self._result

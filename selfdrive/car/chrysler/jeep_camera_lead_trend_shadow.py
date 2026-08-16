"""Passive camera-range trend observer for the White-Panda Jeep.

This module is deliberately telemetry-only.  It does not expose an
acceleration, brake request, planner lead, or CAN message.  A candidate is
reported only after a short, fresh, high-confidence camera-only range history
shows a continuous severe closing trend.  Raw FCA radar may annotate the
result, but it is never used to decide whether the camera candidate exists or
how severe it is.
"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
import math
import statistics


MIN_MODEL_PROBABILITY = 0.95
MIN_EGO_SPEED_MPS = 12.0
MIN_DISTINCT_SAMPLES = 5
MIN_WINDOW_SPAN_NS = 250_000_000
MAX_WINDOW_SPAN_NS = 400_000_000
MAX_SAMPLE_GAP_NS = 100_000_000
MAX_MODEL_AGE_NS = 150_000_000
MIN_OBSERVED_CLOSING_MPS = 4.0
MAX_CAMERA_TTC_S = 6.0

# Range continuity is intentionally tighter than the final candidate gates.
# It permits small model jitter, but rejects an abrupt lead-identity switch.
# One E221 camera frame rose 0.63 m amid four strong decreases in the same
# 250 ms window.  Treat up to 0.75 m as model jitter, then require at least
# 80% decreasing intervals plus a stable robust slope before qualifying.
MAX_DISTANCE_RISE_M = 0.75
# The model range is noisier than a physical radar range.  E221 contains
# coherent 1.7-3.3 m drops per 50 ms; the prediction-error, monotonic-fraction,
# and robust-slope gates below still have to agree across the full window.
MAX_PLAUSIBLE_CLOSING_MPS = 70.0
RANGE_STEP_SLACK_M = 0.60
MAX_PREDICTION_ERROR_M = 2.0
MAX_PREDICTION_ERROR_FRACTION = 0.04
MIN_DECREASING_FRACTION = 0.80
MAX_SLOPE_MAD_MPS = 3.0
MAX_SLOPE_MAD_FRACTION = 0.50
MIN_ENDPOINT_CLOSING_MPS = 3.5

# Radar is an annotation only.  These constants cannot affect the camera
# candidate calculation below.
MAX_RADAR_AGE_NS = 150_000_000
MAX_RADAR_DISTANCE_ERROR_M = 5.0
MAX_RADAR_DISTANCE_ERROR_FRACTION = 0.15


def jeep_camera_lead_trend_runtime_eligible(
    *,
    scoped_to_jeep_wp_op_long: bool,
    inputs_valid: bool,
    model_age_ns: int,
    controls_enabled: bool,
    long_controls_active: bool,
    cruise_available: bool,
    cruise_enabled: bool,
    forward_gear: bool,
    brake_pressed: bool,
    gas_pressed: bool,
    stock_aeb: bool,
    acc_faulted: bool,
) -> bool:
  """Fail-closed live gate for collecting shadow samples."""
  try:
    model_age_ns = int(model_age_ns)
  except (TypeError, ValueError):
    return False
  return bool(
    scoped_to_jeep_wp_op_long
    and inputs_valid
    and 0 <= model_age_ns <= MAX_MODEL_AGE_NS
    and controls_enabled
    and long_controls_active
    and cruise_available
    and cruise_enabled
    and forward_gear
    and not brake_pressed
    and not gas_pressed
    and not stock_aeb
    and not acc_faulted
  )


@dataclass(frozen=True)
class JeepCameraLeadTrendShadowResult:
  hazard_candidate: bool
  reason: str
  distinct_sample: bool
  sample_count: int
  window_span_s: float
  d_rel_m: float | None
  observed_closing_mps: float | None
  camera_ttc_s: float | None
  monotonic_fraction: float
  slope_mad_mps: float | None
  model_age_s: float
  radar_corroborated: bool
  radar_d_rel_m: float | None
  radar_v_rel_mps: float | None
  radar_distance_error_m: float | None


@dataclass(frozen=True)
class JeepCameraLeadTrendShadowWindow:
  distinct_samples: int
  eligible_samples: int
  hazard_samples: int
  hazard_events: int
  radar_corroborated_hazard_samples: int
  reason_counts: tuple[tuple[str, int], ...]


@dataclass(frozen=True)
class _CameraSample:
  model_time_ns: int
  d_rel_m: float


def _finite(*values: float) -> bool:
  try:
    return all(math.isfinite(float(value)) for value in values)
  except (TypeError, ValueError):
    return False


def _robust_range_slope(samples: tuple[_CameraSample, ...]) -> tuple[float, float]:
  slopes = []
  for index, first in enumerate(samples[:-1]):
    for second in samples[index + 1:]:
      dt_s = (second.model_time_ns - first.model_time_ns) / 1e9
      if dt_s > 0.0:
        slopes.append((second.d_rel_m - first.d_rel_m) / dt_s)
  slope_mps = statistics.median(slopes)
  slope_mad_mps = statistics.median(abs(value - slope_mps) for value in slopes)
  return slope_mps, slope_mad_mps


class JeepCameraLeadTrendShadow:
  """Observe a severe camera dRel trend without producing control output."""

  def __init__(self):
    self._samples: deque[_CameraSample] = deque()
    self._last_model_time_ns: int | None = None
    self._hazard_active = False
    self._distinct_samples = 0
    self._eligible_samples = 0
    self._hazard_samples = 0
    self._hazard_events = 0
    self._radar_corroborated_hazard_samples = 0
    self._reason_counts: Counter[str] = Counter()
    self._result = self._make_result("uninitialized")

  @property
  def result(self) -> JeepCameraLeadTrendShadowResult:
    return self._result

  def _make_result(
      self,
      reason: str,
      *,
      hazard_candidate: bool = False,
      distinct_sample: bool = False,
      observed_closing_mps: float | None = None,
      camera_ttc_s: float | None = None,
      monotonic_fraction: float = 0.0,
      slope_mad_mps: float | None = None,
      model_age_ns: int = 0,
      radar_d_rel_m: float | None = None,
      radar_v_rel_mps: float | None = None,
      radar_age_ns: int | None = None,
  ) -> JeepCameraLeadTrendShadowResult:
    latest_d_rel_m = self._samples[-1].d_rel_m if self._samples else None
    window_span_s = (
      (self._samples[-1].model_time_ns - self._samples[0].model_time_ns) / 1e9
      if len(self._samples) >= 2 else 0.0
    )

    # This annotation is intentionally evaluated only after the camera-only
    # hazard decision has been finalized above the call site.  It cannot
    # create, suppress, or change the severity of a candidate.
    radar_corroborated = False
    radar_distance_error_m = None
    if (
        latest_d_rel_m is not None
        and radar_age_ns is not None
        and 0 <= int(radar_age_ns) <= MAX_RADAR_AGE_NS
        and _finite(radar_d_rel_m, radar_v_rel_mps)
        and float(radar_d_rel_m) > 0.0
    ):
      radar_d_rel_m = float(radar_d_rel_m)
      radar_v_rel_mps = float(radar_v_rel_mps)
      radar_distance_error_m = abs(radar_d_rel_m - latest_d_rel_m)
      radar_corroborated = radar_distance_error_m <= max(
        MAX_RADAR_DISTANCE_ERROR_M,
        MAX_RADAR_DISTANCE_ERROR_FRACTION * latest_d_rel_m,
      )
    else:
      radar_d_rel_m = None
      radar_v_rel_mps = None

    return JeepCameraLeadTrendShadowResult(
      hazard_candidate=bool(hazard_candidate),
      reason=reason,
      distinct_sample=bool(distinct_sample),
      sample_count=len(self._samples),
      window_span_s=window_span_s,
      d_rel_m=latest_d_rel_m,
      observed_closing_mps=observed_closing_mps,
      camera_ttc_s=camera_ttc_s,
      monotonic_fraction=monotonic_fraction,
      slope_mad_mps=slope_mad_mps,
      model_age_s=max(0.0, float(model_age_ns) / 1e9),
      radar_corroborated=radar_corroborated,
      radar_d_rel_m=radar_d_rel_m,
      radar_v_rel_mps=radar_v_rel_mps,
      radar_distance_error_m=radar_distance_error_m,
    )

  def _record_result(self, result: JeepCameraLeadTrendShadowResult):
    self._reason_counts[result.reason] += 1
    if result.distinct_sample:
      self._distinct_samples += 1
      self._eligible_samples += 1
      if result.hazard_candidate:
        self._hazard_samples += 1
        if result.radar_corroborated:
          self._radar_corroborated_hazard_samples += 1
        if not self._hazard_active:
          self._hazard_events += 1
      self._hazard_active = result.hazard_candidate
    self._result = result
    return result

  def reset(self, reason: str = "reset") -> JeepCameraLeadTrendShadowResult:
    self._samples.clear()
    self._hazard_active = False
    result = self._make_result(reason)
    return self._record_result(result)

  def snapshot(self, reset: bool = True) -> JeepCameraLeadTrendShadowWindow:
    window = JeepCameraLeadTrendShadowWindow(
      distinct_samples=self._distinct_samples,
      eligible_samples=self._eligible_samples,
      hazard_samples=self._hazard_samples,
      hazard_events=self._hazard_events,
      radar_corroborated_hazard_samples=self._radar_corroborated_hazard_samples,
      reason_counts=tuple(sorted(self._reason_counts.items())),
    )
    if reset:
      self._distinct_samples = 0
      self._eligible_samples = 0
      self._hazard_samples = 0
      self._hazard_events = 0
      self._radar_corroborated_hazard_samples = 0
      self._reason_counts.clear()
    return window

  def update(
      self,
      *,
      eligible: bool,
      model_mono_time_ns: int,
      model_age_ns: int,
      v_ego_mps: float,
      vision_status: bool,
      vision_radar: bool,
      model_probability: float,
      d_rel_m: float,
      radar_d_rel_m: float | None = None,
      radar_v_rel_mps: float | None = None,
      radar_age_ns: int | None = None,
  ) -> JeepCameraLeadTrendShadowResult:
    if not eligible:
      return self.reset("ineligible")

    try:
      model_mono_time_ns = int(model_mono_time_ns)
      model_age_ns = int(model_age_ns)
      v_ego_mps = float(v_ego_mps)
      model_probability = float(model_probability)
      d_rel_m = float(d_rel_m)
    except (TypeError, ValueError):
      return self.reset("invalid_values")

    if (
        model_mono_time_ns <= 0
        or not _finite(v_ego_mps, model_probability, d_rel_m)
        or model_age_ns < 0
        or model_age_ns > MAX_MODEL_AGE_NS
    ):
      return self.reset("stale_or_invalid_model")

    if self._last_model_time_ns == model_mono_time_ns:
      prior = self._result
      result = self._make_result(
        "duplicate_model_frame",
        hazard_candidate=prior.hazard_candidate,
        observed_closing_mps=prior.observed_closing_mps,
        camera_ttc_s=prior.camera_ttc_s,
        monotonic_fraction=prior.monotonic_fraction,
        slope_mad_mps=prior.slope_mad_mps,
        model_age_ns=model_age_ns,
        radar_d_rel_m=radar_d_rel_m,
        radar_v_rel_mps=radar_v_rel_mps,
        radar_age_ns=radar_age_ns,
      )
      return self._record_result(result)

    if (
        self._last_model_time_ns is not None
        and model_mono_time_ns < self._last_model_time_ns
    ):
      self._last_model_time_ns = model_mono_time_ns
      return self.reset("out_of_order_model_frame")
    self._last_model_time_ns = model_mono_time_ns

    if not vision_status or vision_radar:
      return self.reset("not_camera_only")
    if model_probability < MIN_MODEL_PROBABILITY:
      return self.reset("low_probability")
    if v_ego_mps < MIN_EGO_SPEED_MPS:
      return self.reset("low_ego_speed")
    if d_rel_m <= 0.0:
      return self.reset("invalid_distance")

    reset_reason = None
    if self._samples:
      previous = self._samples[-1]
      gap_ns = model_mono_time_ns - previous.model_time_ns
      if gap_ns > MAX_SAMPLE_GAP_NS:
        reset_reason = "sample_gap"
      else:
        dt_s = gap_ns / 1e9
        distance_delta_m = d_rel_m - previous.d_rel_m
        maximum_drop_m = MAX_PLAUSIBLE_CLOSING_MPS * dt_s + RANGE_STEP_SLACK_M
        if distance_delta_m > MAX_DISTANCE_RISE_M:
          reset_reason = "range_rise"
        elif -distance_delta_m > maximum_drop_m:
          reset_reason = "range_jump"
        elif len(self._samples) >= 3:
          prior_slope_mps, _ = _robust_range_slope(tuple(self._samples))
          predicted_distance_m = previous.d_rel_m + prior_slope_mps * dt_s
          prediction_tolerance_m = max(
            MAX_PREDICTION_ERROR_M,
            MAX_PREDICTION_ERROR_FRACTION * previous.d_rel_m,
          )
          if abs(d_rel_m - predicted_distance_m) > prediction_tolerance_m:
            reset_reason = "identity_discontinuity"

    if reset_reason is not None:
      self._samples.clear()
      self._hazard_active = False

    self._samples.append(_CameraSample(model_mono_time_ns, d_rel_m))
    while (
        len(self._samples) > 1
        and model_mono_time_ns - self._samples[0].model_time_ns
          > MAX_WINDOW_SPAN_NS
    ):
      self._samples.popleft()

    if reset_reason is not None:
      result = self._make_result(
        reset_reason,
        distinct_sample=True,
        model_age_ns=model_age_ns,
        radar_d_rel_m=radar_d_rel_m,
        radar_v_rel_mps=radar_v_rel_mps,
        radar_age_ns=radar_age_ns,
      )
      return self._record_result(result)

    if len(self._samples) < MIN_DISTINCT_SAMPLES:
      result = self._make_result(
        "collecting_samples",
        distinct_sample=True,
        model_age_ns=model_age_ns,
        radar_d_rel_m=radar_d_rel_m,
        radar_v_rel_mps=radar_v_rel_mps,
        radar_age_ns=radar_age_ns,
      )
      return self._record_result(result)

    samples = tuple(self._samples)
    span_ns = samples[-1].model_time_ns - samples[0].model_time_ns
    if span_ns < MIN_WINDOW_SPAN_NS:
      result = self._make_result(
        "window_too_short",
        distinct_sample=True,
        model_age_ns=model_age_ns,
        radar_d_rel_m=radar_d_rel_m,
        radar_v_rel_mps=radar_v_rel_mps,
        radar_age_ns=radar_age_ns,
      )
      return self._record_result(result)

    slope_mps, slope_mad_mps = _robust_range_slope(samples)
    observed_closing_mps = max(0.0, -slope_mps)
    endpoint_closing_mps = (
      (samples[0].d_rel_m - samples[-1].d_rel_m) / (span_ns / 1e9)
    )
    decreasing_intervals = sum(
      second.d_rel_m < first.d_rel_m - 0.01
      for first, second in zip(samples, samples[1:], strict=False)
    )
    monotonic_fraction = decreasing_intervals / (len(samples) - 1)
    camera_ttc_s = (
      samples[-1].d_rel_m / observed_closing_mps
      if observed_closing_mps > 0.0 else math.inf
    )

    reason = "camera_trend_candidate"
    hazard_candidate = True
    if monotonic_fraction < MIN_DECREASING_FRACTION:
      reason = "non_monotonic"
      hazard_candidate = False
    elif slope_mad_mps > max(
        MAX_SLOPE_MAD_MPS,
        MAX_SLOPE_MAD_FRACTION * observed_closing_mps,
    ):
      reason = "unstable_slope"
      hazard_candidate = False
    elif endpoint_closing_mps < MIN_ENDPOINT_CLOSING_MPS:
      reason = "weak_endpoint_trend"
      hazard_candidate = False
    elif observed_closing_mps < MIN_OBSERVED_CLOSING_MPS:
      reason = "closing_below_threshold"
      hazard_candidate = False
    elif camera_ttc_s > MAX_CAMERA_TTC_S:
      reason = "ttc_above_threshold"
      hazard_candidate = False

    # Finalize the camera-only decision before adding raw-radar annotation.
    result = self._make_result(
      reason,
      hazard_candidate=hazard_candidate,
      distinct_sample=True,
      observed_closing_mps=observed_closing_mps,
      camera_ttc_s=camera_ttc_s,
      monotonic_fraction=monotonic_fraction,
      slope_mad_mps=slope_mad_mps,
      model_age_ns=model_age_ns,
      radar_d_rel_m=radar_d_rel_m,
      radar_v_rel_mps=radar_v_rel_mps,
      radar_age_ns=radar_age_ns,
    )
    return self._record_result(result)

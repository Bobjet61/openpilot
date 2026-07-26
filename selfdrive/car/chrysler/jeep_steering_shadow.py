"""Passive Jeep steering-limit measurements for diagnostic logging only."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class JeepSteeringWindow:
  samples: int
  active_samples: int
  request_near_full_samples: int
  request_at_ceiling_samples: int
  applied_at_ceiling_samples: int
  error_limited_samples: int
  rate_limited_samples: int
  suppressed_samples: int
  driver_override_samples: int
  eps_over_limit_samples: int
  steer_required_samples: int
  temporary_fault_samples: int
  permanent_fault_samples: int
  limiter_mismatch_samples: int
  max_requested_normalized: float
  max_requested_raw: int
  max_limited_raw: int
  max_applied_raw: int
  max_eps_torque: float
  max_driver_torque: float
  max_request_limited_gap: int
  max_request_applied_gap: int
  longest_request_ceiling_ms: int
  longest_applied_ceiling_ms: int


@dataclass(frozen=True)
class JeepSteeringRateCandidateWindow:
  samples: int
  active_samples: int
  changed_samples: int
  improved_samples: int
  worse_samples: int
  equal_samples: int
  current_panda_rate_violation_samples: int
  candidate_ceiling_samples: int
  max_candidate_raw: int
  max_candidate_delta: int
  max_candidate_divergence: int
  mean_current_request_gap: float
  mean_candidate_request_gap: float


def clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


def finite_abs(value: float) -> float:
  return abs(value) if math.isfinite(value) else 0.0


class JeepSteeringShadow:
  """Classify limits without changing the controller's steering command."""

  def __init__(
      self,
      steer_max: int,
      steer_delta_up: int,
      steer_delta_down: int,
      steer_error_max: int,
      driver_threshold: int,
      sample_hz: int = 50,
  ):
    self.steer_max = steer_max
    self.steer_delta_up = steer_delta_up
    self.steer_delta_down = steer_delta_down
    self.steer_error_max = steer_error_max
    self.driver_threshold = driver_threshold
    self.sample_hz = sample_hz
    self._request_ceiling_run = 0
    self._applied_ceiling_run = 0
    self._reset_window()

  def _reset_window(self):
    self.samples = 0
    self.active_samples = 0
    self.request_near_full_samples = 0
    self.request_at_ceiling_samples = 0
    self.applied_at_ceiling_samples = 0
    self.error_limited_samples = 0
    self.rate_limited_samples = 0
    self.suppressed_samples = 0
    self.driver_override_samples = 0
    self.eps_over_limit_samples = 0
    self.steer_required_samples = 0
    self.temporary_fault_samples = 0
    self.permanent_fault_samples = 0
    self.limiter_mismatch_samples = 0
    self.max_requested_normalized = 0.0
    self.max_requested_raw = 0
    self.max_limited_raw = 0
    self.max_applied_raw = 0
    self.max_eps_torque = 0.0
    self.max_driver_torque = 0.0
    self.max_request_limited_gap = 0
    self.max_request_applied_gap = 0
    self.longest_request_ceiling_samples = 0
    self.longest_applied_ceiling_samples = 0

  def _error_limited_target(
      self,
      requested_raw: int,
      eps_torque: float,
  ) -> int:
    if not math.isfinite(eps_torque):
      eps_torque = 0.0
    max_allowed = min(
      max(eps_torque + self.steer_error_max, self.steer_error_max),
      self.steer_max,
    )
    min_allowed = max(
      min(eps_torque - self.steer_error_max, -self.steer_error_max),
      -self.steer_max,
    )
    return int(round(clip(requested_raw, min_allowed, max_allowed)))

  def _rate_limited_target(
      self,
      error_limited_target: int,
      previous_applied_raw: int,
  ) -> int:
    if previous_applied_raw > 0:
      lower = max(
        previous_applied_raw - self.steer_delta_down,
        -self.steer_delta_up,
      )
      upper = previous_applied_raw + self.steer_delta_up
    else:
      lower = previous_applied_raw - self.steer_delta_up
      upper = min(
        previous_applied_raw + self.steer_delta_down,
        self.steer_delta_up,
      )
    return int(round(clip(error_limited_target, lower, upper)))

  def update(
      self,
      *,
      requested_normalized: float,
      requested_raw: int,
      limited_raw: int,
      applied_raw: int,
      previous_applied_raw: int,
      eps_torque: float,
      driver_torque: float,
      control_allowed: bool,
      steer_required: bool,
      temporary_fault: bool,
      permanent_fault: bool,
  ):
    error_limited_target = self._error_limited_target(
      requested_raw,
      eps_torque,
    )
    rate_limited_target = self._rate_limited_target(
      error_limited_target,
      previous_applied_raw,
    )
    error_limited = error_limited_target != requested_raw
    rate_limited = rate_limited_target != error_limited_target
    request_at_ceiling = abs(requested_raw) >= self.steer_max
    applied_at_ceiling = abs(applied_raw) >= self.steer_max

    self.samples += 1
    self.active_samples += int(control_allowed)
    self.request_near_full_samples += int(
      finite_abs(requested_normalized) >= 0.99,
    )
    self.request_at_ceiling_samples += int(request_at_ceiling)
    self.applied_at_ceiling_samples += int(applied_at_ceiling)
    self.error_limited_samples += int(error_limited)
    self.rate_limited_samples += int(rate_limited)
    self.suppressed_samples += int(
      not control_allowed and limited_raw != 0,
    )
    self.driver_override_samples += int(
      finite_abs(driver_torque) > self.driver_threshold,
    )
    self.eps_over_limit_samples += int(
      finite_abs(eps_torque) > self.steer_max,
    )
    self.steer_required_samples += int(steer_required)
    self.temporary_fault_samples += int(temporary_fault)
    self.permanent_fault_samples += int(permanent_fault)
    self.limiter_mismatch_samples += int(
      limited_raw != rate_limited_target,
    )

    self.max_requested_normalized = max(
      self.max_requested_normalized,
      finite_abs(requested_normalized),
    )
    self.max_requested_raw = max(
      self.max_requested_raw,
      abs(requested_raw),
    )
    self.max_limited_raw = max(self.max_limited_raw, abs(limited_raw))
    self.max_applied_raw = max(self.max_applied_raw, abs(applied_raw))
    self.max_eps_torque = max(
      self.max_eps_torque,
      finite_abs(eps_torque),
    )
    self.max_driver_torque = max(
      self.max_driver_torque,
      finite_abs(driver_torque),
    )
    self.max_request_limited_gap = max(
      self.max_request_limited_gap,
      abs(requested_raw - limited_raw),
    )
    self.max_request_applied_gap = max(
      self.max_request_applied_gap,
      abs(requested_raw - applied_raw),
    )

    self._request_ceiling_run = (
      self._request_ceiling_run + 1 if request_at_ceiling else 0
    )
    self._applied_ceiling_run = (
      self._applied_ceiling_run + 1 if applied_at_ceiling else 0
    )
    self.longest_request_ceiling_samples = max(
      self.longest_request_ceiling_samples,
      self._request_ceiling_run,
    )
    self.longest_applied_ceiling_samples = max(
      self.longest_applied_ceiling_samples,
      self._applied_ceiling_run,
    )

  def snapshot(self) -> JeepSteeringWindow:
    window = JeepSteeringWindow(
      samples=self.samples,
      active_samples=self.active_samples,
      request_near_full_samples=self.request_near_full_samples,
      request_at_ceiling_samples=self.request_at_ceiling_samples,
      applied_at_ceiling_samples=self.applied_at_ceiling_samples,
      error_limited_samples=self.error_limited_samples,
      rate_limited_samples=self.rate_limited_samples,
      suppressed_samples=self.suppressed_samples,
      driver_override_samples=self.driver_override_samples,
      eps_over_limit_samples=self.eps_over_limit_samples,
      steer_required_samples=self.steer_required_samples,
      temporary_fault_samples=self.temporary_fault_samples,
      permanent_fault_samples=self.permanent_fault_samples,
      limiter_mismatch_samples=self.limiter_mismatch_samples,
      max_requested_normalized=self.max_requested_normalized,
      max_requested_raw=self.max_requested_raw,
      max_limited_raw=self.max_limited_raw,
      max_applied_raw=self.max_applied_raw,
      max_eps_torque=self.max_eps_torque,
      max_driver_torque=self.max_driver_torque,
      max_request_limited_gap=self.max_request_limited_gap,
      max_request_applied_gap=self.max_request_applied_gap,
      longest_request_ceiling_ms=round(
        self.longest_request_ceiling_samples * 1000 / self.sample_hz,
      ),
      longest_applied_ceiling_ms=round(
        self.longest_applied_ceiling_samples * 1000 / self.sample_hz,
      ),
    )
    self._reset_window()
    return window


class JeepSteeringRateCandidateShadow:
  """Compare a faster in-memory slew rate without changing applied steering."""

  def __init__(
      self,
      *,
      steer_max: int,
      candidate_delta_up: int,
      candidate_delta_down: int,
      steer_error_max: int,
      installed_delta_limit: int,
  ):
    self.steer_max = steer_max
    self.candidate_delta_up = candidate_delta_up
    self.candidate_delta_down = candidate_delta_down
    self.steer_error_max = steer_error_max
    self.installed_delta_limit = installed_delta_limit
    self.candidate_applied_last = 0
    self._reset_window()

  def _reset_window(self):
    self.samples = 0
    self.active_samples = 0
    self.changed_samples = 0
    self.improved_samples = 0
    self.worse_samples = 0
    self.equal_samples = 0
    self.current_panda_rate_violation_samples = 0
    self.candidate_ceiling_samples = 0
    self.max_candidate_raw = 0
    self.max_candidate_delta = 0
    self.max_candidate_divergence = 0
    self.current_request_gap_sum = 0
    self.candidate_request_gap_sum = 0

  def _error_limited_target(
      self,
      requested_raw: int,
      eps_torque: float,
  ) -> int:
    if not math.isfinite(eps_torque):
      eps_torque = 0.0
    max_allowed = min(
      max(eps_torque + self.steer_error_max, self.steer_error_max),
      self.steer_max,
    )
    min_allowed = max(
      min(eps_torque - self.steer_error_max, -self.steer_error_max),
      -self.steer_max,
    )
    return int(round(clip(requested_raw, min_allowed, max_allowed)))

  def _rate_limited_target(
      self,
      target: int,
      previous: int,
  ) -> int:
    if previous > 0:
      lower = max(
        previous - self.candidate_delta_down,
        -self.candidate_delta_up,
      )
      upper = previous + self.candidate_delta_up
    else:
      lower = previous - self.candidate_delta_up
      upper = min(
        previous + self.candidate_delta_down,
        self.candidate_delta_up,
      )
    return int(round(clip(target, lower, upper)))

  def update(
      self,
      *,
      requested_raw: int,
      installed_applied_raw: int,
      eps_torque: float,
      control_allowed: bool,
  ) -> int:
    previous_candidate = self.candidate_applied_last
    error_target = self._error_limited_target(
      requested_raw,
      eps_torque,
    )
    candidate_limited = self._rate_limited_target(
      error_target,
      previous_candidate,
    )
    candidate_applied = (
      candidate_limited if control_allowed else 0
    )

    self.samples += 1
    self.active_samples += int(control_allowed)
    self.changed_samples += int(
      candidate_applied != installed_applied_raw,
    )
    if control_allowed:
      current_gap = abs(requested_raw - installed_applied_raw)
      candidate_gap = abs(requested_raw - candidate_applied)
      self.current_request_gap_sum += current_gap
      self.candidate_request_gap_sum += candidate_gap
      self.improved_samples += int(candidate_gap < current_gap)
      self.worse_samples += int(candidate_gap > current_gap)
      self.equal_samples += int(candidate_gap == current_gap)
      self.current_panda_rate_violation_samples += int(
        abs(candidate_applied - previous_candidate)
        > self.installed_delta_limit
      )

    self.candidate_ceiling_samples += int(
      abs(candidate_applied) >= self.steer_max,
    )
    self.max_candidate_raw = max(
      self.max_candidate_raw,
      abs(candidate_applied),
    )
    self.max_candidate_delta = max(
      self.max_candidate_delta,
      (
        abs(candidate_applied - previous_candidate)
        if control_allowed else 0
      ),
    )
    self.max_candidate_divergence = max(
      self.max_candidate_divergence,
      abs(candidate_applied - installed_applied_raw),
    )
    self.candidate_applied_last = candidate_applied
    return candidate_applied

  def snapshot(self) -> JeepSteeringRateCandidateWindow:
    denominator = max(self.active_samples, 1)
    window = JeepSteeringRateCandidateWindow(
      samples=self.samples,
      active_samples=self.active_samples,
      changed_samples=self.changed_samples,
      improved_samples=self.improved_samples,
      worse_samples=self.worse_samples,
      equal_samples=self.equal_samples,
      current_panda_rate_violation_samples=(
        self.current_panda_rate_violation_samples
      ),
      candidate_ceiling_samples=self.candidate_ceiling_samples,
      max_candidate_raw=self.max_candidate_raw,
      max_candidate_delta=self.max_candidate_delta,
      max_candidate_divergence=self.max_candidate_divergence,
      mean_current_request_gap=(
        self.current_request_gap_sum / denominator
      ),
      mean_candidate_request_gap=(
        self.candidate_request_gap_sum / denominator
      ),
    )
    self._reset_window()
    return window

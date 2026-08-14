"""Bounded guard for the first unstable Jeep steering-path reversal."""

from dataclasses import dataclass
import math


MIN_SPEED_MPS = 8.0
LOW_LANE_CONFIDENCE = 0.40
ENTER_CURVATURE = 0.0008
ESTABLISHED_CURVATURE = 0.0015
ESTABLISHED_CYCLES = 5       # 100 ms at the 50 Hz steering command rate
GUARD_CYCLES = 50            # one second; driver override clears immediately
GUARD_MAX_RAW = 100          # below the Jeep's ordinary 261 command ceiling
GUARD_DELTA_RAW = 3          # response 5 remains unchanged outside the guard


def _clip(value, lower, upper):
  return min(max(value, lower), upper)


def _sign(value):
  return (value > 0) - (value < 0)


@dataclass(frozen=True)
class JeepSteeringDiscontinuityResult:
  requested_raw: int
  active: bool
  activated: bool
  lane_confidence: float
  established_sign: int


class JeepSteeringDiscontinuityGuard:
  """Reduce the first low-confidence path flip without changing normal rate 5.

  The guard does not infer road geometry from lane probability alone. It arms
  only after curvature has been stable on one side, then the requested path
  crosses to the other side while both inner lane probabilities are weak.
  Normal high-confidence curve reversals remain byte-for-byte unchanged.
  """

  def __init__(self):
    self.reset()

  def reset(self):
    self.established_sign = 0
    self.establishing_sign = 0
    self.establishing_cycles = 0
    self.guard_remaining = 0

  def update(
      self,
      *,
      requested_raw: int,
      previous_applied_raw: int,
      desired_curvature: float,
      left_lane_probability: float,
      right_lane_probability: float,
      speed_mps: float,
      control_allowed: bool,
      steering_pressed: bool,
      model_valid: bool,
  ) -> JeepSteeringDiscontinuityResult:
    finite = all(math.isfinite(value) for value in (
      desired_curvature,
      left_lane_probability,
      right_lane_probability,
      speed_mps,
    ))
    eligible = (
      finite and model_valid and control_allowed and not steering_pressed
      and speed_mps >= MIN_SPEED_MPS
    )
    if not eligible:
      self.reset()
      return JeepSteeringDiscontinuityResult(
        requested_raw=requested_raw,
        active=False,
        activated=False,
        lane_confidence=1.0,
        established_sign=0,
      )

    # Both inner lane lines must be weak. A single missing/occluded line is
    # common on legitimate curves and is not enough evidence to intervene.
    lane_confidence = max(
      left_lane_probability,
      right_lane_probability,
    )
    current_sign = (
      _sign(desired_curvature)
      if abs(desired_curvature) >= ENTER_CURVATURE else 0
    )
    activated = False
    if (
        self.established_sign != 0
        and current_sign != 0
        and current_sign != self.established_sign
        and lane_confidence < LOW_LANE_CONFIDENCE
    ):
      self.guard_remaining = GUARD_CYCLES
      self.established_sign = current_sign
      self.establishing_sign = current_sign
      self.establishing_cycles = 0
      activated = True
    elif self.guard_remaining > 0:
      self.guard_remaining -= 1

    strong_sign = (
      _sign(desired_curvature)
      if abs(desired_curvature) >= ESTABLISHED_CURVATURE else 0
    )
    if strong_sign == 0:
      self.establishing_sign = 0
      self.establishing_cycles = 0
    elif strong_sign == self.establishing_sign:
      self.establishing_cycles += 1
    else:
      self.establishing_sign = strong_sign
      self.establishing_cycles = 1
    if (
        self.establishing_cycles >= ESTABLISHED_CYCLES
        and self.guard_remaining == 0
    ):
      self.established_sign = self.establishing_sign

    active = self.guard_remaining > 0
    guarded_raw = requested_raw
    if active:
      guarded_raw = int(_clip(
        requested_raw,
        -GUARD_MAX_RAW,
        GUARD_MAX_RAW,
      ))
      guarded_raw = int(_clip(
        guarded_raw,
        previous_applied_raw - GUARD_DELTA_RAW,
        previous_applied_raw + GUARD_DELTA_RAW,
      ))

    return JeepSteeringDiscontinuityResult(
      requested_raw=guarded_raw,
      active=active,
      activated=activated,
      lane_confidence=lane_confidence,
      established_sign=self.established_sign,
    )

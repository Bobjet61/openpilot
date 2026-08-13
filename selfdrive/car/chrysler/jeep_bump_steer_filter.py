"""Bounded Jeep steering-feedback filter for short vertical road impulses."""

from dataclasses import dataclass
import math


CONTROL_DT = 0.01
MIN_ACTIVE_SPEED_MPS = 25.0
VERTICAL_BASELINE_TAU_S = 0.75
MEASUREMENT_FILTER_TAU_S = 0.16
ACTIVATION_TAU_S = 0.05
RELEASE_TAU_S = 0.25
VERTICAL_IMPULSE_THRESHOLD_MPS2 = 2.5
IMPULSE_HOLD_S = 0.45
MAX_MEASUREMENT_CORRECTION_MPS2 = 0.35
STRENGTH_EPSILON = 1e-3


def _alpha(dt: float, tau: float) -> float:
  return 1.0 - math.exp(-dt / tau)


def _clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


@dataclass(frozen=True)
class JeepBumpSteerFilterState:
  output_lateral_accel: float
  correction_lateral_accel: float
  strength: float
  vertical_impulse_mps2: float
  impulse_detected: bool
  active: bool


class JeepBumpSteerFilter:
  """Suppress short steering-angle feedback transients after a road impulse.

  Normal steering measurements pass through exactly. A large vertical impulse
  at freeway speed briefly blends in a low-pass measurement, with a hard cap
  on how far control feedback may differ from the raw measurement.
  """

  def __init__(self, dt: float = CONTROL_DT):
    self.dt = dt
    self.hold_frames = max(1, round(IMPULSE_HOLD_S / dt))
    self.vertical_alpha = _alpha(dt, VERTICAL_BASELINE_TAU_S)
    self.measurement_alpha = _alpha(dt, MEASUREMENT_FILTER_TAU_S)
    self.activation_alpha = _alpha(dt, ACTIVATION_TAU_S)
    self.release_alpha = _alpha(dt, RELEASE_TAU_S)
    self.reset()

  def reset(self) -> None:
    self.initialized = False
    self.vertical_baseline = 0.0
    self.filtered_measurement = 0.0
    self.strength = 0.0
    self.hold_remaining = 0
    self.state = JeepBumpSteerFilterState(
      output_lateral_accel=0.0,
      correction_lateral_accel=0.0,
      strength=0.0,
      vertical_impulse_mps2=0.0,
      impulse_detected=False,
      active=False,
    )

  def _passthrough(self, raw_lateral_accel: float) -> float:
    self.reset()
    self.state = JeepBumpSteerFilterState(
      output_lateral_accel=raw_lateral_accel,
      correction_lateral_accel=0.0,
      strength=0.0,
      vertical_impulse_mps2=0.0,
      impulse_detected=False,
      active=False,
    )
    return raw_lateral_accel

  def update(
      self,
      raw_lateral_accel: float,
      vertical_accel: float,
      speed_mps: float,
      controls_active: bool,
      steering_pressed: bool,
  ) -> float:
    inputs = (raw_lateral_accel, vertical_accel, speed_mps)
    if not all(math.isfinite(value) for value in inputs):
      return self._passthrough(raw_lateral_accel)

    eligible = (
      controls_active
      and not steering_pressed
      and speed_mps >= MIN_ACTIVE_SPEED_MPS
    )
    if not eligible:
      return self._passthrough(raw_lateral_accel)

    if not self.initialized:
      self.initialized = True
      self.vertical_baseline = vertical_accel
      self.filtered_measurement = raw_lateral_accel

    vertical_impulse = vertical_accel - self.vertical_baseline
    self.vertical_baseline += self.vertical_alpha * vertical_impulse
    self.filtered_measurement += self.measurement_alpha * (
      raw_lateral_accel - self.filtered_measurement
    )

    impulse_detected = abs(vertical_impulse) >= VERTICAL_IMPULSE_THRESHOLD_MPS2
    if impulse_detected:
      self.hold_remaining = self.hold_frames
    elif self.hold_remaining > 0:
      self.hold_remaining -= 1

    target_strength = 1.0 if self.hold_remaining > 0 else 0.0
    strength_alpha = (
      self.activation_alpha if target_strength > self.strength
      else self.release_alpha
    )
    self.strength += strength_alpha * (target_strength - self.strength)
    if target_strength == 0.0 and self.strength < STRENGTH_EPSILON:
      self.strength = 0.0

    correction = self.strength * _clip(
      self.filtered_measurement - raw_lateral_accel,
      -MAX_MEASUREMENT_CORRECTION_MPS2,
      MAX_MEASUREMENT_CORRECTION_MPS2,
    )
    output = raw_lateral_accel + correction
    filter_active = self.strength > 0.0

    self.state = JeepBumpSteerFilterState(
      output_lateral_accel=output,
      correction_lateral_accel=correction,
      strength=self.strength,
      vertical_impulse_mps2=vertical_impulse,
      impulse_detected=impulse_detected,
      active=filter_active,
    )
    return output

"""Safety-gated longitudinal recovery after a driver accelerator override."""

from __future__ import annotations

import math


JEEP_WP_S20_FLAG = 2
JEEP_GAS_RELEASE_RECOVERY_DURATION_S = 5.0
JEEP_GAS_RELEASE_PLAN_MAX_AGE_S = 0.2
JEEP_GAS_RELEASE_MIN_SPEED_MPS = 5.0
JEEP_GAS_RELEASE_MAX_OVERSPEED_MPS = 1.5
JEEP_GAS_RELEASE_MAX_PLANNED_DECEL_MPS2 = -0.25
JEEP_GAS_RELEASE_MIN_FEEDBACK_SCALE = 0.25


def jeep_gas_release_context_allowed(
    *,
    jeep_op_long: bool,
    plans_valid: bool,
    main_source: str,
    sp_source: str,
    has_lead: bool,
    fcw: bool,
    vision_turn_state: str,
    speed_limit_state: str,
    turn_speed_state: str,
    hard_brake_predicted: bool,
    force_decel: bool,
    brake_pressed: bool,
    acc_faulted: bool,
    stock_aeb: bool,
    standstill: bool,
) -> bool:
  """Allow recovery only during unconstrained, healthy free cruising."""
  return bool(
    jeep_op_long
    and plans_valid
    and main_source == "cruise"
    and sp_source == "cruise"
    and not has_lead
    and not fcw
    and vision_turn_state == "disabled"
    and speed_limit_state == "inactive"
    and turn_speed_state == "inactive"
    and not hard_brake_predicted
    and not force_decel
    and not brake_pressed
    and not acc_faulted
    and not stock_aeb
    and not standstill
  )


def attenuate_negative_pid_output(
    *,
    feedforward: float,
    proportional: float,
    integral: float,
    derivative: float,
    ordinary_output: float,
    feedback_scale: float,
    neg_limit: float,
    pos_limit: float,
) -> float:
  """Attenuate negative P/I output without changing the PID's state.

  Negative planner feed-forward and derivative feedback are never weakened.
  Keeping the PID's ordinary internal state lets a cancelled recovery return
  to the exact baseline output immediately, without an integrator tail.
  """
  ordinary_output = max(
    float(neg_limit),
    min(float(pos_limit), float(ordinary_output)),
  )
  if ordinary_output >= 0.0:
    return ordinary_output

  feedback_scale = max(0.0, min(1.0, float(feedback_scale)))
  proportional_out = (
    proportional * feedback_scale
    if proportional < 0.0 else proportional
  )
  integral_out = integral * feedback_scale if integral < 0.0 else integral
  output = feedforward + proportional_out + integral_out + derivative
  output = max(ordinary_output, min(0.0, output))
  if feedforward < 0.0 and ordinary_output <= feedforward:
    # Preserve planner braking only when ordinary PID was already honoring it;
    # never turn an ordinary coast/accel command into braking.
    output = min(output, feedforward)
  return max(ordinary_output, min(0.0, output))


class JeepGasReleaseRecovery:
  """Blend negative PID feedback back in after a guarded gas release.

  The planner feed-forward acceleration is intentionally outside this helper.
  Any disqualifying context cancels the recovery instead of pausing it.
  """

  def __init__(self, enabled: bool):
    self.enabled = bool(enabled)
    self.gas_pressed_prev = False
    self.elapsed_s: float | None = None
    self.feedback_scale = 1.0

  @property
  def active(self) -> bool:
    return self.elapsed_s is not None

  def clear(self) -> None:
    self.elapsed_s = None
    self.feedback_scale = 1.0

  def update(
      self,
      *,
      active: bool,
      gas_pressed: bool,
      context_allowed: bool,
      v_ego: float,
      v_cruise: float,
      planned_accel: float,
      dt_s: float,
  ) -> float:
    gas_pressed = bool(gas_pressed)
    gas_released = self.gas_pressed_prev and not gas_pressed
    self.gas_pressed_prev = gas_pressed

    finite_inputs = all(math.isfinite(float(value)) for value in (
      v_ego, v_cruise, planned_accel, dt_s,
    ))
    eligible_now = bool(
      self.enabled
      and active
      and not gas_pressed
      and context_allowed
      and finite_inputs
      and float(v_ego) >= JEEP_GAS_RELEASE_MIN_SPEED_MPS
      and float(v_ego) <= (
        float(v_cruise) + JEEP_GAS_RELEASE_MAX_OVERSPEED_MPS
      )
      and float(planned_accel) > JEEP_GAS_RELEASE_MAX_PLANNED_DECEL_MPS2
    )

    if gas_released and eligible_now:
      # Arm even when the vehicle is still just below the planned trajectory;
      # route 8 did not develop its excess speed until several seconds later.
      self.elapsed_s = 0.0
    elif not eligible_now:
      self.clear()

    if self.elapsed_s is None:
      return self.feedback_scale

    progress = min(
      1.0,
      self.elapsed_s / JEEP_GAS_RELEASE_RECOVERY_DURATION_S,
    )
    # Reintroduce feedback slowly at first, then fully by the bounded timeout.
    self.feedback_scale = JEEP_GAS_RELEASE_MIN_FEEDBACK_SCALE + (
      1.0 - JEEP_GAS_RELEASE_MIN_FEEDBACK_SCALE
    ) * progress * progress
    self.elapsed_s += max(0.0, float(dt_s))
    if self.elapsed_s >= JEEP_GAS_RELEASE_RECOVERY_DURATION_S:
      self.clear()
    return self.feedback_scale

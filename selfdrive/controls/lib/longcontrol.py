from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from openpilot.selfdrive.controls.lib.longitudinal_gas_release_recovery import (
  JEEP_WP_S20_FLAG,
  JeepGasReleaseRecovery,
  attenuate_negative_pid_output,
)
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego, v_target,
                             v_target_1sec, brake_pressed, cruise_standstill):
  # Ignore cruise standstill if car has a gas interceptor
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptorDEPRECATED
  accelerating = v_target_1sec > v_target
  planned_stop = (v_target < CP.vEgoStopping and
                  v_target_1sec < CP.vEgoStopping and
                  not accelerating)
  stay_stopped = (v_ego < CP.vEgoStopping and
                  (brake_pressed or cruise_standstill))
  stopping_condition = planned_stop or stay_stopped

  starting_condition = (v_target_1sec > CP.vEgoStarting and
                        accelerating and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state in (LongCtrlState.off, LongCtrlState.pid):
      long_control_state = LongCtrlState.pid
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off  # initialized to off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.v_pid = 0.0
    self.last_output_accel = 0.0
    jeep_wp_op_long = bool(
      CP.carName == "chrysler"
      and CP.openpilotLongitudinalControl
      and (CP.spFlags & JEEP_WP_S20_FLAG)
    )
    self.jeep_gas_release_recovery = JeepGasReleaseRecovery(
      jeep_wp_op_long,
    )
    self.jeep_gas_release_feedback_scale = 1.0
    self.jeep_gas_release_baseline_output_accel = 0.0
    self.jeep_gas_release_planned_accel = 0.0

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(
      self,
      active,
      CS,
      long_plan,
      accel_limits,
      t_since_plan,
      *,
      jeep_gas_release_recovery_allowed=False,
      v_cruise=float("nan"),
  ):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target_now = interp(t_since_plan, CONTROL_N_T_IDX, speeds)
      a_target_now = interp(t_since_plan, CONTROL_N_T_IDX, long_plan.accels)

      v_target_lower = interp(self.CP.longitudinalActuatorDelayLowerBound + t_since_plan, CONTROL_N_T_IDX, speeds)
      a_target_lower = 2 * (v_target_lower - v_target_now) / self.CP.longitudinalActuatorDelayLowerBound - a_target_now

      v_target_upper = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan, CONTROL_N_T_IDX, speeds)
      a_target_upper = 2 * (v_target_upper - v_target_now) / self.CP.longitudinalActuatorDelayUpperBound - a_target_now

      v_target = min(v_target_lower, v_target_upper)
      a_target = min(a_target_lower, a_target_upper)

      v_target_1sec = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan + 1.0, CONTROL_N_T_IDX, speeds)
    else:
      v_target = 0.0
      v_target_now = 0.0
      v_target_1sec = 0.0
      a_target = 0.0

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]
    self.jeep_gas_release_planned_accel = a_target

    recovery_was_active = self.jeep_gas_release_recovery.active
    self.jeep_gas_release_feedback_scale = (
      self.jeep_gas_release_recovery.update(
        active=active,
        gas_pressed=CS.gasPressed,
        context_allowed=jeep_gas_release_recovery_allowed,
        v_ego=CS.vEgo,
        v_cruise=v_cruise,
        planned_accel=a_target,
        dt_s=DT_CTRL,
      )
    )
    recovery_cancelled = bool(
      recovery_was_active and not self.jeep_gas_release_recovery.active
    )

    output_accel = (
      self.jeep_gas_release_baseline_output_accel
      if recovery_cancelled else self.last_output_accel
    )
    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       v_target, v_target_1sec, CS.brakePressed,
                                                       CS.cruiseState.standstill)

    if self.long_control_state == LongCtrlState.off:
      self.reset(CS.vEgo)
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target_now

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      # TODO too complex, needs to be simplified and tested on toyotas
      prevent_overshoot = not self.CP.stoppingControl and CS.vEgo < 1.5 and v_target_1sec < 0.7 and v_target_1sec < self.v_pid
      deadzone = interp(CS.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      error = self.v_pid - CS.vEgo
      error_deadzone = apply_deadzone(error, deadzone)
      freeze_integrator = prevent_overshoot
      output_accel = self.pid.update(error_deadzone, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)
      self.jeep_gas_release_baseline_output_accel = output_accel
      if self.jeep_gas_release_recovery.active:
        output_accel = attenuate_negative_pid_output(
          feedforward=self.pid.f,
          proportional=self.pid.p,
          integral=self.pid.i,
          derivative=self.pid.d,
          ordinary_output=self.jeep_gas_release_baseline_output_accel,
          feedback_scale=self.jeep_gas_release_feedback_scale,
          neg_limit=accel_limits[0],
          pos_limit=accel_limits[1],
        )

    if self.long_control_state != LongCtrlState.pid:
      self.jeep_gas_release_baseline_output_accel = output_accel

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    return self.last_output_accel

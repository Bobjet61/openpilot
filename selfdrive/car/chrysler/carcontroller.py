from collections import Counter

import cereal.messaging as messaging
from cereal import car
from common.conversions import Conversions as CV
from opendbc.can.packer import CANPacker
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car import apply_meas_steer_torque_limits
from openpilot.selfdrive.car.chrysler import chryslercan
from openpilot.selfdrive.car.chrysler.jeep_camera_lead_trend_shadow import (
  JeepCameraLeadTrendShadow,
  jeep_camera_lead_trend_runtime_eligible,
)
from openpilot.selfdrive.car.chrysler.jeep_radar_shadow import JeepVisionLead
from openpilot.selfdrive.car.chrysler.jeep_longitudinal import (
  JEEP_LONG_ACTUATION_COMPILED,
  JeepLongitudinalShadow,
  JeepLongitudinalTransportScheduler,
  jeep_factory_sng_hold_motion_safe,
  jeep_factory_sng_lead_moving,
  jeep_factory_sng_vision_lead_moving,
)
from openpilot.selfdrive.car.chrysler.jeep_longitudinal_planner_shadow import JeepLongitudinalPlanShadow
from openpilot.selfdrive.car.chrysler.jeep_radar_brake_guard import JeepRadarBrakeContinuityGuard
from openpilot.selfdrive.car.chrysler.jeep_steering_discontinuity_guard import JeepSteeringDiscontinuityGuard
from openpilot.selfdrive.car.chrysler.jeep_steering_shadow import JeepSteeringRateCandidateShadow, JeepSteeringShadow
from openpilot.selfdrive.car.chrysler.values import CAR, RAM_CARS, RAM_DT, STEER_THRESHOLD, CarControllerParams, ChryslerFlags, ChryslerFlagsSP
from openpilot.selfdrive.car.interfaces import CarControllerBase, FORWARD_GEARS
from openpilot.selfdrive.controls.lib.drive_helpers import FCA_V_CRUISE_MIN

BUTTONS_STATES = ["accelCruise", "decelCruise", "cancel", "resumeCruise"]

JEEP_LONG_CARS = {
  CAR.JEEP_GRAND_CHEROKEE,
  CAR.JEEP_GRAND_CHEROKEE_2019,
}

B6Y_STANDSTILL_HOLD_DECEL = -2.0
# Factory SNG is only a hold/release bridge: once independent lead motion is
# observed, let the stock ACC resume without adding a perceptible software
# delay. Both guarded detection paths may trigger on their first fresh
# observation; transmission follows on the next valid SCCM counter.
B6Y_LEAD_CONFIRM_CYCLES = 1
B6Y_VISION_LEAD_CONFIRM_CYCLES = 1
B6Y_RESUME_RETRY_FRAMES = 50
B6Y_RESUME_MAX_ATTEMPTS = 3
B6Y_RESUME_PULSE_COUNTERS = 8
JEEP_LKAS_ENABLE_CONFIRM_FRAMES = 50


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.frame = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.lkas_control_bit_prev = False
    self.jeep_lkas_enable_request_frame = -1
    self.last_button_frame = 0
    self.b6y_last_das_3_counter = -1
    self.b6y_last_resume_frame = -100
    self.b6y_lead_moving_frames = 0
    self.b6y_vision_lead_moving_frames = 0
    self.b6y_minimum_held_lead_distance = float("inf")
    self.b6y_approach_armed = False
    self.jeep_radar_shadow_updated = False
    self.b6y_resume_attempts = 0
    self.b6y_resume_pulse_remaining = 0
    self.b6y_lead_departure_latched = False
    self.jeep_long_shadow = JeepLongitudinalShadow()
    self.jeep_long_envelope = self.jeep_long_shadow.update(0.0, eligible=False)
    self.jeep_long_shadow_frames = []
    self.jeep_long_transport_frames = []
    self.jeep_long_transport_scheduler = JeepLongitudinalTransportScheduler()
    self.jeep_long_plan_sm = (
      messaging.SubMaster(["longitudinalPlan"])
      if CP.carFingerprint in JEEP_LONG_CARS else None
    )
    self.jeep_long_plan_shadow = (
      JeepLongitudinalPlanShadow(CP)
      if CP.carFingerprint in JEEP_LONG_CARS else None
    )
    self.jeep_long_plan_result = None
    self.jeep_radar_shadow_sm = (
      messaging.SubMaster(["radarState"])
      if CP.carFingerprint in JEEP_LONG_CARS else None
    )
    self.jeep_radar_shadow_last_cycle = 0
    self.jeep_radar_shadow_update_frame = -1000
    self.jeep_radar_shadow_selection = None
    self.jeep_radar_shadow_vision = None
    self.jeep_radar_shadow_reason_counts = Counter()
    self.jeep_closing_brake_floor = None
    self.jeep_radar_brake_guard = (
      JeepRadarBrakeContinuityGuard()
      if CP.carFingerprint in JEEP_LONG_CARS else None
    )
    self.jeep_radar_brake_guard_result = None
    self.jeep_camera_lead_trend_shadow = (
      JeepCameraLeadTrendShadow()
      if CP.carFingerprint in JEEP_LONG_CARS else None
    )
    self.jeep_camera_lead_trend_result = None
    self.jeep_steering_model_sm = (
      messaging.SubMaster(["modelV2"])
      if CP.carFingerprint in JEEP_LONG_CARS else None
    )
    self.jeep_steering_discontinuity_guard = (
      JeepSteeringDiscontinuityGuard()
      if CP.carFingerprint in JEEP_LONG_CARS else None
    )

    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(CP)
    self.jeep_steering_shadow = (
      JeepSteeringShadow(
        steer_max=self.params.STEER_MAX,
        steer_delta_up=self.params.STEER_DELTA_UP,
        steer_delta_down=self.params.STEER_DELTA_DOWN,
        steer_error_max=self.params.STEER_ERROR_MAX,
        driver_threshold=STEER_THRESHOLD,
      )
      if CP.carFingerprint in JEEP_LONG_CARS else None
    )
    self.jeep_steering_rate5_shadow = (
      JeepSteeringRateCandidateShadow(
        steer_max=self.params.STEER_MAX,
        candidate_delta_up=5,
        candidate_delta_down=5,
        steer_error_max=self.params.STEER_ERROR_MAX,
        installed_delta_limit=self.params.STEER_DELTA_UP,
      )
      if CP.carFingerprint in JEEP_LONG_CARS else None
    )

    self.sm = messaging.SubMaster(['longitudinalPlanSP'])
    self.param_s = Params()
    self.is_metric = self.param_s.get_bool("IsMetric")
    self.speed_limit_control_enabled = False
    self.last_speed_limit_sign_tap = False
    self.last_speed_limit_sign_tap_prev = False
    self.speed_limit = 0.
    self.speed_limit_offset = 0
    self.timer = 0
    self.final_speed_kph = 0
    self.init_speed = 0
    self.current_speed = 0
    self.v_set_dis = 0
    self.v_cruise_min = 0
    self.button_type = 0
    self.button_select = 0
    self.button_count = 0
    self.target_speed = 0
    self.t_interval = 7
    self.slc_active_stock = False
    self.sl_force_active_timer = 0
    self.v_tsc_state = 0
    self.slc_state = 0
    self.m_tsc_state = 0
    self.cruise_button = None
    self.speed_diff = 0
    self.v_tsc = 0
    self.m_tsc = 0
    self.steady_speed = 0
    self.button_frame = 0

  def update(self, CC, CS, now_nanos):
    if not self.CP.pcmCruiseSpeed:
      self.sm.update(0)

      if self.sm.updated['longitudinalPlanSP']:
        self.v_tsc_state = self.sm['longitudinalPlanSP'].visionTurnControllerState
        self.slc_state = self.sm['longitudinalPlanSP'].speedLimitControlState
        self.m_tsc_state = self.sm['longitudinalPlanSP'].turnSpeedControlState
        self.speed_limit = self.sm['longitudinalPlanSP'].speedLimit
        self.speed_limit_offset = self.sm['longitudinalPlanSP'].speedLimitOffset
        self.v_tsc = self.sm['longitudinalPlanSP'].visionTurnSpeed
        self.m_tsc = self.sm['longitudinalPlanSP'].turnSpeed

      if self.frame % 200 == 0:
        self.speed_limit_control_enabled = self.param_s.get_bool("EnableSlc")
        self.is_metric = self.param_s.get_bool("IsMetric")
      self.last_speed_limit_sign_tap = self.param_s.get_bool("LastSpeedLimitSignTap")
      self.v_cruise_min = FCA_V_CRUISE_MIN[self.is_metric] * (CV.KPH_TO_MPH if not self.is_metric else 1)

    can_sends = []
    self.update_jeep_radar_shadow(CS)

    if not self.CP.pcmCruiseSpeed:
      if not self.last_speed_limit_sign_tap_prev and self.last_speed_limit_sign_tap:
        self.sl_force_active_timer = self.frame
        self.param_s.put_bool_nonblocking("LastSpeedLimitSignTap", False)
      self.last_speed_limit_sign_tap_prev = self.last_speed_limit_sign_tap

      sl_force_active = self.speed_limit_control_enabled and (self.frame < (self.sl_force_active_timer * DT_CTRL + 2.0))
      sl_inactive = not sl_force_active and (not self.speed_limit_control_enabled or (True if self.slc_state == 0 else False))
      sl_temp_inactive = not sl_force_active and (self.speed_limit_control_enabled and (True if self.slc_state == 1 else False))
      slc_active = not sl_inactive and not sl_temp_inactive

      self.slc_active_stock = slc_active

    lkas_active = CC.latActive and CS.madsEnabled
    jeep_long_vehicle_eligible, jeep_long_vehicle_reason = (
      self.jeep_long_vehicle_eligibility(CC, CS)
    )
    self.update_jeep_camera_lead_trend_shadow(CC, CS, now_nanos)
    if self.jeep_radar_brake_guard is not None:
      radar_shadow = CS.jeep_radar_shadow
      self.jeep_radar_brake_guard_result = (
        self.jeep_radar_brake_guard.update(
          eligible=jeep_long_vehicle_eligible,
          v_ego=CS.out.vEgo,
          now_nanos=now_nanos,
          radar_cycle=radar_shadow.cycle_count,
          cycle_complete=radar_shadow.last_cycle_complete,
          tracks=radar_shadow.tracks,
          vision=self.jeep_radar_shadow_vision,
          selection=self.jeep_radar_shadow_selection,
          planner_decelerating=CC.actuators.accel <= -0.40,
        )
      )
    self.update_jeep_long_plan_shadow(
      CS,
      now_nanos,
      jeep_long_vehicle_eligible,
      jeep_long_vehicle_reason,
    )
    if self.frame % 2 == 0:
      requested_accel = (
        CC.actuators.accel
        if (
          JEEP_LONG_ACTUATION_COMPILED
          and self.CP.openpilotLongitudinalControl
        ) else (
          self.jeep_long_plan_result.controller_accel_mps2
          if self.jeep_long_plan_result is not None else 0.0
        )
      )
      dropout_floor = (
        self.jeep_radar_brake_guard_result.brake_floor_mps2
        if self.jeep_radar_brake_guard_result is not None else None
      )
      # The former stateless fused-closing floor is deliberately not composed
      # here. Archive replay showed that brief vision/radar pairing fragments
      # could turn an ordinary planner request into abrupt braking while the
      # production lead remained healthy. Only the stateful degraded/low-
      # confidence lead-continuity guard may strengthen the production request.
      self.jeep_closing_brake_floor = dropout_floor
      if (
          JEEP_LONG_ACTUATION_COMPILED
          and self.CP.openpilotLongitudinalControl
          and self.jeep_closing_brake_floor is not None
      ):
        requested_accel = min(
          requested_accel,
          self.jeep_closing_brake_floor,
        )
      # CC.actuators.accel is already the output of openpilot's production
      # longitudinal controller. The separate plan subscriber remains useful
      # for telemetry, but its transient service-valid flag must not reset the
      # actuator jerk ramp or alternate between stock and openpilot commands.
      self.jeep_long_envelope = self.jeep_long_shadow.update(
        requested_accel,
        jeep_long_vehicle_eligible,
        CS.out.vEgo,
        CC.orientationNED[1] if len(CC.orientationNED) > 1 else float("nan"),
      )

    # Offer the most recent 50 Hz envelope on every 100 Hz controller frame.
    # The monotonic 25 ms scheduler floor produces about 33 Hz without ever
    # weakening either Panda's watchdog. Current vehicle eligibility is also
    # checked here so a pedal/door/gear override cannot replay the prior frame.
    self.jeep_long_shadow_frames = []
    self.jeep_long_transport_frames = []
    transport_counter = self.jeep_long_transport_scheduler.next_counter(
      now_nanos,
      self.jeep_long_envelope.transport_enabled and jeep_long_vehicle_eligible,
    )
    if transport_counter is not None:
      if self.jeep_long_envelope.host_enabled:
        self.jeep_long_shadow_frames = (
          chryslercan.create_wp_long_shadow_messages(
            self.packer, self.jeep_long_envelope, transport_counter)
        )
        can_sends.extend(self.jeep_long_shadow_frames)
        self.jeep_long_shadow.note_transport_sent(
          self.jeep_long_envelope,
        )
      else:
        self.jeep_long_transport_frames = (
          chryslercan.create_wp_long_transport_messages(
            self.packer, transport_counter)
        )
        can_sends.extend(self.jeep_long_transport_frames)

    if self.frame % 100 == 0 and self.CP.spFlags & ChryslerFlagsSP.SP_WP_S20:
      cloudlog.info(
        f"Jeep long shadow: eligible={self.jeep_long_envelope.eligible}, "
        f"requested={self.jeep_long_envelope.requested_accel:.3f}, "
        f"closing_floor={self.jeep_closing_brake_floor}, "
        f"dropout_guard_reason={getattr(self.jeep_radar_brake_guard_result, 'reason', 'unavailable')}, "
        f"dropout_guard_age={getattr(self.jeep_radar_brake_guard_result, 'dropout_age_s', 0.0):.3f}, "
        f"dropout_guard_bias={getattr(self.jeep_radar_brake_guard_result, 'range_bias_m', 0.0):.2f}, "
        f"limited={self.jeep_long_envelope.limited_accel:.3f}, "
        f"brake={self.jeep_long_envelope.brake_active}, "
        f"brake_cmd={self.jeep_long_envelope.brake_accel_mps2:.3f}, "
        f"engine={self.jeep_long_envelope.engine_active}, "
        f"torque={self.jeep_long_envelope.engine_torque_nm:.1f}, "
        f"pitch={self.jeep_long_envelope.filtered_pitch_rad:.4f}, "
        f"grade_torque={self.jeep_long_envelope.grade_torque_nm:.1f}, "
        f"speed={CS.out.vEgo:.3f}, "
        f"a_ego={CS.out.aEgo:.3f}, "
        f"set_speed={CC.hudControl.setSpeed:.3f}, "
        f"control_state={CC.actuators.longControlState}, "
        f"mode={self.jeep_long_envelope.command_mode}, "
        f"brake_latched={self.jeep_long_envelope.brake_latched}, "
        f"stop={self.jeep_long_envelope.stop_request}, "
        f"go={self.jeep_long_envelope.go_request}, "
        f"low_speed_state={self.jeep_long_envelope.low_speed_state}, "
        f"transport={self.jeep_long_envelope.transport_enabled}, "
        f"host_enabled={self.jeep_long_envelope.host_enabled}"
      )
      self.log_wp_long_diagnostic(CS)
      self.log_jeep_long_plan_shadow(CS)
      self.log_jeep_radar_shadow(CS)
      self.log_jeep_camera_lead_trend_shadow()
      self.log_jeep_steering_shadow()
      self.log_jeep_steering_rate5_shadow()

    if self.frame % 10 == 0 and self.CP.carFingerprint not in RAM_CARS:
      can_sends.append(chryslercan.create_lkas_heartbit(self.packer, CS.lkas_disabled, CS.lkas_heartbit))

    ram_cars = self.CP.carFingerprint in RAM_CARS


    das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0
    # cruise buttons
    if CS.button_counter != self.last_button_frame:
      self.last_button_frame = CS.button_counter

      if ram_cars:
        if CS.buttonStates["cancel"]:
          can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter, das_bus, self.CP, cancel=True))
        else:
          can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter, das_bus, self.CP,
                                                             cruise_buttons_msg=CS.cruise_buttons,
                                                             cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

      # ACC cancellation
      elif CC.cruiseControl.cancel:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, self.CP, cancel=True))

      # ACC resume from standstill
      elif CC.cruiseControl.resume:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, self.CP, resume=True))


      if not (CC.cruiseControl.cancel or CC.cruiseControl.resume) and not self.CP.pcmCruiseSpeed and CS.out.cruiseState.enabled:
        self.button_frame += 1
        button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
        if ram_cars:
          self.cruise_button = self.get_cruise_buttons(CS, CC.vCruise)
        elif button_counter_offset is not None:
          self.cruise_button = self.get_cruise_buttons(CS, CC.vCruise)

        if self.cruise_button is not None:
          if ram_cars:
            can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter, das_bus, self.CP, buttons=self.cruise_button))
          elif button_counter_offset is not None:
            can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + button_counter_offset, das_bus, self.CP, buttons=self.cruise_button))

    if self.CP.spFlags & ChryslerFlagsSP.SP_JEEP_FACTORY_SNG:
      self.update_b6y_standstill_hold(CC, CS, can_sends)

    # HUD alerts
    if self.frame % 25 == 0:
      if CS.lkas_car_model != -1:
        can_sends.append(chryslercan.create_lkas_hud(self.packer, self.CP, lkas_active, CS.madsEnabled, CC.hudControl.visualAlert,
                                                     self.hud_count, CS.lkas_car_model, CS.auto_high_beam))
        self.hud_count += 1

    # steering
    if self.frame % self.params.STEER_STEP == 0:

      # TODO: can we make this more sane? why is it different for all the cars?
      lkas_control_bit = self.lkas_control_bit_prev
      if self.CP.carFingerprint in RAM_DT:
        if self.CP.minEnableSpeed <= CS.out.vEgo <= self.CP.minEnableSpeed + 0.5:
          lkas_control_bit = True
        if (self.CP.minEnableSpeed >= 14.5) and (CS.out.gearShifter != 2):
          lkas_control_bit = False
      elif self.CP.spFlags & ChryslerFlagsSP.SP_WP_S20:
        jeep_lkas_requested = (
          CC.latActive
          and CS.out.gearShifter in FORWARD_GEARS
          and not CS.out.steerFaultTemporary
          and not CS.out.steerFaultPermanent
        )
        if not jeep_lkas_requested:
          self.jeep_lkas_enable_request_frame = -1
          lkas_control_bit = False
        elif self.lkas_control_bit_prev:
          lkas_control_bit = True
        else:
          if self.jeep_lkas_enable_request_frame < 0:
            self.jeep_lkas_enable_request_frame = self.frame
          # The route-56 fault followed an LKAS enable pulse lasting only two
          # command cycles during an ACC/mode-button transition. Require the
          # lateral request to remain stable before presenting a rising edge
          # to the EPS; falling edges and driver/fault revocation stay immediate.
          lkas_control_bit = (
            self.frame - self.jeep_lkas_enable_request_frame
            >= JEEP_LKAS_ENABLE_CONFIRM_FRAMES
          )
      elif CS.out.vEgo > self.CP.minSteerSpeed:
        lkas_control_bit = True
      elif self.CP.flags & ChryslerFlags.HIGHER_MIN_STEERING_SPEED:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
          lkas_control_bit = False
      elif self.CP.carFingerprint in RAM_CARS:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 0.5):
          lkas_control_bit = False

      # EPS faults if LKAS re-enables too quickly
      lkas_control_bit = lkas_control_bit and (self.frame - self.last_lkas_falling_edge > 200) and not CS.out.steerFaultTemporary and not CS.out.steerFaultPermanent

      if not lkas_control_bit and self.lkas_control_bit_prev:
        self.last_lkas_falling_edge = self.frame

      # Steer torque. Response 5 remains the ordinary Jeep limiter. Only the
      # first low-confidence reversal of an already-established path enters a
      # separately bounded one-second discontinuity guard.
      previous_apply_steer = self.apply_steer_last
      control_allowed = (
        lkas_active
        and lkas_control_bit
        and self.lkas_control_bit_prev
      )
      requested_steer = int(round(
        CC.actuators.steer * self.params.STEER_MAX,
      ))
      new_steer = requested_steer
      if (
          self.jeep_steering_discontinuity_guard is not None
          and self.jeep_steering_model_sm is not None
      ):
        self.jeep_steering_model_sm.update(0)
        model_valid = (
          self.jeep_steering_model_sm.seen["modelV2"]
          and self.jeep_steering_model_sm.valid["modelV2"]
          and self.jeep_steering_model_sm.alive["modelV2"]
        )
        left_lane_probability = 1.0
        right_lane_probability = 1.0
        if model_valid:
          lane_probabilities = list(
            self.jeep_steering_model_sm["modelV2"].laneLineProbs,
          )
          model_valid = len(lane_probabilities) >= 3
          if model_valid:
            left_lane_probability = lane_probabilities[1]
            right_lane_probability = lane_probabilities[2]
        guard_result = self.jeep_steering_discontinuity_guard.update(
          requested_raw=requested_steer,
          previous_applied_raw=previous_apply_steer,
          desired_curvature=CC.actuators.curvature,
          left_lane_probability=left_lane_probability,
          right_lane_probability=right_lane_probability,
          speed_mps=CS.out.vEgo,
          control_allowed=control_allowed,
          steering_pressed=CS.out.steeringPressed,
          model_valid=model_valid,
        )
        new_steer = guard_result.requested_raw
        if guard_result.activated:
          cloudlog.warning(
            "Jeep steering discontinuity guard: "
            f"requested={requested_steer},guarded={new_steer},"
            f"curvature={CC.actuators.curvature:.6f},"
            f"lane_confidence={guard_result.lane_confidence:.3f}"
          )
      limited_steer = apply_meas_steer_torque_limits(
        new_steer,
        previous_apply_steer,
        CS.out.steeringTorqueEps,
        self.params,
      )
      apply_steer = limited_steer
      if not control_allowed:
        apply_steer = 0
      if self.jeep_steering_shadow is not None:
        self.jeep_steering_shadow.update(
          requested_normalized=CC.actuators.steer,
          requested_raw=new_steer,
          limited_raw=limited_steer,
          applied_raw=apply_steer,
          previous_applied_raw=previous_apply_steer,
          eps_torque=CS.out.steeringTorqueEps,
          driver_torque=CS.out.steeringTorque,
          control_allowed=control_allowed,
          steer_required=(
            CC.hudControl.visualAlert
            == car.CarControl.HUDControl.VisualAlert.steerRequired
          ),
          temporary_fault=CS.out.steerFaultTemporary,
          permanent_fault=CS.out.steerFaultPermanent,
        )
      if self.jeep_steering_rate5_shadow is not None:
        self.jeep_steering_rate5_shadow.update(
          requested_raw=new_steer,
          installed_applied_raw=apply_steer,
          eps_torque=CS.out.steeringTorqueEps,
          control_allowed=control_allowed,
        )
      self.apply_steer_last = apply_steer
      self.lkas_control_bit_prev = lkas_control_bit

      can_sends.append(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_steer), lkas_control_bit))

    self.frame += 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    return new_actuators, can_sends

  def update_b6y_standstill_hold(self, CC, CS, can_sends):
    """Bridge the stock ACC standstill timeout with the proven b6 brake hold."""
    factory_sng = bool(
      self.CP.spFlags & ChryslerFlagsSP.SP_JEEP_FACTORY_SNG
    )
    if not factory_sng or not CS.das_3:
      return

    counter = CS.das_3.get("COUNTER")
    counter_changed = counter != self.b6y_last_das_3_counter
    self.b6y_last_das_3_counter = counter

    approach_eligible = (
      CC.enabled and CS.cruise_active_actual and CS.acc_decelerating
      and CS.out.vEgo < 3.0 and not CS.out.accFaulted and not CS.out.stockAeb
    )
    if approach_eligible:
      self.b6y_approach_armed = True
    elif (CC.cruiseControl.cancel or CS.out.gasPressed or
          CS.out.vEgo > 5.0 or CS.out.accFaulted or CS.out.stockAeb):
      self.b6y_approach_armed = False

    if (not CS.b6y_hold_active and self.b6y_approach_armed and CC.enabled and
        CS.out.standstill and not CS.out.accFaulted and
        not CS.out.stockAeb):
      CS.b6y_hold_active = True
      self.b6y_last_resume_frame = self.frame - B6Y_RESUME_RETRY_FRAMES
      self.b6y_lead_moving_frames = 0
      self.b6y_vision_lead_moving_frames = 0
      self.b6y_minimum_held_lead_distance = float("inf")
      self.b6y_resume_attempts = 0
      self.b6y_resume_pulse_remaining = 0
      self.b6y_lead_departure_latched = False
      cloudlog.info(
        "B6Y hold: armed after stock ACC decelerated to standstill"
      )

    if (CS.b6y_hold_active and
        (CC.cruiseControl.cancel or
         CS.out.gasPressed or CS.out.brakePressed or
         not CS.forward_gear or
         not jeep_factory_sng_hold_motion_safe(CS.out.vEgo) or
         not CS.out.cruiseState.available or
         CS.out.accFaulted or CS.out.stockAeb)):
      CS.b6y_hold_active = False
      self.b6y_lead_moving_frames = 0
      self.b6y_vision_lead_moving_frames = 0
      self.b6y_minimum_held_lead_distance = float("inf")
      self.b6y_approach_armed = False
      self.b6y_resume_attempts = 0
      self.b6y_resume_pulse_remaining = 0
      self.b6y_lead_departure_latched = False
      cloudlog.info("B6Y hold: released")
      return

    if not CS.b6y_hold_active or CS.cruise_active_actual:
      return

    counter_offset = 2 if counter_changed else 3
    can_sends.append(chryslercan.create_b6y_standstill_hold(
      self.packer,
      counter_offset,
      CS.das_3,
    ))

    selection = self.jeep_radar_shadow_selection
    vision = self.jeep_radar_shadow_vision
    lead_moving = (
      selection is not None and selection.reason == "selected" and
      selection.track is not None and vision is not None and
      jeep_factory_sng_lead_moving(
        vision.status, vision.d_rel, vision.v_rel, vision.model_prob,
        selection.track.d_rel, selection.track.v_rel,
      )
    )
    if self.jeep_radar_shadow_updated:
      if (vision is not None and vision.status and
          vision.model_prob >= 0.90 and
          2.0 <= vision.d_rel <= 25.0):
        self.b6y_minimum_held_lead_distance = min(
          self.b6y_minimum_held_lead_distance, vision.d_rel,
        )
      self.b6y_lead_moving_frames = (
        min(B6Y_LEAD_CONFIRM_CYCLES, self.b6y_lead_moving_frames + 1)
        if lead_moving else 0
      )
      vision_lead_moving = (
        vision is not None and jeep_factory_sng_vision_lead_moving(
          vision.status, vision.d_rel, vision.v_rel, vision.model_prob,
          self.b6y_minimum_held_lead_distance,
        )
      )
      self.b6y_vision_lead_moving_frames = (
        min(B6Y_VISION_LEAD_CONFIRM_CYCLES,
            self.b6y_vision_lead_moving_frames + 1)
        if vision_lead_moving else 0
      )

    # Retry the counter-synchronized RESUME sequence at most three times, 0.5
    # seconds apart, after independent lead-motion confirmation.
    resume_ready = (
      self.b6y_lead_moving_frames >= B6Y_LEAD_CONFIRM_CYCLES
      or self.b6y_vision_lead_moving_frames >= B6Y_VISION_LEAD_CONFIRM_CYCLES
    )
    # A departing lead can quickly leave the close-range association after the
    # first pulse. Keep that independently confirmed departure latched for the
    # bounded retry sequence; pedals, faults, cancel, gear, and motion gates
    # above still release the hold immediately.
    self.b6y_lead_departure_latched |= resume_ready
    retry_ready = self.frame - self.b6y_last_resume_frame >= B6Y_RESUME_RETRY_FRAMES
    if (self.b6y_lead_departure_latched and retry_ready and
        self.b6y_resume_attempts < B6Y_RESUME_MAX_ATTEMPTS and
        self.b6y_resume_pulse_remaining == 0):
      self.b6y_resume_pulse_remaining = B6Y_RESUME_PULSE_COUNTERS
      self.b6y_last_resume_frame = self.frame
      self.b6y_resume_attempts += 1
      cloudlog.info(
        f"B6Y hold: RESUME pulse attempt={self.b6y_resume_attempts}"
      )

    driver_button_pressed = any(CS.buttonStates.get(name, False) for name in BUTTONS_STATES)
    if driver_button_pressed:
      self.b6y_resume_pulse_remaining = 0

    # The Jeep ignored a single 20 ms synthesized RESUME frame in route 5d.
    # A physical press that it recognized occupied seven consecutive SCCM
    # counters. Emit eight fresh, counter-synchronized frames per bounded attempt.
    if (self.b6y_resume_pulse_remaining > 0 and counter_changed and
        not driver_button_pressed):
      can_sends.append(chryslercan.create_cruise_buttons(
        self.packer,
        CS.button_counter + 1,
        0,
        self.CP,
        resume=True,
      ))
      self.b6y_resume_pulse_remaining -= 1

    if self.frame % 50 == 0:
      cloudlog.info(
        f"B6Y hold: direct brake={B6Y_STANDSTILL_HOLD_DECEL:.1f}, counter_offset={counter_offset}"
      )

  def jeep_long_vehicle_eligibility(self, CC, CS):
    if self.CP.carFingerprint not in JEEP_LONG_CARS:
      return False, "unsupported_vehicle"
    if not self.CP.openpilotLongitudinalControl:
      return False, "factory_acc_mode"
    if not self.CP.spFlags & ChryslerFlagsSP.SP_WP_S20:
      return False, "white_panda_flag_missing"
    if not CC.enabled:
      return False, "controls_disabled"
    if not CC.longActive:
      return False, "long_controls_inactive"
    if CS.out.gearShifter not in FORWARD_GEARS:
      return False, "gear"
    if not CS.out.cruiseState.available:
      return False, "cruise_unavailable"
    if not CS.out.cruiseState.enabled:
      return False, "op_long_inactive"
    if CS.out.accFaulted:
      return False, "acc_fault"
    if CS.out.brakePressed:
      return False, "brake_pressed"
    if CS.out.gasPressed:
      return False, "gas_pressed"
    if CS.out.stockAeb:
      return False, "stock_aeb"
    return True, "eligible"

  def update_jeep_long_plan_shadow(
      self,
      CS,
      now_nanos,
      vehicle_eligible,
      vehicle_reason,
  ):
    if (
        self.jeep_long_plan_sm is None
        or self.jeep_long_plan_shadow is None
    ):
      return

    self.jeep_long_plan_sm.update(0)
    seen = self.jeep_long_plan_sm.seen["longitudinalPlan"]
    service_valid = (
      seen and self.jeep_long_plan_sm.valid["longitudinalPlan"]
    )
    plan_mono_time = self.jeep_long_plan_sm.logMonoTime[
      "longitudinalPlan"
    ]
    plan_age_s = (
      (now_nanos - plan_mono_time) / 1e9
      if seen else float("inf")
    )
    self.jeep_long_plan_result = self.jeep_long_plan_shadow.update(
      plan=self.jeep_long_plan_sm["longitudinalPlan"],
      car_state=CS.out,
      seen=seen,
      service_valid=service_valid,
      plan_age_s=plan_age_s,
      vehicle_eligible=vehicle_eligible,
      vehicle_reason=vehicle_reason,
      radar_selection=self.jeep_radar_shadow_selection,
    )

  def update_jeep_radar_shadow(self, CS):
    self.jeep_radar_shadow_updated = False
    if self.jeep_radar_shadow_sm is None:
      return

    self.jeep_radar_shadow_sm.update(0)
    radar_shadow = CS.jeep_radar_shadow
    if radar_shadow.cycle_count == self.jeep_radar_shadow_last_cycle:
      return
    self.jeep_radar_shadow_last_cycle = radar_shadow.cycle_count
    self.jeep_radar_shadow_update_frame = self.frame
    self.jeep_radar_shadow_updated = True

    radar_state_valid = (
      self.jeep_radar_shadow_sm.seen["radarState"]
      and self.jeep_radar_shadow_sm.valid["radarState"]
    )
    if radar_state_valid:
      lead = self.jeep_radar_shadow_sm["radarState"].leadOne
      vision = JeepVisionLead(
        status=lead.status,
        d_rel=lead.dRel,
        v_rel=lead.vRel,
        model_prob=lead.modelProb,
        radar=lead.radar,
      )
    else:
      vision = JeepVisionLead(
        status=False,
        d_rel=0.0,
        v_rel=0.0,
        model_prob=0.0,
      )

    selection = radar_shadow.select(vision, CS.out.vEgo)
    self.jeep_radar_shadow_selection = selection
    self.jeep_radar_shadow_vision = vision
    self.jeep_radar_shadow_reason_counts[selection.reason] += 1

  def update_jeep_camera_lead_trend_shadow(self, CC, CS, now_nanos):
    """Update the passive camera-range observer; never alter control state."""
    if (
        self.jeep_camera_lead_trend_shadow is None
        or self.jeep_radar_shadow_sm is None
    ):
      return

    radar_state_seen = self.jeep_radar_shadow_sm.seen["radarState"]
    radar_state_valid = (
      radar_state_seen
      and self.jeep_radar_shadow_sm.valid["radarState"]
    )
    radar_state = self.jeep_radar_shadow_sm["radarState"]
    model_mono_time_ns = int(radar_state.mdMonoTime) if radar_state_valid else 0
    model_age_ns = (
      now_nanos - model_mono_time_ns
      if model_mono_time_ns > 0 else 2**63 - 1
    )
    lead = radar_state.leadOne

    scoped_to_jeep_wp_op_long = bool(
      JEEP_LONG_ACTUATION_COMPILED
      and self.CP.carFingerprint in JEEP_LONG_CARS
      and self.CP.openpilotLongitudinalControl
      and self.CP.spFlags & ChryslerFlagsSP.SP_WP_S20
    )
    eligible = jeep_camera_lead_trend_runtime_eligible(
      scoped_to_jeep_wp_op_long=scoped_to_jeep_wp_op_long,
      inputs_valid=radar_state_valid and model_mono_time_ns > 0,
      model_age_ns=model_age_ns,
      controls_enabled=CC.enabled,
      long_controls_active=CC.longActive,
      cruise_available=CS.out.cruiseState.available,
      cruise_enabled=CS.out.cruiseState.enabled,
      forward_gear=CS.out.gearShifter in FORWARD_GEARS,
      brake_pressed=CS.out.brakePressed,
      gas_pressed=CS.out.gasPressed,
      stock_aeb=CS.out.stockAeb,
      acc_faulted=CS.out.accFaulted,
    )

    # Raw FCA radar is passed only as an optional annotation.  Selection and
    # freshness cannot affect any camera candidate gate or control output.
    selection = self.jeep_radar_shadow_selection
    radar_track = (
      selection.track
      if selection is not None and selection.track is not None else None
    )
    radar_age_ns = (
      int(max(0, self.frame - self.jeep_radar_shadow_update_frame) * DT_CTRL * 1e9)
      if radar_track is not None else None
    )
    self.jeep_camera_lead_trend_result = (
      self.jeep_camera_lead_trend_shadow.update(
        eligible=eligible,
        model_mono_time_ns=model_mono_time_ns,
        model_age_ns=model_age_ns,
        v_ego_mps=CS.out.vEgo,
        vision_status=lead.status,
        vision_radar=lead.radar,
        model_probability=lead.modelProb,
        d_rel_m=lead.dRel,
        radar_d_rel_m=(radar_track.d_rel if radar_track is not None else None),
        radar_v_rel_mps=(radar_track.v_rel if radar_track is not None else None),
        radar_age_ns=radar_age_ns,
      )
    )

  def log_jeep_long_plan_shadow(self, CS):
    if (
        self.jeep_long_plan_shadow is None
        or self.jeep_long_plan_result is None
    ):
      return

    result = self.jeep_long_plan_result
    window = self.jeep_long_plan_shadow.snapshot()
    cloudlog.info(
      f"Jeep long plan shadow: samples={window.samples},"
      f"plan_valid={window.plan_valid_samples},"
      f"eligible={window.eligible_samples},"
      f"lead={window.lead_samples},"
      f"radar_supported={window.radar_supported_samples},"
      f"lead_confirmed={window.lead_confirmed_samples},"
      f"lead_unconfirmed={window.lead_unconfirmed_samples},"
      f"radar_only={window.radar_only_samples},"
      f"fcw={window.fcw_samples},"
      f"brake_request={window.brake_request_samples},"
      f"engine_request={window.engine_request_samples},"
      f"max_brake={window.max_brake_mps2:.3f},"
      f"max_accel={window.max_accel_mps2:.3f},"
      f"last_reason={result.reason},"
      f"source={result.source},"
      f"has_lead={result.has_lead},"
      f"lead_state={result.lead_state},"
      f"plan_age={result.plan_age_s:.3f},"
      f"target_v={result.target_speed_mps:.3f},"
      f"target_a={result.target_accel_mps2:.3f},"
      f"controller_a={result.controller_accel_mps2:.3f},"
      f"control_state={result.control_state},"
      f"a_ego={CS.out.aEgo:.3f},"
      f"radar_d={result.radar_d_rel:.2f},"
      f"radar_v={result.radar_v_rel:.2f},"
      f"transport={self.jeep_long_envelope.transport_enabled},"
      f"host_enabled={self.jeep_long_envelope.host_enabled}"
    )

  @staticmethod
  def log_wp_long_diagnostic(CS):
    diagnostic = CS.wp_long_diagnostic
    if diagnostic is None:
      return
    cloudlog.info(
      f"Jeep WP long diagnostic: valid={diagnostic.valid},"
      f"applied={diagnostic.applied},"
      f"host_requested={diagnostic.host_requested},"
      f"brake_requested={diagnostic.brake_requested},"
      f"engine_requested={diagnostic.engine_requested},"
      f"failure_mask=0x{diagnostic.failure_mask:04x},"
      f"failure_reasons={'|'.join(diagnostic.failure_reasons) or 'none'},"
      f"private_counter={diagnostic.private_counter},"
      f"stock_counter={diagnostic.stock_counter}"
    )
    command = CS.wp_long_command_diagnostic
    if command is not None:
      cloudlog.info(
        f"Jeep WP command diagnostic: valid={command.valid},"
        f"version={command.version},"
        f"stock_engine={command.stock_engine_active},"
        f"stock_torque={command.stock_engine_torque_nm:.2f},"
        f"output_engine={command.output_engine_active},"
        f"output_torque={command.output_engine_torque_nm:.2f},"
        f"stock_acc_available={command.stock_acc_available},"
        f"stock_acc_active={command.stock_acc_active},"
        f"stock_accel={command.stock_accel_mps2:.4f}"
      )
    owner = CS.wp_long_owner_diagnostic
    if owner is not None:
      cloudlog.info(
        f"Jeep WP long owner: valid={owner.valid},"
        f"owner={owner.owner_name},"
        f"stock_valid={owner.stock_valid},"
        f"stock_available={owner.stock_available},"
        f"stock_active={owner.stock_active},"
        f"stock_fault={owner.stock_fault},"
        f"stock_collision={owner.stock_collision},"
        f"cancel_injected={owner.cancel_injected}"
      )

  def log_jeep_radar_shadow(self, CS):
    selection = self.jeep_radar_shadow_selection
    vision = self.jeep_radar_shadow_vision
    radar_shadow = CS.jeep_radar_shadow
    reason_counts = ",".join(
      f"{reason}:{count}"
      for reason, count in sorted(
        self.jeep_radar_shadow_reason_counts.items(),
      )
    ) or "none"

    if selection is None or vision is None:
      last_result = "not_evaluated"
    elif selection.track is None:
      score = (
        "none" if selection.score is None
        else f"{selection.score:.3f}"
      )
      last_result = (
        f"{selection.reason},score={score},"
        f"vision_d={vision.d_rel:.2f},vision_v={vision.v_rel:.2f},"
        f"prob={vision.model_prob:.3f}"
      )
    else:
      last_result = (
        f"selected,track={selection.track.index},"
        f"score={selection.score:.3f},"
        f"radar_d={selection.track.d_rel:.2f},"
        f"radar_v={selection.track.v_rel:.2f},"
        f"vision_d={vision.d_rel:.2f},vision_v={vision.v_rel:.2f},"
        f"prob={vision.model_prob:.3f}"
      )

    cloudlog.info(
      f"Jeep radar shadow: cycles={radar_shadow.cycle_count},"
      f"complete={radar_shadow.complete_cycle_count},"
      f"tracks={len(radar_shadow.tracks)},"
      f"window={reason_counts},last={last_result}"
    )
    self.jeep_radar_shadow_reason_counts.clear()

  def log_jeep_camera_lead_trend_shadow(self):
    if (
        self.jeep_camera_lead_trend_shadow is None
        or self.jeep_camera_lead_trend_result is None
    ):
      return

    result = self.jeep_camera_lead_trend_result
    window = self.jeep_camera_lead_trend_shadow.snapshot()
    reason_counts = "|".join(
      f"{reason}:{count}" for reason, count in window.reason_counts
    ) or "none"
    closing = (
      "none" if result.observed_closing_mps is None
      else f"{result.observed_closing_mps:.3f}"
    )
    ttc = (
      "none" if result.camera_ttc_s is None
      else f"{result.camera_ttc_s:.3f}"
    )
    radar_error = (
      "none" if result.radar_distance_error_m is None
      else f"{result.radar_distance_error_m:.3f}"
    )
    cloudlog.info(
      f"Jeep camera lead trend shadow: distinct={window.distinct_samples},"
      f"eligible={window.eligible_samples},"
      f"hazard_samples={window.hazard_samples},"
      f"hazard_events={window.hazard_events},"
      f"radar_corroborated_hazards="
      f"{window.radar_corroborated_hazard_samples},"
      f"reasons={reason_counts},"
      f"candidate={result.hazard_candidate},"
      f"last_reason={result.reason},"
      f"samples={result.sample_count},"
      f"span={result.window_span_s:.3f},"
      f"camera_d={result.d_rel_m},"
      f"camera_closing={closing},"
      f"camera_ttc={ttc},"
      f"monotonic={result.monotonic_fraction:.3f},"
      f"radar_corroborated={result.radar_corroborated},"
      f"radar_d={result.radar_d_rel_m},"
      f"radar_v={result.radar_v_rel_mps},"
      f"radar_error={radar_error}"
    )

  def log_jeep_steering_shadow(self):
    if self.jeep_steering_shadow is None:
      return

    window = self.jeep_steering_shadow.snapshot()
    cloudlog.info(
      f"Jeep steer shadow: samples={window.samples},"
      f"active={window.active_samples},"
      f"request_near_full={window.request_near_full_samples},"
      f"request_at_261={window.request_at_ceiling_samples},"
      f"applied_at_261={window.applied_at_ceiling_samples},"
      f"error_limited={window.error_limited_samples},"
      f"rate_limited={window.rate_limited_samples},"
      f"suppressed={window.suppressed_samples},"
      f"driver_override={window.driver_override_samples},"
      f"eps_over_261={window.eps_over_limit_samples},"
      f"steer_required={window.steer_required_samples},"
      f"temporary_fault={window.temporary_fault_samples},"
      f"permanent_fault={window.permanent_fault_samples},"
      f"limiter_mismatch={window.limiter_mismatch_samples},"
      f"max_request_norm={window.max_requested_normalized:.3f},"
      f"max_request_raw={window.max_requested_raw},"
      f"max_limited_raw={window.max_limited_raw},"
      f"max_applied_raw={window.max_applied_raw},"
      f"max_eps={window.max_eps_torque:.1f},"
      f"max_driver={window.max_driver_torque:.1f},"
      f"max_request_limited_gap={window.max_request_limited_gap},"
      f"max_request_applied_gap={window.max_request_applied_gap},"
      f"longest_request_261_ms={window.longest_request_ceiling_ms},"
      f"longest_applied_261_ms={window.longest_applied_ceiling_ms}"
    )

  def log_jeep_steering_rate5_shadow(self):
    if self.jeep_steering_rate5_shadow is None:
      return

    window = self.jeep_steering_rate5_shadow.snapshot()
    cloudlog.info(
      f"Jeep steer rate5 shadow: samples={window.samples},"
      f"active={window.active_samples},"
      f"changed={window.changed_samples},"
      f"improved={window.improved_samples},"
      f"worse={window.worse_samples},"
      f"equal={window.equal_samples},"
      f"panda_rate_violation="
      f"{window.current_panda_rate_violation_samples},"
      f"candidate_at_261={window.candidate_ceiling_samples},"
      f"max_candidate={window.max_candidate_raw},"
      f"max_delta={window.max_candidate_delta},"
      f"max_divergence={window.max_candidate_divergence},"
      f"mean_gap_rate3={window.mean_current_request_gap:.3f},"
      f"mean_gap_rate5={window.mean_candidate_request_gap:.3f},"
      f"applied_rate={self.params.STEER_DELTA_UP},candidate_rate=5,"
      f"candidate_applied=True"
    )

  # multikyd methods, sunnyhaibin logic
  def get_cruise_buttons_status(self, CS):
    if not CS.out.cruiseState.enabled or any(CS.buttonStates[button_state] for button_state in BUTTONS_STATES):
      self.timer = 40
    elif self.timer:
      self.timer -= 1
    else:
      return 1
    return 0

  def get_target_speed(self, v_cruise_kph_prev):
    v_cruise_kph = v_cruise_kph_prev
    if self.slc_state > 1:
      v_cruise_kph = (self.speed_limit + self.speed_limit_offset) * CV.MS_TO_KPH
      if not self.slc_active_stock:
        v_cruise_kph = v_cruise_kph_prev
    return v_cruise_kph

  def get_button_type(self, button_type):
    self.type_status = "type_" + str(button_type)
    self.button_picker = getattr(self, self.type_status, lambda: "default")
    return self.button_picker()

  def reset_button(self):
    if self.button_type != 3:
      self.button_type = 0

  def type_default(self):
    self.button_type = 0
    return None

  def type_0(self):
    self.button_count = 0
    self.target_speed = self.init_speed
    self.speed_diff = self.target_speed - self.v_set_dis
    if self.target_speed > self.v_set_dis:
      self.button_type = 1
    elif self.target_speed < self.v_set_dis and self.v_set_dis > self.v_cruise_min:
      self.button_type = 2
    return None

  def type_1(self):
    cruise_button = 1
    self.button_count += 1
    if self.target_speed <= self.v_set_dis:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 5:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_2(self):
    cruise_button = 2
    self.button_count += 1
    if self.target_speed >= self.v_set_dis or self.v_set_dis <= self.v_cruise_min:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 5:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_3(self):
    cruise_button = None
    self.button_count += 1
    if self.button_count > self.t_interval:
      self.button_type = 0
    return cruise_button

  def get_curve_speed(self, target_speed_kph, v_cruise_kph_prev):
    if self.v_tsc_state != 0:
      vision_v_cruise_kph = self.v_tsc * CV.MS_TO_KPH
      if int(vision_v_cruise_kph) == int(v_cruise_kph_prev):
        vision_v_cruise_kph = 255
    else:
      vision_v_cruise_kph = 255
    if self.m_tsc_state > 1:
      map_v_cruise_kph = self.m_tsc * CV.MS_TO_KPH
      if int(map_v_cruise_kph) == 0.0:
        map_v_cruise_kph = 255
    else:
      map_v_cruise_kph = 255
    curve_speed = self.curve_speed_hysteresis(min(vision_v_cruise_kph, map_v_cruise_kph) + 2 * CV.MPH_TO_KPH)
    return min(target_speed_kph, curve_speed)

  def get_button_control(self, CS, final_speed, v_cruise_kph_prev):
    self.init_speed = round(min(final_speed, v_cruise_kph_prev) * (CV.KPH_TO_MPH if not self.is_metric else 1))
    self.v_set_dis = round(CS.out.cruiseState.speed * (CV.MS_TO_MPH if not self.is_metric else CV.MS_TO_KPH))
    cruise_button = self.get_button_type(self.button_type)
    return cruise_button

  def curve_speed_hysteresis(self, cur_speed: float, hyst=(0.75 * CV.MPH_TO_KPH)):
    if cur_speed > self.steady_speed:
      self.steady_speed = cur_speed
    elif cur_speed < self.steady_speed - hyst:
      self.steady_speed = cur_speed
    return self.steady_speed

  def get_cruise_buttons(self, CS, v_cruise_kph_prev):
    cruise_button = None
    if not self.get_cruise_buttons_status(CS):
      pass
    elif CS.out.cruiseState.enabled:
      set_speed_kph = self.get_target_speed(v_cruise_kph_prev)
      if self.slc_state > 1:
        target_speed_kph = set_speed_kph
      else:
        target_speed_kph = min(v_cruise_kph_prev, set_speed_kph)
      if self.v_tsc_state != 0 or self.m_tsc_state > 1:
        self.final_speed_kph = self.get_curve_speed(target_speed_kph, v_cruise_kph_prev)
      else:
        self.final_speed_kph = target_speed_kph

      cruise_button = self.get_button_control(CS, self.final_speed_kph, v_cruise_kph_prev)  # MPH/KPH based button presses
    return cruise_button

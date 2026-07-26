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
from openpilot.selfdrive.car.chrysler.jeep_radar_shadow import JeepVisionLead
from openpilot.selfdrive.car.chrysler.jeep_longitudinal import JeepLongitudinalShadow
from openpilot.selfdrive.car.chrysler.jeep_steering_shadow import JeepSteeringShadow
from openpilot.selfdrive.car.chrysler.values import CAR, RAM_CARS, RAM_DT, STEER_THRESHOLD, CarControllerParams, ChryslerFlags, ChryslerFlagsSP
from openpilot.selfdrive.car.interfaces import CarControllerBase, FORWARD_GEARS
from openpilot.selfdrive.controls.lib.drive_helpers import FCA_V_CRUISE_MIN

BUTTONS_STATES = ["accelCruise", "decelCruise", "cancel", "resumeCruise"]

BRAKE_HOLD_CARS = {
  CAR.JEEP_GRAND_CHEROKEE,
  CAR.JEEP_GRAND_CHEROKEE_2019,
}


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.frame = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.lkas_control_bit_prev = False
    self.last_button_frame = 0
    self.bh_hold_decel = -2.0
    self.last_das_3_counter = -1
    self.bh_last_resume_frame = -100
    self.jeep_long_shadow = JeepLongitudinalShadow()
    self.jeep_long_envelope = self.jeep_long_shadow.update(0.0, eligible=False)
    self.jeep_long_shadow_frames = []
    self.jeep_radar_shadow_sm = (
      messaging.SubMaster(["radarState"])
      if CP.carFingerprint in BRAKE_HOLD_CARS else None
    )
    self.jeep_radar_shadow_last_cycle = 0
    self.jeep_radar_shadow_selection = None
    self.jeep_radar_shadow_vision = None
    self.jeep_radar_shadow_reason_counts = Counter()

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
      if CP.carFingerprint in BRAKE_HOLD_CARS else None
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
    jeep_long_eligible = (
      self.CP.carFingerprint in BRAKE_HOLD_CARS and
      bool(self.CP.spFlags & ChryslerFlagsSP.SP_WP_S20) and
      CC.enabled and
      CS.out.gearShifter in FORWARD_GEARS and
      CS.out.cruiseState.available and
      not CS.out.accFaulted and
      not CS.out.brakePressed and
      not CS.out.gasPressed and
      not CS.out.stockAeb
    )
    if self.frame % 2 == 0:
      self.jeep_long_envelope = self.jeep_long_shadow.update(CC.actuators.accel, jeep_long_eligible)
      self.jeep_long_shadow_frames = chryslercan.create_wp_long_shadow_messages(
        self.packer, self.jeep_long_envelope, self.frame // 2)
      if self.jeep_long_envelope.transport_enabled:
        can_sends.extend(self.jeep_long_shadow_frames)

    if self.frame % 100 == 0 and self.CP.spFlags & ChryslerFlagsSP.SP_WP_S20:
      cloudlog.info(
        f"Jeep long shadow: eligible={self.jeep_long_envelope.eligible}, "
        f"requested={self.jeep_long_envelope.requested_accel:.3f}, "
        f"limited={self.jeep_long_envelope.limited_accel:.3f}, "
        f"brake={self.jeep_long_envelope.brake_active}, "
        f"engine={self.jeep_long_envelope.engine_active}, "
        f"torque={self.jeep_long_envelope.engine_torque_nm:.1f}, "
        f"transport={self.jeep_long_envelope.transport_enabled}, "
        f"host_enabled={self.jeep_long_envelope.host_enabled}"
      )
      self.log_jeep_radar_shadow(CS)
      self.log_jeep_steering_shadow()

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
        lkas_control_bit = CC.latActive and CS.out.gearShifter in FORWARD_GEARS
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

      # steer torque
      previous_apply_steer = self.apply_steer_last
      new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
      limited_steer = apply_meas_steer_torque_limits(
        new_steer,
        previous_apply_steer,
        CS.out.steeringTorqueEps,
        self.params,
      )
      control_allowed = (
        lkas_active
        and lkas_control_bit
        and self.lkas_control_bit_prev
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
      self.apply_steer_last = apply_steer
      self.lkas_control_bit_prev = lkas_control_bit

      can_sends.append(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_steer), lkas_control_bit))

    if self.CP.carFingerprint in BRAKE_HOLD_CARS:
      self.brake_hold(CC, CS, can_sends)

    self.frame += 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    return new_actuators, can_sends

  def update_jeep_radar_shadow(self, CS):
    if self.jeep_radar_shadow_sm is None:
      return

    self.jeep_radar_shadow_sm.update(0)
    radar_shadow = CS.jeep_radar_shadow
    if radar_shadow.cycle_count == self.jeep_radar_shadow_last_cycle:
      return
    self.jeep_radar_shadow_last_cycle = radar_shadow.cycle_count

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

  def brake_hold(self, CC, CS, can_sends):
    """Maintain stock ACC braking after the Jeep's stop-and-go timeout."""
    if not CS.das_3:
      return

    counter = CS.das_3.get("COUNTER")
    counter_changed = counter != self.last_das_3_counter
    self.last_das_3_counter = counter

    if (not CS.brake_hold and CS.cruise_active_actual and
        CS.acc_decelerating and CS.out.standstill):
      CS.brake_hold = True
      self.bh_last_resume_frame = self.frame - 10
      cloudlog.info("Brake hold: ACTIVATING - ACC decelerating to standstill")

    if (CS.brake_hold and
        (CC.cruiseControl.cancel or CS.out.gasPressed or
         CS.out.brakePressed or not CS.forward_gear or
         not CS.out.standstill)):
      CS.brake_hold = False
      cloudlog.info("Brake hold: DEACTIVATING")
      return

    if not CS.brake_hold:
      return

    if CS.cruise_active_actual:
      if CS.out.standstill:
        self.bh_hold_decel = min(
          self.bh_hold_decel,
          CS.das_3.get("ACC_DECEL", -2.0),
        )
      else:
        self.bh_hold_decel = -2.0
      return

    counter_offset = 2 if counter_changed else 3
    can_sends.append(chryslercan.das_3_command(
      self.packer,
      counter_offset,
      False,  # go
      False,  # torque request
      None,   # torque
      2,      # maximum requested gear
      False,  # standstill flag
      self.bh_hold_decel,
      False,  # brake preparation
      CS.das_3,
    ))

    if self.frame - self.bh_last_resume_frame >= 10:
      can_sends.append(chryslercan.create_cruise_buttons(
        self.packer,
        CS.button_counter + 1,
        0,
        self.CP,
        resume=True,
      ))
      self.bh_last_resume_frame = self.frame

    if self.frame % 50 == 0:
      cloudlog.info(
        f"Brake hold: Sending DAS_3 - decel={self.bh_hold_decel}, "
        f"counter_offset={counter_offset}"
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

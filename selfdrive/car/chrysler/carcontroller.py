from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_meas_steer_torque_limits
from openpilot.selfdrive.car.chrysler import chryslercan
from openpilot.selfdrive.car.chrysler.values import RAM_CARS, RAM_DT, CarControllerParams, ChryslerFlags, CAR
from openpilot.selfdrive.car.chrysler.values import JEEPS as CHRYSLER_JEEPS
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.common.swaglog import cloudlog


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.frame = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.lkas_control_bit_prev = False
    self.last_button_frame = 0

    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(CP)

    # Brake hold (Jeep SNG workaround) - matching jvePilot implementation
    self.bh_hold_decel = -2.0  # Default brake decel value like jvePilot
    self.last_das_3_counter = -1

  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    can_sends = []

    lkas_active = CC.latActive and self.lkas_control_bit_prev

    # cruise buttons
    if (self.frame - self.last_button_frame)*DT_CTRL > 0.05:
      das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0

      # ACC cancellation (disabled for brake hold compatibility - matches jvePilot)
      # if CC.cruiseControl.cancel:
      #   self.last_button_frame = self.frame
      #   can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, CS.button_message, cancel=True))

      # ACC resume from standstill (disabled for brake hold compatibility - matches jvePilot)
      # elif CC.cruiseControl.resume:
      #   self.last_button_frame = self.frame
      #   can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, CS.button_message, resume=True))

    # HUD alerts
    if self.frame % 25 == 0:
      if CS.lkas_car_model != -1:
        can_sends.append(chryslercan.create_lkas_hud(self.packer, self.CP, lkas_active, CC.hudControl.visualAlert,
                                                     self.hud_count, CS.lkas_car_model, CS.auto_high_beam, CC.latActive,
                                                     CC.hudControl.leftLaneVisible, CC.hudControl.rightLaneVisible, CC.hudControl.leftLaneClose,
                                                     CC.hudControl.rightLaneClose, CC.hudControl.leftLaneDepart, CC.hudControl.rightLaneDepart))
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
      elif CS.out.vEgo > self.CP.minSteerSpeed:
        lkas_control_bit = True
      elif self.CP.flags & ChryslerFlags.HIGHER_MIN_STEERING_SPEED:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
          lkas_control_bit = False
      elif self.CP.carFingerprint in RAM_CARS:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 0.5):
          lkas_control_bit = False

      # EPS faults if LKAS re-enables too quickly
      lkas_control_bit = lkas_control_bit and (self.frame - self.last_lkas_falling_edge > 200)

      if not lkas_control_bit and self.lkas_control_bit_prev:
        self.last_lkas_falling_edge = self.frame
      self.lkas_control_bit_prev = lkas_control_bit

      # steer torque
      new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
      apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params)
      if not lkas_active or not lkas_control_bit:
        apply_steer = 0
      self.apply_steer_last = apply_steer

      can_sends.append(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_steer), lkas_control_bit))

    # Jeep Brake Hold: keep standstill asserted (and optional small brake) after SNG auto-cancel
    # This mirrors the jvePilot implementation for proper brake hold functionality
    try:
      is_jeep = (self.CP.carFingerprint in CHRYSLER_JEEPS)
    except Exception:
      is_jeep = False

    # Jeep Brake Hold: keep standstill asserted (and optional small brake) after SNG auto-cancel
    # This mirrors the jvePilot implementation exactly
    if is_jeep and getattr(frogpilot_toggles, 'jeep_brake_hold', False):
      self.brake_hold(CC, CS, can_sends)

    self.frame += 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    return new_actuators, can_sends

  def brake_hold(self, CC, CS, can_sends):
    """Jeep brake hold implementation matching jvePilot exactly

    Activates when ACC is decelerating to a stop, then maintains brake pressure
    after ACC times out due to SNG limitation.
    """
    # Track DAS_3 counter changes for proper message timing
    counter_changed = (CS.das_3.get('COUNTER') != self.last_das_3_counter)
    self.last_das_3_counter = CS.das_3.get('COUNTER')

    # Diagnostic snapshot every ~1 second
    if self.frame % 50 == 0:
      cloudlog.info(
        "Brake hold debug: "
        f"hold={CS.brake_hold}, "
        f"cruise_active_actual={CS.cruise_active_actual}, "
        f"acc_decelerating={CS.acc_decelerating}, "
        f"standstill={CS.out.standstill}, "
        f"cruise_enabled={CS.out.cruiseState.enabled}, "
        f"gas={CS.out.gasPressed}, "
        f"brake={CS.out.brakePressed}, "
        f"forward_gear={CS.forward_gear}, "
        f"das3_counter={CS.das_3.get('COUNTER')}, "
        f"acc_decel={CS.das_3.get('ACC_DECEL')}"
      )

    # Brake hold activation: engage when ACC is decelerating to a stop
    if (not CS.brake_hold and
        CS.cruise_active_actual and CS.acc_decelerating and CS.out.standstill):
      CS.brake_hold = True
      cloudlog.info("Brake hold: ACTIVATING - ACC decelerating to standstill")
    # Brake hold deactivation: release ONLY on driver intervention (not on ACC/openpilot state changes)
    # This allows brake hold to persist even when ACC times out or openpilot disables
    if (CS.brake_hold and
        (CC.cruiseControl.cancel or CS.out.gasPressed or
         CS.out.brakePressed or not CS.forward_gear or not CS.out.standstill)):
      CS.brake_hold = False
      cloudlog.info("Brake hold: DEACTIVATING")
      return

    # Send DAS_3 brake hold command when active (matching jvePilot)
    if CS.brake_hold:
      das_bus = 0  # Jeep/Pacifica on bus 0

      # Track decel like jvePilot (uses actual ACC_DECEL value, not calculated)
      if CS.cruise_active_actual:
        self.bh_hold_decel = min(self.bh_hold_decel, CS.das_3.get('ACC_DECEL', -2.0)) if CS.out.standstill else -2.0
      else:
        # Send brake hold message with proper parameters (matching jvePilot)
        counter_offset = 2 if counter_changed else 3
        msg = chryslercan.das_3_command(self.packer, counter_offset,
                                        False,  # go
                                        False,  # torque_req
                                        None,   # torque
                                        2,      # max_gear
                                        False,  # stop (standstill)
                                        self.bh_hold_decel,  # brake
                                        False,  # brake_prep
                                        CS.das_3)
        can_sends.append(msg)

        # Send Resume button press every 10 frames (~0.2 seconds) to attempt ACC resume
        if self.frame % 10 == 0:
          resume_msg = chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus,
                                                         CS.button_message, resume=True)
          can_sends.append(resume_msg)

        # Log every 50 frames (~1 second) to avoid spam
        if self.frame % 50 == 0:
          cloudlog.info(f"Brake hold: Sending DAS_3 - decel={self.bh_hold_decel}, counter_offset={counter_offset}")

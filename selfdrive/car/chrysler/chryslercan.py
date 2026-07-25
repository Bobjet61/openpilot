from cereal import car
from openpilot.selfdrive.car.chrysler.values import RAM_CARS

GearShifter = car.CarState.GearShifter
VisualAlert = car.CarControl.HUDControl.VisualAlert

def create_lkas_hud(packer, CP, lkas_active, mads_enabled, hud_alert, hud_count, car_model, auto_high_beam):
  # LKAS_HUD - Controls what lane-keeping icon is displayed

  # == Color ==
  # 0 hidden?
  # 1 white
  # 2 green
  # 3 ldw

  # == Lines ==
  # 03 white Lines
  # 04 grey lines
  # 09 left lane close
  # 0A right lane close
  # 0B left Lane very close
  # 0C right Lane very close
  # 0D left cross cross
  # 0E right lane cross

  # == Alerts ==
  # 7 Normal
  # 6 lane departure place hands on wheel

  color = 2 if lkas_active else 1 if mads_enabled and not lkas_active else 0
  lines = 3 if lkas_active else 0
  alerts = 7 if lkas_active else 0

  if hud_count < (1 * 4):  # first 3 seconds, 4Hz
    alerts = 1

  if hud_alert in (VisualAlert.ldw, VisualAlert.steerRequired):
    color = 4
    lines = 0
    alerts = 6

  values = {
    "LKAS_ICON_COLOR": color,
    "CAR_MODEL": car_model,
    "LKAS_LANE_LINES": lines,
    "LKAS_ALERTS": alerts,
  }

  if CP.carFingerprint in RAM_CARS:
    values['AUTO_HIGH_BEAM_ON'] = auto_high_beam
    values['LKAS_DISABLED'] = 0 if mads_enabled else 1

  return packer.make_can_msg("DAS_6", 0, values)


def create_lkas_command(packer, CP, apply_steer, lkas_control_bit):
  # LKAS_COMMAND Lane-keeping signal to turn the wheel
  enabled_val = 2 if CP.carFingerprint in RAM_CARS else 1
  values = {
    "STEERING_TORQUE": apply_steer,
    "LKAS_CONTROL_BIT": enabled_val if lkas_control_bit else 0,
  }
  return packer.make_can_msg("LKAS_COMMAND", 0, values)


def create_cruise_buttons(packer, frame, bus, CP, cruise_buttons_msg=None, buttons=0, cancel=False, resume=False):

  acc_accel = 1 if buttons == 1 else 0
  acc_decel = 1 if buttons == 2 else 0

  values = {
    "ACC_Cancel": cancel,
    "ACC_Resume": resume,
    "ACC_Accel": acc_accel,
    "ACC_Decel": acc_decel,
    "COUNTER": frame % 0x10,
  }

  if buttons == 0 and not (cancel or resume) and CP.carFingerprint in RAM_CARS:
    values = cruise_buttons_msg.copy()
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)


def das_3_command(packer, counter_offset, go, torque_req, torque, max_gear, stop, brake, brake_prep, das_3):
  """Create DAS_3 command message like jvePilot implementation."""
  values = das_3.copy()
  values["ACC_AVAILABLE"] = 1
  values["ACC_ACTIVE"] = 1
  values["COUNTER"] = (das_3["COUNTER"] + counter_offset) % 0x10

  if go is not None:
    values["ACC_GO"] = go

  if stop is not None:
    values["ACC_STANDSTILL"] = stop

  if brake is not None:
    values["ACC_DECEL_REQ"] = 1
    values["ACC_DECEL"] = brake
    values["ACC_BRK_PREP"] = brake_prep

  if torque is not None:
    values["ENGINE_TORQUE_REQUEST_MAX"] = torque_req
    values["ENGINE_TORQUE_REQUEST"] = torque

  if max_gear is not None:
    values["GR_MAX_REQ"] = max_gear

  return packer.make_can_msg("DAS_3", 0, values)


def create_wp_long_shadow_messages(packer, envelope, counter):
  """Pack calibrated Jeep White Panda commands with OP longitudinal disabled."""
  counter %= 0x10
  brake_values = {
    "ACC_STOP": 0,
    "ACC_GO": 0,
    # Stock DAS_3 uses its +4.0 m/s^2 encoded maximum as the inactive
    # deceleration sentinel whenever ACC_DECEL_REQ is zero.
    "ACC_DECEL_CMD": envelope.limited_accel if envelope.brake_active else 4.0,
    "ACC_AVAILABLE": envelope.eligible,
    "ACC_ENABLED": envelope.eligible,
    # Stock logs show brake-prep is not a normal-braking enable bit. The
    # initial moving-only shadow excludes brake-prep and stop/go completely.
    "ACC_BRK_PREP": 0,
    "COMMAND_TYPE": 1 if envelope.brake_active else 0,
    "COUNTER": counter,
  }
  dash_values = {
    "ACC_DISP_MSG": 0,
    "ACC_SET_SPEED_KPH": 0,
    "ACC_SET_SPEED_MPH": 0,
    "OP_LONG_ENABLE": 0,
    "CRUISE_STATE": 0,
    "CRUISE_ICON": 0,
    "LEAD_DIST": 255,
  }
  torque_values = {
    "ENGINE_TORQUE_REQUEST_MAX": envelope.engine_active,
    "ENGINE_TORQUE_REQUEST": envelope.engine_torque_nm,
    "COUNTER": counter,
  }
  return [
    packer.make_can_msg("WP_ACC_BRAKE_CMD", 0, brake_values),
    packer.make_can_msg("WP_ACC_DASH_CMD", 0, dash_values),
    packer.make_can_msg("WP_ACC_TORQUE_CMD", 0, torque_values),
  ]


def create_lkas_heartbit(packer, lkas_disabled, lkas_heartbit):
  # LKAS_HEARTBIT (697) LKAS heartbeat
  values = lkas_heartbit.copy()  # forward what we parsed
  values["LKAS_DISABLED"] = 1 if lkas_disabled else 0
  return packer.make_can_msg("LKAS_HEARTBIT", 0, values)

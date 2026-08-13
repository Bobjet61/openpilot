import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

LONG_PATH = Path(__file__).resolve().parents[1] / "jeep_longitudinal.py"
PLANNER_PATH = (
  Path(__file__).resolve().parents[1]
  / "jeep_longitudinal_planner_shadow.py"
)
CARCONTROLLER_PATH = Path(__file__).resolve().parents[1] / "carcontroller.py"
CARSTATE_PATH = Path(__file__).resolve().parents[1] / "carstate.py"
INTERFACE_PATH = Path(__file__).resolve().parents[1] / "interface.py"
CHRYSLERCAN_PATH = Path(__file__).resolve().parents[1] / "chryslercan.py"
CONTROLSD_PATH = Path(__file__).resolve().parents[3] / "controls" / "controlsd.py"
DRIVE_HELPERS_PATH = Path(__file__).resolve().parents[3] / "controls" / "lib" / "drive_helpers.py"
LONG_SPEC = importlib.util.spec_from_file_location("jeep_longitudinal_under_test", LONG_PATH)
assert LONG_SPEC is not None and LONG_SPEC.loader is not None
LONG = importlib.util.module_from_spec(LONG_SPEC)
sys.modules[LONG_SPEC.name] = LONG
LONG_SPEC.loader.exec_module(LONG)

ACCEL_MAX = LONG.ACCEL_MAX
ACCEL_MIN = LONG.ACCEL_MIN
JEEP_LONG_ACTUATION_COMPILED = LONG.JEEP_LONG_ACTUATION_COMPILED
JEEP_LONG_REJECT_DIAGNOSTICS_COMPILED = (
  LONG.JEEP_LONG_REJECT_DIAGNOSTICS_COMPILED
)
JEEP_LONG_SHADOW_TRANSPORT_COMPILED = LONG.JEEP_LONG_SHADOW_TRANSPORT_COMPILED
JeepLongitudinalShadow = LONG.JeepLongitudinalShadow
JeepLongitudinalTransportScheduler = LONG.JeepLongitudinalTransportScheduler
TRANSPORT_MIN_SEND_INTERVAL_NS = LONG.TRANSPORT_MIN_SEND_INTERVAL_NS
decode_wp_long_diagnostic = LONG.decode_wp_long_diagnostic
decode_wp_long_command_diagnostic = LONG.decode_wp_long_command_diagnostic
decode_wp_long_owner_diagnostic = LONG.decode_wp_long_owner_diagnostic
jeep_acc_faulted = LONG.jeep_acc_faulted
jeep_factory_sng_lead_moving = LONG.jeep_factory_sng_lead_moving
jeep_factory_sng_vision_lead_moving = LONG.jeep_factory_sng_vision_lead_moving
jeep_long_actuation_enabled = LONG.jeep_long_actuation_enabled
jeep_long_mode_safety_param = LONG.jeep_long_mode_safety_param
select_jeep_longitudinal_mode = LONG.select_jeep_longitudinal_mode
fca_checksum = LONG.fca_checksum
jeep_long_shadow_safety_param = LONG.jeep_long_shadow_safety_param
engine_torque_max_for_speed = LONG.engine_torque_max_for_speed


class PrivateMessagePacker:
  """Small byte-level packer for the three private transport frames."""

  ADDRESSES = {
    "WP_ACC_BRAKE_CMD": 0x1F6,
    "WP_ACC_DASH_CMD": 0x1F7,
    "WP_ACC_TORQUE_CMD": 0x272,
  }

  def make_can_msg(self, name, bus, values):
    dat = bytearray(8)
    if name == "WP_ACC_BRAKE_CMD":
      dat[0] |= int(values.get("ACC_STOP", 0)) << 5
      dat[0] |= int(values.get("ACC_GO", 0)) << 6
      decel_raw = round((values.get("ACC_DECEL_CMD", -16.0) + 16.0) / 0.004885)
      dat[2] |= (decel_raw >> 8) & 0xF
      dat[2] |= int(values.get("ACC_AVAILABLE", 0)) << 4
      dat[2] |= int(values.get("ACC_ENABLED", 0)) << 5
      dat[3] = decel_raw & 0xFF
      dat[4] |= int(values.get("COMMAND_TYPE", 0)) << 4
      dat[6] |= int(values.get("ACC_BRK_PREP", 0)) << 1
    elif name == "WP_ACC_DASH_CMD":
      dat[3] |= int(values.get("OP_LONG_ENABLE", 0))
    elif name == "WP_ACC_TORQUE_CMD":
      torque_raw = round((values.get("ENGINE_TORQUE_REQUEST", -500.0) + 500.0) / 0.25)
      dat[4] |= int(values.get("ENGINE_TORQUE_REQUEST_MAX", 0)) << 7
      dat[4] |= (torque_raw >> 8) & 0x7F
      dat[5] = torque_raw & 0xFF
    else:
      raise ValueError(name)

    dat[6] |= (int(values.get("COUNTER", 0)) & 0xF) << 4
    dat[7] = int(values.get("CHECKSUM", 0))
    return self.ADDRESSES[name], bus, bytes(dat), 0


class RecordingPacker:
  def make_can_msg(self, name, bus, values):
    return name, bus, values.copy()


def load_chryslercan():
  cereal = ModuleType("cereal")
  cereal.car = SimpleNamespace(
    CarState=SimpleNamespace(GearShifter=SimpleNamespace()),
    CarControl=SimpleNamespace(
      HUDControl=SimpleNamespace(VisualAlert=SimpleNamespace()),
    ),
  )
  long_module = ModuleType(
    "openpilot.selfdrive.car.chrysler.jeep_longitudinal",
  )
  long_module.fca_checksum = fca_checksum
  values_module = ModuleType("openpilot.selfdrive.car.chrysler.values")
  values_module.RAM_CARS = set()
  with patch.dict(sys.modules, {
    "cereal": cereal,
    "openpilot.selfdrive.car.chrysler.jeep_longitudinal": long_module,
    "openpilot.selfdrive.car.chrysler.values": values_module,
  }):
    spec = importlib.util.spec_from_file_location(
      "chryslercan_transport_under_test",
      CHRYSLERCAN_PATH,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
  return module


class TestJeepLongitudinalShadow(unittest.TestCase):
  def test_b6w_combines_das3_and_dashboard_das4_faults(self):
    self.assertFalse(jeep_acc_faulted(0, 0))
    self.assertTrue(jeep_acc_faulted(1, 0))
    self.assertTrue(jeep_acc_faulted(0, 1))
    self.assertTrue(jeep_acc_faulted(2, 1))

  def test_white_panda_diagnostic_decodes_applied_brake_cycle(self):
    diagnostic = decode_wp_long_diagnostic(0xB7, 0, 0, 0xA4)
    self.assertTrue(diagnostic.valid)
    self.assertTrue(diagnostic.applied)
    self.assertTrue(diagnostic.host_requested)
    self.assertTrue(diagnostic.brake_requested)
    self.assertFalse(diagnostic.engine_requested)
    self.assertEqual(diagnostic.failure_mask, 0)
    self.assertEqual(diagnostic.failure_reasons, ())
    self.assertEqual(diagnostic.private_counter, 10)
    self.assertEqual(diagnostic.stock_counter, 4)

  def test_white_panda_diagnostic_names_multiple_failures(self):
    failure_mask = (1 << 5) | (1 << 11) | (1 << 15)
    diagnostic = decode_wp_long_diagnostic(
      0xB2,
      failure_mask & 0xFF,
      failure_mask >> 8,
      0x31,
    )
    self.assertTrue(diagnostic.valid)
    self.assertFalse(diagnostic.applied)
    self.assertEqual(
      diagnostic.failure_reasons,
      ("speed_not_fresh", "speed_too_low", "command_envelope"),
    )

  def test_legacy_zero_white_panda_beacon_is_unsupported(self):
    diagnostic = decode_wp_long_diagnostic(0, 0, 0, 0)
    self.assertFalse(diagnostic.valid)
    self.assertEqual(diagnostic.failure_reasons, ("unsupported_beacon",))

  def test_white_panda_command_diagnostic_compares_factory_and_output(self):
    diagnostic = decode_wp_long_command_diagnostic(
      0xC1, 1,
      1, 160.75,
      1, 100.0,
      1, 1, -0.65,
    )
    self.assertTrue(diagnostic.valid)
    self.assertTrue(diagnostic.stock_engine_active)
    self.assertAlmostEqual(diagnostic.stock_engine_torque_nm, 160.75)
    self.assertTrue(diagnostic.output_engine_active)
    self.assertAlmostEqual(diagnostic.output_engine_torque_nm, 100.0)
    self.assertTrue(diagnostic.stock_acc_available)
    self.assertTrue(diagnostic.stock_acc_active)
    self.assertAlmostEqual(diagnostic.stock_accel_mps2, -0.65)

    unsupported = decode_wp_long_command_diagnostic(
      0xC1, 2,
      0, 0.0,
      0, 0.0,
      0, 0, 0.0,
    )
    self.assertFalse(unsupported.valid)

  def test_white_panda_owner_diagnostic_decodes_handoff_state(self):
    diagnostic = decode_wp_long_owner_diagnostic(0xD2, 0x19, 0)
    self.assertTrue(diagnostic.valid)
    self.assertEqual(diagnostic.owner_state, 2)
    self.assertEqual(diagnostic.owner_name, "openpilot")
    self.assertTrue(diagnostic.stock_valid)
    self.assertTrue(diagnostic.stock_available)
    self.assertFalse(diagnostic.stock_active)
    self.assertEqual(diagnostic.stock_fault, 0)
    self.assertFalse(diagnostic.stock_collision)
    self.assertTrue(diagnostic.cancel_injected)

  def test_white_panda_owner_diagnostic_rejects_unknown_signature(self):
    diagnostic = decode_wp_long_owner_diagnostic(0, 0xFF, 3)
    self.assertFalse(diagnostic.valid)
    self.assertEqual(diagnostic.owner_name, "unsupported")
    self.assertFalse(diagnostic.stock_valid)
    self.assertEqual(diagnostic.stock_fault, 0)

  def test_b7o_keeps_transport_diagnostics_but_disables_actuation(self):
    self.assertFalse(JEEP_LONG_ACTUATION_COMPILED)
    self.assertTrue(JEEP_LONG_REJECT_DIAGNOSTICS_COMPILED)
    self.assertTrue(JEEP_LONG_SHADOW_TRANSPORT_COMPILED)
    result = JeepLongitudinalShadow().update(-1.0, eligible=True)
    self.assertTrue(result.transport_enabled)
    self.assertFalse(result.host_enabled)

  def test_b7o_offroad_toggle_cannot_enable_actuation(self):
    self.assertFalse(jeep_long_actuation_enabled(False))
    self.assertFalse(jeep_long_actuation_enabled(True))

  def test_restart_gated_mode_selector_is_mutually_exclusive(self):
    expected = {
      (False, False, False): "factory",
      (False, False, True): "factory",
      (False, True, False): "factory_stop_and_go",
      (False, True, True): "factory_stop_and_go",
      (True, False, False): "experimental_shadow",
      (True, False, True): "experimental",
      (True, True, False): "experimental_shadow",
      (True, True, True): "experimental",
    }
    for inputs, expected_name in expected.items():
      experimental, factory_sng, compiled = inputs
      with self.subTest(inputs=inputs):
        mode = select_jeep_longitudinal_mode(
          experimental, factory_sng, compiled,
        )
        self.assertEqual(mode.name, expected_name)
        self.assertEqual(mode.factory_acc, not mode.openpilot_long)
        self.assertEqual(
          sum((mode.factory_stop_and_go, mode.openpilot_long)),
          1 if expected_name in ("factory_stop_and_go", "experimental") else 0,
        )
        self.assertEqual(
          mode.conflicting_toggles,
          experimental and factory_sng,
        )

  def test_shadow_experimental_request_keeps_factory_vehicle_owner(self):
    mode = select_jeep_longitudinal_mode(
      experimental_long=True,
      custom_stock_long=True,
      actuation_compiled=False,
    )
    self.assertTrue(mode.factory_acc)
    self.assertTrue(mode.experimental_shadow)
    self.assertFalse(mode.factory_stop_and_go)
    self.assertFalse(mode.openpilot_long)

  def test_b7o_omits_vehicle_flags_with_actuation_compiled_out(self):
    base = 0x80
    shadow, diagnostic, actuation = 0x01, 0x02, 0x04
    self.assertEqual(
      jeep_long_mode_safety_param(
        base, False, shadow, diagnostic, actuation,
      ),
      base,
    )
    self.assertEqual(
      jeep_long_mode_safety_param(
        base, True, shadow, diagnostic, actuation,
      ),
      base,
    )

  def test_factory_sng_resume_requires_vision_and_radar_agreement(self):
    self.assertTrue(jeep_factory_sng_lead_moving(
      True, 12.0, 1.0, 0.95, 12.5, 0.9,
    ))
    self.assertFalse(jeep_factory_sng_lead_moving(
      False, 12.0, 1.0, 0.95, 12.5, 0.9,
    ))
    self.assertFalse(jeep_factory_sng_lead_moving(
      True, 12.0, 1.0, 0.69, 12.5, 0.9,
    ))
    self.assertFalse(jeep_factory_sng_lead_moving(
      True, 12.0, 0.49, 0.95, 12.5, 0.9,
    ))
    self.assertFalse(jeep_factory_sng_lead_moving(
      True, 12.0, 1.0, 0.95, 12.5, 0.49,
    ))
    self.assertFalse(jeep_factory_sng_lead_moving(
      True, 12.0, 2.6, 0.95, 12.5, 0.9,
    ))
    self.assertFalse(jeep_factory_sng_lead_moving(
      True, 12.0, 1.0, 0.95, 20.0, 0.9,
    ))
    self.assertFalse(jeep_factory_sng_lead_moving(
      True, math.nan, 1.0, 0.95, 12.5, 0.9,
    ))

  def test_factory_sng_vision_fallback_requires_motion_and_range_gain(self):
    self.assertTrue(jeep_factory_sng_vision_lead_moving(
      True, 7.0, 0.8, 0.98, 6.3,
    ))
    self.assertFalse(jeep_factory_sng_vision_lead_moving(
      True, 6.6, 0.8, 0.98, 6.3,
    ))
    self.assertFalse(jeep_factory_sng_vision_lead_moving(
      True, 7.0, 0.5, 0.98, 6.3,
    ))
    self.assertFalse(jeep_factory_sng_vision_lead_moving(
      True, 7.0, 0.8, 0.89, 6.3,
    ))
    self.assertFalse(jeep_factory_sng_vision_lead_moving(
      True, 26.0, 0.8, 0.98, 20.0,
    ))

  def test_committed_vehicle_path_sends_only_guarded_active_frames(self):
    carcontroller_source = CARCONTROLLER_PATH.read_text(encoding="utf-8")
    guarded_transport_append = (
      "if transport_counter is not None:\n"
      "      if self.jeep_long_envelope.host_enabled:"
    )
    self.assertIn(guarded_transport_append, carcontroller_source)
    self.assertIn(
      "self.jeep_long_envelope.transport_enabled and jeep_long_vehicle_eligible",
      carcontroller_source,
    )
    self.assertEqual(
      carcontroller_source.count(
        "can_sends.extend(self.jeep_long_transport_frames)",
      ),
      1,
    )
    self.assertEqual(
      carcontroller_source.count(
        "can_sends.extend(self.jeep_long_shadow_frames)",
      ),
      1,
    )
    self.assertIn(
      "self.jeep_long_transport_scheduler.next_counter(",
      carcontroller_source,
    )
    self.assertIn(
      "self.jeep_long_shadow.note_transport_sent(",
      carcontroller_source,
    )
    self.assertNotIn("MIN_ACTIVE_SPEED_MPS", carcontroller_source)
    self.assertIn("if not CC.longActive:", carcontroller_source)
    self.assertIn("CC.actuators.accel", carcontroller_source)
    self.assertIn(
      "requested_accel,\n        jeep_long_vehicle_eligible,\n        CS.out.vEgo,",
      carcontroller_source,
    )
    self.assertNotIn(
      "self.jeep_long_plan_result.eligible",
      carcontroller_source,
    )
    self.assertNotIn("JeepRadarLongitudinalAssist", carcontroller_source)
    self.assertNotIn("self.jeep_radar_assist", carcontroller_source)

    interface_source = INTERFACE_PATH.read_text(encoding="utf-8")
    self.assertIn(
      "ret.experimentalLongitudinalAvailable = JEEP_LONG_ACTUATION_COMPILED",
      interface_source,
    )
    self.assertIn(
      "jeep_long_mode = select_jeep_longitudinal_mode(",
      interface_source,
    )
    self.assertIn("jeep_op_long = jeep_long_mode.openpilot_long", interface_source)
    self.assertIn(
      "jeep_factory_sng = jeep_long_mode.factory_stop_and_go",
      interface_source,
    )
    self.assertIn(
      "ret.openpilotLongitudinalControl = jeep_op_long",
      interface_source,
    )
    self.assertIn(
      "ret.pcmCruise = not jeep_op_long",
      interface_source,
    )
    self.assertIn("ret.customStockLongAvailable = True", interface_source)
    self.assertIn("Panda.FLAG_CHRYSLER_JEEP_FACTORY_SNG", interface_source)
    self.assertIn("ChryslerFlagsSP.SP_JEEP_FACTORY_SNG", interface_source)
    self.assertIn("if jeep_op_long:", interface_source)
    self.assertIn(
      "ret.safetyConfigs[0].safetyParam = jeep_long_mode_safety_param(",
      interface_source,
    )
    self.assertIn(
      'if not self.CP.openpilotLongitudinalControl:\n'
      '      return False, "factory_acc_mode"',
      carcontroller_source,
    )
    self.assertIn("ret.longitudinalTuning.deadzoneV = [0.1]", interface_source)
    self.assertIn("ret.longitudinalTuning.kpV = [0.6]", interface_source)
    self.assertIn("ret.longitudinalTuning.kiV = [0.2]", interface_source)
    self.assertIn(
      "ret.longitudinalActuatorDelayLowerBound = 0.15",
      interface_source,
    )
    self.assertNotIn("ret.pcmCruiseSpeed =", interface_source)
    self.assertIn("resume_button=(ButtonType.resumeCruise,)", interface_source)

    controls_source = CONTROLSD_PATH.read_text(encoding="utf-8")
    self.assertEqual(
      controls_source.count("is_uninitialized_resume_button(self.CP, be.type)"),
      2,
    )
    drive_helpers_source = DRIVE_HELPERS_PATH.read_text(encoding="utf-8")
    self.assertIn(
      'CP.carName == "chrysler" and CP.openpilotLongitudinalControl and CP.pcmCruiseSpeed',
      drive_helpers_source,
    )
    self.assertIn("return button_type == ButtonType.resumeCruise", drive_helpers_source)

    planner_source = PLANNER_PATH.read_text(encoding="utf-8")
    self.assertIn(
      "controller_accel_mps2=controller_accel",
      planner_source,
    )
    for forbidden in (
      "CANPacker(",
      "PubMaster(",
      "sendcan.",
      "can_sends.append",
      "can_sends.extend",
    ):
      self.assertNotIn(forbidden, planner_source)

  def test_b6q_replaces_the_factory_resume_bridge(self):
    carcontroller_source = CARCONTROLLER_PATH.read_text(encoding="utf-8")
    chryslercan_source = CHRYSLERCAN_PATH.read_text(encoding="utf-8")

    # The legacy helper is used only by the separately flagged factory
    # stop-and-go mode; openpilot-long retains its own state machine.
    self.assertIn("def update_b6y_standstill_hold(", carcontroller_source)
    self.assertEqual(carcontroller_source.count("self.update_b6y_standstill_hold("), 1)
    self.assertIn("SP_JEEP_FACTORY_SNG", carcontroller_source)
    self.assertIn("B6Y_LEAD_CONFIRM_CYCLES = 3", carcontroller_source)
    self.assertIn("if self.jeep_radar_shadow_updated:", carcontroller_source)
    self.assertIn("jeep_factory_sng_lead_moving(", carcontroller_source)
    self.assertIn("B6Y_RESUME_MAX_ATTEMPTS = 3", carcontroller_source)
    self.assertIn("B6Y_RESUME_PULSE_COUNTERS = 6", carcontroller_source)
    self.assertIn("self.b6y_resume_pulse_remaining > 0 and counter_changed", carcontroller_source)
    self.assertIn("driver_button_pressed", carcontroller_source)
    self.assertIn("B6Y_VISION_LEAD_CONFIRM_CYCLES = 5", carcontroller_source)
    self.assertIn("jeep_factory_sng_vision_lead_moving(", carcontroller_source)
    self.assertIn("self.b6y_approach_armed = True", carcontroller_source)
    self.assertIn("JEEP_LKAS_ENABLE_CONFIRM_FRAMES = 50", carcontroller_source)
    self.assertIn("self.jeep_lkas_enable_request_frame = -1", carcontroller_source)
    self.assertIn("elif self.lkas_control_bit_prev:", carcontroller_source)
    self.assertIn(
      "self.frame - self.jeep_lkas_enable_request_frame\n"
      "            >= JEEP_LKAS_ENABLE_CONFIRM_FRAMES",
      carcontroller_source,
    )
    self.assertIn('"ACC_STOP": envelope.stop_request', chryslercan_source)
    self.assertIn('"ACC_GO": envelope.go_request', chryslercan_source)

  def test_b6y_hold_message_has_no_propulsion_or_go_request(self):
    chryslercan = load_chryslercan()
    stock = {
      "ENGINE_TORQUE_REQUEST": 12.5,
      "ENGINE_TORQUE_REQUEST_MAX": 1,
      "ACC_STANDSTILL": 1,
      "ACC_GO": 1,
      "ACC_DECEL": 4.0,
      "ACC_AVAILABLE": 1,
      "ACC_ACTIVE": 0,
      "GR_MAX_REQ": 8,
      "ACC_DECEL_REQ": 0,
      "ACC_BRK_PREP": 1,
      "COUNTER": 15,
    }
    name, bus, values = chryslercan.create_b6y_standstill_hold(
      RecordingPacker(), 2, stock,
    )
    self.assertEqual((name, bus), ("DAS_3", 0))
    self.assertEqual(values["COUNTER"], 1)
    self.assertEqual(values["ACC_DECEL"], -2.0)
    self.assertEqual(values["ACC_DECEL_REQ"], 1)
    self.assertEqual(values["ACC_AVAILABLE"], 1)
    self.assertEqual(values["ACC_ACTIVE"], 1)
    self.assertEqual(values["GR_MAX_REQ"], 2)
    self.assertEqual(values["ENGINE_TORQUE_REQUEST_MAX"], 0)
    self.assertEqual(values["ACC_GO"], 0)
    self.assertEqual(values["ACC_STANDSTILL"], 0)
    self.assertEqual(values["ACC_BRK_PREP"], 0)
    self.assertEqual(values["ENGINE_TORQUE_REQUEST"], 12.5)

  def test_transport_and_actuation_are_independent_fail_closed_gates(self):
    with patch.object(
      LONG, "JEEP_LONG_SHADOW_TRANSPORT_COMPILED", True,
    ), patch.object(
      LONG, "JEEP_LONG_ACTUATION_COMPILED", False,
    ):
      transport_only = JeepLongitudinalShadow().update(-1.0, eligible=True)
      self.assertTrue(transport_only.transport_enabled)
      self.assertFalse(transport_only.host_enabled)

    with patch.object(
      LONG, "JEEP_LONG_SHADOW_TRANSPORT_COMPILED", False,
    ), patch.object(
      LONG, "JEEP_LONG_ACTUATION_COMPILED", True,
    ):
      invalid_actuation_only = JeepLongitudinalShadow().update(
        -1.0, eligible=True,
      )
      self.assertFalse(invalid_actuation_only.transport_enabled)
      self.assertFalse(invalid_actuation_only.host_enabled)

    with patch.object(
      LONG, "JEEP_LONG_SHADOW_TRANSPORT_COMPILED", True,
    ), patch.object(
      LONG, "JEEP_LONG_ACTUATION_COMPILED", True,
    ):
      hypothetical_both = JeepLongitudinalShadow().update(
        -1.0, eligible=True,
      )
      self.assertTrue(hypothetical_both.transport_enabled)
      self.assertTrue(hypothetical_both.host_enabled)

  def test_ineligible_state_blocks_hypothetical_transport(self):
    with patch.object(
      LONG, "JEEP_LONG_SHADOW_TRANSPORT_COMPILED", True,
    ), patch.object(
      LONG, "JEEP_LONG_ACTUATION_COMPILED", True,
    ):
      result = JeepLongitudinalShadow().update(-1.0, eligible=False)
      self.assertFalse(result.transport_enabled)
      self.assertFalse(result.host_enabled)

  def test_b6o_adds_all_independent_longitudinal_safety_flags(self):
    self.assertEqual(jeep_long_shadow_safety_param(0, 4), 4)
    self.assertEqual(jeep_long_shadow_safety_param(32, 4, 16, 64), 52)
    with patch.object(LONG, "JEEP_LONG_ACTUATION_COMPILED", True):
      self.assertEqual(jeep_long_shadow_safety_param(32, 4, 16, 64), 116)
    with patch.object(LONG, "JEEP_LONG_ACTUATION_COMPILED", False):
      self.assertEqual(jeep_long_shadow_safety_param(32, 4, 16, 64), 52)
    with patch.object(LONG, "JEEP_LONG_SHADOW_TRANSPORT_COMPILED", False):
      self.assertEqual(jeep_long_shadow_safety_param(32, 4, 16, 64), 32)

  def test_requested_accel_is_clipped(self):
    positive = JeepLongitudinalShadow().update(20.0, eligible=True)
    negative = JeepLongitudinalShadow().update(-20.0, eligible=True)
    self.assertEqual(positive.requested_accel, ACCEL_MAX)
    self.assertEqual(negative.requested_accel, ACCEL_MIN)

  def test_jerk_limits(self):
    shadow = JeepLongitudinalShadow()
    self.assertAlmostEqual(shadow.update(1.0, eligible=True).limited_accel, 0.02)
    self.assertAlmostEqual(shadow.update(-3.0, eligible=True).limited_accel, -0.02)

  def test_ineligible_fails_to_zero(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(60):
      shadow.update(1.0, eligible=True, speed_mps=15.0)
    result = shadow.update(1.0, eligible=False)
    self.assertEqual(result.limited_accel, 0.0)
    self.assertFalse(result.brake_active)
    self.assertFalse(result.engine_active)
    self.assertEqual(result.brake_accel_mps2, 0.0)
    self.assertEqual(result.command_mode, "inactive")
    self.assertFalse(result.brake_latched)
    self.assertFalse(shadow.propulsion_latched)

  def test_brake_and_engine_are_mutually_exclusive(self):
    brake_shadow = JeepLongitudinalShadow()
    for _ in range(10):
      brake = brake_shadow.update(-1.0, eligible=True)
    self.assertTrue(brake.brake_active)
    self.assertFalse(brake.engine_active)

    engine_shadow = JeepLongitudinalShadow()
    for _ in range(10):
      engine = engine_shadow.update(1.0, eligible=True, speed_mps=15.0)
    self.assertFalse(engine.brake_active)
    self.assertTrue(engine.engine_active)
    self.assertGreater(engine.engine_torque_nm, 0.0)

  def test_b6q_mild_negative_request_remains_coast(self):
    shadow = JeepLongitudinalShadow()
    first = shadow.update(-0.30, eligible=True, speed_mps=15.0)
    self.assertFalse(first.engine_active)
    for _ in range(100):
      result = shadow.update(-0.30, eligible=True, speed_mps=15.0)
    self.assertAlmostEqual(result.limited_accel, -0.30)
    self.assertFalse(result.brake_latched)
    self.assertFalse(result.brake_active)
    self.assertFalse(result.engine_active)
    self.assertEqual(result.command_mode, "coast")

  def test_b6q_moderate_brake_requires_confirmation(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(LONG.BRAKE_ENTRY_CONFIRM_CYCLES - 1):
      result = shadow.update(-0.50, eligible=True, speed_mps=15.0)
      self.assertFalse(result.brake_latched)
    result = shadow.update(-0.50, eligible=True, speed_mps=15.0)
    self.assertTrue(result.brake_latched)
    self.assertAlmostEqual(result.limited_accel, LONG.BRAKE_ENTER_ACCEL)

  def test_b6q_strong_raw_brake_bypasses_confirmation(self):
    shadow = JeepLongitudinalShadow()
    first = shadow.update(-1.0, eligible=True, speed_mps=15.0)
    self.assertTrue(first.brake_latched)
    self.assertEqual(first.limited_accel, -0.04)
    for _ in range(10):
      result = shadow.update(-1.0, eligible=True, speed_mps=15.0)
      self.assertTrue(result.brake_latched)
      if result.brake_active:
        break
    self.assertTrue(result.brake_active)

  def test_b6q_one_cycle_strong_brake_spike_does_not_stick(self):
    shadow = JeepLongitudinalShadow()
    spike = shadow.update(-1.0, eligible=True, speed_mps=15.0)
    self.assertTrue(spike.brake_latched)
    recovered = shadow.update(0.0, eligible=True, speed_mps=15.0)
    self.assertFalse(recovered.brake_latched)
    self.assertFalse(recovered.brake_active)

  def test_b6q_brake_confirmation_resets_outside_entry_band(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(8):
      shadow.update(-0.50, eligible=True, speed_mps=15.0)
    reset = shadow.update(-0.20, eligible=True, speed_mps=15.0)
    self.assertFalse(reset.brake_latched)
    self.assertEqual(shadow.brake_entry_confirm_cycles, 0)

  def test_b6s_propulsion_hysteresis_suppresses_boundary_chatter(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(80):
      result = shadow.update(0.5, eligible=True, speed_mps=20.0)
    self.assertTrue(result.engine_active)
    self.assertTrue(shadow.propulsion_latched)

    # These requests cover b6r's remaining engine/coast chatter cluster. Once
    # propulsion is active, none crosses the separate -0.32 exit threshold.
    for accel in (-0.03, -0.08, -0.13, -0.19, -0.28, -0.01):
      for _ in range(10):
        result = shadow.update(accel, eligible=True, speed_mps=20.0)
        self.assertFalse(result.brake_active)
        self.assertTrue(shadow.propulsion_latched)
    self.assertTrue(result.engine_active)

    # A real negative correction retires propulsion and it cannot restart on
    # boundary noise. A later positive request explicitly re-arms it.
    for _ in range(80):
      result = shadow.update(-0.33, eligible=True, speed_mps=20.0)
    self.assertFalse(shadow.propulsion_latched)
    self.assertFalse(result.engine_active)
    for _ in range(20):
      result = shadow.update(-0.02, eligible=True, speed_mps=20.0)
    self.assertFalse(shadow.propulsion_latched)
    self.assertFalse(result.engine_active)
    for _ in range(20):
      result = shadow.update(0.03, eligible=True, speed_mps=20.0)
    self.assertTrue(shadow.propulsion_latched)
    self.assertTrue(result.engine_active)

  def test_low_speed_stop_holds_without_propulsion(self):
    shadow = JeepLongitudinalShadow()
    for speed in (3.0, 2.0, 1.0, 0.5, 0.1, 0.0):
      for _ in range(25):
        result = shadow.update(-1.0, eligible=True, speed_mps=speed)
        self.assertFalse(result.engine_active)
        self.assertFalse(result.go_request)

    self.assertEqual(result.low_speed_state, "hold")
    self.assertEqual(result.command_mode, "hold")
    self.assertTrue(result.stop_request)
    self.assertTrue(result.brake_active)
    self.assertFalse(result.engine_active)
    self.assertAlmostEqual(result.brake_accel_mps2, -2.0)

  def test_true_standstill_uses_full_hold_before_owner_handoff(self):
    shadow = JeepLongitudinalShadow()
    result = shadow.update(1.0, eligible=True, speed_mps=0.0)
    self.assertEqual(result.low_speed_state, "hold")
    self.assertTrue(result.stop_request)
    self.assertTrue(result.brake_active)
    self.assertFalse(result.engine_active)
    self.assertEqual(result.brake_accel_mps2, -2.0)

  def test_low_speed_launch_releases_brake_before_five_cycle_go_pulse(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(100):
      held = shadow.update(-1.0, eligible=True, speed_mps=0.0)
    self.assertTrue(held.stop_request)

    results = []
    for _ in range(250):
      result = shadow.update(1.0, eligible=True, speed_mps=0.0)
      results.append(result)
      shadow.note_transport_sent(result)
      if result.low_speed_state == "creep":
        break

    go_results = [result for result in results if result.go_request]
    self.assertEqual(len(go_results), 5)
    self.assertTrue(all(not result.brake_active for result in go_results))
    self.assertTrue(all(not result.engine_active for result in go_results))
    self.assertTrue(all(not result.stop_request for result in go_results))
    first_go = next(i for i, result in enumerate(results) if result.go_request)
    pre_go = results[:first_go]
    self.assertTrue(any(result.brake_active for result in pre_go))
    self.assertTrue(any(
      not result.brake_active and result.low_speed_state == "release"
      for result in pre_go
    ))
    self.assertTrue(all(not result.engine_active for result in pre_go))

  def test_low_speed_launch_torque_requires_completed_go_and_is_bounded(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(100):
      shadow.update(-1.0, eligible=True, speed_mps=0.0)
    for _ in range(250):
      result = shadow.update(1.0, eligible=True, speed_mps=0.0)
      shadow.note_transport_sent(result)
      self.assertFalse(result.engine_active)
      if result.low_speed_state == "creep":
        break

    # The first cycle after the neutral GO pulse can use the separately capped
    # factory-like launch envelope, even before wheel speed reaches 0.78 m/s.
    for _ in range(10):
      result = shadow.update(
        1.0, eligible=True, speed_mps=0.0, pitch_rad=math.radians(3.0),
      )
      if result.engine_active:
        break
    self.assertEqual(result.low_speed_state, "creep")
    self.assertTrue(result.engine_active)
    self.assertFalse(result.brake_active)
    self.assertFalse(result.stop_request)
    self.assertFalse(result.go_request)
    self.assertLessEqual(
      result.engine_torque_nm, LONG.LOW_SPEED_LAUNCH_TORQUE_MAX_NM,
    )

    for _ in range(10):
      result = shadow.update(1.0, eligible=True, speed_mps=0.5)
      self.assertTrue(result.engine_active)
      self.assertLessEqual(
        result.engine_torque_nm, LONG.LOW_SPEED_LAUNCH_TORQUE_MAX_NM,
      )
    result = shadow.update(1.0, eligible=True, speed_mps=0.8)
    self.assertEqual(result.low_speed_state, "drive")
    self.assertTrue(result.engine_active)

  def test_failed_creep_reapplies_hold_and_requires_request_reset(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(100):
      shadow.update(-1.0, eligible=True, speed_mps=0.0)
    saw_launch_torque = False
    for _ in range(400):
      result = shadow.update(1.0, eligible=True, speed_mps=0.0)
      shadow.note_transport_sent(result)
      saw_launch_torque |= result.engine_active
      if result.low_speed_state == "blocked" and result.brake_active:
        break
    self.assertTrue(saw_launch_torque)
    self.assertEqual(result.low_speed_state, "blocked")
    self.assertTrue(result.stop_request)
    self.assertTrue(result.brake_active)
    self.assertFalse(result.go_request)
    self.assertFalse(result.engine_active)

    # A continuously positive planner request cannot produce repeated GO
    # pulses. The latch clears only after the controller withdraws it.
    for _ in range(100):
      result = shadow.update(1.0, eligible=True, speed_mps=0.0)
      self.assertFalse(result.go_request)
      self.assertEqual(result.low_speed_state, "blocked")
    for _ in range(60):
      result = shadow.update(-0.5, eligible=True, speed_mps=0.0)
    self.assertEqual(result.low_speed_state, "hold")

  def test_transport_commits_neutral_release_before_any_go_cycle(self):
    shadow = JeepLongitudinalShadow()
    scheduler = JeepLongitudinalTransportScheduler()
    sent = []
    now_nanos = 1_000_000_000

    for _ in range(100):
      result = shadow.update(-1.0, eligible=True, speed_mps=0.0)
      counter = scheduler.next_counter(now_nanos, result.transport_enabled)
      if counter is not None:
        sent.append(result)
        shadow.note_transport_sent(result)
      now_nanos += 20_000_000

    for _ in range(300):
      result = shadow.update(1.0, eligible=True, speed_mps=0.0)
      counter = scheduler.next_counter(now_nanos, result.transport_enabled)
      if counter is not None:
        sent.append(result)
        shadow.note_transport_sent(result)
      now_nanos += 20_000_000
      if any(envelope.go_request for envelope in sent):
        break

    first_go = next(
      index for index, envelope in enumerate(sent) if envelope.go_request
    )
    released = sent[first_go - 1]
    self.assertEqual(released.low_speed_state, "release")
    self.assertFalse(released.brake_active)
    self.assertFalse(released.engine_active)
    self.assertFalse(released.stop_request)
    self.assertFalse(released.go_request)

  def test_low_speed_state_fails_off_on_authority_loss(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(100):
      shadow.update(-1.0, eligible=True, speed_mps=0.0)
    result = shadow.update(-1.0, eligible=False, speed_mps=0.0)
    self.assertEqual(result.low_speed_state, "drive")
    self.assertFalse(result.transport_enabled)
    self.assertFalse(result.brake_active)
    self.assertFalse(result.engine_active)
    self.assertFalse(result.stop_request)
    self.assertFalse(result.go_request)

  def test_engine_torque_is_zero_inside_deadband_at_zero_speed(self):
    result = JeepLongitudinalShadow().update(0.04, eligible=True)
    self.assertFalse(result.engine_active)
    self.assertEqual(result.engine_torque_nm, 0.0)

  def test_b6m_capture_brake_calibration_is_applied(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(60):
      result = shadow.update(-1.0, eligible=True, speed_mps=15.0)
    self.assertTrue(result.brake_active)
    self.assertFalse(result.engine_active)
    self.assertAlmostEqual(result.brake_accel_mps2, -1.0188, places=4)

  def test_b6z_restores_fault_free_flat_road_propulsion_gain(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(60):
      result = shadow.update(0.5, eligible=True, speed_mps=15.0)
    self.assertEqual(result.limited_accel, 0.5)
    self.assertTrue(result.engine_active)
    self.assertAlmostEqual(result.engine_torque_nm, 157.50, places=2)
    self.assertEqual(result.grade_torque_nm, 0.0)

  def test_grade_feed_forward_is_filtered_and_bounded(self):
    self.assertEqual(LONG.ENGINE_TORQUE_GRADE_FILTER_TAU_S, 1.5)
    shadow = JeepLongitudinalShadow()
    first = shadow.update(
      0.5, eligible=True, speed_mps=20.0, pitch_rad=math.radians(4.0),
    )
    self.assertGreater(first.grade_torque_nm, 0.0)
    self.assertLess(first.grade_torque_nm, LONG.ENGINE_TORQUE_GRADE_MAX_NM)
    for _ in range(160):
      result = shadow.update(
        0.5, eligible=True, speed_mps=20.0, pitch_rad=math.radians(8.0),
      )
    self.assertEqual(result.grade_torque_nm, LONG.ENGINE_TORQUE_GRADE_MAX_NM)
    self.assertLessEqual(
      abs(result.filtered_pitch_rad),
      LONG.ENGINE_TORQUE_GRADE_PITCH_LIMIT_RAD,
    )

  def test_b6z_retains_the_fault_free_b6w_running_ceiling(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(160):
      result = shadow.update(
        ACCEL_MAX,
        eligible=True,
        speed_mps=40.0,
        pitch_rad=math.radians(4.0),
      )
    self.assertEqual(result.limited_accel, ACCEL_MAX)
    self.assertTrue(result.engine_active)
    self.assertEqual(result.engine_torque_nm, 440.0)

  def test_b6z_running_torque_ceiling_is_speed_shaped(self):
    self.assertEqual(engine_torque_max_for_speed(0.0), 250.0)
    self.assertEqual(engine_torque_max_for_speed(1.0), 270.0)
    self.assertEqual(engine_torque_max_for_speed(7.5), 400.0)
    self.assertEqual(engine_torque_max_for_speed(9.5), 440.0)
    self.assertEqual(engine_torque_max_for_speed(16.22), 440.0)

  def test_b6z_fault_window_torque_rise_is_bounded(self):
    shadow = JeepLongitudinalShadow()
    previous_torque = 0.0
    for _ in range(200):
      result = shadow.update(
        ACCEL_MAX,
        eligible=True,
        speed_mps=16.22,
        pitch_rad=math.radians(4.0),
      )
      self.assertLessEqual(
        result.engine_torque_nm - previous_torque,
        LONG.ENGINE_TORQUE_RATE_UP_NM_PER_S * LONG.COMMAND_DT + 1e-9,
      )
      self.assertLessEqual(result.engine_torque_nm, 440.0)
      previous_torque = result.engine_torque_nm
    self.assertEqual(result.engine_torque_nm, 440.0)

  def test_low_speed_torque_is_bounded_after_confirmed_creep(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(120):
      result = shadow.update(
        ACCEL_MAX,
        eligible=True,
        speed_mps=1.0,
        pitch_rad=math.radians(4.0),
      )
    self.assertTrue(result.engine_active)
    self.assertLessEqual(result.engine_torque_nm, 270.0)
    self.assertEqual(result.grade_torque_nm, 0.0)

  def test_invalid_calibration_input_fails_off(self):
    result = JeepLongitudinalShadow().update(
      float("nan"), eligible=True, speed_mps=15.0,
    )
    self.assertFalse(result.eligible)
    self.assertFalse(result.transport_enabled)
    self.assertFalse(result.host_enabled)
    self.assertFalse(result.brake_active)
    self.assertFalse(result.engine_active)

  def test_recorded_b6n_boundary_cluster_stays_out_of_braking(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(50):
      result = shadow.update(0.03, eligible=True, speed_mps=23.6)
    self.assertTrue(result.engine_active)

    # Segment 6 around 53-56 seconds repeatedly crossed b6n's old -0.05
    # boundary and alternated roughly 224 Nm with braking. Five controller
    # cycles approximate each retained 10 Hz qlog sample.
    recorded_accels = (
      -0.0392, -0.1008, -0.0562, -0.0061, -0.0210,
      -0.0504, -0.0182, -0.0448, -0.1075, -0.1196, -0.0487,
    )
    modes = []
    for accel in recorded_accels:
      for _ in range(5):
        result = shadow.update(accel, eligible=True, speed_mps=23.6)
        modes.append(result.command_mode)
        self.assertFalse(result.brake_active)
        self.assertGreaterEqual(result.engine_torque_nm, 0.0)
    self.assertNotIn("brake", modes)
    self.assertNotIn("coast", modes)

  def test_propulsion_and_braking_are_separated_by_coast_and_slew(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(80):
      result = shadow.update(0.5, eligible=True, speed_mps=23.6)
    self.assertTrue(result.engine_active)

    previous = result
    saw_coast = False
    saw_brake = False
    brake_latched_cycle = None
    brake_active_cycle = None
    for cycle in range(180):
      result = shadow.update(-1.0, eligible=True, speed_mps=23.6)
      self.assertFalse(result.engine_active and result.brake_active)
      self.assertLessEqual(
        abs(result.engine_torque_nm - previous.engine_torque_nm),
        LONG.BRAKE_TRANSITION_TORQUE_RATE_DOWN_NM_PER_S * LONG.COMMAND_DT
        + LONG.ENGINE_TORQUE_ZERO_EPSILON_NM + 0.0001,
      )
      self.assertLessEqual(
        abs(result.brake_accel_mps2 - previous.brake_accel_mps2), 0.0401,
      )
      if result.command_mode == "coast":
        saw_coast = True
      if result.brake_latched and brake_latched_cycle is None:
        brake_latched_cycle = cycle
      if result.brake_active:
        saw_brake = True
        if brake_active_cycle is None:
          brake_active_cycle = cycle
        self.assertEqual(result.engine_torque_nm, 0.0)
      previous = result
    self.assertTrue(saw_coast)
    self.assertTrue(saw_brake)
    self.assertIsNotNone(brake_latched_cycle)
    self.assertIsNotNone(brake_active_cycle)
    self.assertLessEqual(brake_active_cycle - brake_latched_cycle, 20)

    saw_coast = False
    saw_engine = False
    for _ in range(220):
      result = shadow.update(0.5, eligible=True, speed_mps=23.6)
      self.assertFalse(result.engine_active and result.brake_active)
      self.assertLessEqual(
        abs(result.engine_torque_nm - previous.engine_torque_nm), 12.0001,
      )
      self.assertLessEqual(
        abs(result.brake_accel_mps2 - previous.brake_accel_mps2), 0.0401,
      )
      if result.command_mode == "coast":
        saw_coast = True
      if result.engine_active:
        saw_engine = True
        self.assertEqual(result.brake_accel_mps2, 0.0)
      previous = result
    self.assertTrue(saw_coast)
    self.assertTrue(saw_engine)

  def test_brake_hysteresis_does_not_release_on_small_rebound(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(100):
      result = shadow.update(-1.0, eligible=True, speed_mps=15.0)
    self.assertTrue(result.brake_latched)
    self.assertTrue(result.brake_active)

    while result.limited_accel < -0.15:
      result = shadow.update(-0.15, eligible=True, speed_mps=15.0)
    for _ in range(10):
      result = shadow.update(-0.15, eligible=True, speed_mps=15.0)
      self.assertTrue(result.brake_latched)

    while result.limited_accel < -0.07:
      result = shadow.update(0.0, eligible=True, speed_mps=15.0)
    self.assertFalse(result.brake_latched)

  def test_fca_checksum_ignores_only_final_byte(self):
    payload = bytearray.fromhex("1020304050607000")
    checksum = fca_checksum(payload)
    payload[-1] = checksum
    self.assertEqual(fca_checksum(payload), checksum)
    payload[1] ^= 1
    self.assertNotEqual(fca_checksum(payload), checksum)

  def test_transmitted_probe_bytes_are_strictly_neutral(self):
    chryslercan = load_chryslercan()
    brake, dash, torque = chryslercan.create_wp_long_transport_messages(
      PrivateMessagePacker(), 13,
    )

    self.assertEqual((brake[0], dash[0], torque[0]), (0x1F6, 0x1F7, 0x272))
    self.assertEqual(((brake[2][2] & 0xF) << 8) | brake[2][3], 4094)
    self.assertEqual((brake[2][4] >> 4) & 0x7, 0)
    self.assertEqual((brake[2][6] >> 1) & 0x1, 0)
    self.assertEqual(dash[2][3] & 0x1, 0)
    self.assertEqual(torque[2][4] >> 7, 0)
    self.assertEqual(((torque[2][4] & 0x7F) << 8) | torque[2][5], 2000)
    self.assertEqual(
      tuple(msg[2][6] >> 4 for msg in (brake, dash, torque)),
      (13, 13, 13),
    )
    self.assertTrue(
      all(msg[2][7] == fca_checksum(msg[2])
          for msg in (brake, dash, torque)),
    )

  def test_state_machine_stop_release_go_bytes_match_guard_protocol(self):
    chryslercan = load_chryslercan()
    shadow = JeepLongitudinalShadow()
    for _ in range(100):
      envelope = shadow.update(-1.0, eligible=True, speed_mps=0.0)

    stop, _, stop_torque = chryslercan.create_wp_long_shadow_messages(
      PrivateMessagePacker(), envelope, 4,
    )
    self.assertEqual((stop[2][0] >> 5) & 0x3, 0x1)
    self.assertEqual((stop[2][4] >> 4) & 0x7, 1)
    self.assertEqual(stop_torque[2][4] >> 7, 0)

    for _ in range(250):
      envelope = shadow.update(1.0, eligible=True, speed_mps=0.0)
      shadow.note_transport_sent(envelope)
      if envelope.go_request:
        break
    self.assertTrue(envelope.go_request)
    go, _, go_torque = chryslercan.create_wp_long_shadow_messages(
      PrivateMessagePacker(), envelope, 6,
    )
    self.assertEqual((go[2][0] >> 5) & 0x3, 0x2)
    self.assertEqual((go[2][4] >> 4) & 0x7, 0)
    self.assertEqual(go_torque[2][4] >> 7, 0)
    self.assertTrue(all(
      msg[2][7] == fca_checksum(msg[2]) for msg in (stop, stop_torque, go, go_torque)
    ))

  def test_transport_scheduler_enforces_margin_and_counts_only_sends(self):
    self.assertEqual(TRANSPORT_MIN_SEND_INTERVAL_NS, 25_000_000)
    scheduler = JeepLongitudinalTransportScheduler()
    self.assertEqual(scheduler.next_counter(1_000_000_000, True), 0)
    self.assertIsNone(
      scheduler.next_counter(
        1_000_000_000 + TRANSPORT_MIN_SEND_INTERVAL_NS - 1,
        True,
      )
    )
    self.assertEqual(
      scheduler.next_counter(
        1_000_000_000 + TRANSPORT_MIN_SEND_INTERVAL_NS,
        True,
      ),
      1,
    )

  def test_transport_scheduler_does_not_advance_while_disabled(self):
    scheduler = JeepLongitudinalTransportScheduler()
    self.assertIsNone(scheduler.next_counter(1_000_000_000, False))
    self.assertEqual(scheduler.next_counter(1_100_000_000, True), 0)
    self.assertIsNone(scheduler.next_counter(1_110_000_000, False))
    self.assertEqual(scheduler.next_counter(1_200_000_000, True), 1)

  def test_route45_two_pedal_overrides_do_not_latch_host_transport_off(self):
    """The host must re-offer a valid cycle after each explicit re-enable."""
    shadow = JeepLongitudinalShadow()
    scheduler = JeepLongitudinalTransportScheduler()
    now = 1_000_000_000

    for expected_counter in (0, 1):
      active = shadow.update(-0.5, eligible=True, speed_mps=15.0)
      self.assertTrue(active.transport_enabled)
      self.assertFalse(active.host_enabled)
      self.assertEqual(
        scheduler.next_counter(now, active.transport_enabled),
        expected_counter,
      )

      # Gas/brake authority loss withdraws every output and does not consume a
      # counter. This mirrors both driver overrides in route 45.
      overridden = shadow.update(-0.5, eligible=False, speed_mps=15.0)
      self.assertFalse(overridden.transport_enabled)
      self.assertFalse(overridden.host_enabled)
      self.assertFalse(overridden.brake_active)
      self.assertFalse(overridden.engine_active)
      self.assertIsNone(
        scheduler.next_counter(now + 10_000_000, False),
      )
      now += 100_000_000

  def test_transport_scheduler_wraps_counter(self):
    scheduler = JeepLongitudinalTransportScheduler()
    start = 1_000_000_000
    counters = [
      scheduler.next_counter(
        start + index * TRANSPORT_MIN_SEND_INTERVAL_NS,
        True,
      )
      for index in range(18)
    ]
    self.assertEqual(counters, list(range(16)) + [0, 1])

  def test_transport_scheduler_absorbs_recorded_jitter_patterns(self):
    scheduler = JeepLongitudinalTransportScheduler()
    timestamps_ms = (0, 20, 40, 54, 74, 94, 134, 154)
    sent = [
      (timestamp, counter)
      for timestamp in timestamps_ms
      if (
        counter := scheduler.next_counter(
          1_000_000_000 + timestamp * 1_000_000,
          True,
        )
      ) is not None
    ]
    self.assertEqual(
      sent,
      [(0, 0), (40, 1), (74, 2), (134, 3)],
    )
    self.assertTrue(
      all(
        current[0] - previous[0] >= 25
        for previous, current in zip(sent, sent[1:])
      )
    )

  def test_transport_scheduler_uses_100hz_offers_for_nominal_33hz(self):
    scheduler = JeepLongitudinalTransportScheduler()
    timestamps_ms = range(0, 101, 10)
    sent = [
      (timestamp, counter)
      for timestamp in timestamps_ms
      if (
        counter := scheduler.next_counter(
          1_000_000_000 + timestamp * 1_000_000,
          True,
        )
      ) is not None
    ]
    self.assertEqual(sent, [(0, 0), (30, 1), (60, 2), (90, 3)])
    self.assertTrue(all(
      current[0] - previous[0] >= 25
      for previous, current in zip(sent, sent[1:])
    ))


if __name__ == "__main__":
  unittest.main()

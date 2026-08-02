import importlib.util
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
fca_checksum = LONG.fca_checksum
jeep_long_shadow_safety_param = LONG.jeep_long_shadow_safety_param


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

  def test_b6o_enables_independently_guarded_transport_and_actuation(self):
    self.assertTrue(JEEP_LONG_ACTUATION_COMPILED)
    self.assertTrue(JEEP_LONG_REJECT_DIAGNOSTICS_COMPILED)
    self.assertTrue(JEEP_LONG_SHADOW_TRANSPORT_COMPILED)
    result = JeepLongitudinalShadow().update(-1.0, eligible=True)
    self.assertTrue(result.transport_enabled)
    self.assertTrue(result.host_enabled)

  def test_committed_vehicle_path_sends_only_guarded_active_frames(self):
    carcontroller_source = CARCONTROLLER_PATH.read_text(encoding="utf-8")
    guarded_transport_append = (
      "if transport_counter is not None:\n"
      "        if self.jeep_long_envelope.host_enabled:"
    )
    self.assertIn(guarded_transport_append, carcontroller_source)
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
      "if CS.out.standstill or CS.out.vEgo < MIN_ACTIVE_SPEED_MPS:",
      carcontroller_source,
    )
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

    interface_source = INTERFACE_PATH.read_text(encoding="utf-8")
    self.assertIn(
      "ret.experimentalLongitudinalAvailable = JEEP_LONG_ACTUATION_COMPILED",
      interface_source,
    )
    self.assertIn(
      "ret.openpilotLongitudinalControl = JEEP_LONG_ACTUATION_COMPILED",
      interface_source,
    )

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

  def test_b6y_hold_is_direct_brake_only_and_gated(self):
    carcontroller_source = CARCONTROLLER_PATH.read_text(encoding="utf-8")
    carstate_source = CARSTATE_PATH.read_text(encoding="utf-8")
    chryslercan_source = CHRYSLERCAN_PATH.read_text(encoding="utf-8")
    controlsd_source = CONTROLSD_PATH.read_text(encoding="utf-8")

    self.assertIn("def update_b6y_standstill_hold(", carcontroller_source)
    self.assertIn("CC.enabled and CC.longActive", carcontroller_source)
    self.assertIn("CS.cruise_active_actual and CS.acc_decelerating", carcontroller_source)
    self.assertIn("not CS.forward_gear or not CS.out.standstill", carcontroller_source)
    self.assertIn("CS.out.accFaulted or CS.out.stockAeb", carcontroller_source)
    self.assertIn("chryslercan.create_b6y_standstill_hold(", carcontroller_source)
    self.assertIn("if not CS.b6y_hold_active or CS.cruise_active_actual:", carcontroller_source)
    self.assertIn("if CS.out.standstill or CS.out.vEgo < MIN_ACTIVE_SPEED_MPS:", carcontroller_source)
    self.assertIn("def create_b6y_standstill_hold(", chryslercan_source)
    self.assertIn('"ENGINE_TORQUE_REQUEST_MAX": 0', chryslercan_source)
    self.assertIn('"ACC_DECEL": -2.0', chryslercan_source)
    self.assertIn('"ACC_GO": 0', chryslercan_source)

    self.assertIn("self.b6y_hold_active = False", carstate_source)
    self.assertIn("self.cruise_active_actual = ret.cruiseState.enabled", carstate_source)
    self.assertIn("self.das_3 = dict(cp_cruise.vl[\"DAS_3\"])", carstate_source)
    # The custom bridge is not exposed as the generic brake-hold state:
    # that generic event intentionally disengages openpilot longitudinal.
    self.assertNotIn("ret.brakeHoldActive", carstate_source)
    self.assertNotIn("CS.brakeHoldActive", controlsd_source)
    self.assertIn(
      "cruise_mismatch = CS.cruiseState.enabled and not self.enabled",
      controlsd_source,
    )
    self.assertIn("elif CC.cruiseControl.resume:", carcontroller_source)
    self.assertIn(
      "self.CP, resume=True))",
      carcontroller_source,
    )

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
    with (
      patch.object(LONG, "JEEP_LONG_SHADOW_TRANSPORT_COMPILED", True),
      patch.object(LONG, "JEEP_LONG_ACTUATION_COMPILED", False),
    ):
      transport_only = JeepLongitudinalShadow().update(-1.0, eligible=True)
      self.assertTrue(transport_only.transport_enabled)
      self.assertFalse(transport_only.host_enabled)

    with (
      patch.object(LONG, "JEEP_LONG_SHADOW_TRANSPORT_COMPILED", False),
      patch.object(LONG, "JEEP_LONG_ACTUATION_COMPILED", True),
    ):
      invalid_actuation_only = JeepLongitudinalShadow().update(
        -1.0, eligible=True,
      )
      self.assertFalse(invalid_actuation_only.transport_enabled)
      self.assertFalse(invalid_actuation_only.host_enabled)

    with (
      patch.object(LONG, "JEEP_LONG_SHADOW_TRANSPORT_COMPILED", True),
      patch.object(LONG, "JEEP_LONG_ACTUATION_COMPILED", True),
    ):
      hypothetical_both = JeepLongitudinalShadow().update(
        -1.0, eligible=True,
      )
      self.assertTrue(hypothetical_both.transport_enabled)
      self.assertTrue(hypothetical_both.host_enabled)

  def test_ineligible_state_blocks_hypothetical_transport(self):
    with (
      patch.object(LONG, "JEEP_LONG_SHADOW_TRANSPORT_COMPILED", True),
      patch.object(LONG, "JEEP_LONG_ACTUATION_COMPILED", True),
    ):
      result = JeepLongitudinalShadow().update(-1.0, eligible=False)
      self.assertFalse(result.transport_enabled)
      self.assertFalse(result.host_enabled)

  def test_b6o_adds_all_independent_longitudinal_safety_flags(self):
    self.assertEqual(jeep_long_shadow_safety_param(0, 4), 4)
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

  def test_b6m_capture_speed_aware_propulsion_fit_is_applied(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(60):
      result = shadow.update(0.5, eligible=True, speed_mps=15.0)
    self.assertEqual(result.limited_accel, 0.5)
    self.assertTrue(result.engine_active)
    self.assertAlmostEqual(result.engine_torque_nm, 210.05, places=2)

  def test_host_torque_ceiling_covers_matched_oem_range_only(self):
    shadow = JeepLongitudinalShadow()
    for _ in range(80):
      result = shadow.update(ACCEL_MAX, eligible=True, speed_mps=40.0)
    self.assertEqual(result.limited_accel, ACCEL_MAX)
    self.assertTrue(result.engine_active)
    self.assertEqual(result.engine_torque_nm, 425.0)

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
      result = shadow.update(0.0, eligible=True, speed_mps=23.6)
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
        abs(result.engine_torque_nm - previous.engine_torque_nm), 12.0001,
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


if __name__ == "__main__":
  unittest.main()

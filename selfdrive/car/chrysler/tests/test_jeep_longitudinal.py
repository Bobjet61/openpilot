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
INTERFACE_PATH = Path(__file__).resolve().parents[1] / "interface.py"
CHRYSLERCAN_PATH = Path(__file__).resolve().parents[1] / "chryslercan.py"
LONG_SPEC = importlib.util.spec_from_file_location("jeep_longitudinal_under_test", LONG_PATH)
assert LONG_SPEC is not None and LONG_SPEC.loader is not None
LONG = importlib.util.module_from_spec(LONG_SPEC)
sys.modules[LONG_SPEC.name] = LONG
LONG_SPEC.loader.exec_module(LONG)

ACCEL_MAX = LONG.ACCEL_MAX
ACCEL_MIN = LONG.ACCEL_MIN
JEEP_LONG_ACTUATION_COMPILED = LONG.JEEP_LONG_ACTUATION_COMPILED
JEEP_LONG_SHADOW_TRANSPORT_COMPILED = LONG.JEEP_LONG_SHADOW_TRANSPORT_COMPILED
JeepLongitudinalShadow = LONG.JeepLongitudinalShadow
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
  def test_transport_is_on_but_actuation_is_compile_time_off(self):
    self.assertFalse(JEEP_LONG_ACTUATION_COMPILED)
    self.assertTrue(JEEP_LONG_SHADOW_TRANSPORT_COMPILED)
    result = JeepLongitudinalShadow().update(-1.0, eligible=True)
    self.assertTrue(result.transport_enabled)
    self.assertFalse(result.host_enabled)

  def test_committed_vehicle_path_keeps_shadow_frames_and_longitudinal_off(self):
    carcontroller_source = CARCONTROLLER_PATH.read_text(encoding="utf-8")
    guarded_transport_append = (
      "if self.jeep_long_envelope.transport_enabled:\n"
      "        self.jeep_long_transport_frames = ("
    )
    self.assertIn(guarded_transport_append, carcontroller_source)
    self.assertEqual(
      carcontroller_source.count(
        "can_sends.extend(self.jeep_long_transport_frames)",
      ),
      1,
    )
    self.assertNotIn(
      "can_sends.extend(self.jeep_long_shadow_frames)",
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

  def test_transport_and_actuation_are_independent_fail_closed_gates(self):
    with patch.object(LONG, "JEEP_LONG_SHADOW_TRANSPORT_COMPILED", True):
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

  def test_committed_transport_adds_only_shadow_param(self):
    self.assertEqual(jeep_long_shadow_safety_param(0, 4), 4)
    self.assertEqual(jeep_long_shadow_safety_param(2, 4), 6)

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
    shadow.update(1.0, eligible=True)
    result = shadow.update(1.0, eligible=False)
    self.assertEqual(result.limited_accel, 0.0)
    self.assertFalse(result.brake_active)
    self.assertFalse(result.engine_active)

  def test_brake_and_engine_are_mutually_exclusive(self):
    brake_shadow = JeepLongitudinalShadow()
    for _ in range(10):
      brake = brake_shadow.update(-1.0, eligible=True)
    self.assertTrue(brake.brake_active)
    self.assertFalse(brake.engine_active)

    engine_shadow = JeepLongitudinalShadow()
    for _ in range(10):
      engine = engine_shadow.update(1.0, eligible=True)
    self.assertFalse(engine.brake_active)
    self.assertTrue(engine.engine_active)
    self.assertGreater(engine.engine_torque_nm, 0.0)

  def test_engine_torque_is_zero_inside_deadband(self):
    result = JeepLongitudinalShadow().update(0.04, eligible=True)
    self.assertFalse(result.engine_active)
    self.assertEqual(result.engine_torque_nm, 0.0)

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


if __name__ == "__main__":
  unittest.main()

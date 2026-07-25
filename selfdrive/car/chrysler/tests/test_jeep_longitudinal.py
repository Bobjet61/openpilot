import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

LONG_PATH = Path(__file__).resolve().parents[1] / "jeep_longitudinal.py"
CARCONTROLLER_PATH = Path(__file__).resolve().parents[1] / "carcontroller.py"
INTERFACE_PATH = Path(__file__).resolve().parents[1] / "interface.py"
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


class TestJeepLongitudinalShadow(unittest.TestCase):
  def test_actuation_is_compile_time_off(self):
    self.assertFalse(JEEP_LONG_ACTUATION_COMPILED)
    self.assertFalse(JEEP_LONG_SHADOW_TRANSPORT_COMPILED)
    result = JeepLongitudinalShadow().update(-1.0, eligible=True)
    self.assertFalse(result.transport_enabled)
    self.assertFalse(result.host_enabled)

  def test_committed_vehicle_path_keeps_shadow_frames_and_longitudinal_off(self):
    carcontroller_source = CARCONTROLLER_PATH.read_text(encoding="utf-8")
    guarded_append = (
      "if self.jeep_long_envelope.transport_enabled:\n"
      "        can_sends.extend(self.jeep_long_shadow_frames)"
    )
    self.assertIn(guarded_append, carcontroller_source)
    self.assertEqual(
      carcontroller_source.count(
        "can_sends.extend(self.jeep_long_shadow_frames)",
      ),
      1,
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

  def test_committed_transport_gate_leaves_panda_param_off(self):
    self.assertEqual(jeep_long_shadow_safety_param(0, 4), 0)
    self.assertEqual(jeep_long_shadow_safety_param(2, 4), 2)

  def test_hypothetical_transport_adds_only_shadow_param(self):
    with patch.object(LONG, "JEEP_LONG_SHADOW_TRANSPORT_COMPILED", True):
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


if __name__ == "__main__":
  unittest.main()

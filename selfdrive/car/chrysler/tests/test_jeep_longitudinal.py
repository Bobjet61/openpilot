import importlib.util
from pathlib import Path
import sys
import unittest

LONG_PATH = Path(__file__).resolve().parents[1] / "jeep_longitudinal.py"
LONG_SPEC = importlib.util.spec_from_file_location("jeep_longitudinal_under_test", LONG_PATH)
assert LONG_SPEC is not None and LONG_SPEC.loader is not None
LONG = importlib.util.module_from_spec(LONG_SPEC)
sys.modules[LONG_SPEC.name] = LONG
LONG_SPEC.loader.exec_module(LONG)

ACCEL_MAX = LONG.ACCEL_MAX
ACCEL_MIN = LONG.ACCEL_MIN
JEEP_LONG_ACTUATION_COMPILED = LONG.JEEP_LONG_ACTUATION_COMPILED
JeepLongitudinalShadow = LONG.JeepLongitudinalShadow


class TestJeepLongitudinalShadow(unittest.TestCase):
  def test_actuation_is_compile_time_off(self):
    self.assertFalse(JEEP_LONG_ACTUATION_COMPILED)
    result = JeepLongitudinalShadow().update(-1.0, eligible=True)
    self.assertFalse(result.host_enabled)

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


if __name__ == "__main__":
  unittest.main()

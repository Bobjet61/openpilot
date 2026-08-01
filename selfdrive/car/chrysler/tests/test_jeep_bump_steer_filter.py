import importlib.util
import math
from pathlib import Path
import sys
import unittest


FILTER_PATH = (
  Path(__file__).resolve().parents[1] / "jeep_bump_steer_filter.py"
)
SPEC = importlib.util.spec_from_file_location(
  "jeep_bump_steer_filter_under_test", FILTER_PATH,
)
assert SPEC is not None and SPEC.loader is not None
FILTER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = FILTER
SPEC.loader.exec_module(FILTER)

JeepBumpSteerFilter = FILTER.JeepBumpSteerFilter
MAX_CORRECTION = FILTER.MAX_MEASUREMENT_CORRECTION_MPS2


class TestJeepBumpSteerFilter(unittest.TestCase):
  def test_normal_road_passes_measurement_through_exactly(self):
    bump_filter = JeepBumpSteerFilter()
    for frame in range(500):
      raw = math.sin(frame * 0.01) * 1.5
      output = bump_filter.update(raw, 0.2, 34.0, True, False)
      self.assertEqual(output, raw)
      self.assertFalse(bump_filter.state.active)

  def test_vertical_impulse_activates_bounded_correction(self):
    bump_filter = JeepBumpSteerFilter()
    for _ in range(100):
      bump_filter.update(1.0, 0.0, 34.0, True, False)

    corrections = []
    detections = 0
    for frame in range(80):
      vertical = -5.9 if frame == 0 else 0.0
      raw = 0.0 if frame < 20 else 1.0
      output = bump_filter.update(raw, vertical, 34.0, True, False)
      corrections.append(output - raw)
      detections += int(bump_filter.state.impulse_detected)

    self.assertGreater(detections, 0)
    self.assertTrue(any(abs(value) > 0.05 for value in corrections))
    self.assertLessEqual(max(abs(value) for value in corrections), MAX_CORRECTION)

  def test_low_speed_impulse_is_exact_passthrough(self):
    bump_filter = JeepBumpSteerFilter()
    bump_filter.update(0.5, 0.0, 15.0, True, False)
    output = bump_filter.update(-0.8, 6.0, 15.0, True, False)
    self.assertEqual(output, -0.8)
    self.assertFalse(bump_filter.state.active)

  def test_driver_override_is_exact_passthrough(self):
    bump_filter = JeepBumpSteerFilter()
    bump_filter.update(0.5, 0.0, 34.0, True, False)
    bump_filter.update(-0.5, 6.0, 34.0, True, False)
    output = bump_filter.update(0.9, 0.0, 34.0, True, True)
    self.assertEqual(output, 0.9)
    self.assertEqual(bump_filter.state.correction_lateral_accel, 0.0)
    self.assertFalse(bump_filter.initialized)

    output = bump_filter.update(-0.6, 0.0, 34.0, True, False)
    self.assertEqual(output, -0.6)
    self.assertEqual(bump_filter.state.strength, 0.0)
    self.assertFalse(bump_filter.state.active)

  def test_release_returns_to_exact_passthrough(self):
    bump_filter = JeepBumpSteerFilter()
    for _ in range(100):
      bump_filter.update(0.5, 0.0, 34.0, True, False)
    bump_filter.update(-0.8, 6.0, 34.0, True, False)

    for _ in range(300):
      output = bump_filter.update(0.7, 0.0, 34.0, True, False)

    self.assertEqual(output, 0.7)
    self.assertEqual(bump_filter.state.strength, 0.0)
    self.assertFalse(bump_filter.state.active)

  def test_invalid_sensor_input_resets_and_passes_raw(self):
    bump_filter = JeepBumpSteerFilter()
    bump_filter.update(0.5, 0.0, 34.0, True, False)
    output = bump_filter.update(0.7, float("nan"), 34.0, True, False)
    self.assertEqual(output, 0.7)
    self.assertFalse(bump_filter.initialized)


if __name__ == "__main__":
  unittest.main()

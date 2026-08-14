import importlib.util
from pathlib import Path
import sys
import unittest


PATH = Path(__file__).resolve().parents[1] / "jeep_steering_discontinuity_guard.py"
CONTROLLER_PATH = Path(__file__).resolve().parents[1] / "carcontroller.py"
SPEC = importlib.util.spec_from_file_location("jeep_steering_discontinuity_guard_test", PATH)
assert SPEC is not None and SPEC.loader is not None
GUARD = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = GUARD
SPEC.loader.exec_module(GUARD)


class TestJeepSteeringDiscontinuityGuard(unittest.TestCase):
  def setUp(self):
    self.guard = GUARD.JeepSteeringDiscontinuityGuard()

  def update(self, curvature, **overrides):
    values = {
      "requested_raw": -150 if curvature > 0 else 150,
      "previous_applied_raw": -80 if curvature > 0 else 80,
      "desired_curvature": curvature,
      "left_lane_probability": 0.9,
      "right_lane_probability": 0.9,
      "speed_mps": 13.0,
      "control_allowed": True,
      "steering_pressed": False,
      "model_valid": True,
    }
    values.update(overrides)
    return self.guard.update(**values)

  def establish(self, curvature=0.003):
    for _ in range(GUARD.ESTABLISHED_CYCLES):
      result = self.update(curvature)
    return result

  def test_high_confidence_curve_reversal_preserves_response_five_request(self):
    self.establish()
    result = self.update(-0.002, requested_raw=220, previous_applied_raw=-80)
    self.assertFalse(result.active)
    self.assertEqual(result.requested_raw, 220)

  def test_first_low_confidence_reversal_is_guarded_immediately(self):
    self.establish()
    result = self.update(
      -0.0012,
      requested_raw=220,
      previous_applied_raw=-80,
      left_lane_probability=0.20,
      right_lane_probability=0.25,
    )
    self.assertTrue(result.activated)
    self.assertTrue(result.active)
    self.assertEqual(result.requested_raw, -77)

  def test_guard_caps_authority_and_uses_slower_bounded_slew(self):
    self.establish()
    self.update(
      -0.0012,
      requested_raw=220,
      previous_applied_raw=-80,
      left_lane_probability=0.20,
      right_lane_probability=0.25,
    )
    result = self.update(
      -0.004,
      requested_raw=261,
      previous_applied_raw=98,
      left_lane_probability=0.20,
      right_lane_probability=0.25,
    )
    self.assertTrue(result.active)
    self.assertEqual(result.requested_raw, 100)

  def test_low_confidence_without_a_sign_change_is_unchanged(self):
    self.establish()
    result = self.update(
      0.004,
      requested_raw=-230,
      previous_applied_raw=-80,
      left_lane_probability=0.1,
      right_lane_probability=0.1,
    )
    self.assertFalse(result.active)
    self.assertEqual(result.requested_raw, -230)

  def test_one_confident_inner_lane_does_not_trigger_guard(self):
    self.establish()
    result = self.update(
      -0.002,
      requested_raw=220,
      previous_applied_raw=-80,
      left_lane_probability=0.2,
      right_lane_probability=0.8,
    )
    self.assertFalse(result.active)
    self.assertEqual(result.requested_raw, 220)

  def test_low_speed_driver_override_and_invalid_model_disable_guard(self):
    for override in (
      {"speed_mps": 7.9},
      {"steering_pressed": True},
      {"model_valid": False},
    ):
      self.establish()
      result = self.update(
        -0.002,
        left_lane_probability=0.1,
        right_lane_probability=0.1,
        **override,
      )
      self.assertFalse(result.active)
      self.assertEqual(result.requested_raw, 150)

  def test_controller_applies_guard_before_the_existing_torque_limiter(self):
    source = CONTROLLER_PATH.read_text(encoding="utf-8")
    guard_index = source.index(
      "self.jeep_steering_discontinuity_guard.update(",
    )
    limiter_index = source.index(
      "limited_steer = apply_meas_steer_torque_limits(",
      guard_index,
    )
    self.assertLess(guard_index, limiter_index)
    self.assertIn("new_steer = guard_result.requested_raw", source)


if __name__ == "__main__":
  unittest.main()

import ast
import importlib.util
from pathlib import Path
import sys
import unittest


CHRYSLER_PATH = Path(__file__).resolve().parents[1]
SHADOW_PATH = CHRYSLER_PATH / "jeep_steering_shadow.py"
CONTROLLER_PATH = CHRYSLER_PATH / "carcontroller.py"
VALUES_PATH = CHRYSLER_PATH / "values.py"

SHADOW_SPEC = importlib.util.spec_from_file_location(
  "jeep_steering_shadow_under_test",
  SHADOW_PATH,
)
assert SHADOW_SPEC is not None and SHADOW_SPEC.loader is not None
SHADOW = importlib.util.module_from_spec(SHADOW_SPEC)
sys.modules[SHADOW_SPEC.name] = SHADOW
SHADOW_SPEC.loader.exec_module(SHADOW)


class TestJeepSteeringShadow(unittest.TestCase):
  def setUp(self):
    self.shadow = SHADOW.JeepSteeringShadow(
      steer_max=261,
      steer_delta_up=3,
      steer_delta_down=3,
      steer_error_max=80,
      driver_threshold=120,
    )

  def update(self, **overrides):
    values = {
      "requested_normalized": 50 / 261,
      "requested_raw": 50,
      "limited_raw": 50,
      "applied_raw": 50,
      "previous_applied_raw": 47,
      "eps_torque": 0.0,
      "driver_torque": 0.0,
      "control_allowed": True,
      "steer_required": False,
      "temporary_fault": False,
      "permanent_fault": False,
    }
    values.update(overrides)
    self.shadow.update(**values)

  @staticmethod
  def baseline_limiter(requested, previous, eps_torque):
    max_allowed = min(max(eps_torque + 80, 80), 261)
    min_allowed = max(min(eps_torque - 80, -80), -261)
    error_limited = min(max(requested, min_allowed), max_allowed)
    if previous > 0:
      lower = max(previous - 3, -3)
      upper = previous + 3
    else:
      lower = previous - 3
      upper = min(previous + 3, 3)
    return int(round(min(max(error_limited, lower), upper)))

  def test_unlimited_sample_has_no_limit_classification(self):
    self.update()
    window = self.shadow.snapshot()
    self.assertEqual(window.samples, 1)
    self.assertEqual(window.active_samples, 1)
    self.assertEqual(window.error_limited_samples, 0)
    self.assertEqual(window.rate_limited_samples, 0)
    self.assertEqual(window.limiter_mismatch_samples, 0)
    self.assertEqual(window.max_applied_raw, 50)

  def test_measured_eps_error_limit_is_distinguished(self):
    self.update(
      requested_normalized=1.0,
      requested_raw=261,
      limited_raw=80,
      applied_raw=80,
      previous_applied_raw=80,
    )
    window = self.shadow.snapshot()
    self.assertEqual(window.error_limited_samples, 1)
    self.assertEqual(window.rate_limited_samples, 0)
    self.assertEqual(window.limiter_mismatch_samples, 0)
    self.assertEqual(window.max_request_limited_gap, 181)

  def test_command_rate_limit_is_distinguished(self):
    self.update(
      limited_raw=3,
      applied_raw=3,
      previous_applied_raw=0,
    )
    window = self.shadow.snapshot()
    self.assertEqual(window.error_limited_samples, 0)
    self.assertEqual(window.rate_limited_samples, 1)
    self.assertEqual(window.limiter_mismatch_samples, 0)

  def test_disabled_control_suppression_is_recorded(self):
    self.update(
      limited_raw=3,
      applied_raw=0,
      previous_applied_raw=0,
      control_allowed=False,
    )
    window = self.shadow.snapshot()
    self.assertEqual(window.suppressed_samples, 1)
    self.assertEqual(window.max_request_applied_gap, 50)

  def test_ceiling_duration_and_eps_feedback_are_separate(self):
    for _ in range(10):
      self.update(
        requested_normalized=1.0,
        requested_raw=261,
        limited_raw=261,
        applied_raw=261,
        previous_applied_raw=258,
        eps_torque=277.0,
      )
    window = self.shadow.snapshot()
    self.assertEqual(window.request_at_ceiling_samples, 10)
    self.assertEqual(window.applied_at_ceiling_samples, 10)
    self.assertEqual(window.eps_over_limit_samples, 10)
    self.assertEqual(window.max_eps_torque, 277.0)
    self.assertEqual(window.longest_request_ceiling_ms, 200)
    self.assertEqual(window.longest_applied_ceiling_ms, 200)

  def test_driver_warning_and_fault_indicators_are_recorded(self):
    self.update(
      driver_torque=121.0,
      steer_required=True,
      temporary_fault=True,
      permanent_fault=True,
    )
    window = self.shadow.snapshot()
    self.assertEqual(window.driver_override_samples, 1)
    self.assertEqual(window.steer_required_samples, 1)
    self.assertEqual(window.temporary_fault_samples, 1)
    self.assertEqual(window.permanent_fault_samples, 1)

  def test_exact_limiter_reconstruction_has_no_mismatches(self):
    for requested in (-261, -160, -50, 0, 50, 160, 261):
      for previous in (-261, -80, -3, 0, 3, 80, 261):
        for eps_torque in (-300.0, -80.0, 0.0, 80.0, 300.0):
          limited = self.baseline_limiter(
            requested,
            previous,
            eps_torque,
          )
          self.update(
            requested_normalized=requested / 261,
            requested_raw=requested,
            limited_raw=limited,
            applied_raw=limited,
            previous_applied_raw=previous,
            eps_torque=eps_torque,
          )
    window = self.shadow.snapshot()
    self.assertEqual(window.samples, 245)
    self.assertEqual(window.limiter_mismatch_samples, 0)

  def test_nonfinite_eps_feedback_cannot_break_diagnostics(self):
    self.update(
      limited_raw=3,
      applied_raw=3,
      previous_applied_raw=0,
      eps_torque=float("nan"),
    )
    window = self.shadow.snapshot()
    self.assertEqual(window.samples, 1)
    self.assertEqual(window.max_eps_torque, 0.0)

  def test_diagnostic_has_no_output_path_and_limit_is_unchanged(self):
    shadow_source = SHADOW_PATH.read_text(encoding="utf-8")
    for forbidden in (
      "CANPacker",
      "PubMaster",
      "sendcan",
      "can_sends",
      "create_lkas_command",
    ):
      self.assertNotIn(forbidden, shadow_source)

    values_source = VALUES_PATH.read_text(encoding="utf-8")
    self.assertIn("self.STEER_MAX = 261", values_source)

    controller_tree = ast.parse(
      CONTROLLER_PATH.read_text(encoding="utf-8"),
    )
    logging_methods = [
      node
      for node in ast.walk(controller_tree)
      if (
        isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == "log_jeep_steering_shadow"
      )
    ]
    self.assertEqual(len(logging_methods), 1)
    referenced_names = {
      node.id
      for node in ast.walk(logging_methods[0])
      if isinstance(node, ast.Name)
    }
    self.assertTrue(
      {"can_sends", "new_actuators", "apply_steer"}.isdisjoint(
        referenced_names,
      ),
    )


if __name__ == "__main__":
  unittest.main()

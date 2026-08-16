import importlib.util
from pathlib import Path
import sys
import unittest


CHRYSLER_PATH = Path(__file__).resolve().parents[1]
ESTIMATOR_PATH = CHRYSLER_PATH / "jeep_vision_lead_decel.py"
RADARD_PATH = CHRYSLER_PATH.parents[1] / "controls" / "radard.py"

SPEC = importlib.util.spec_from_file_location("jeep_vision_lead_decel_under_test", ESTIMATOR_PATH)
assert SPEC is not None and SPEC.loader is not None
ESTIMATOR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = ESTIMATOR
SPEC.loader.exec_module(ESTIMATOR)

DT_NS = 50_000_000
BASE_NS = 1_000_000_000


def update(
    estimator,
    frame,
    *,
    probability=0.99,
    distance=45.0,
    v_ego=17.0,
    v_lead=14.5,
    a_now=-1.0,
    a_2s=-1.4,
    a_std_now=0.3,
    a_std_2s=0.5,
    model_time_ns=None,
):
  if model_time_ns is None:
    model_time_ns = BASE_NS + frame * DT_NS
  return estimator.update(
    model_mono_time_ns=model_time_ns,
    v_ego_mps=v_ego,
    model_v_ego_mps=v_ego,
    probability=probability,
    x=[distance],
    v=[v_lead],
    a=[a_now, a_2s],
    a_std=[a_std_now, a_std_2s],
  )


class TestJeepVisionLeadDecelEstimator(unittest.TestCase):
  def test_runtime_gate_fails_closed_and_does_not_include_gas_override(self):
    base = {
      "scoped_to_jeep_wp_op_long": True,
      "inputs_valid": True,
      "model_age_ns": ESTIMATOR.MAX_MODEL_AGE_NS,
      "cruise_enabled": True,
      "forward_gear": True,
      "brake_pressed": False,
      "stock_aeb": False,
      "acc_faulted": False,
    }
    self.assertTrue(ESTIMATOR.jeep_vision_lead_decel_runtime_eligible(**base))
    failures = {
      "scoped_to_jeep_wp_op_long": False,
      "inputs_valid": False,
      "model_age_ns": ESTIMATOR.MAX_MODEL_AGE_NS + 1,
      "cruise_enabled": False,
      "forward_gear": False,
      "brake_pressed": True,
      "stock_aeb": True,
      "acc_faulted": True,
    }
    for field, value in failures.items():
      with self.subTest(field=field):
        candidate = dict(base)
        candidate[field] = value
        self.assertFalse(ESTIMATOR.jeep_vision_lead_decel_runtime_eligible(**candidate))

  def test_requires_three_distinct_continuous_frames(self):
    estimator = ESTIMATOR.JeepVisionLeadDecelEstimator()
    first = update(estimator, 0)
    second = update(estimator, 1, distance=44.875)
    self.assertFalse(first.confirmed)
    self.assertFalse(second.confirmed)
    self.assertEqual(second.consecutive_frames, 2)

    duplicate = update(estimator, 1, distance=44.875)
    self.assertFalse(duplicate.confirmed)
    self.assertEqual(duplicate.consecutive_frames, 2)

    third = update(estimator, 2, distance=44.75)
    self.assertTrue(third.confirmed)
    self.assertAlmostEqual(third.a_lead_mps2, -1.0)

  def test_requires_sustained_certain_deceleration(self):
    cases = (
      {"probability": 0.89},
      {"v_lead": 16.1},
      {"a_now": -0.49},
      {"a_2s": -0.49},
      {"a_std_now": 1.01},
      {"a_std_2s": 1.01},
      {"v_ego": 2.9, "v_lead": 0.0},
    )
    for kwargs in cases:
      with self.subTest(kwargs=kwargs):
        estimator = ESTIMATOR.JeepVisionLeadDecelEstimator()
        for frame in range(ESTIMATOR.CONFIRM_FRAMES + 1):
          result = update(estimator, frame, **kwargs)
        self.assertFalse(result.confirmed)
        self.assertEqual(result.a_lead_mps2, 0.0)

  def test_gate_boundaries_are_inclusive(self):
    estimator = ESTIMATOR.JeepVisionLeadDecelEstimator()
    for frame in range(ESTIMATOR.CONFIRM_FRAMES):
      result = update(
        estimator,
        frame,
        probability=ESTIMATOR.MIN_MODEL_PROBABILITY,
        v_ego=ESTIMATOR.MIN_EGO_SPEED_MPS,
        v_lead=ESTIMATOR.MIN_EGO_SPEED_MPS - ESTIMATOR.MIN_CLOSING_SPEED_MPS,
        a_now=-ESTIMATOR.MIN_SUSTAINED_DECEL_MPS2,
        a_2s=-ESTIMATOR.MIN_SUSTAINED_DECEL_MPS2,
        a_std_now=ESTIMATOR.MAX_ACCEL_STD_MPS2,
        a_std_2s=ESTIMATOR.MAX_ACCEL_STD_MPS2,
      )
    self.assertTrue(result.confirmed)
    self.assertAlmostEqual(result.a_lead_mps2, -ESTIMATOR.MIN_SUSTAINED_DECEL_MPS2)

  def test_uses_less_negative_prediction_and_hard_lower_bound(self):
    estimator = ESTIMATOR.JeepVisionLeadDecelEstimator()
    for frame in range(ESTIMATOR.CONFIRM_FRAMES):
      result = update(estimator, frame, a_now=-1.7, a_2s=-0.8)
    self.assertAlmostEqual(result.a_lead_mps2, -0.8)

    estimator = ESTIMATOR.JeepVisionLeadDecelEstimator()
    for frame in range(ESTIMATOR.CONFIRM_FRAMES):
      result = update(estimator, frame, a_now=-3.5, a_2s=-3.0)
    self.assertAlmostEqual(result.a_lead_mps2, ESTIMATOR.MIN_LEAD_ACCEL_MPS2)

  def test_identity_jump_and_model_gap_restart_confirmation(self):
    estimator = ESTIMATOR.JeepVisionLeadDecelEstimator()
    update(estimator, 0)
    update(estimator, 1, distance=44.875)
    jump = update(estimator, 2, distance=70.0)
    self.assertFalse(jump.confirmed)
    self.assertEqual(jump.consecutive_frames, 1)

    update(estimator, 3, distance=69.875)
    gap = update(
      estimator,
      4,
      distance=69.75,
      model_time_ns=BASE_NS + 3 * DT_NS + ESTIMATOR.MAX_MODEL_FRAME_GAP_NS + 1,
    )
    self.assertFalse(gap.confirmed)
    self.assertEqual(gap.consecutive_frames, 1)
    self.assertEqual(gap.reason, "model_frame_gap")

  def test_velocity_jump_restarts_confirmation(self):
    estimator = ESTIMATOR.JeepVisionLeadDecelEstimator()
    update(estimator, 0)
    update(estimator, 1, distance=44.875)
    result = update(estimator, 2, distance=44.75, v_lead=8.0)
    self.assertFalse(result.confirmed)
    self.assertEqual(result.consecutive_frames, 1)

  def test_out_of_order_timestamp_resets(self):
    estimator = ESTIMATOR.JeepVisionLeadDecelEstimator()
    update(estimator, 0)
    update(estimator, 1, distance=44.875)
    result = update(estimator, 2, model_time_ns=BASE_NS - 1)
    self.assertFalse(result.confirmed)
    self.assertEqual(result.reason, "out_of_order_model_frame")
    self.assertEqual(result.consecutive_frames, 0)

  def test_gate_loss_immediately_returns_to_zero(self):
    estimator = ESTIMATOR.JeepVisionLeadDecelEstimator()
    for frame in range(ESTIMATOR.CONFIRM_FRAMES):
      result = update(estimator, frame)
    self.assertTrue(result.confirmed)

    lost = update(estimator, ESTIMATOR.CONFIRM_FRAMES, probability=0.7)
    self.assertFalse(lost.confirmed)
    self.assertEqual(lost.a_lead_mps2, 0.0)
    self.assertEqual(lost.consecutive_frames, 0)

  def test_nonfinite_or_short_arrays_fail_closed(self):
    estimator = ESTIMATOR.JeepVisionLeadDecelEstimator()
    invalid = estimator.update(
      model_mono_time_ns=BASE_NS,
      v_ego_mps=17.0,
      model_v_ego_mps=17.0,
      probability=0.99,
      x=[float("nan")],
      v=[14.0],
      a=[-1.0],
      a_std=[0.2],
    )
    self.assertFalse(invalid.confirmed)
    self.assertEqual(invalid.a_lead_mps2, 0.0)

    malformed = estimator.update(
      model_mono_time_ns=BASE_NS + DT_NS,
      v_ego_mps=17.0,
      model_v_ego_mps=17.0,
      probability=0.99,
      x=None,
      v=[14.0],
      a=[-1.0, -1.0],
      a_std=[0.2, 0.2],
    )
    self.assertFalse(malformed.confirmed)
    self.assertEqual(malformed.a_lead_mps2, 0.0)

  def test_route6b_representative_frames_confirm(self):
    estimator = ESTIMATOR.JeepVisionLeadDecelEstimator()
    route_frames = (
      (46.31, 15.32, -0.93, -1.48, 0.37, 0.50),
      (46.22, 15.22, -0.98, -1.55, 0.34, 0.49),
      (46.13, 15.11, -1.03, -1.62, 0.33, 0.48),
    )
    for frame, values in enumerate(route_frames):
      distance, v_lead, a_now, a_2s, a_std_now, a_std_2s = values
      result = update(
        estimator,
        frame,
        distance=distance,
        v_ego=16.91,
        v_lead=v_lead,
        a_now=a_now,
        a_2s=a_2s,
        a_std_now=a_std_now,
        a_std_2s=a_std_2s,
      )
    self.assertTrue(result.confirmed)
    self.assertAlmostEqual(result.a_lead_mps2, -1.03)

  def test_radard_scope_and_camera_only_integration_are_explicit(self):
    source = RADARD_PATH.read_text(encoding="utf-8")
    self.assertIn("CHRYSLER_CAR.JEEP_GRAND_CHEROKEE", source)
    self.assertIn("CHRYSLER_CAR.JEEP_GRAND_CHEROKEE_2019", source)
    self.assertIn("CP.openpilotLongitudinalControl", source)
    self.assertIn("CP.spFlags & ChryslerFlagsSP.SP_WP_S20", source)
    self.assertIn("sm.all_checks(['modelV2', 'carState'])", source)
    self.assertIn("jeep_vision_lead_decel_runtime_eligible", source)
    self.assertIn("sm['carState'].cruiseState.enabled", source)
    self.assertIn("brake_pressed=sm['carState'].brakePressed", source)
    self.assertNotIn("and not sm['carState'].gasPressed", source)
    self.assertIn('"longitudinal_ineligible"', source)
    self.assertIn('vision_decel_estimator.reset(model_mono_time_ns, "radar_track")', source)
    self.assertIn("a_lead_k = estimate.a_lead_mps2", source)
    lead_two_call = "leads_v3[1], model_v_ego, self.CP, low_speed_override=False"
    self.assertIn(lead_two_call, source)


if __name__ == "__main__":
  unittest.main()

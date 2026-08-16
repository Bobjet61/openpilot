import importlib.util
from pathlib import Path
import sys
import unittest


CHRYSLER_PATH = Path(__file__).resolve().parents[1]
SHADOW_PATH = CHRYSLER_PATH / "jeep_camera_lead_trend_shadow.py"
CONTROLLER_PATH = CHRYSLER_PATH / "carcontroller.py"

SPEC = importlib.util.spec_from_file_location(
  "jeep_camera_lead_trend_shadow_under_test",
  SHADOW_PATH,
)
assert SPEC is not None and SPEC.loader is not None
SHADOW = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SHADOW
SPEC.loader.exec_module(SHADOW)

BASE_NS = 1_000_000_000
DT_NS = 50_000_000


def update(
    shadow,
    frame,
    *,
    distance=60.0,
    eligible=True,
    model_age_ns=20_000_000,
    v_ego=20.0,
    vision_status=True,
    vision_radar=False,
    probability=0.99,
    model_time_ns=None,
    radar_d=None,
    radar_v=None,
    radar_age_ns=None,
):
  if model_time_ns is None:
    model_time_ns = BASE_NS + frame * DT_NS
  return shadow.update(
    eligible=eligible,
    model_mono_time_ns=model_time_ns,
    model_age_ns=model_age_ns,
    v_ego_mps=v_ego,
    vision_status=vision_status,
    vision_radar=vision_radar,
    model_probability=probability,
    d_rel_m=distance,
    radar_d_rel_m=radar_d,
    radar_v_rel_mps=radar_v,
    radar_age_ns=radar_age_ns,
  )


def replay_distances(shadow, distances, **kwargs):
  result = None
  for frame, distance in enumerate(distances):
    result = update(shadow, frame, distance=distance, **kwargs)
  return result


class TestJeepCameraLeadTrendShadow(unittest.TestCase):
  def test_runtime_gate_fails_closed_for_scope_and_live_blockers(self):
    base = {
      "scoped_to_jeep_wp_op_long": True,
      "inputs_valid": True,
      "model_age_ns": SHADOW.MAX_MODEL_AGE_NS,
      "controls_enabled": True,
      "long_controls_active": True,
      "cruise_available": True,
      "cruise_enabled": True,
      "forward_gear": True,
      "brake_pressed": False,
      "gas_pressed": False,
      "stock_aeb": False,
      "acc_faulted": False,
    }
    self.assertTrue(SHADOW.jeep_camera_lead_trend_runtime_eligible(**base))
    failures = {
      "scoped_to_jeep_wp_op_long": False,
      "inputs_valid": False,
      "model_age_ns": SHADOW.MAX_MODEL_AGE_NS + 1,
      "controls_enabled": False,
      "long_controls_active": False,
      "cruise_available": False,
      "cruise_enabled": False,
      "forward_gear": False,
      "brake_pressed": True,
      "gas_pressed": True,
      "stock_aeb": True,
      "acc_faulted": True,
    }
    for field, value in failures.items():
      with self.subTest(field=field):
        candidate = dict(base)
        candidate[field] = value
        self.assertFalse(
          SHADOW.jeep_camera_lead_trend_runtime_eligible(**candidate),
        )

  def test_e221_shaped_camera_range_trend_is_positive(self):
    # Exact high-confidence camera-only range/probability window from E221.
    # The final 0.63 m rise is bounded model jitter amid four strong decreases.
    distances = (78.17, 75.65, 73.95, 72.18, 69.70, 70.33)
    probabilities = (0.951, 0.963, 0.955, 0.954, 0.964, 0.953)
    shadow = SHADOW.JeepCameraLeadTrendShadow()
    for frame, (distance, probability) in enumerate(
        zip(distances, probabilities, strict=True),
    ):
      result = update(
        shadow,
        frame,
        distance=distance,
        probability=probability,
      )

    self.assertTrue(result.hazard_candidate)
    self.assertEqual(result.reason, "camera_trend_candidate")
    self.assertGreaterEqual(result.sample_count, SHADOW.MIN_DISTINCT_SAMPLES)
    self.assertGreaterEqual(result.window_span_s, 0.25)
    self.assertLessEqual(result.window_span_s, 0.40)
    self.assertGreaterEqual(
      result.observed_closing_mps,
      SHADOW.MIN_OBSERVED_CLOSING_MPS,
    )
    self.assertLessEqual(result.camera_ttc_s, SHADOW.MAX_CAMERA_TTC_S)
    self.assertGreaterEqual(
      result.monotonic_fraction,
      SHADOW.MIN_DECREASING_FRACTION,
    )

  def test_requires_five_distinct_samples_and_minimum_time_span(self):
    shadow = SHADOW.JeepCameraLeadTrendShadow()
    for frame, distance in enumerate((30.0, 29.7, 29.4, 29.1, 28.8)):
      result = update(shadow, frame, distance=distance)
    self.assertFalse(result.hazard_candidate)
    self.assertEqual(result.reason, "window_too_short")
    self.assertEqual(result.sample_count, 5)
    self.assertAlmostEqual(result.window_span_s, 0.20)

    duplicate = update(shadow, 4, distance=20.0)
    self.assertFalse(duplicate.distinct_sample)
    self.assertEqual(duplicate.sample_count, 5)
    self.assertEqual(duplicate.reason, "duplicate_model_frame")

    sixth = update(shadow, 5, distance=28.5)
    self.assertTrue(sixth.hazard_candidate)
    self.assertEqual(sixth.sample_count, 6)

  def test_e352_shaped_camera_range_is_negative_even_with_adjacent_radar(self):
    # Exact E352 camera-only shape: the supposed lead is moving away overall
    # and changes range direction.  An aggressive adjacent raw-radar return
    # cannot turn it into a camera trend candidate.
    distances = (22.35, 22.06, 21.64, 21.43, 22.96, 22.15, 22.94)
    shadow = SHADOW.JeepCameraLeadTrendShadow()
    result = replay_distances(
      shadow,
      distances,
      radar_d=27.0,
      radar_v=-16.0,
      radar_age_ns=20_000_000,
    )
    self.assertFalse(result.hazard_candidate)
    # Range agreement alone cannot establish lane membership.  Even when the
    # optional annotation is true, it cannot override the negative camera trend.
    self.assertTrue(result.radar_corroborated)

  def test_flat_camera_range_is_negative(self):
    distances = (44.00, 43.99, 43.98, 43.97, 43.96, 43.95, 43.94)
    result = replay_distances(
      SHADOW.JeepCameraLeadTrendShadow(), distances,
    )
    self.assertFalse(result.hazard_candidate)
    self.assertIn(result.reason, ("closing_below_threshold", "non_monotonic"))

  def test_adjacent_fast_radar_cannot_raise_camera_severity(self):
    distances = (30.0, 29.9, 29.8, 29.7, 29.6, 29.5)
    without_radar = replay_distances(
      SHADOW.JeepCameraLeadTrendShadow(),
      distances,
    )
    with_adjacent_radar = replay_distances(
      SHADOW.JeepCameraLeadTrendShadow(),
      distances,
      radar_d=29.5,
      radar_v=-20.0,
      radar_age_ns=10_000_000,
    )
    self.assertFalse(without_radar.hazard_candidate)
    self.assertFalse(with_adjacent_radar.hazard_candidate)
    self.assertEqual(
      without_radar.observed_closing_mps,
      with_adjacent_radar.observed_closing_mps,
    )
    self.assertEqual(without_radar.camera_ttc_s, with_adjacent_radar.camera_ttc_s)

  def test_matching_radar_only_adds_corroboration_annotation(self):
    distances = (32.0, 31.65, 31.30, 30.95, 30.60, 30.25)
    without_radar = replay_distances(
      SHADOW.JeepCameraLeadTrendShadow(), distances,
    )
    with_radar = replay_distances(
      SHADOW.JeepCameraLeadTrendShadow(),
      distances,
      radar_d=30.0,
      radar_v=-7.0,
      radar_age_ns=20_000_000,
    )
    self.assertEqual(
      without_radar.hazard_candidate,
      with_radar.hazard_candidate,
    )
    self.assertEqual(
      without_radar.observed_closing_mps,
      with_radar.observed_closing_mps,
    )
    self.assertEqual(without_radar.camera_ttc_s, with_radar.camera_ttc_s)
    self.assertFalse(without_radar.radar_corroborated)
    self.assertTrue(with_radar.radar_corroborated)

  def test_camera_ttc_gate_rejects_fast_closing_distant_lead(self):
    distances = (100.0, 99.7, 99.4, 99.1, 98.8, 98.5)
    result = replay_distances(SHADOW.JeepCameraLeadTrendShadow(), distances)
    self.assertFalse(result.hazard_candidate)
    self.assertEqual(result.reason, "ttc_above_threshold")
    self.assertGreater(result.camera_ttc_s, SHADOW.MAX_CAMERA_TTC_S)

  def test_probability_speed_and_camera_only_gates_reset_history(self):
    failures = (
      {"probability": SHADOW.MIN_MODEL_PROBABILITY - 0.01},
      {"v_ego": SHADOW.MIN_EGO_SPEED_MPS - 0.01},
      {"vision_status": False},
      {"vision_radar": True},
    )
    for failure in failures:
      with self.subTest(failure=failure):
        shadow = SHADOW.JeepCameraLeadTrendShadow()
        update(shadow, 0, distance=40.0)
        update(shadow, 1, distance=39.6)
        result = update(shadow, 2, distance=39.2, **failure)
        self.assertFalse(result.hazard_candidate)
        self.assertEqual(result.sample_count, 0)

  def test_stale_model_and_live_ineligibility_reset_history(self):
    shadow = SHADOW.JeepCameraLeadTrendShadow()
    update(shadow, 0, distance=40.0)
    update(shadow, 1, distance=39.6)
    stale = update(
      shadow,
      2,
      distance=39.2,
      model_age_ns=SHADOW.MAX_MODEL_AGE_NS + 1,
    )
    self.assertEqual(stale.reason, "stale_or_invalid_model")
    self.assertEqual(stale.sample_count, 0)

    update(shadow, 3, distance=38.8)
    ineligible = update(shadow, 4, distance=38.4, eligible=False)
    self.assertEqual(ineligible.reason, "ineligible")
    self.assertEqual(ineligible.sample_count, 0)

  def test_frame_gap_restarts_from_current_sample(self):
    shadow = SHADOW.JeepCameraLeadTrendShadow()
    update(shadow, 0, distance=50.0)
    update(shadow, 1, distance=49.7)
    result = update(
      shadow,
      2,
      distance=49.0,
      model_time_ns=BASE_NS + DT_NS + SHADOW.MAX_SAMPLE_GAP_NS + 1,
    )
    self.assertEqual(result.reason, "sample_gap")
    self.assertEqual(result.sample_count, 1)
    self.assertFalse(result.hazard_candidate)

  def test_range_jump_rise_and_out_of_order_reset(self):
    shadow = SHADOW.JeepCameraLeadTrendShadow()
    update(shadow, 0, distance=60.0)
    jump = update(shadow, 1, distance=50.0)
    self.assertEqual(jump.reason, "range_jump")
    self.assertEqual(jump.sample_count, 1)

    rise = update(shadow, 2, distance=51.0)
    self.assertEqual(rise.reason, "range_rise")
    self.assertEqual(rise.sample_count, 1)

    out_of_order = update(
      shadow,
      3,
      distance=50.5,
      model_time_ns=BASE_NS - 1,
    )
    self.assertEqual(out_of_order.reason, "out_of_order_model_frame")
    self.assertEqual(out_of_order.sample_count, 0)

  def test_non_monotonic_or_unstable_camera_history_is_negative(self):
    # All rises remain inside the jitter bound, but too few intervals show a
    # genuine decreasing trend to qualify.
    distances = (30.00, 30.10, 30.00, 30.10, 30.00, 29.90)
    non_monotonic = replay_distances(
      SHADOW.JeepCameraLeadTrendShadow(), distances,
    )
    self.assertFalse(non_monotonic.hazard_candidate)
    self.assertEqual(non_monotonic.reason, "non_monotonic")

  def test_window_snapshot_counts_events_not_duplicate_controller_ticks(self):
    shadow = SHADOW.JeepCameraLeadTrendShadow()
    result = replay_distances(
      shadow,
      (32.0, 31.6, 31.2, 30.8, 30.4, 30.0),
    )
    self.assertTrue(result.hazard_candidate)
    update(shadow, 5, distance=30.0)
    window = shadow.snapshot()
    self.assertEqual(window.distinct_samples, 6)
    self.assertEqual(window.hazard_events, 1)
    self.assertGreaterEqual(window.hazard_samples, 1)
    self.assertEqual(shadow.snapshot().distinct_samples, 0)

  def test_result_has_no_control_authority_surface(self):
    fields = SHADOW.JeepCameraLeadTrendShadowResult.__dataclass_fields__
    forbidden_fragments = ("accel", "brake", "torque", "request", "floor")
    for field in fields:
      self.assertFalse(any(fragment in field for fragment in forbidden_fragments))

    source = SHADOW_PATH.read_text(encoding="utf-8")
    for forbidden in (
        "CANPacker",
        "create_wp_long",
        "requested_accel",
        "brake_floor_mps2",
        "can_sends",
    ):
      self.assertNotIn(forbidden, source)

  def test_controller_integration_is_telemetry_only_and_jeep_scoped(self):
    source = CONTROLLER_PATH.read_text(encoding="utf-8")
    self.assertIn("JeepCameraLeadTrendShadow", source)
    self.assertIn("jeep_camera_lead_trend_runtime_eligible", source)
    self.assertIn("if CP.carFingerprint in JEEP_LONG_CARS else None", source)
    self.assertIn("log_jeep_camera_lead_trend_shadow", source)
    self.assertNotIn(
      "self.jeep_camera_lead_trend_result.hazard_candidate\n"
      "          and JEEP_LONG_ACTUATION_COMPILED",
      source,
    )


if __name__ == "__main__":
  unittest.main()

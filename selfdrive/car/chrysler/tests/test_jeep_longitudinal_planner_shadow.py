import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
import types
import unittest

# The Windows analysis runtime omits device-only realtime dependencies.
# LongControl and drive_helpers need only these two production constants here.
if sys.platform == "win32":
  realtime_stub = types.ModuleType("openpilot.common.realtime")
  realtime_stub.DT_CTRL = 0.01
  realtime_stub.DT_MDL = 0.05
  sys.modules["openpilot.common.realtime"] = realtime_stub

from cereal import car
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N


CHRYSLER_PATH = Path(__file__).resolve().parents[1]
LONG_PATH = CHRYSLER_PATH / "jeep_longitudinal.py"
PLANNER_PATH = CHRYSLER_PATH / "jeep_longitudinal_planner_shadow.py"
CONTROLLER_PATH = CHRYSLER_PATH / "carcontroller.py"

# Load the worktree module under its production name. This also makes the test
# work in Windows checkouts where openpilot's repository symlinks are text
# files rather than filesystem links.
LONG_MODULE_NAME = "openpilot.selfdrive.car.chrysler.jeep_longitudinal"
LONG_SPEC = importlib.util.spec_from_file_location(
  LONG_MODULE_NAME,
  LONG_PATH,
)
assert LONG_SPEC is not None and LONG_SPEC.loader is not None
LONG = importlib.util.module_from_spec(LONG_SPEC)
sys.modules[LONG_MODULE_NAME] = LONG
LONG_SPEC.loader.exec_module(LONG)

PLANNER_SPEC = importlib.util.spec_from_file_location(
  "jeep_longitudinal_planner_shadow_under_test",
  PLANNER_PATH,
)
assert PLANNER_SPEC is not None and PLANNER_SPEC.loader is not None
PLANNER = importlib.util.module_from_spec(PLANNER_SPEC)
sys.modules[PLANNER_SPEC.name] = PLANNER
PLANNER_SPEC.loader.exec_module(PLANNER)


def make_car_params():
  CP = car.CarParams.new_message()
  CP.stopAccel = -2.0
  CP.startAccel = -0.8
  CP.stoppingDecelRate = 0.8
  CP.vEgoStopping = 0.5
  CP.vEgoStarting = 0.5
  CP.stoppingControl = True
  CP.startingState = False
  CP.longitudinalTuning.deadzoneBP = [0.0]
  CP.longitudinalTuning.deadzoneV = [0.0]
  CP.longitudinalTuning.kf = 1.0
  CP.longitudinalTuning.kpBP = [0.0]
  CP.longitudinalTuning.kpV = [1.0]
  CP.longitudinalTuning.kiBP = [0.0]
  CP.longitudinalTuning.kiV = [1.0]
  CP.longitudinalActuatorDelayLowerBound = 0.15
  CP.longitudinalActuatorDelayUpperBound = 0.15
  return CP


def make_plan(
    *,
    speed=10.0,
    accel=0.5,
    has_lead=False,
    fcw=False,
    source="cruise",
):
  return SimpleNamespace(
    speeds=[speed] * CONTROL_N,
    accels=[accel] * CONTROL_N,
    hasLead=has_lead,
    fcw=fcw,
    longitudinalPlanSource=source,
  )


def make_car_state(v_ego=8.0):
  return SimpleNamespace(
    vEgo=v_ego,
    brakePressed=False,
    cruiseState=SimpleNamespace(standstill=False),
  )


class TestJeepLongitudinalPlanShadow(unittest.TestCase):
  def setUp(self):
    self.shadow = PLANNER.JeepLongitudinalPlanShadow(
      make_car_params(),
    )

  def update(self, **overrides):
    values = {
      "plan": make_plan(),
      "car_state": make_car_state(),
      "seen": True,
      "service_valid": True,
      "plan_age_s": 0.05,
      "vehicle_eligible": True,
      "vehicle_reason": "eligible",
      "radar_selection": None,
    }
    values.update(overrides)
    return self.shadow.update(**values)

  def test_valid_plan_runs_production_long_control_in_memory(self):
    result = self.update()
    self.assertTrue(result.plan_valid)
    self.assertTrue(result.eligible)
    self.assertEqual(result.reason, "eligible")
    self.assertGreater(result.controller_accel_mps2, 0.0)
    self.assertLessEqual(result.controller_accel_mps2, LONG.ACCEL_MAX)
    self.assertEqual(result.target_speed_mps, 10.0)

  def test_stale_plan_fails_closed_and_resets_output(self):
    self.update()
    result = self.update(plan_age_s=PLANNER.PLAN_MAX_AGE_S + 0.01)
    self.assertFalse(result.plan_valid)
    self.assertFalse(result.eligible)
    self.assertEqual(result.reason, "plan_stale")
    self.assertEqual(result.controller_accel_mps2, 0.0)

  def test_vehicle_ineligibility_wins_over_valid_plan(self):
    result = self.update(
      vehicle_eligible=False,
      vehicle_reason="brake_pressed",
    )
    self.assertTrue(result.plan_valid)
    self.assertFalse(result.eligible)
    self.assertEqual(result.reason, "brake_pressed")
    self.assertEqual(result.controller_accel_mps2, 0.0)

  def test_invalid_and_nonfinite_trajectories_fail_closed(self):
    malformed = make_plan()
    malformed.speeds = malformed.speeds[:-1]
    result = self.update(plan=malformed)
    self.assertEqual(result.reason, "plan_speeds_invalid")
    self.assertEqual(result.controller_accel_mps2, 0.0)

    nonfinite = make_plan()
    nonfinite.accels[0] = float("nan")
    result = self.update(plan=nonfinite)
    self.assertEqual(result.reason, "plan_accels_invalid")
    self.assertEqual(result.controller_accel_mps2, 0.0)

  def test_lead_confirmation_is_diagnostic_only(self):
    radar = SimpleNamespace(
      track=SimpleNamespace(d_rel=22.0, v_rel=-1.5),
    )
    confirmed = self.update(
      plan=make_plan(has_lead=True),
      radar_selection=radar,
    )
    self.assertEqual(confirmed.lead_state, "confirmed")
    self.assertEqual(confirmed.radar_d_rel, 22.0)

    unconfirmed = self.update(plan=make_plan(has_lead=True))
    self.assertEqual(unconfirmed.lead_state, "unconfirmed")

    radar_only = self.update(
      plan=make_plan(has_lead=False),
      radar_selection=radar,
    )
    self.assertEqual(radar_only.lead_state, "radar_only")

  def test_window_counts_and_resets(self):
    self.update(
      plan=make_plan(has_lead=True, fcw=True),
      radar_selection=SimpleNamespace(
        track=SimpleNamespace(d_rel=20.0, v_rel=-2.0),
      ),
    )
    self.update(
      vehicle_eligible=False,
      vehicle_reason="gas_pressed",
    )
    window = self.shadow.snapshot()
    self.assertEqual(window.samples, 2)
    self.assertEqual(window.plan_valid_samples, 2)
    self.assertEqual(window.eligible_samples, 1)
    self.assertEqual(window.lead_confirmed_samples, 1)
    self.assertEqual(window.fcw_samples, 1)
    self.assertEqual(self.shadow.snapshot().samples, 0)

  def test_module_and_controller_have_no_unguarded_output_path(self):
    planner_tree = ast.parse(
      PLANNER_PATH.read_text(encoding="utf-8"),
    )
    used_names = {
      node.id
      for node in ast.walk(planner_tree)
      if isinstance(node, ast.Name)
    }
    used_attributes = {
      node.attr
      for node in ast.walk(planner_tree)
      if isinstance(node, ast.Attribute)
    }
    for forbidden in (
      "CANPacker",
      "PubMaster",
      "sendcan",
      "can_sends",
      "create_wp_long_shadow_messages",
    ):
      self.assertNotIn(forbidden, used_names | used_attributes)

    controller_source = CONTROLLER_PATH.read_text(encoding="utf-8")
    self.assertIn(
      "self.jeep_long_plan_result.controller_accel_mps2",
      controller_source,
    )
    self.assertEqual(
      controller_source.count(
        "can_sends.extend(self.jeep_long_shadow_frames)",
      ),
      1,
    )
    self.assertIn(
      "if self.jeep_long_envelope.transport_enabled:",
      controller_source,
    )


if __name__ == "__main__":
  unittest.main()

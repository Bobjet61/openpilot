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
RECOVERY_PATH = (
  CHRYSLER_PATH.parents[1]
  / "controls"
  / "lib"
  / "longitudinal_gas_release_recovery.py"
)

RECOVERY_MODULE_NAME = (
  "openpilot.selfdrive.controls.lib.longitudinal_gas_release_recovery"
)
RECOVERY_SPEC = importlib.util.spec_from_file_location(
  RECOVERY_MODULE_NAME,
  RECOVERY_PATH,
)
assert RECOVERY_SPEC is not None and RECOVERY_SPEC.loader is not None
RECOVERY = importlib.util.module_from_spec(RECOVERY_SPEC)
sys.modules[RECOVERY_MODULE_NAME] = RECOVERY
RECOVERY_SPEC.loader.exec_module(RECOVERY)

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


def make_car_params(*, jeep_op_long=False):
  CP = car.CarParams.new_message()
  CP.carName = "chrysler" if jeep_op_long else "mock"
  CP.openpilotLongitudinalControl = jeep_op_long
  CP.spFlags = RECOVERY.JEEP_WP_S20_FLAG if jeep_op_long else 0
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
    gasPressed=False,
    brakePressed=False,
    cruiseState=SimpleNamespace(standstill=False),
  )


def make_linear_plan(*, speed, accel):
  return SimpleNamespace(
    speeds=[
      speed + accel * time_s
      for time_s in PLANNER.CONTROL_N_T_IDX
    ],
    accels=[accel] * CONTROL_N,
  )


def gas_release_context_allowed(**overrides):
  values = {
    "jeep_op_long": True,
    "plans_valid": True,
    "main_source": "cruise",
    "sp_source": "cruise",
    "has_lead": False,
    "fcw": False,
    "vision_turn_state": "disabled",
    "speed_limit_state": "inactive",
    "turn_speed_state": "inactive",
    "hard_brake_predicted": False,
    "force_decel": False,
    "brake_pressed": False,
    "acc_faulted": False,
    "stock_aeb": False,
    "standstill": False,
  }
  values.update(overrides)
  return RECOVERY.jeep_gas_release_context_allowed(**values)


class TestJeepGasReleaseRecovery(unittest.TestCase):
  def test_context_predicate_fails_closed(self):
    self.assertTrue(gas_release_context_allowed())

    disqualifying_contexts = {
      "non_jeep_op_long": {"jeep_op_long": False},
      "invalid_plan": {"plans_valid": False},
      "lead_source": {"main_source": "lead0"},
      "e2e_source": {"main_source": "e2e"},
      "sp_turn_source": {"sp_source": "turn"},
      "lead": {"has_lead": True},
      "fcw": {"fcw": True},
      "vision_turn": {"vision_turn_state": "turning"},
      "speed_limit": {"speed_limit_state": "active"},
      "turn_speed": {"turn_speed_state": "active"},
      "hard_brake": {"hard_brake_predicted": True},
      "force_decel": {"force_decel": True},
      "brake_pressed": {"brake_pressed": True},
      "acc_fault": {"acc_faulted": True},
      "stock_aeb": {"stock_aeb": True},
      "standstill": {"standstill": True},
    }
    for name, overrides in disqualifying_contexts.items():
      with self.subTest(name=name):
        self.assertFalse(gas_release_context_allowed(**overrides))

  def test_attenuation_changes_only_negative_pid_feedback(self):
    output = RECOVERY.attenuate_negative_pid_output(
      feedforward=-0.6,
      proportional=-1.2,
      integral=-0.4,
      derivative=-0.1,
      ordinary_output=-2.3,
      feedback_scale=0.25,
      neg_limit=-3.0,
      pos_limit=2.0,
    )
    self.assertAlmostEqual(output, -1.1)

    # Negative planner feed-forward and derivative feedback remain intact even
    # at the smallest possible P/I scale.
    self.assertAlmostEqual(
      RECOVERY.attenuate_negative_pid_output(
        feedforward=-0.6,
        proportional=-1.2,
        integral=-0.4,
        derivative=-0.1,
        ordinary_output=-2.3,
        feedback_scale=0.0,
        neg_limit=-3.0,
        pos_limit=2.0,
      ),
      -0.7,
    )

    positive_output = RECOVERY.attenuate_negative_pid_output(
      feedforward=0.1,
      proportional=0.8,
      integral=0.2,
      derivative=0.0,
      ordinary_output=1.1,
      feedback_scale=0.0,
      neg_limit=-3.0,
      pos_limit=2.0,
    )
    self.assertAlmostEqual(positive_output, 1.1)

  def test_attenuation_never_reverses_command_sign(self):
    ordinary_accel = RECOVERY.attenuate_negative_pid_output(
      feedforward=-0.2,
      proportional=0.5,
      integral=0.0,
      derivative=0.0,
      ordinary_output=0.3,
      feedback_scale=0.25,
      neg_limit=-3.0,
      pos_limit=2.0,
    )
    self.assertAlmostEqual(ordinary_accel, 0.3)

    softened_brake = RECOVERY.attenuate_negative_pid_output(
      feedforward=0.3,
      proportional=-0.5,
      integral=0.0,
      derivative=0.0,
      ordinary_output=-0.2,
      feedback_scale=0.25,
      neg_limit=-3.0,
      pos_limit=2.0,
    )
    self.assertLessEqual(softened_brake, 0.0)
    self.assertGreaterEqual(softened_brake, -0.2)

    ordinary_light_brake = RECOVERY.attenuate_negative_pid_output(
      feedforward=-0.6,
      proportional=0.5,
      integral=0.0,
      derivative=0.0,
      ordinary_output=-0.1,
      feedback_scale=0.25,
      neg_limit=-3.0,
      pos_limit=2.0,
    )
    self.assertAlmostEqual(ordinary_light_brake, -0.1)

  def test_long_control_preserves_feedforward_and_positive_feedback(self):
    CP = make_car_params(jeep_op_long=True)
    CP.longitudinalTuning.kiV = [0.0]
    accel_limits = [-3.0, 2.0]

    recovery_control = PLANNER.LongControl(CP)
    overspeed_state = make_car_state(v_ego=11.0)
    overspeed_state.gasPressed = True
    recovery_control.update(
      False,
      overspeed_state,
      make_linear_plan(speed=10.0, accel=-0.2),
      accel_limits,
      0.0,
      jeep_gas_release_recovery_allowed=True,
      v_cruise=10.0,
    )
    overspeed_state.gasPressed = False
    recovery_output = recovery_control.update(
      True,
      overspeed_state,
      make_linear_plan(speed=10.0, accel=-0.2),
      accel_limits,
      0.0,
      jeep_gas_release_recovery_allowed=True,
      v_cruise=10.0,
    )
    self.assertEqual(
      recovery_control.jeep_gas_release_feedback_scale,
      RECOVERY.JEEP_GAS_RELEASE_MIN_FEEDBACK_SCALE,
    )
    self.assertAlmostEqual(recovery_control.pid.i, 0.0)
    self.assertAlmostEqual(recovery_control.pid.f, -0.2)
    self.assertLessEqual(recovery_output, recovery_control.pid.f)

    baseline_control = PLANNER.LongControl(CP)
    baseline_output = baseline_control.update(
      True,
      make_car_state(v_ego=11.0),
      make_linear_plan(speed=10.0, accel=-0.2),
      accel_limits,
      0.0,
      jeep_gas_release_recovery_allowed=True,
      v_cruise=10.0,
    )
    self.assertAlmostEqual(baseline_control.pid.f, -0.2)
    self.assertAlmostEqual(baseline_control.pid.p, recovery_control.pid.p)
    self.assertLess(baseline_output, recovery_output)

    positive_recovery = PLANNER.LongControl(CP)
    positive_state = make_car_state(v_ego=9.0)
    positive_state.gasPressed = True
    positive_recovery.update(
      False,
      positive_state,
      make_linear_plan(speed=10.0, accel=0.2),
      accel_limits,
      0.0,
      jeep_gas_release_recovery_allowed=True,
      v_cruise=10.0,
    )
    positive_state.gasPressed = False
    positive_output = positive_recovery.update(
      True,
      positive_state,
      make_linear_plan(speed=10.0, accel=0.2),
      accel_limits,
      0.0,
      jeep_gas_release_recovery_allowed=True,
      v_cruise=10.0,
    )
    positive_baseline = PLANNER.LongControl(CP)
    baseline_positive_output = positive_baseline.update(
      True,
      make_car_state(v_ego=9.0),
      make_linear_plan(speed=10.0, accel=0.2),
      accel_limits,
      0.0,
      jeep_gas_release_recovery_allowed=True,
      v_cruise=10.0,
    )
    self.assertAlmostEqual(positive_recovery.pid.p, positive_baseline.pid.p)
    self.assertAlmostEqual(positive_recovery.pid.f, positive_baseline.pid.f)
    self.assertAlmostEqual(positive_output, baseline_positive_output)

  def test_timeout_restores_feedback_and_hazard_cancels(self):
    recovery = RECOVERY.JeepGasReleaseRecovery(enabled=True)
    common = {
      "active": True,
      "context_allowed": True,
      "v_ego": 20.0,
      "v_cruise": 20.0,
      "planned_accel": 0.0,
      "dt_s": RECOVERY.JEEP_GAS_RELEASE_RECOVERY_DURATION_S / 2.0,
    }
    recovery.update(gas_pressed=True, **common)
    self.assertEqual(
      recovery.update(gas_pressed=False, **common),
      RECOVERY.JEEP_GAS_RELEASE_MIN_FEEDBACK_SCALE,
    )
    self.assertTrue(recovery.active)
    recovery.update(gas_pressed=False, **common)
    self.assertEqual(recovery.update(gas_pressed=False, **common), 1.0)
    self.assertFalse(recovery.active)

    recovery.update(gas_pressed=True, **common)
    self.assertEqual(
      recovery.update(gas_pressed=False, **common),
      RECOVERY.JEEP_GAS_RELEASE_MIN_FEEDBACK_SCALE,
    )
    self.assertEqual(
      recovery.update(
        gas_pressed=False,
        **(common | {"context_allowed": False}),
      ),
      1.0,
    )
    self.assertFalse(recovery.active)
    self.assertEqual(recovery.update(gas_pressed=False, **common), 1.0)

  def test_significant_planned_braking_bypasses_recovery(self):
    CP = make_car_params(jeep_op_long=True)
    state = make_car_state(v_ego=11.0)
    state.gasPressed = True
    recovery_control = PLANNER.LongControl(CP)
    baseline_control = PLANNER.LongControl(CP)
    mild_plan = make_linear_plan(speed=10.0, accel=0.0)

    recovery_control.update(
      False,
      state,
      mild_plan,
      [-3.0, 2.0],
      0.0,
      jeep_gas_release_recovery_allowed=True,
      v_cruise=10.0,
    )
    baseline_control.update(
      False,
      state,
      mild_plan,
      [-3.0, 2.0],
      0.0,
      jeep_gas_release_recovery_allowed=False,
      v_cruise=10.0,
    )
    state.gasPressed = False
    recovery_control.update(
      True,
      state,
      mild_plan,
      [-3.0, 2.0],
      0.0,
      jeep_gas_release_recovery_allowed=True,
      v_cruise=10.0,
    )
    baseline_control.update(
      True,
      state,
      mild_plan,
      [-3.0, 2.0],
      0.0,
      jeep_gas_release_recovery_allowed=False,
      v_cruise=10.0,
    )

    self.assertTrue(recovery_control.jeep_gas_release_recovery.active)
    plan = make_linear_plan(speed=10.0, accel=-0.6)
    recovery_output = recovery_control.update(
      True,
      state,
      plan,
      [-3.0, 2.0],
      0.0,
      jeep_gas_release_recovery_allowed=True,
      v_cruise=10.0,
    )
    baseline_output = baseline_control.update(
      True,
      state,
      plan,
      [-3.0, 2.0],
      0.0,
      jeep_gas_release_recovery_allowed=False,
      v_cruise=10.0,
    )
    self.assertFalse(recovery_control.jeep_gas_release_recovery.active)
    self.assertEqual(recovery_control.jeep_gas_release_feedback_scale, 1.0)
    self.assertAlmostEqual(recovery_control.pid.f, baseline_control.pid.f)
    self.assertAlmostEqual(recovery_control.pid.p, baseline_control.pid.p)
    self.assertAlmostEqual(recovery_control.pid.i, baseline_control.pid.i)
    self.assertAlmostEqual(recovery_output, baseline_output)

  def test_stopping_transition_restores_baseline_output(self):
    CP = make_car_params(jeep_op_long=True)
    recovery_control = PLANNER.LongControl(CP)
    baseline_control = PLANNER.LongControl(CP)
    state = make_car_state(v_ego=11.0)
    state.gasPressed = True
    cruise_plan = make_linear_plan(speed=10.0, accel=0.0)

    for control, allowed in (
      (recovery_control, True),
      (baseline_control, False),
    ):
      control.update(
        False,
        state,
        cruise_plan,
        [-3.0, 2.0],
        0.0,
        jeep_gas_release_recovery_allowed=allowed,
        v_cruise=10.0,
      )
    state.gasPressed = False
    recovery_release_output = recovery_control.update(
      True,
      state,
      cruise_plan,
      [-3.0, 2.0],
      0.0,
      jeep_gas_release_recovery_allowed=True,
      v_cruise=10.0,
    )
    baseline_release_output = baseline_control.update(
      True,
      state,
      cruise_plan,
      [-3.0, 2.0],
      0.0,
      jeep_gas_release_recovery_allowed=False,
      v_cruise=10.0,
    )
    self.assertGreater(recovery_release_output, baseline_release_output)

    stopping_plan = make_plan(speed=0.0, accel=-1.0)
    recovery_stop_output = recovery_control.update(
      True,
      state,
      stopping_plan,
      [-3.0, 2.0],
      0.0,
      jeep_gas_release_recovery_allowed=False,
      v_cruise=10.0,
    )
    baseline_stop_output = baseline_control.update(
      True,
      state,
      stopping_plan,
      [-3.0, 2.0],
      0.0,
      jeep_gas_release_recovery_allowed=False,
      v_cruise=10.0,
    )
    self.assertEqual(
      recovery_control.long_control_state,
      PLANNER.LongCtrlState.stopping,
    )
    self.assertAlmostEqual(recovery_stop_output, baseline_stop_output)

  def test_disabled_non_jeep_recovery_never_arms(self):
    recovery = RECOVERY.JeepGasReleaseRecovery(enabled=False)
    common = {
      "active": True,
      "context_allowed": True,
      "v_ego": 20.0,
      "v_cruise": 20.0,
      "planned_accel": 0.0,
      "dt_s": 0.01,
    }
    self.assertEqual(recovery.update(gas_pressed=True, **common), 1.0)
    self.assertEqual(recovery.update(gas_pressed=False, **common), 1.0)
    self.assertFalse(recovery.active)

    CP = make_car_params(jeep_op_long=False)
    CP.longitudinalTuning.kiV = [0.0]
    control = PLANNER.LongControl(CP)
    state = make_car_state(v_ego=11.0)
    state.gasPressed = True
    plan = make_linear_plan(speed=10.0, accel=0.0)
    control.update(
      False,
      state,
      plan,
      [-3.0, 2.0],
      0.0,
      jeep_gas_release_recovery_allowed=True,
      v_cruise=10.0,
    )
    state.gasPressed = False
    control.update(
      True,
      state,
      plan,
      [-3.0, 2.0],
      0.0,
      jeep_gas_release_recovery_allowed=True,
      v_cruise=10.0,
    )
    self.assertEqual(control.jeep_gas_release_feedback_scale, 1.0)
    self.assertLess(control.pid.p, 0.0)


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
    self.assertEqual(
      controller_source.count(
        "can_sends.extend(self.jeep_long_transport_frames)",
      ),
      1,
    )
    self.assertIn(
      "if transport_counter is not None:",
      controller_source,
    )
    self.assertIn(
      "if self.jeep_long_envelope.host_enabled:",
      controller_source,
    )


if __name__ == "__main__":
  unittest.main()

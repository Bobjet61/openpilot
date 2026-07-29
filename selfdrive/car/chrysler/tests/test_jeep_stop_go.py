import importlib.util
from pathlib import Path
import sys
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "jeep_stop_go.py"
SPEC = importlib.util.spec_from_file_location("jeep_stop_go_under_test", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
STOP_GO = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = STOP_GO
SPEC.loader.exec_module(STOP_GO)


class TestJeepStopGoHold(unittest.TestCase):
  def setUp(self):
    self.hold = STOP_GO.JeepStopGoHold()
    self.base = {
      "frame": 100,
      "supported": True,
      "forward_gear": True,
      "cruise_available": True,
      "stock_acc_enabled": True,
      "controls_enabled": True,
      "long_active": True,
      "standstill": True,
      "v_ego_mps": 0.0,
      "stopping": True,
      "requested_accel_mps2": -3.0,
      "lead_departure_confirmed": False,
      "cancel": False,
      "gas_pressed": False,
      "brake_pressed": False,
      "acc_faulted": False,
      "stock_aeb": False,
    }

  def update(self, **changes):
    values = self.base.copy()
    values.update(changes)
    return self.hold.update(**values)

  def test_latches_only_from_an_active_acc_stop(self):
    for change in (
      {"controls_enabled": False},
      {"long_active": False},
      {"stock_acc_enabled": False},
      {"standstill": False},
      {"stopping": False},
      {"requested_accel_mps2": 0.0},
    ):
      self.hold = STOP_GO.JeepStopGoHold()
      self.assertFalse(self.update(**change).active)

    result = self.update()
    self.assertTrue(result.active)
    self.assertFalse(result.hold_command)
    self.assertEqual(result.reason, "stock_holding")

  def test_requests_hold_only_after_stock_acc_times_out(self):
    self.update()
    result = self.update(frame=250, stock_acc_enabled=False,
                         controls_enabled=True, long_active=False)
    self.assertTrue(result.active)
    self.assertTrue(result.hold_command)
    self.assertEqual(result.reason, "holding")

  def test_resume_requires_confirmed_lead_departure(self):
    self.update()
    no_lead = self.update(frame=250, stock_acc_enabled=False,
                          long_active=False)
    self.assertTrue(no_lead.hold_command)
    self.assertFalse(no_lead.send_resume)

    lead = self.update(frame=251, stock_acc_enabled=False, long_active=False,
                       lead_departure_confirmed=True)
    self.assertTrue(lead.send_resume)
    self.assertTrue(lead.hold_command)
    self.assertTrue(lead.launch_pending)
    self.assertEqual(lead.reason, "resume_pending_hold")

  def test_resume_is_rate_limited_without_releasing_hold(self):
    self.update()
    first = self.update(frame=200, stock_acc_enabled=False, long_active=False,
                        lead_departure_confirmed=True)
    self.assertTrue(first.send_resume)
    self.assertTrue(first.hold_command)
    rate_limited = self.update(
      frame=225, stock_acc_enabled=False, long_active=False,
      lead_departure_confirmed=True,
    )
    self.assertFalse(rate_limited.send_resume)
    self.assertTrue(rate_limited.hold_command)
    still_holding = self.update(
      frame=251, stock_acc_enabled=False, long_active=False,
      lead_departure_confirmed=False,
    )
    self.assertTrue(still_holding.hold_command)
    self.assertTrue(still_holding.launch_pending)

  def test_stock_acc_acknowledgement_ends_private_hold_before_launch(self):
    self.update()
    requested = self.update(
      frame=200,
      stock_acc_enabled=False,
      long_active=False,
      lead_departure_confirmed=True,
    )
    self.assertTrue(requested.hold_command)

    acknowledged = self.update(
      frame=201,
      stock_acc_enabled=True,
      long_active=False,
      lead_departure_confirmed=False,
    )
    self.assertTrue(acknowledged.active)
    self.assertFalse(acknowledged.hold_command)
    self.assertFalse(acknowledged.launch_pending)
    self.assertEqual(acknowledged.reason, "stock_holding")

    launched = self.update(
      frame=250,
      stock_acc_enabled=True,
      long_active=True,
      standstill=False,
      v_ego_mps=0.6,
    )
    self.assertFalse(launched.active)

  def test_driver_inputs_and_faults_release_immediately(self):
    for change in (
      {"cancel": True},
      {"gas_pressed": True},
      {"brake_pressed": True},
      {"forward_gear": False},
      {"cruise_available": False},
      {"acc_faulted": True},
      {"stock_aeb": True},
      {"controls_enabled": False},
    ):
      self.hold = STOP_GO.JeepStopGoHold()
      self.update()
      result = self.update(frame=200, stock_acc_enabled=False,
                           long_active=False, **change)
      self.assertFalse(result.active, change)
      self.assertFalse(result.hold_command, change)

  def test_movement_releases_but_stopped_hold_does_not_time_out(self):
    self.update()
    self.assertFalse(self.update(
      frame=200, stock_acc_enabled=True, standstill=False, v_ego_mps=0.6,
    ).active)

    self.hold = STOP_GO.JeepStopGoHold()
    self.update()
    result = self.update(
      frame=100 + 60 * 60 * 100,
      stock_acc_enabled=False,
      long_active=False,
    )
    self.assertTrue(result.active)
    self.assertTrue(result.hold_command)

  def test_unacknowledged_roll_never_clears_hold(self):
    self.update()
    result = self.update(
      frame=200,
      stock_acc_enabled=False,
      long_active=False,
      standstill=False,
      v_ego_mps=1.0,
    )
    self.assertTrue(result.active)
    self.assertTrue(result.hold_command)

  def test_resume_attempt_limit_keeps_brakes_applied(self):
    self.update()
    for attempt in range(STOP_GO.MAX_RESUME_ATTEMPTS):
      result = self.update(
        frame=200 + attempt * STOP_GO.RESUME_INTERVAL_FRAMES,
        stock_acc_enabled=False,
        long_active=False,
        lead_departure_confirmed=True,
      )
      self.assertTrue(result.send_resume)
      self.assertTrue(result.hold_command)

    limited = self.update(
      frame=200 + STOP_GO.MAX_RESUME_ATTEMPTS * STOP_GO.RESUME_INTERVAL_FRAMES,
      stock_acc_enabled=False,
      long_active=False,
      lead_departure_confirmed=True,
    )
    self.assertFalse(limited.send_resume)
    self.assertTrue(limited.hold_command)
    self.assertEqual(limited.reason, "resume_limit_hold")


class TestJeepFullLongLaunchGuard(unittest.TestCase):
  def setUp(self):
    self.guard = STOP_GO.JeepFullLongLaunchGuard()
    self.base = {
      "frame": 100,
      "mode_enabled": True,
      "supported": True,
      "forward_gear": True,
      "cruise_available": True,
      "controls_enabled": True,
      "long_active": True,
      "standstill": True,
      "v_ego_mps": 0.0,
      "requested_accel_mps2": 0.5,
      "plan_valid": True,
      "plan_has_lead": True,
      "lead_departure_confirmed": True,
      "cancel": False,
      "gas_pressed": False,
      "brake_pressed": False,
      "acc_faulted": False,
      "stock_aeb": False,
    }

  def update(self, **changes):
    values = self.base.copy()
    values.update(changes)
    return self.guard.update(**values)

  def arm(self):
    first = self.update(frame=100)
    self.assertTrue(first.send_resume)
    self.assertFalse(first.armed)
    second = self.update(frame=110)
    self.assertTrue(second.send_resume)
    self.assertFalse(second.armed)
    third = self.update(frame=120)
    self.assertTrue(third.send_resume)
    self.assertFalse(third.armed)
    return self.update(frame=135)

  def test_confirmed_lead_departure_runs_settled_resume_handshake(self):
    first = self.update()
    self.assertFalse(first.armed)
    self.assertTrue(first.send_resume)
    self.assertEqual(first.resume_attempts, 1)
    self.assertFalse(self.update(frame=109).send_resume)
    self.assertTrue(self.update(frame=110).send_resume)
    self.assertTrue(self.update(frame=120).send_resume)
    self.assertFalse(self.update(frame=134).armed)
    armed = self.update(frame=135)
    self.assertTrue(armed.armed)
    self.assertEqual(armed.reason, "auto_lead_departure_armed")

    self.guard = STOP_GO.JeepFullLongLaunchGuard()
    waiting = self.update(lead_departure_confirmed=False)
    self.assertFalse(waiting.armed)
    self.assertFalse(waiting.send_resume)
    self.assertEqual(waiting.reason, "awaiting_lead_departure")

  def test_arm_helper(self):
    armed = self.arm()
    self.assertTrue(armed.armed)
    self.assertFalse(armed.hold_active)

  def test_brake_hold_recovery_latch_is_bounded(self):
    waiting = self.update(
      lead_departure_confirmed=False,
      requested_accel_mps2=-1.0,
    )
    self.assertTrue(waiting.hold_active)

    creeping = self.update(
      frame=101,
      standstill=False,
      v_ego_mps=STOP_GO.FULL_LONG_LAUNCH_ARM_MAX_SPEED_MPS,
      lead_departure_confirmed=False,
      requested_accel_mps2=-1.0,
    )
    self.assertTrue(creeping.hold_active)
    self.assertFalse(creeping.armed)
    self.assertFalse(creeping.send_resume)

    outside_recovery = self.update(
      frame=102,
      standstill=False,
      v_ego_mps=STOP_GO.FULL_LONG_LAUNCH_ARM_MAX_SPEED_MPS + 0.01,
      lead_departure_confirmed=False,
      requested_accel_mps2=-1.0,
    )
    self.assertFalse(outside_recovery.hold_active)

  def test_brake_hold_recovery_latch_clears_on_driver_brake(self):
    waiting = self.update(
      lead_departure_confirmed=False,
      requested_accel_mps2=-1.0,
    )
    self.assertTrue(waiting.hold_active)
    interrupted = self.update(
      frame=101,
      lead_departure_confirmed=False,
      requested_accel_mps2=-1.0,
      brake_pressed=True,
    )
    self.assertFalse(interrupted.hold_active)
    self.assertEqual(interrupted.reason, "brake")

  def test_arm_requires_stopped_valid_positive_lead_plan(self):
    for change in (
      {"standstill": False},
      {"v_ego_mps": 0.3},
      {"requested_accel_mps2": 0.0},
      {"plan_valid": False},
      {"plan_has_lead": False},
      {"lead_departure_confirmed": False},
    ):
      self.guard = STOP_GO.JeepFullLongLaunchGuard()
      result = self.update(**change)
      self.assertFalse(result.armed, change)
      self.assertFalse(result.send_resume, change)

  def test_nonpositive_plan_aborts_pending_handshake(self):
    first = self.update(frame=100)
    self.assertTrue(first.send_resume)
    aborted = self.update(frame=101, requested_accel_mps2=0.0)
    self.assertFalse(aborted.armed)
    self.assertFalse(aborted.send_resume)
    self.assertEqual(aborted.reason, "launch_plan_not_positive")

  def test_launch_is_one_shot_and_times_out(self):
    self.assertTrue(self.arm().armed)
    self.assertTrue(self.update(
      frame=135 + STOP_GO.FULL_LONG_LAUNCH_WINDOW_FRAMES - 1,
      standstill=False,
      v_ego_mps=1.0,
    ).armed)
    timed_out = self.update(
      frame=135 + STOP_GO.FULL_LONG_LAUNCH_WINDOW_FRAMES,
      standstill=False,
      v_ego_mps=1.0,
    )
    self.assertFalse(timed_out.armed)
    self.assertEqual(timed_out.reason, "launch_timeout")

  def test_launch_completes_at_low_speed_boundary(self):
    self.assertTrue(self.arm().armed)
    complete = self.update(
      frame=200,
      standstill=False,
      v_ego_mps=STOP_GO.FULL_LONG_LAUNCH_COMPLETE_MPS,
    )
    self.assertFalse(complete.armed)
    self.assertEqual(complete.reason, "launch_complete")

  def test_every_hazard_disarms(self):
    for change in (
      {"mode_enabled": False},
      {"supported": False},
      {"forward_gear": False},
      {"cruise_available": False},
      {"controls_enabled": False},
      {"long_active": False},
      {"plan_valid": False},
      {"plan_has_lead": False},
      {"cancel": True},
      {"gas_pressed": True},
      {"brake_pressed": True},
      {"acc_faulted": True},
      {"stock_aeb": True},
    ):
      self.guard = STOP_GO.JeepFullLongLaunchGuard()
      self.assertTrue(self.arm().armed)
      self.assertFalse(self.update(frame=136, **change).armed, change)


if __name__ == "__main__":
  unittest.main()

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
    self.assertFalse(lead.hold_command)
    self.assertTrue(lead.launch_pending)

  def test_resume_release_is_short_and_rate_limited(self):
    self.update()
    first = self.update(frame=200, stock_acc_enabled=False, long_active=False,
                        lead_departure_confirmed=True)
    self.assertTrue(first.send_resume)
    self.assertFalse(self.update(
      frame=225, stock_acc_enabled=False, long_active=False,
      lead_departure_confirmed=True,
    ).send_resume)
    after_release = self.update(
      frame=251, stock_acc_enabled=False, long_active=False,
      lead_departure_confirmed=False,
    )
    self.assertTrue(after_release.hold_command)

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

  def test_sub_threshold_creep_reapplies_hold_without_launch(self):
    self.update()
    result = self.update(
      frame=200,
      stock_acc_enabled=False,
      long_active=False,
      standstill=False,
      v_ego_mps=0.2,
    )
    self.assertTrue(result.active)
    self.assertTrue(result.hold_command)


if __name__ == "__main__":
  unittest.main()

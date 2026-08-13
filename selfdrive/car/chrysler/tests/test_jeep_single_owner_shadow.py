import importlib.util
import itertools
from pathlib import Path
import unittest


PATH = Path(__file__).resolve().parents[1] / "jeep_single_owner_shadow.py"
SPEC = importlib.util.spec_from_file_location("jeep_single_owner_shadow_test", PATH)
assert SPEC is not None and SPEC.loader is not None
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


def sample(**overrides):
  values = dict(
    monotonic_time_s=0.0, requested=True, controls_enabled=True, stock_available=True,
    stock_isolated=True, isolation_healthy=True,
    stock_command_observed=False, stock_fault=False, collision=False,
    gas_pressed=False, brake_pressed=False, forward_gear=True,
    door_open=False, seatbelt_unlatched=False, speed_mps=15.0,
    requested_accel_mps2=-0.5,
  )
  values.update(overrides)
  return MOD.SingleOwnerInput(**values)


class TestJeepSingleOwnerShadow(unittest.TestCase):
  @staticmethod
  def advance(shadow, count=30, **overrides):
    return [
      shadow.update(sample(monotonic_time_s=i * 0.1, **overrides))
      for i in range(count)
    ]

  def test_never_allows_actuation(self):
    shadow = MOD.JeepSingleOwnerShadow()
    results = self.advance(shadow)
    self.assertTrue(any(r.owner == MOD.LongOwner.OPENPILOT_SHADOW for r in results))
    self.assertTrue(all(not r.actuation_allowed for r in results))

  def test_waits_for_stock_owner_to_be_quiet(self):
    shadow = MOD.JeepSingleOwnerShadow()
    for i in range(30):
      result = shadow.update(sample(monotonic_time_s=i * 0.1, stock_command_observed=True))
    self.assertEqual(result.owner, MOD.LongOwner.ARMING)

  def test_never_arms_without_explicit_stock_isolation(self):
    shadow = MOD.JeepSingleOwnerShadow()
    for i in range(100):
      result = shadow.update(sample(monotonic_time_s=i * 0.1, stock_isolated=False))
    self.assertEqual(result.owner, MOD.LongOwner.BLOCKED)
    self.assertEqual(result.reason, "stock_not_isolated")
    self.assertFalse(result.actuation_allowed)

  def test_isolation_fault_blocks_immediately(self):
    shadow = MOD.JeepSingleOwnerShadow()
    self.advance(shadow)
    result = shadow.update(sample(monotonic_time_s=3.0, isolation_healthy=False))
    self.assertEqual(result.owner, MOD.LongOwner.BLOCKED)
    self.assertEqual(result.reason, "isolation_fault")

  def test_driver_override_blocks_immediately(self):
    shadow = MOD.JeepSingleOwnerShadow()
    self.advance(shadow)
    result = shadow.update(sample(monotonic_time_s=3.0, brake_pressed=True))
    self.assertEqual(result.owner, MOD.LongOwner.BLOCKED)
    self.assertEqual(result.reason, "driver_brake")
    self.assertEqual(result.requested_accel_mps2, 0.0)

  def test_stock_reappearance_revokes_shadow_output(self):
    shadow = MOD.JeepSingleOwnerShadow()
    self.advance(shadow)
    result = shadow.update(sample(monotonic_time_s=3.0, stock_command_observed=True))
    self.assertEqual(result.owner, MOD.LongOwner.REVOKING)
    self.assertEqual(result.requested_accel_mps2, 0.0)

  def test_stock_reappearance_latches_until_control_reset(self):
    shadow = MOD.JeepSingleOwnerShadow()
    self.advance(shadow)
    shadow.update(sample(monotonic_time_s=3.0, stock_command_observed=True))
    result = shadow.update(sample(monotonic_time_s=3.1, stock_command_observed=False))
    self.assertEqual(result.owner, MOD.LongOwner.BLOCKED)
    self.assertEqual(result.reason, "control_reset_required")
    self.assertEqual(result.requested_accel_mps2, 0.0)

    shadow.update(sample(monotonic_time_s=3.2, controls_enabled=False))
    result = shadow.update(sample(monotonic_time_s=3.3))
    self.assertEqual(result.owner, MOD.LongOwner.ARMING)

  def test_handoff_uses_elapsed_time_not_sample_count(self):
    shadow = MOD.JeepSingleOwnerShadow()
    for i in range(100):
      result = shadow.update(sample(monotonic_time_s=i * 0.005))
    self.assertEqual(result.owner, MOD.LongOwner.ARMING)
    result = shadow.update(sample(monotonic_time_s=1.01))
    self.assertEqual(result.owner, MOD.LongOwner.OPENPILOT_SHADOW)

  def test_nonmonotonic_time_latches_reset(self):
    shadow = MOD.JeepSingleOwnerShadow()
    shadow.update(sample(monotonic_time_s=1.0))
    result = shadow.update(sample(monotonic_time_s=0.9))
    self.assertEqual(result.owner, MOD.LongOwner.BLOCKED)
    self.assertEqual(result.reason, "invalid_time")

  def test_every_live_blocker_revokes_shadow_immediately(self):
    blockers = {
      "controls_enabled": False,
      "stock_available": False,
      "stock_isolated": False,
      "isolation_healthy": False,
      "stock_fault": True,
      "collision": True,
      "gas_pressed": True,
      "brake_pressed": True,
      "forward_gear": False,
      "door_open": True,
      "seatbelt_unlatched": True,
    }
    for field, value in blockers.items():
      with self.subTest(field=field):
        shadow = MOD.JeepSingleOwnerShadow()
        self.advance(shadow)
        result = shadow.update(sample(monotonic_time_s=3.0, **{field: value}))
        self.assertNotEqual(result.owner, MOD.LongOwner.OPENPILOT_SHADOW)
        self.assertEqual(result.requested_accel_mps2, 0.0)
        self.assertFalse(result.actuation_allowed)

  def test_boolean_gate_combinations_never_bypass_invariants(self):
    fields = (
      "controls_enabled", "stock_available", "stock_isolated", "isolation_healthy",
      "stock_fault", "collision", "gas_pressed", "brake_pressed",
      "forward_gear", "door_open", "seatbelt_unlatched",
    )
    for values in itertools.product((False, True), repeat=len(fields)):
      overrides = dict(zip(fields, values, strict=True))
      shadow = MOD.JeepSingleOwnerShadow()
      results = [
        shadow.update(sample(monotonic_time_s=i * 0.1, **overrides))
        for i in range(15)
      ]
      for result in results:
        self.assertFalse(result.actuation_allowed)
        if result.owner == MOD.LongOwner.OPENPILOT_SHADOW:
          self.assertTrue(overrides["controls_enabled"])
          self.assertTrue(overrides["stock_available"])
          self.assertTrue(overrides["stock_isolated"])
          self.assertTrue(overrides["isolation_healthy"])
          self.assertFalse(overrides["stock_fault"])
          self.assertFalse(overrides["collision"])
          self.assertFalse(overrides["gas_pressed"])
          self.assertFalse(overrides["brake_pressed"])
          self.assertTrue(overrides["forward_gear"])
          self.assertFalse(overrides["door_open"])
          self.assertFalse(overrides["seatbelt_unlatched"])


if __name__ == "__main__":
  unittest.main()

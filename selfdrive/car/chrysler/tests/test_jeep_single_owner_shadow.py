import importlib.util
from pathlib import Path
import unittest


PATH = Path(__file__).resolve().parents[1] / "jeep_single_owner_shadow.py"
SPEC = importlib.util.spec_from_file_location("jeep_single_owner_shadow_test", PATH)
assert SPEC is not None and SPEC.loader is not None
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


def sample(**overrides):
  values = dict(
    requested=True, controls_enabled=True, stock_available=True,
    stock_active=False, stock_fault=False, collision=False,
    gas_pressed=False, brake_pressed=False, forward_gear=True,
    door_open=False, seatbelt_unlatched=False, speed_mps=15.0,
    requested_accel_mps2=-0.5,
  )
  values.update(overrides)
  return MOD.SingleOwnerInput(**values)


class TestJeepSingleOwnerShadow(unittest.TestCase):
  def test_never_allows_actuation(self):
    shadow = MOD.JeepSingleOwnerShadow()
    results = [shadow.update(sample()) for _ in range(30)]
    self.assertTrue(any(r.owner == MOD.LongOwner.OPENPILOT_SHADOW for r in results))
    self.assertTrue(all(not r.actuation_allowed for r in results))

  def test_waits_for_stock_owner_to_be_quiet(self):
    shadow = MOD.JeepSingleOwnerShadow()
    for _ in range(30):
      result = shadow.update(sample(stock_active=True))
    self.assertEqual(result.owner, MOD.LongOwner.ARMING)

  def test_driver_override_blocks_immediately(self):
    shadow = MOD.JeepSingleOwnerShadow()
    for _ in range(30):
      shadow.update(sample())
    result = shadow.update(sample(brake_pressed=True))
    self.assertEqual(result.owner, MOD.LongOwner.BLOCKED)
    self.assertEqual(result.reason, "driver_brake")
    self.assertEqual(result.requested_accel_mps2, 0.0)

  def test_stock_reappearance_revokes_shadow_output(self):
    shadow = MOD.JeepSingleOwnerShadow()
    for _ in range(30):
      shadow.update(sample())
    result = shadow.update(sample(stock_active=True))
    self.assertEqual(result.owner, MOD.LongOwner.REVOKING)
    self.assertEqual(result.requested_accel_mps2, 0.0)


if __name__ == "__main__":
  unittest.main()

import importlib.util
from pathlib import Path
import sys
import unittest

REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT))

from common.conversions import Conversions as CV

APA5_PATH = Path(__file__).resolve().parents[1] / "apa5.py"
APA5_SPEC = importlib.util.spec_from_file_location("apa5_under_test", APA5_PATH)
assert APA5_SPEC is not None and APA5_SPEC.loader is not None
APA5 = importlib.util.module_from_spec(APA5_SPEC)
sys.modules[APA5_SPEC.name] = APA5
APA5_SPEC.loader.exec_module(APA5)

APA_CUTOFF_MPH = APA5.APA_CUTOFF_MPH
APA_REENTRY_MPH = APA5.APA_REENTRY_MPH
APA5_ACTUATION_COMPILED = APA5.APA5_ACTUATION_COMPILED
STEER_TYPE_APA = APA5.STEER_TYPE_APA
STEER_TYPE_LKAS = APA5.STEER_TYPE_LKAS
STEER_TYPE_NONE = APA5.STEER_TYPE_NONE
Apa5ModeController = APA5.Apa5ModeController
apa_torque_multiplier = APA5.apa_torque_multiplier
extra_apa_authority = APA5.extra_apa_authority


class TestApa5Envelope(unittest.TestCase):
  def test_authority_endpoints(self):
    self.assertEqual(extra_apa_authority(0.0), 1.0)
    self.assertEqual(extra_apa_authority(4.0), 1.0)
    self.assertAlmostEqual(extra_apa_authority(4.5), 0.5)
    self.assertEqual(extra_apa_authority(5.0), 0.0)
    self.assertEqual(extra_apa_authority(80.0), 0.0)

  def test_multiplier_is_lkas_equivalent_at_cutoff(self):
    self.assertEqual(apa_torque_multiplier(0.0), 4.0)
    self.assertEqual(apa_torque_multiplier(4.0), 4.0)
    self.assertAlmostEqual(apa_torque_multiplier(4.5), 2.5)
    self.assertEqual(apa_torque_multiplier(5.0), 1.0)

  def test_handoff_and_reentry_hysteresis(self):
    mode = Apa5ModeController()
    mode.update(3.0 * CV.MPH_TO_MS, eligible=True)
    below_cutoff = mode.update(4.9 * CV.MPH_TO_MS, eligible=True)
    self.assertEqual(below_cutoff.shadow_steer_type, STEER_TYPE_APA)

    at_cutoff = mode.update(APA_CUTOFF_MPH * CV.MPH_TO_MS, eligible=True)
    self.assertEqual(at_cutoff.shadow_steer_type, STEER_TYPE_LKAS)

    still_in_hysteresis = mode.update(4.5 * CV.MPH_TO_MS, eligible=True)
    self.assertEqual(still_in_hysteresis.shadow_steer_type, STEER_TYPE_LKAS)

    reentered = mode.update(APA_REENTRY_MPH * CV.MPH_TO_MS, eligible=True)
    self.assertEqual(reentered.shadow_steer_type, STEER_TYPE_APA)

  def test_ineligible_is_lkas_and_actuation_is_default_off(self):
    mode = Apa5ModeController()
    result = mode.update(2.0 * CV.MPH_TO_MS, eligible=False)
    self.assertEqual(result.shadow_steer_type, STEER_TYPE_LKAS)
    self.assertFalse(APA5_ACTUATION_COMPILED)
    self.assertEqual(result.commanded_steer_type, STEER_TYPE_NONE)


if __name__ == "__main__":
  unittest.main()

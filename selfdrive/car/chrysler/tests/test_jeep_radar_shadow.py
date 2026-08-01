import ast
import importlib.util
from pathlib import Path
import sys
import unittest


CHRYSLER_PATH = Path(__file__).resolve().parents[1]
RADAR_PATH = CHRYSLER_PATH / "jeep_radar_shadow.py"
CARSTATE_PATH = CHRYSLER_PATH / "carstate.py"
CARCONTROLLER_PATH = CHRYSLER_PATH / "carcontroller.py"
INTERFACE_PATH = CHRYSLER_PATH / "interface.py"
LONG_PATH = CHRYSLER_PATH / "jeep_longitudinal.py"

RADAR_SPEC = importlib.util.spec_from_file_location(
  "jeep_radar_shadow_under_test",
  RADAR_PATH,
)
assert RADAR_SPEC is not None and RADAR_SPEC.loader is not None
RADAR = importlib.util.module_from_spec(RADAR_SPEC)
sys.modules[RADAR_SPEC.name] = RADAR
RADAR_SPEC.loader.exec_module(RADAR)


class FakeParser:
  def __init__(self, distances=None, relative_speeds=None):
    distances = distances or [20.0 + index for index in range(10)]
    relative_speeds = relative_speeds or [-1.0] * 10
    self.vl = {}
    for index, address in enumerate(RADAR.RADAR_MSGS_C):
      self.vl[address] = {"LONG_DIST": distances[index]}
    for index, address in enumerate(RADAR.RADAR_MSGS_D):
      self.vl[address] = {"REL_SPEED": relative_speeds[index]}


class TestJeepRadarShadow(unittest.TestCase):
  def setUp(self):
    self.shadow = RADAR.JeepRadarShadow()

  def test_complete_cycle_decodes_range_and_speed_only(self):
    parser = FakeParser()
    triggered = self.shadow.update_can(
      parser,
      RADAR.RADAR_REQUIRED_MSGS,
    )
    self.assertTrue(triggered)
    self.assertTrue(self.shadow.last_cycle_complete)
    self.assertEqual(self.shadow.complete_cycle_count, 1)
    self.assertEqual(len(self.shadow.tracks), 10)
    self.assertEqual(self.shadow.tracks[0].d_rel, 20.0)
    self.assertEqual(self.shadow.tracks[0].v_rel, -1.0)
    self.assertFalse(hasattr(self.shadow.tracks[0], "y_rel"))

  def test_incomplete_cycle_fails_closed_and_clears_stale_tracks(self):
    parser = FakeParser()
    self.shadow.update_can(parser, RADAR.RADAR_REQUIRED_MSGS)
    self.assertTrue(self.shadow.tracks)

    self.shadow.update_can(parser, {RADAR.RADAR_TRIGGER_MSG})
    self.assertFalse(self.shadow.last_cycle_complete)
    self.assertEqual(self.shadow.tracks, ())

  def test_exact_vision_match_is_selected(self):
    parser = FakeParser(
      distances=[42.0] + [80.0 + index for index in range(9)],
      relative_speeds=[-3.0] + [5.0] * 9,
    )
    self.shadow.update_can(parser, RADAR.RADAR_REQUIRED_MSGS)
    result = self.shadow.select(
      RADAR.JeepVisionLead(True, 42.0, -3.0, 0.95),
      v_ego=25.0,
    )
    self.assertEqual(result.reason, "selected")
    self.assertEqual(result.track.index, 0)
    self.assertEqual(result.score, 0.0)

  def test_low_probability_vision_lead_abstains(self):
    self.shadow.update_can(FakeParser(), RADAR.RADAR_REQUIRED_MSGS)
    result = self.shadow.select(
      RADAR.JeepVisionLead(True, 20.0, -1.0, 0.69),
      v_ego=20.0,
    )
    self.assertEqual(result.reason, "vision_not_eligible")
    self.assertIsNone(result.track)

  def test_radar_fused_lead_is_not_accepted_as_vision_gate(self):
    self.shadow.update_can(FakeParser(), RADAR.RADAR_REQUIRED_MSGS)
    result = self.shadow.select(
      RADAR.JeepVisionLead(True, 20.0, -1.0, 0.95, radar=True),
      v_ego=20.0,
    )
    self.assertEqual(result.reason, "vision_not_eligible")

  def test_oncoming_proxy_is_rejected(self):
    parser = FakeParser(
      distances=[30.0] + [80.0 + index for index in range(9)],
      relative_speeds=[-22.0] + [5.0] * 9,
    )
    self.shadow.update_can(parser, RADAR.RADAR_REQUIRED_MSGS)
    result = self.shadow.select(
      RADAR.JeepVisionLead(True, 30.0, -22.0, 0.95),
      v_ego=18.0,
    )
    self.assertEqual(result.reason, "no_gated_candidate")
    self.assertIsNone(result.track)

  def test_weak_combined_match_abstains(self):
    parser = FakeParser(
      distances=[34.9] + [80.0 + index for index in range(9)],
      relative_speeds=[3.4] + [10.0] * 9,
    )
    self.shadow.update_can(parser, RADAR.RADAR_REQUIRED_MSGS)
    result = self.shadow.select(
      RADAR.JeepVisionLead(True, 30.0, -1.0, 0.95),
      v_ego=20.0,
    )
    self.assertEqual(result.reason, "weak_candidate")
    self.assertIsNone(result.track)

  def test_materially_different_near_tie_abstains(self):
    parser = FakeParser(
      distances=[38.0, 42.0] + [90.0 + index for index in range(8)],
      relative_speeds=[-2.0, -2.0] + [10.0] * 8,
    )
    self.shadow.update_can(parser, RADAR.RADAR_REQUIRED_MSGS)
    result = self.shadow.select(
      RADAR.JeepVisionLead(True, 40.0, -2.0, 0.95),
      v_ego=20.0,
    )
    self.assertEqual(result.reason, "ambiguous_acquisition")
    self.assertIsNone(result.track)

  def test_diagnostic_source_has_no_output_or_planner_dependency(self):
    radar_source = RADAR_PATH.read_text(encoding="utf-8")
    for forbidden in (
      "CANPacker",
      "PubMaster",
      "sendcan",
      "can_sends",
      "longitudinalPlan",
    ):
      self.assertNotIn(forbidden, radar_source)

    interface_source = INTERFACE_PATH.read_text(encoding="utf-8")
    self.assertIn("ret.radarUnavailable = True", interface_source)
    interface_tree = ast.parse(interface_source)
    self.assertFalse(
      any(
        isinstance(node, ast.Attribute) and node.attr == "can_parsers"
        for node in ast.walk(interface_tree)
      ),
    )

    carstate_source = CARSTATE_PATH.read_text(encoding="utf-8")
    self.assertIn(
      "messages = [(address, 0)",
      carstate_source,
    )

    controller_source = CARCONTROLLER_PATH.read_text(encoding="utf-8")
    controller_tree = ast.parse(controller_source)
    diagnostic_methods = [
      node
      for node in ast.walk(controller_tree)
      if (
        isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name in (
          "update_jeep_radar_shadow",
          "log_jeep_radar_shadow",
        )
      )
    ]
    self.assertEqual(len(diagnostic_methods), 2)
    for method in diagnostic_methods:
      referenced_names = {
        node.id
        for node in ast.walk(method)
        if isinstance(node, ast.Name)
      }
      self.assertTrue(
        {"can_sends", "new_actuators", "apply_steer"}.isdisjoint(
          referenced_names,
        ),
      )

  def test_b6k_recovery_keeps_longitudinal_transport_compiled_off(self):
    long_source = LONG_PATH.read_text(encoding="utf-8")
    self.assertIn(
      "JEEP_LONG_SHADOW_TRANSPORT_COMPILED = False",
      long_source,
    )
    self.assertIn(
      "JEEP_LONG_ACTUATION_COMPILED = False",
      long_source,
    )

    controller_source = CARCONTROLLER_PATH.read_text(encoding="utf-8")
    self.assertIn(
      "if transport_counter is not None:",
      controller_source,
    )
    self.assertIn(
      "can_sends.extend(self.jeep_long_shadow_frames)",
      controller_source,
    )


if __name__ == "__main__":
  unittest.main()

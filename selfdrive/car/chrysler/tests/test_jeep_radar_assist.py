#!/usr/bin/env python3
import unittest

from openpilot.selfdrive.car.chrysler.jeep_radar_assist import (
  JeepRadarLongitudinalAssist,
  LAUNCH_ACCEL_MPS2,
)
from openpilot.selfdrive.car.chrysler.jeep_radar_shadow import (
  JeepRadarSelection,
  JeepRadarTrack,
  JeepVisionLead,
)
from openpilot.selfdrive.car.chrysler.jeep_longitudinal import JeepLongitudinalShadow


class TestJeepRadarLongitudinalAssist(unittest.TestCase):
  @staticmethod
  def selected(distance, radar_v_rel, vision_v_rel=None, probability=0.99):
    vision_v_rel = radar_v_rel if vision_v_rel is None else vision_v_rel
    track = JeepRadarTrack(2, distance, radar_v_rel)
    return (
      JeepRadarSelection(track, "selected", 0.1),
      JeepVisionLead(True, distance, vision_v_rel, probability),
    )

  @staticmethod
  def update(assist, cycle, selection, vision, **kwargs):
    return assist.update(
      planner_accel_mps2=kwargs.get("planner_accel_mps2", 0.4),
      speed_mps=kwargs.get("speed_mps", 25.0),
      eligible=kwargs.get("eligible", True),
      selection=selection,
      vision=vision,
      radar_cycle=cycle,
      now_nanos=1_000_000_000 + cycle * 50_000_000,
    )

  def test_ambiguous_or_radar_only_target_cannot_act(self):
    assist = JeepRadarLongitudinalAssist()
    vision = JeepVisionLead(True, 40.0, -5.0, 0.99)
    for cycle in range(1, 6):
      result = self.update(
        assist,
        cycle,
        JeepRadarSelection(None, "ambiguous_acquisition", 0.1),
        vision,
      )
    self.assertFalse(result.active)
    self.assertEqual(result.requested_accel_mps2, 0.4)

  def test_three_stable_matches_can_strengthen_braking(self):
    assist = JeepRadarLongitudinalAssist()
    selection, vision = self.selected(55.0, -7.0, -4.5)
    first = self.update(assist, 1, selection, vision)
    second = self.update(assist, 2, selection, vision)
    third = self.update(assist, 3, selection, vision)
    self.assertFalse(first.active)
    self.assertFalse(second.active)
    self.assertTrue(third.active)
    self.assertEqual(third.mode, "matched_lead_brake")
    self.assertLess(third.requested_accel_mps2, 0.0)
    self.assertGreaterEqual(third.requested_accel_mps2, -2.5)

  def test_planner_keeps_more_conservative_request(self):
    assist = JeepRadarLongitudinalAssist()
    selection, vision = self.selected(80.0, -2.0, -2.0)
    for cycle in range(1, 4):
      result = self.update(
        assist,
        cycle,
        selection,
        vision,
        planner_accel_mps2=-2.0,
      )
    self.assertFalse(result.active)
    self.assertEqual(result.requested_accel_mps2, -2.0)

  def test_matched_moving_lead_triggers_bounded_launch(self):
    assist = JeepRadarLongitudinalAssist()
    selection, vision = self.selected(18.0, 1.5, 1.0)
    for cycle in range(1, 5):
      result = self.update(
        assist,
        cycle,
        selection,
        vision,
        planner_accel_mps2=-2.0,
        speed_mps=0.0,
      )
    self.assertTrue(result.active)
    self.assertEqual(result.mode, "matched_lead_launch")
    self.assertEqual(result.requested_accel_mps2, LAUNCH_ACCEL_MPS2)

  def test_launch_requires_both_vision_and_radar_motion(self):
    assist = JeepRadarLongitudinalAssist()
    selection, vision = self.selected(18.0, 1.5, 0.0)
    for cycle in range(1, 7):
      result = self.update(
        assist,
        cycle,
        selection,
        vision,
        planner_accel_mps2=-2.0,
        speed_mps=0.0,
      )
    self.assertFalse(result.active)
    self.assertEqual(result.requested_accel_mps2, -2.0)

  def test_stale_match_fails_back_to_planner(self):
    assist = JeepRadarLongitudinalAssist()
    selection, vision = self.selected(55.0, -7.0, -4.5)
    for cycle in range(1, 4):
      self.update(assist, cycle, selection, vision)
    result = assist.update(
      planner_accel_mps2=0.4,
      speed_mps=25.0,
      eligible=True,
      selection=selection,
      vision=vision,
      radar_cycle=3,
      now_nanos=2_000_000_000,
    )
    self.assertFalse(result.active)
    self.assertEqual(result.mode, "no_fresh_match")
    self.assertEqual(result.requested_accel_mps2, 0.4)

  def test_matched_lead_launch_reaches_guarded_go_without_resume_button(self):
    assist = JeepRadarLongitudinalAssist()
    envelope_generator = JeepLongitudinalShadow()
    selection, vision = self.selected(18.0, 1.5, 1.0)
    for _ in range(100):
      envelope_generator.update(-2.0, eligible=True, speed_mps=0.0)

    saw_go = False
    saw_creep_torque = False
    cycle = 0
    for frame in range(250):
      if frame % 2 == 0:
        cycle += 1
      result = assist.update(
        planner_accel_mps2=-2.0,
        speed_mps=0.0,
        eligible=True,
        selection=selection,
        vision=vision,
        radar_cycle=cycle,
        now_nanos=1_000_000_000 + frame * 20_000_000,
      )
      envelope = envelope_generator.update(
        result.requested_accel_mps2,
        eligible=True,
        speed_mps=0.0,
      )
      envelope_generator.note_transport_sent(envelope)
      self.assertFalse(envelope.brake_active and envelope.engine_active)
      saw_go |= envelope.go_request
      saw_creep_torque |= (
        envelope.low_speed_state == "creep" and envelope.engine_active
      )
      if saw_go and saw_creep_torque:
        break

    self.assertTrue(saw_go)
    self.assertTrue(saw_creep_torque)


if __name__ == "__main__":
  unittest.main()

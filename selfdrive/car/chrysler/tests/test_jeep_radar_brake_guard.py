import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest


CHRYSLER_PATH = Path(__file__).resolve().parents[1]
GUARD_PATH = CHRYSLER_PATH / "jeep_radar_brake_guard.py"
CONTROLLER_PATH = CHRYSLER_PATH / "carcontroller.py"

GUARD_SPEC = importlib.util.spec_from_file_location(
  "jeep_radar_brake_guard_under_test",
  GUARD_PATH,
)
assert GUARD_SPEC is not None and GUARD_SPEC.loader is not None
GUARD = importlib.util.module_from_spec(GUARD_SPEC)
sys.modules[GUARD_SPEC.name] = GUARD
GUARD_SPEC.loader.exec_module(GUARD)


BASE_NS = 1_000_000_000
RADAR_DT_NS = 50_000_000
RADAR_DT_S = RADAR_DT_NS / 1e9


def make_track(index=2, d_rel=30.0, v_rel=-4.0):
  return SimpleNamespace(index=index, d_rel=d_rel, v_rel=v_rel)


def make_vision(d_rel=30.0, v_rel=-4.0, probability=0.99, *, status=True):
  return SimpleNamespace(
    status=status,
    d_rel=d_rel,
    v_rel=v_rel,
    model_prob=probability,
    radar=False,
  )


def selected(track):
  return SimpleNamespace(track=track, reason="selected", score=0.0)


def no_selection(reason="vision_not_eligible"):
  return SimpleNamespace(track=None, reason=reason, score=None)


def absent_vision():
  return make_vision(0.0, 0.0, 0.0, status=False)


def update_guard(
    guard,
    *,
    now_nanos,
    radar_cycle,
    track=None,
    vision=None,
    selection=None,
    eligible=True,
    cycle_complete=True,
    planner_decelerating=True,
    v_ego=10.0,
    tracks=None,
):
  if tracks is None:
    tracks = () if track is None else (track,)
  if vision is None:
    vision = absent_vision()
  if selection is None:
    selection = no_selection()
  return guard.update(
    eligible=eligible,
    v_ego=v_ego,
    now_nanos=now_nanos,
    radar_cycle=radar_cycle,
    cycle_complete=cycle_complete,
    tracks=tracks,
    vision=vision,
    selection=selection,
    planner_decelerating=planner_decelerating,
  )


def seed_fused_track(
    guard,
    *,
    cycles=GUARD.CONFIRM_CYCLES,
    start_ns=BASE_NS,
    start_distance=30.0,
    v_rel=-4.0,
    v_ego=10.0,
    index=2,
    radar_range_bias=0.0,
):
  result = None
  track = None
  now_nanos = start_ns
  for step in range(cycles):
    now_nanos = start_ns + step * RADAR_DT_NS
    radar_d = start_distance + v_rel * step * RADAR_DT_S
    track = make_track(index, radar_d, v_rel)
    vision = make_vision(radar_d - radar_range_bias, v_rel)
    result = update_guard(
      guard,
      now_nanos=now_nanos,
      radar_cycle=step + 1,
      track=track,
      vision=vision,
      selection=selected(track),
      v_ego=v_ego,
    )
  assert result is not None and track is not None
  return SimpleNamespace(
    result=result,
    now_nanos=now_nanos,
    radar_cycle=cycles,
    track=track,
    v_ego=v_ego,
    v_rel=v_rel,
  )


class TestJeepRadarBrakeContinuityGuard(unittest.TestCase):
  def test_confirmation_requires_distinct_clean_radar_cycles(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard, cycles=GUARD.CONFIRM_CYCLES - 1)
    self.assertFalse(seed.result.confirmed)

    dropout_track = make_track(
      seed.track.index,
      seed.track.d_rel + seed.v_rel * RADAR_DT_S,
      seed.v_rel,
    )
    result = update_guard(
      guard,
      now_nanos=seed.now_nanos + RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 1,
      track=dropout_track,
    )
    self.assertEqual(result.reason, "fusion_not_confirmed")
    self.assertFalse(result.active)
    self.assertIsNone(result.brake_floor_mps2)

    guard = GUARD.JeepRadarBrakeContinuityGuard()
    track = make_track()
    first = update_guard(
      guard,
      now_nanos=BASE_NS,
      radar_cycle=1,
      track=track,
      vision=make_vision(),
      selection=selected(track),
    )
    self.assertEqual(first.reason, "confirming_fusion")
    for repeat in range(1, GUARD.CONFIRM_CYCLES + 1):
      repeated = update_guard(
        guard,
        now_nanos=BASE_NS + repeat * 5_000_000,
        radar_cycle=1,
        track=track,
        vision=make_vision(),
        selection=selected(track),
      )
      self.assertFalse(repeated.confirmed)

  def test_required_clean_cycles_seed_but_do_not_brake_until_dropout(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard)
    self.assertEqual(seed.result.reason, "fused_confirmed")
    self.assertTrue(seed.result.confirmed)
    self.assertFalse(seed.result.active)
    self.assertIsNone(seed.result.brake_floor_mps2)

    track = make_track(
      seed.track.index,
      seed.track.d_rel + seed.v_rel * RADAR_DT_S,
      seed.v_rel,
    )
    dropout = update_guard(
      guard,
      now_nanos=seed.now_nanos + RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 1,
      track=track,
      # A low-confidence outward jump is not a credible replacement object.
      vision=make_vision(78.0, seed.v_rel, probability=0.69),
      selection=no_selection("no_gated_candidate"),
    )
    self.assertEqual(dropout.reason, "dropout_continuity")
    self.assertTrue(dropout.active)
    self.assertLess(dropout.brake_floor_mps2, 0.0)

  def test_route8_shaped_three_second_dropout_remains_braking(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(
      guard,
      start_distance=29.0,
      v_rel=-3.6,
      v_ego=10.0,
    )
    dropout_start_ns = seed.now_nanos + RADAR_DT_NS
    previous_floor = None
    result = None

    # 62 samples span 3.05 seconds from the first lost association.  The first
    # sample reproduces the outward vision jump; the remaining samples model
    # the production lead disappearing while the same raw target closes.
    for step in range(62):
      now_nanos = dropout_start_ns + step * RADAR_DT_NS
      elapsed_since_seed_s = (now_nanos - seed.now_nanos) / 1e9
      radar_d = seed.track.d_rel + seed.v_rel * elapsed_since_seed_s
      track = make_track(seed.track.index, radar_d, seed.v_rel)
      vision = (
        make_vision(78.0, seed.v_rel, probability=0.69)
        if step == 0 else absent_vision()
      )
      selection = no_selection(
        "no_gated_candidate" if step == 0 else "vision_not_eligible",
      )
      result = update_guard(
        guard,
        now_nanos=now_nanos,
        radar_cycle=seed.radar_cycle + 1 + step,
        track=track,
        vision=vision,
        selection=selection,
        planner_decelerating=step == 0,
        v_ego=seed.v_ego,
      )
      self.assertTrue(result.active)
      self.assertEqual(result.reason, "dropout_continuity")
      self.assertIsNotNone(result.brake_floor_mps2)
      self.assertLessEqual(result.brake_floor_mps2, 0.0)
      if previous_floor is not None:
        self.assertLessEqual(result.brake_floor_mps2, previous_floor + 1e-9)
      previous_floor = result.brake_floor_mps2

    self.assertIsNotNone(result)
    self.assertAlmostEqual(result.dropout_age_s, 3.05, places=6)
    self.assertLess(result.d_rel, 18.0)

  def test_dropout_and_raw_freshness_boundaries_expire_fail_closed(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard, start_distance=34.0, v_rel=-3.0)
    dropout_ns = seed.now_nanos + RADAR_DT_NS
    dropout_d = seed.track.d_rel + seed.v_rel * RADAR_DT_S
    dropout_track = make_track(seed.track.index, dropout_d, seed.v_rel)
    first = update_guard(
      guard,
      now_nanos=dropout_ns,
      radar_cycle=seed.radar_cycle + 1,
      track=dropout_track,
    )
    self.assertTrue(first.active)

    # The hard cap is measured from the last independently supported cycle,
    # not from the first radar-only cycle.
    exact_timeout_ns = seed.now_nanos + GUARD.DROPOUT_TIMEOUT_NS
    exact = first
    cycle = seed.radar_cycle + 1
    now_nanos = dropout_ns
    current_d = dropout_d
    while now_nanos < exact_timeout_ns:
      step_ns = min(RADAR_DT_NS, exact_timeout_ns - now_nanos)
      now_nanos += step_ns
      current_d += seed.v_rel * (step_ns / 1e9)
      cycle += 1
      exact = update_guard(
        guard,
        now_nanos=now_nanos,
        radar_cycle=cycle,
        track=make_track(seed.track.index, current_d, seed.v_rel),
      )
    self.assertTrue(exact.active)
    expired = update_guard(
      guard,
      now_nanos=exact_timeout_ns + 1,
      radar_cycle=cycle + 1,
      track=make_track(
        seed.track.index,
        current_d + seed.v_rel / 1e9,
        seed.v_rel,
      ),
    )
    self.assertEqual(expired.reason, "dropout_timeout")
    self.assertFalse(expired.active)
    self.assertIsNone(expired.brake_floor_mps2)

    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard)
    dropout_ns = seed.now_nanos + RADAR_DT_NS
    track = make_track(
      seed.track.index,
      seed.track.d_rel + seed.v_rel * RADAR_DT_S,
      seed.v_rel,
    )
    update_guard(
      guard,
      now_nanos=dropout_ns,
      radar_cycle=seed.radar_cycle + 1,
      track=track,
    )
    exact_fresh = update_guard(
      guard,
      now_nanos=dropout_ns + GUARD.RAW_STALE_NS,
      radar_cycle=seed.radar_cycle + 1,
      track=track,
    )
    self.assertTrue(exact_fresh.active)
    stale = update_guard(
      guard,
      now_nanos=dropout_ns + GUARD.RAW_STALE_NS + 1,
      radar_cycle=seed.radar_cycle + 1,
      track=track,
    )
    self.assertEqual(stale.reason, "radar_stale")
    self.assertFalse(stale.active)

  def test_absolute_authority_timeout_requires_fresh_strong_confirmation(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard, start_distance=30.0, v_rel=-3.0)
    active_start_ns = seed.now_nanos + RADAR_DT_NS
    cycle = seed.radar_cycle + 1
    current_d = seed.track.d_rel + seed.v_rel * RADAR_DT_S

    first = update_guard(
      guard,
      now_nanos=active_start_ns,
      radar_cycle=cycle,
      track=make_track(seed.track.index, current_d, seed.v_rel),
      vision=make_vision(current_d, seed.v_rel, probability=0.60),
      selection=no_selection("no_gated_candidate"),
    )
    self.assertTrue(first.active)

    exact_timeout_ns = active_start_ns + GUARD.ABSOLUTE_ACTIVE_TIMEOUT_NS
    result = first
    now_nanos = active_start_ns
    while now_nanos + RADAR_DT_NS < exact_timeout_ns:
      now_nanos += RADAR_DT_NS
      cycle += 1
      current_d += seed.v_rel * RADAR_DT_S
      result = update_guard(
        guard,
        now_nanos=now_nanos,
        radar_cycle=cycle,
        track=make_track(seed.track.index, current_d, seed.v_rel),
        vision=make_vision(current_d, seed.v_rel, probability=0.60),
        selection=no_selection("no_gated_candidate"),
        planner_decelerating=False,
      )
      self.assertTrue(result.active)
      self.assertIsNotNone(result.brake_floor_mps2)

    self.assertLess(now_nanos, exact_timeout_ns)
    cycle += 1
    current_d += seed.v_rel * (
      (exact_timeout_ns - now_nanos) / 1e9
    )
    expired = update_guard(
      guard,
      now_nanos=exact_timeout_ns,
      radar_cycle=cycle,
      track=make_track(seed.track.index, current_d, seed.v_rel),
      vision=make_vision(current_d, seed.v_rel, probability=0.60),
      selection=no_selection("no_gated_candidate"),
      planner_decelerating=False,
    )
    self.assertEqual(expired.reason, "absolute_active_timeout")
    self.assertFalse(expired.active)
    self.assertFalse(expired.confirmed)
    self.assertIsNone(expired.brake_floor_mps2)

    raw_only = update_guard(
      guard,
      now_nanos=exact_timeout_ns + RADAR_DT_NS,
      radar_cycle=cycle + 1,
      track=make_track(
        seed.track.index,
        current_d + seed.v_rel * RADAR_DT_S,
        seed.v_rel,
      ),
      planner_decelerating=True,
    )
    self.assertEqual(raw_only.reason, "fusion_not_confirmed")
    self.assertFalse(raw_only.active)
    self.assertIsNone(raw_only.brake_floor_mps2)

  def test_incomplete_missing_invalid_and_jumping_tracks_revoke(self):
    cases = (
      (
        "incomplete_cycle",
        {"cycle_complete": False, "tracks": ()},
        "incomplete_radar_cycle",
      ),
      (
        "missing_confirmed_slot",
        {"tracks": (make_track(7, 25.0, -4.0),)},
        "track_discontinuity",
      ),
      (
        "nonfinite_track",
        {"track": make_track(2, float("nan"), -4.0)},
        "track_discontinuity",
      ),
      (
        "range_jump",
        {"track": make_track(2, 50.0, -4.0)},
        "track_discontinuity",
      ),
      (
        "speed_jump",
        {"track": make_track(2, 29.8, 3.0)},
        "track_discontinuity",
      ),
    )
    for name, overrides, expected_reason in cases:
      with self.subTest(name=name):
        guard = GUARD.JeepRadarBrakeContinuityGuard()
        seed = seed_fused_track(guard)
        default_track = make_track(
          seed.track.index,
          seed.track.d_rel + seed.v_rel * RADAR_DT_S,
          seed.v_rel,
        )
        kwargs = {
          "track": default_track,
          "tracks": (default_track,),
          "cycle_complete": True,
        }
        kwargs.update(overrides)
        if "track" in overrides and "tracks" not in overrides:
          kwargs["tracks"] = (kwargs["track"],)
        result = update_guard(
          guard,
          now_nanos=seed.now_nanos + RADAR_DT_NS,
          radar_cycle=seed.radar_cycle + 1,
          **kwargs,
        )
        self.assertEqual(result.reason, expected_reason)
        self.assertFalse(result.active)
        self.assertFalse(result.confirmed)
        self.assertIsNone(result.brake_floor_mps2)

  def test_unique_kinematic_match_survives_raw_radar_slot_hops(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard, index=0)

    first_d = seed.track.d_rel + seed.v_rel * RADAR_DT_S
    first_hop = make_track(1, first_d, seed.v_rel)
    result = update_guard(
      guard,
      now_nanos=seed.now_nanos + RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 1,
      track=first_hop,
      tracks=(first_hop,),
    )
    self.assertTrue(result.active)
    self.assertEqual(result.reason, "dropout_continuity")
    self.assertEqual(result.track_index, 1)

    second_hop = make_track(
      0,
      first_hop.d_rel + first_hop.v_rel * RADAR_DT_S,
      first_hop.v_rel,
    )
    result = update_guard(
      guard,
      now_nanos=seed.now_nanos + 2 * RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 2,
      track=second_hop,
      tracks=(second_hop,),
      planner_decelerating=False,
    )
    self.assertTrue(result.active)
    self.assertEqual(result.reason, "dropout_continuity")
    self.assertEqual(result.track_index, 0)

  def test_ambiguous_kinematic_reassociation_resets(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard)
    predicted_d = seed.track.d_rel + seed.v_rel * RADAR_DT_S
    candidates = (
      make_track(0, predicted_d - 0.05, seed.v_rel),
      make_track(1, predicted_d + 0.05, seed.v_rel),
    )
    result = update_guard(
      guard,
      now_nanos=seed.now_nanos + RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 1,
      tracks=candidates,
    )
    self.assertEqual(result.reason, "ambiguous_track_continuity")
    self.assertFalse(result.active)
    self.assertFalse(result.confirmed)
    self.assertIsNone(result.brake_floor_mps2)

  def test_close_stationary_track_loss_freezes_but_never_strengthens_floor(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(
      guard,
      start_distance=17.0,
      v_rel=-8.0,
      v_ego=10.0,
    )
    dropout_track = make_track(
      seed.track.index,
      seed.track.d_rel + seed.v_rel * RADAR_DT_S,
      seed.v_rel,
    )
    active = update_guard(
      guard,
      now_nanos=seed.now_nanos + RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 1,
      track=dropout_track,
    )
    self.assertTrue(active.active)
    self.assertIsNotNone(active.brake_floor_mps2)

    lost_ns = seed.now_nanos + 2 * RADAR_DT_NS
    lost = update_guard(
      guard,
      now_nanos=lost_ns,
      radar_cycle=seed.radar_cycle + 2,
      tracks=(),
      cycle_complete=True,
      planner_decelerating=False,
    )
    self.assertEqual(lost.reason, "bounded_stale_brake_hold")
    self.assertTrue(lost.active)
    self.assertEqual(lost.brake_floor_mps2, active.brake_floor_mps2)

    # Empty but timely complete cycles prove the radar transport is alive. The
    # frozen value may persist, but it must never become stronger.
    held = lost
    cycle = seed.radar_cycle + 2
    now_nanos = lost_ns
    support_expiry_ns = seed.now_nanos + GUARD.DROPOUT_TIMEOUT_NS
    while now_nanos + RADAR_DT_NS <= support_expiry_ns:
      now_nanos += RADAR_DT_NS
      cycle += 1
      held = update_guard(
        guard,
        now_nanos=now_nanos,
        radar_cycle=cycle,
        tracks=(),
        planner_decelerating=False,
      )
      self.assertEqual(held.reason, "bounded_stale_brake_hold")
      self.assertEqual(held.brake_floor_mps2, active.brake_floor_mps2)

    expired = update_guard(
      guard,
      now_nanos=support_expiry_ns + 1,
      radar_cycle=cycle + 1,
      tracks=(),
      planner_decelerating=False,
    )
    self.assertEqual(expired.reason, "dropout_timeout")
    self.assertFalse(expired.active)
    self.assertIsNone(expired.brake_floor_mps2)

  def test_dead_radar_bus_revokes_bounded_stale_hold(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(
      guard,
      start_distance=17.0,
      v_rel=-8.0,
      v_ego=10.0,
    )
    dropout_ns = seed.now_nanos + RADAR_DT_NS
    dropout_track = make_track(
      seed.track.index,
      seed.track.d_rel + seed.v_rel * RADAR_DT_S,
      seed.v_rel,
    )
    update_guard(
      guard,
      now_nanos=dropout_ns,
      radar_cycle=seed.radar_cycle + 1,
      track=dropout_track,
    )
    lost_ns = dropout_ns + RADAR_DT_NS
    frozen = update_guard(
      guard,
      now_nanos=lost_ns,
      radar_cycle=seed.radar_cycle + 2,
      tracks=(),
      planner_decelerating=False,
    )
    self.assertEqual(frozen.reason, "bounded_stale_brake_hold")

    dead_bus = update_guard(
      guard,
      now_nanos=lost_ns + GUARD.RAW_STALE_NS + 1,
      radar_cycle=seed.radar_cycle + 2,
      tracks=(),
      planner_decelerating=False,
    )
    self.assertEqual(dead_bus.reason, "radar_stale")
    self.assertFalse(dead_bus.active)
    self.assertFalse(dead_bus.confirmed)
    self.assertIsNone(dead_bus.brake_floor_mps2)

  def test_new_cycle_after_transport_gap_cannot_continue_confirmation(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    first_track = make_track()
    first = update_guard(
      guard,
      now_nanos=BASE_NS,
      radar_cycle=1,
      track=first_track,
      vision=make_vision(),
      selection=selected(first_track),
    )
    self.assertEqual(first.reason, "confirming_fusion")

    gap_ns = GUARD.RAW_STALE_NS + 1
    next_track = make_track(
      first_track.index,
      first_track.d_rel + first_track.v_rel * (gap_ns / 1e9),
      first_track.v_rel,
    )
    delayed = update_guard(
      guard,
      now_nanos=BASE_NS + gap_ns,
      radar_cycle=2,
      track=next_track,
      vision=make_vision(next_track.d_rel, next_track.v_rel),
      selection=selected(next_track),
    )
    self.assertEqual(delayed.reason, "radar_cycle_gap")
    self.assertFalse(delayed.active)
    self.assertFalse(delayed.confirmed)

  def test_ineligible_cycle_snapshot_cannot_confirm_after_reenable(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    track = make_track()
    ineligible = update_guard(
      guard,
      now_nanos=BASE_NS,
      radar_cycle=1,
      track=track,
      vision=make_vision(),
      selection=selected(track),
      eligible=False,
    )
    self.assertEqual(ineligible.reason, "ineligible")

    same_snapshot = update_guard(
      guard,
      now_nanos=BASE_NS + 10_000_000,
      radar_cycle=1,
      track=track,
      vision=make_vision(),
      selection=selected(track),
      eligible=True,
    )
    self.assertEqual(same_snapshot.reason, "awaiting_radar_cycle")
    self.assertFalse(same_snapshot.confirmed)

    result = same_snapshot
    for step in range(GUARD.CONFIRM_CYCLES):
      now_nanos = BASE_NS + (step + 1) * RADAR_DT_NS
      d_rel = track.d_rel + track.v_rel * ((step + 1) * RADAR_DT_S)
      fresh_track = make_track(track.index, d_rel, track.v_rel)
      result = update_guard(
        guard,
        now_nanos=now_nanos,
        radar_cycle=step + 2,
        track=fresh_track,
        vision=make_vision(d_rel, track.v_rel),
        selection=selected(fresh_track),
      )
      if step < GUARD.CONFIRM_CYCLES - 1:
        self.assertFalse(result.confirmed)
    self.assertEqual(result.reason, "fused_confirmed")
    self.assertTrue(result.confirmed)

  def test_weak_matching_vision_can_refresh_but_cannot_cold_acquire(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    weak_track = make_track()
    weak_vision = make_vision(
      weak_track.d_rel,
      weak_track.v_rel,
      probability=GUARD.MIN_SUPPORT_MODEL_PROB,
    )
    for cycle in range(1, GUARD.CONFIRM_CYCLES + 2):
      result = update_guard(
        guard,
        now_nanos=BASE_NS + (cycle - 1) * RADAR_DT_NS,
        radar_cycle=cycle,
        track=weak_track,
        vision=weak_vision,
        selection=no_selection("no_gated_candidate"),
      )
    self.assertEqual(result.reason, "fusion_not_confirmed")
    self.assertFalse(result.confirmed)

    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard)
    supported_ns = seed.now_nanos + RADAR_DT_NS
    supported_track = make_track(
      seed.track.index,
      seed.track.d_rel + seed.v_rel * RADAR_DT_S,
      seed.v_rel,
    )
    supported = update_guard(
      guard,
      now_nanos=supported_ns,
      radar_cycle=seed.radar_cycle + 1,
      track=supported_track,
      vision=make_vision(
        supported_track.d_rel,
        supported_track.v_rel,
        probability=GUARD.MIN_SUPPORT_MODEL_PROB,
      ),
      selection=no_selection("no_gated_candidate"),
    )
    self.assertTrue(supported.active)
    self.assertEqual(supported.reason, "dropout_continuity")

  def test_eligible_weakly_supported_vision_stays_passive_until_dropout(self):
    self.assertEqual(GUARD.CONFIRM_CYCLES, 3)
    self.assertEqual(GUARD.MAX_SUPPORT_DISTANCE_ERROR_M, 4.5)

    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard)

    # This is outside the strong 3 m fusion tolerance but inside the 4.5 m
    # preservation tolerance needed for subscriber-pairing skew.  The
    # production vision lead remains eligible, so it must never start or
    # sustain the brake-only dropout floor.
    track = seed.track
    now_nanos = seed.now_nanos
    radar_cycle = seed.radar_cycle
    for _ in range(2):
      now_nanos += RADAR_DT_NS
      radar_cycle += 1
      track = make_track(
        track.index,
        track.d_rel + track.v_rel * RADAR_DT_S,
        track.v_rel,
      )
      healthy = update_guard(
        guard,
        now_nanos=now_nanos,
        radar_cycle=radar_cycle,
        track=track,
        vision=make_vision(
          track.d_rel - 4.4,
          track.v_rel,
          probability=0.92,
        ),
        selection=no_selection("no_gated_candidate"),
      )
      self.assertEqual(healthy.reason, "vision_supported_tracking")
      self.assertTrue(healthy.confirmed)
      self.assertFalse(healthy.active)
      self.assertIsNone(healthy.brake_floor_mps2)

    # Once the production lead really drops below eligibility, the remembered
    # raw track may provide continuity, subject to the existing braking gates.
    now_nanos += RADAR_DT_NS
    radar_cycle += 1
    track = make_track(
      track.index,
      track.d_rel + track.v_rel * RADAR_DT_S,
      track.v_rel,
    )
    dropout = update_guard(
      guard,
      now_nanos=now_nanos,
      radar_cycle=radar_cycle,
      track=track,
      vision=absent_vision(),
      selection=no_selection(),
    )
    self.assertEqual(dropout.reason, "dropout_continuity")
    self.assertTrue(dropout.active)
    self.assertIsNotNone(dropout.brake_floor_mps2)
    self.assertLessEqual(dropout.brake_floor_mps2, 0.0)

    # A weak-but-eligible production lead recovery must also remove an
    # already-running dropout floor immediately.
    now_nanos += RADAR_DT_NS
    radar_cycle += 1
    track = make_track(
      track.index,
      track.d_rel + track.v_rel * RADAR_DT_S,
      track.v_rel,
    )
    recovered = update_guard(
      guard,
      now_nanos=now_nanos,
      radar_cycle=radar_cycle,
      track=track,
      vision=make_vision(
        track.d_rel - 4.4,
        track.v_rel,
        probability=0.92,
      ),
      selection=no_selection("no_gated_candidate"),
    )
    self.assertEqual(recovered.reason, "vision_supported_tracking")
    self.assertFalse(recovered.active)
    self.assertIsNone(recovered.brake_floor_mps2)

  def test_credible_disagreeing_vision_replaces_dropout_hypothesis(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard)
    track = make_track(
      seed.track.index,
      seed.track.d_rel + seed.v_rel * RADAR_DT_S,
      seed.v_rel,
    )
    result = update_guard(
      guard,
      now_nanos=seed.now_nanos + RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 1,
      track=track,
      vision=make_vision(78.0, seed.v_rel, probability=0.95),
      selection=no_selection("no_gated_candidate"),
    )
    self.assertEqual(result.reason, "vision_disagreement")
    self.assertFalse(result.active)
    self.assertFalse(result.confirmed)
    self.assertIsNone(result.brake_floor_mps2)

  def test_guard_cannot_relax_braking_or_request_acceleration(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard)
    track = make_track(
      seed.track.index,
      seed.track.d_rel + seed.v_rel * RADAR_DT_S,
      seed.v_rel,
    )
    result = update_guard(
      guard,
      now_nanos=seed.now_nanos + RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 1,
      track=track,
    )
    self.assertIsNotNone(result.brake_floor_mps2)
    self.assertLessEqual(result.brake_floor_mps2, 0.0)
    self.assertEqual(min(-2.0, result.brake_floor_mps2), -2.0)
    self.assertLess(min(1.0, result.brake_floor_mps2), 0.0)

    # A lead that was only mildly closing may seed the history, but it cannot
    # enter radar-only continuity or produce propulsion.
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard, v_rel=-1.5)
    track = make_track(
      seed.track.index,
      seed.track.d_rel + seed.v_rel * RADAR_DT_S,
      seed.v_rel,
    )
    result = update_guard(
      guard,
      now_nanos=seed.now_nanos + RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 1,
      track=track,
      planner_decelerating=True,
    )
    self.assertEqual(result.reason, "dropout_not_braking")
    self.assertFalse(result.active)
    self.assertIsNone(result.brake_floor_mps2)

  def test_dropout_requires_existing_planner_braking(self):
    for planner_decelerating, expected_active in (
        (False, False),
        (True, True),
    ):
      with self.subTest(
          planner_decelerating=planner_decelerating,
      ):
        guard = GUARD.JeepRadarBrakeContinuityGuard()
        seed = seed_fused_track(guard)
        track = make_track(
          seed.track.index,
          seed.track.d_rel + seed.v_rel * RADAR_DT_S,
          seed.v_rel,
        )
        result = update_guard(
          guard,
          now_nanos=seed.now_nanos + RADAR_DT_NS,
          radar_cycle=seed.radar_cycle + 1,
          track=track,
          planner_decelerating=planner_decelerating,
        )
        self.assertEqual(result.active, expected_active)
        self.assertEqual(
          result.reason,
          "dropout_continuity" if expected_active else "dropout_not_braking",
        )

  def test_driver_or_disengagement_reset_requires_fresh_reconfirmation(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard)
    track = make_track(
      seed.track.index,
      seed.track.d_rel + seed.v_rel * RADAR_DT_S,
      seed.v_rel,
    )
    active = update_guard(
      guard,
      now_nanos=seed.now_nanos + RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 1,
      track=track,
    )
    self.assertTrue(active.active)

    reset = update_guard(
      guard,
      now_nanos=seed.now_nanos + RADAR_DT_NS + 10_000_000,
      radar_cycle=seed.radar_cycle + 1,
      track=track,
      eligible=False,
    )
    self.assertEqual(reset.reason, "ineligible")
    self.assertFalse(reset.active)
    self.assertFalse(reset.confirmed)

    raw_only = update_guard(
      guard,
      now_nanos=seed.now_nanos + 2 * RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 2,
      track=make_track(
        seed.track.index,
        track.d_rel + seed.v_rel * RADAR_DT_S,
        seed.v_rel,
      ),
      eligible=True,
      planner_decelerating=True,
    )
    self.assertEqual(raw_only.reason, "fusion_not_confirmed")
    self.assertFalse(raw_only.active)
    self.assertIsNone(raw_only.brake_floor_mps2)

  def test_live_reacquisition_ends_dropout_and_reconfirms_from_zero(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(guard)
    dropout_track = make_track(
      seed.track.index,
      seed.track.d_rel + seed.v_rel * RADAR_DT_S,
      seed.v_rel,
    )
    active = update_guard(
      guard,
      now_nanos=seed.now_nanos + RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 1,
      track=dropout_track,
    )
    self.assertTrue(active.active)

    recovered_track = make_track(
      seed.track.index,
      dropout_track.d_rel + seed.v_rel * RADAR_DT_S,
      seed.v_rel,
    )
    recovered = update_guard(
      guard,
      now_nanos=seed.now_nanos + 2 * RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 2,
      track=recovered_track,
      vision=make_vision(recovered_track.d_rel, seed.v_rel),
      selection=selected(recovered_track),
    )
    self.assertEqual(recovered.reason, "confirming_fusion")
    self.assertFalse(recovered.active)
    self.assertFalse(recovered.confirmed)
    self.assertIsNone(recovered.brake_floor_mps2)

  def test_confirmed_range_bias_is_applied_during_dropout(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    seed = seed_fused_track(
      guard,
      start_distance=32.0,
      radar_range_bias=2.0,
    )
    self.assertAlmostEqual(seed.result.range_bias_m, 2.0)
    raw_d = seed.track.d_rel + seed.v_rel * RADAR_DT_S
    track = make_track(seed.track.index, raw_d, seed.v_rel)
    result = update_guard(
      guard,
      now_nanos=seed.now_nanos + RADAR_DT_NS,
      radar_cycle=seed.radar_cycle + 1,
      track=track,
    )
    self.assertTrue(result.active)
    self.assertAlmostEqual(result.range_bias_m, 2.0)
    self.assertAlmostEqual(result.d_rel, raw_d - 2.0)

  def test_raw_radar_cannot_cold_acquire_and_controller_is_jeep_scoped(self):
    guard = GUARD.JeepRadarBrakeContinuityGuard()
    track = make_track()
    result = update_guard(
      guard,
      now_nanos=BASE_NS,
      radar_cycle=1,
      track=track,
      planner_decelerating=True,
    )
    self.assertEqual(result.reason, "fusion_not_confirmed")
    self.assertFalse(result.active)
    self.assertIsNone(result.brake_floor_mps2)

    controller_source = CONTROLLER_PATH.read_text(encoding="utf-8")
    self.assertIn(
      "self.jeep_radar_brake_guard = (\n"
      "      JeepRadarBrakeContinuityGuard()\n"
      "      if CP.carFingerprint in JEEP_LONG_CARS else None",
      controller_source,
    )
    self.assertIn("self.jeep_closing_brake_floor = dropout_floor", controller_source)
    self.assertNotIn("jeep_closing_brake_floor(\n", controller_source)
    self.assertIn("requested_accel = min(", controller_source)


if __name__ == "__main__":
  unittest.main()

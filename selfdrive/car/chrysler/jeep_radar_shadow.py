"""Passive Jeep radar/vision association for diagnostic logging only.

This module has no publisher, CAN packer, or control output. It decodes the
already-present bus-1 radar observations and compares range and relative speed
with openpilot's vision lead. The result is intentionally not a RadarData
message and cannot be consumed by planning or actuation.
"""

from dataclasses import dataclass
import math
from typing import Any, Iterable


RADAR_MSGS_C = tuple(range(0x2C2, 0x2D4 + 2, 2))
RADAR_MSGS_D = tuple(range(0x2A2, 0x2B4 + 2, 2))
RADAR_TRIGGER_MSG = RADAR_MSGS_C[-1]
RADAR_REQUIRED_MSGS = frozenset(RADAR_MSGS_C + RADAR_MSGS_D)

MIN_MODEL_PROB = 0.70
MIN_ABSOLUTE_TARGET_SPEED_MPS = -2.0
SPEED_GATE_MPS = 5.0
MAX_BASE_SCORE = 1.35
AMBIGUITY_SCORE_MARGIN = 0.08


@dataclass(frozen=True)
class JeepRadarTrack:
  index: int
  d_rel: float
  v_rel: float


@dataclass(frozen=True)
class JeepVisionLead:
  status: bool
  d_rel: float
  v_rel: float
  model_prob: float
  radar: bool = False

  @property
  def eligible(self) -> bool:
    return (
      self.status
      and not self.radar
      and math.isfinite(self.d_rel)
      and self.d_rel > 0.0
      and math.isfinite(self.v_rel)
      and math.isfinite(self.model_prob)
      and self.model_prob >= MIN_MODEL_PROB
    )


@dataclass(frozen=True)
class JeepRadarSelection:
  track: JeepRadarTrack | None
  reason: str
  score: float | None = None


class JeepRadarShadow:
  """Accumulate complete radar cycles and make fail-closed associations."""

  def __init__(self):
    self.tracks: tuple[JeepRadarTrack, ...] = ()
    self.cycle_count = 0
    self.complete_cycle_count = 0
    self.last_cycle_complete = False
    self._updated_messages: set[int] = set()

  def update_can(self, parser: Any, updated_messages: Iterable[int]) -> bool:
    """Read one complete 20-message cycle from a passive CANParser.

    Returns true when a trigger was observed, including an incomplete cycle.
    Incomplete cycles clear the track snapshot instead of reusing stale data.
    """
    updated = set(updated_messages)
    self._updated_messages.update(updated)
    if RADAR_TRIGGER_MSG not in updated:
      return False

    self.cycle_count += 1
    cycle_messages = self._updated_messages
    self._updated_messages = set()
    self.last_cycle_complete = RADAR_REQUIRED_MSGS.issubset(cycle_messages)
    if not self.last_cycle_complete:
      self.tracks = ()
      return True

    tracks: list[JeepRadarTrack] = []
    for index, (c_address, d_address) in enumerate(
        zip(RADAR_MSGS_C, RADAR_MSGS_D, strict=True),
    ):
      c_values = parser.vl[c_address]
      d_values = parser.vl[d_address]
      if "LONG_DIST" not in c_values or "REL_SPEED" not in d_values:
        self.tracks = ()
        self.last_cycle_complete = False
        return True

      d_rel = float(c_values["LONG_DIST"])
      v_rel = float(d_values["REL_SPEED"])
      if d_rel > 0.0 and math.isfinite(d_rel) and math.isfinite(v_rel):
        tracks.append(JeepRadarTrack(index, d_rel, v_rel))

    self.tracks = tuple(tracks)
    self.complete_cycle_count += 1
    return True

  def select(
      self,
      vision: JeepVisionLead,
      v_ego: float,
  ) -> JeepRadarSelection:
    """Select a radar observation only when vision independently supports it."""
    if not vision.eligible or not math.isfinite(v_ego):
      return JeepRadarSelection(None, "vision_not_eligible")

    distance_gate = max(5.0, abs(vision.d_rel) * 0.25)
    candidates: list[tuple[float, JeepRadarTrack]] = []
    for track in self.tracks:
      distance_error = abs(track.d_rel - vision.d_rel)
      speed_error = abs(track.v_rel - vision.v_rel)
      absolute_target_speed = v_ego + track.v_rel
      if (
        distance_error > distance_gate
        or speed_error > SPEED_GATE_MPS
        or absolute_target_speed < MIN_ABSOLUTE_TARGET_SPEED_MPS
      ):
        continue

      score = (
        distance_error / distance_gate
        + speed_error / SPEED_GATE_MPS
      )
      candidates.append((score, track))

    candidates.sort(key=lambda item: (item[0], item[1].d_rel))
    if not candidates:
      return JeepRadarSelection(None, "no_gated_candidate")

    best_score, selected = candidates[0]
    if best_score > MAX_BASE_SCORE:
      return JeepRadarSelection(None, "weak_candidate", best_score)

    if len(candidates) > 1:
      second_score, second = candidates[1]
      materially_different = (
        abs(selected.d_rel - second.d_rel) > 3.0
        or abs(selected.v_rel - second.v_rel) > 3.0
      )
      if (
        second_score - best_score < AMBIGUITY_SCORE_MARGIN
        and materially_different
      ):
        return JeepRadarSelection(
          None,
          "ambiguous_acquisition",
          best_score,
        )

    return JeepRadarSelection(selected, "selected", best_score)

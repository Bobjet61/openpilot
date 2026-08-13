"""Non-actuating single-owner longitudinal handoff model for the Jeep."""

from dataclasses import dataclass
from enum import IntEnum
import math


class LongOwner(IntEnum):
  STOCK = 0
  ARMING = 1
  OPENPILOT_SHADOW = 2
  REVOKING = 3
  BLOCKED = 4


@dataclass(frozen=True)
class SingleOwnerInput:
  monotonic_time_s: float
  requested: bool
  controls_enabled: bool
  stock_available: bool
  stock_isolated: bool
  isolation_healthy: bool
  stock_command_observed: bool
  stock_fault: bool
  collision: bool
  gas_pressed: bool
  brake_pressed: bool
  forward_gear: bool
  door_open: bool
  seatbelt_unlatched: bool
  speed_mps: float
  requested_accel_mps2: float


@dataclass(frozen=True)
class SingleOwnerResult:
  owner: LongOwner
  reason: str
  requested_accel_mps2: float
  would_brake: bool
  would_accelerate: bool
  actuation_allowed: bool = False


class JeepSingleOwnerShadow:
  """Models an exclusive stock-to-openpilot handoff without transmitting CAN."""

  ARM_CONFIRM_S = 1.0
  STOCK_QUIET_S = 1.0

  def __init__(self):
    self.owner = LongOwner.STOCK
    self.arm_started_s: float | None = None
    self.stock_quiet_started_s: float | None = None
    self.last_time_s: float | None = None
    self.rearm_latched = False

  def _reset_handoff(self):
    self.arm_started_s = None
    self.stock_quiet_started_s = None

  @staticmethod
  def _block_reason(sample: SingleOwnerInput) -> str | None:
    if not sample.requested:
      return "not_requested"
    if not sample.controls_enabled:
      return "controls_disabled"
    if not sample.stock_available:
      return "stock_unavailable"
    if sample.stock_fault:
      return "stock_fault"
    if sample.collision:
      return "collision"
    if sample.gas_pressed:
      return "driver_gas"
    if sample.brake_pressed:
      return "driver_brake"
    if not sample.forward_gear:
      return "not_forward_gear"
    if sample.door_open:
      return "door_open"
    if sample.seatbelt_unlatched:
      return "seatbelt_unlatched"
    if not sample.isolation_healthy:
      return "isolation_fault"
    if not sample.stock_isolated:
      return "stock_not_isolated"
    if not math.isfinite(sample.speed_mps):
      return "invalid_speed"
    if not math.isfinite(sample.requested_accel_mps2):
      return "invalid_accel"
    return None

  def update(self, sample: SingleOwnerInput) -> SingleOwnerResult:
    if (not math.isfinite(sample.monotonic_time_s) or
        (self.last_time_s is not None and sample.monotonic_time_s < self.last_time_s)):
      self.owner = LongOwner.BLOCKED
      self.rearm_latched = True
      self._reset_handoff()
      return SingleOwnerResult(self.owner, "invalid_time", 0.0, False, False)
    self.last_time_s = sample.monotonic_time_s

    blocked = self._block_reason(sample)
    if blocked is not None:
      self.owner = LongOwner.STOCK if blocked == "not_requested" else LongOwner.BLOCKED
      self._reset_handoff()
      if blocked in ("not_requested", "controls_disabled"):
        # A deliberate control reset is the only way to clear a latched
        # ownership, collision, stock-fault, or isolation fault.
        self.rearm_latched = False
      elif blocked in ("stock_fault", "collision", "isolation_fault"):
        self.rearm_latched = True
      return SingleOwnerResult(self.owner, blocked, 0.0, False, False)

    if self.rearm_latched:
      self.owner = LongOwner.BLOCKED
      self._reset_handoff()
      return SingleOwnerResult(self.owner, "control_reset_required", 0.0, False, False)

    if self.owner in (LongOwner.STOCK, LongOwner.BLOCKED):
      self.owner = LongOwner.ARMING
      self.arm_started_s = sample.monotonic_time_s
      self.stock_quiet_started_s = (
        sample.monotonic_time_s if not sample.stock_command_observed else None
      )
    elif self.owner == LongOwner.ARMING:
      if sample.stock_command_observed:
        self.stock_quiet_started_s = None
      elif self.stock_quiet_started_s is None:
        self.stock_quiet_started_s = sample.monotonic_time_s
      arm_start = self.arm_started_s if self.arm_started_s is not None else sample.monotonic_time_s
      quiet_start = self.stock_quiet_started_s if self.stock_quiet_started_s is not None else sample.monotonic_time_s
      arm_elapsed = sample.monotonic_time_s - arm_start
      quiet_elapsed = sample.monotonic_time_s - quiet_start
      if arm_elapsed >= self.ARM_CONFIRM_S and quiet_elapsed >= self.STOCK_QUIET_S:
        self.owner = LongOwner.OPENPILOT_SHADOW
    elif self.owner == LongOwner.OPENPILOT_SHADOW and sample.stock_command_observed:
      self.owner = LongOwner.REVOKING
      self.rearm_latched = True
      self._reset_handoff()
    elif self.owner == LongOwner.REVOKING:
      self.owner = LongOwner.BLOCKED

    shadow_output = self.owner == LongOwner.OPENPILOT_SHADOW
    accel = sample.requested_accel_mps2 if shadow_output else 0.0
    reason = {
      LongOwner.ARMING: "waiting_exclusive_handoff",
      LongOwner.OPENPILOT_SHADOW: "exclusive_shadow_owner",
      LongOwner.REVOKING: "stock_owner_reappeared",
      LongOwner.BLOCKED: "ownership_conflict",
      LongOwner.STOCK: "stock_owner",
    }[self.owner]
    return SingleOwnerResult(
      self.owner, reason, accel, accel < -0.05, accel > 0.05,
    )

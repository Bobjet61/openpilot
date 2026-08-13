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
  requested: bool
  controls_enabled: bool
  stock_available: bool
  stock_active: bool
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

  ARM_CONFIRM_SAMPLES = 20
  STOCK_QUIET_SAMPLES = 5
  REVOKE_CONFIRM_SAMPLES = 5

  def __init__(self):
    self.owner = LongOwner.STOCK
    self.arm_samples = 0
    self.stock_quiet_samples = 0
    self.revoke_samples = 0

  @staticmethod
  def _block_reason(sample: SingleOwnerInput) -> str | None:
    if not sample.requested:
      return "not_requested"
    if not sample.controls_enabled:
      return "controls_disabled"
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
    if not math.isfinite(sample.speed_mps):
      return "invalid_speed"
    if not math.isfinite(sample.requested_accel_mps2):
      return "invalid_accel"
    return None

  def update(self, sample: SingleOwnerInput) -> SingleOwnerResult:
    blocked = self._block_reason(sample)
    if blocked is not None:
      self.owner = LongOwner.STOCK if blocked == "not_requested" else LongOwner.BLOCKED
      self.arm_samples = 0
      self.stock_quiet_samples = 0
      self.revoke_samples = 0
      return SingleOwnerResult(self.owner, blocked, 0.0, False, False)

    if self.owner in (LongOwner.STOCK, LongOwner.BLOCKED):
      self.owner = LongOwner.ARMING
      self.arm_samples = 1
      self.stock_quiet_samples = int(not sample.stock_active)
    elif self.owner == LongOwner.ARMING:
      self.arm_samples += 1
      self.stock_quiet_samples = (
        self.stock_quiet_samples + 1 if not sample.stock_active else 0
      )
      if (self.arm_samples >= self.ARM_CONFIRM_SAMPLES and
          self.stock_quiet_samples >= self.STOCK_QUIET_SAMPLES):
        self.owner = LongOwner.OPENPILOT_SHADOW
    elif self.owner == LongOwner.OPENPILOT_SHADOW and sample.stock_active:
      self.owner = LongOwner.REVOKING
      self.revoke_samples = 1
    elif self.owner == LongOwner.REVOKING:
      if sample.stock_active:
        self.revoke_samples += 1
        if self.revoke_samples >= self.REVOKE_CONFIRM_SAMPLES:
          self.owner = LongOwner.BLOCKED
      else:
        self.owner = LongOwner.OPENPILOT_SHADOW
        self.revoke_samples = 0

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

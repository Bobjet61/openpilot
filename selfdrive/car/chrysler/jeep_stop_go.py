"""Fail-closed stock-ACC hold and resume coordinator for the Jeep.

The moving openpilot longitudinal path remains unchanged. This state machine
only latches a stop that openpilot and stock ACC entered together, requests the
already road-tested -2.0 m/s^2 standstill hold after stock ACC times out, and
sends guarded RESUME requests after independent vision/radar evidence that the
lead departed. The private hold remains applied until stock ACC acknowledges.
"""

from dataclasses import dataclass


HOLD_ACCEL_MPS2 = -2.0
RESUME_INTERVAL_FRAMES = 50   # 0.5 seconds
MAX_RESUME_ATTEMPTS = 10
MOVE_RELEASE_SPEED_MPS = 0.5


@dataclass(frozen=True)
class JeepStopGoResult:
  active: bool
  hold_command: bool
  send_resume: bool
  launch_pending: bool
  reason: str
  resume_attempts: int


class JeepStopGoHold:
  """Coordinate a bounded standstill hold without low-speed propulsion."""

  def __init__(self):
    self.active = False
    self.last_resume_frame = -RESUME_INTERVAL_FRAMES
    self.resume_attempts = 0
    self.reason = "inactive"

  def _deactivate(self, reason: str) -> JeepStopGoResult:
    self.active = False
    self.resume_attempts = 0
    self.reason = reason
    return JeepStopGoResult(
      active=False,
      hold_command=False,
      send_resume=False,
      launch_pending=False,
      reason=reason,
      resume_attempts=0,
    )

  def update(
      self,
      *,
      frame: int,
      supported: bool,
      forward_gear: bool,
      cruise_available: bool,
      stock_acc_enabled: bool,
      controls_enabled: bool,
      long_active: bool,
      standstill: bool,
      v_ego_mps: float,
      stopping: bool,
      requested_accel_mps2: float,
      lead_departure_confirmed: bool,
      cancel: bool,
      gas_pressed: bool,
      brake_pressed: bool,
      acc_faulted: bool,
      stock_aeb: bool,
  ) -> JeepStopGoResult:
    if not supported:
      return self._deactivate("unsupported")
    if cancel:
      return self._deactivate("cancel")
    if gas_pressed:
      return self._deactivate("gas")
    if brake_pressed:
      return self._deactivate("brake")
    if not forward_gear:
      return self._deactivate("gear")
    if not cruise_available:
      return self._deactivate("cruise_unavailable")
    if acc_faulted:
      return self._deactivate("acc_fault")
    if stock_aeb:
      return self._deactivate("stock_aeb")
    if self.active and not controls_enabled:
      return self._deactivate("controls_disabled")

    entry = (
      not self.active
      and controls_enabled
      and long_active
      and stock_acc_enabled
      and standstill
      and stopping
      and requested_accel_mps2 < 0.0
    )
    if entry:
      self.active = True
      self.last_resume_frame = frame - RESUME_INTERVAL_FRAMES
      self.resume_attempts = 0
      self.reason = "latched"

    if not self.active:
      return self._deactivate("inactive")
    if (
        stock_acc_enabled
        and not standstill
        and v_ego_mps > MOVE_RELEASE_SPEED_MPS
    ):
      return self._deactivate("vehicle_moving")

    send_resume = False
    if (
        standstill
        and not stock_acc_enabled
        and lead_departure_confirmed
        and self.resume_attempts < MAX_RESUME_ATTEMPTS
        and frame - self.last_resume_frame >= RESUME_INTERVAL_FRAMES
    ):
      send_resume = True
      self.last_resume_frame = frame
      self.resume_attempts += 1
      self.reason = "resume_requested"

    # Never create an open-loop brake gap. A RESUME request is sent while the
    # private brake-only hold remains active. The hold is removed only after
    # stock ACC explicitly reports enabled and becomes the braking/launch gate.
    launch_pending = self.resume_attempts > 0 and not stock_acc_enabled
    hold_command = (
      self.active
      and not stock_acc_enabled
    )
    if hold_command:
      if self.resume_attempts >= MAX_RESUME_ATTEMPTS:
        self.reason = "resume_limit_hold"
      else:
        self.reason = "resume_pending_hold" if launch_pending else "holding"
    elif stock_acc_enabled:
      self.reason = "stock_holding"
    else:
      self.reason = "latched"

    return JeepStopGoResult(
      active=self.active,
      hold_command=hold_command,
      send_resume=send_resume,
      launch_pending=launch_pending,
      reason=self.reason,
      resume_attempts=self.resume_attempts,
    )

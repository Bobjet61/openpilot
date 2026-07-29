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
FULL_LONG_LAUNCH_WINDOW_FRAMES = 800  # 8 seconds at 100 Hz
FULL_LONG_LAUNCH_COMPLETE_MPS = 2.05
FULL_LONG_LAUNCH_ARM_MAX_SPEED_MPS = 0.2
AUTO_LAUNCH_RESUME_INTERVAL_FRAMES = 10  # 0.1 seconds
AUTO_LAUNCH_RESUME_ATTEMPTS = 3
AUTO_LAUNCH_SETTLE_FRAMES = 35  # hold for 0.35 seconds before propulsion
AUTO_LAUNCH_HANDSHAKE_TIMEOUT_FRAMES = 100


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


@dataclass(frozen=True)
class JeepFullLongLaunchResult:
  armed: bool
  send_resume: bool
  reason: str
  arm_frame: int
  resume_attempts: int


class JeepFullLongLaunchGuard:
  """One-shot, lead-departure launch authorization for low-speed propulsion."""

  def __init__(self):
    self.armed = False
    self.arm_frame = -1
    self.handshake_pending = False
    self.handshake_frame = -1
    self.last_resume_frame = -AUTO_LAUNCH_RESUME_INTERVAL_FRAMES
    self.resume_attempts = 0
    self.reason = "mode_disabled"

  def _result(self, send_resume: bool = False) -> JeepFullLongLaunchResult:
    return JeepFullLongLaunchResult(
      armed=self.armed,
      send_resume=send_resume,
      reason=self.reason,
      arm_frame=self.arm_frame,
      resume_attempts=self.resume_attempts,
    )

  def _disarm(self, reason: str) -> JeepFullLongLaunchResult:
    self.armed = False
    self.arm_frame = -1
    self.handshake_pending = False
    self.handshake_frame = -1
    self.last_resume_frame = -AUTO_LAUNCH_RESUME_INTERVAL_FRAMES
    self.resume_attempts = 0
    self.reason = reason
    return self._result()

  def update(
      self,
      *,
      frame: int,
      mode_enabled: bool,
      supported: bool,
      forward_gear: bool,
      cruise_available: bool,
      controls_enabled: bool,
      long_active: bool,
      standstill: bool,
      v_ego_mps: float,
      requested_accel_mps2: float,
      plan_valid: bool,
      plan_has_lead: bool,
      lead_departure_confirmed: bool,
      cancel: bool,
      gas_pressed: bool,
      brake_pressed: bool,
      acc_faulted: bool,
      stock_aeb: bool,
  ) -> JeepFullLongLaunchResult:
    if not mode_enabled:
      return self._disarm("mode_disabled")
    if not supported:
      return self._disarm("unsupported")
    if cancel:
      return self._disarm("cancel")
    if gas_pressed:
      return self._disarm("gas")
    if brake_pressed:
      return self._disarm("brake")
    if not forward_gear:
      return self._disarm("gear")
    if not cruise_available:
      return self._disarm("cruise_unavailable")
    if not controls_enabled:
      return self._disarm("controls_disabled")
    if not long_active:
      return self._disarm("long_controls_inactive")
    if acc_faulted:
      return self._disarm("acc_fault")
    if stock_aeb:
      return self._disarm("stock_aeb")
    if not plan_valid:
      return self._disarm("plan_invalid")
    if (self.armed or self.handshake_pending) and not plan_has_lead:
      return self._disarm("lead_lost")
    if self.armed and v_ego_mps >= FULL_LONG_LAUNCH_COMPLETE_MPS:
      return self._disarm("launch_complete")
    if (
        (self.armed or self.handshake_pending)
        and standstill
        and requested_accel_mps2 <= 0.05
    ):
      return self._disarm("launch_plan_not_positive")
    if (
        self.armed
        and frame - self.arm_frame >= FULL_LONG_LAUNCH_WINDOW_FRAMES
    ):
      return self._disarm("launch_timeout")

    can_begin_handshake = (
      not self.armed
      and not self.handshake_pending
      and standstill
      and v_ego_mps <= FULL_LONG_LAUNCH_ARM_MAX_SPEED_MPS
      and plan_has_lead
      and lead_departure_confirmed
      and requested_accel_mps2 > 0.05
    )
    if can_begin_handshake:
      self.handshake_pending = True
      self.handshake_frame = frame
      self.last_resume_frame = frame
      self.resume_attempts = 1
      self.reason = "auto_resume_handshake"
      return self._result(send_resume=True)

    if self.handshake_pending:
      if not standstill or v_ego_mps > FULL_LONG_LAUNCH_ARM_MAX_SPEED_MPS:
        return self._disarm("moved_before_launch_arm")
      elapsed_frames = frame - self.handshake_frame
      send_resume = (
        self.resume_attempts < AUTO_LAUNCH_RESUME_ATTEMPTS
        and frame - self.last_resume_frame
        >= AUTO_LAUNCH_RESUME_INTERVAL_FRAMES
      )
      if send_resume:
        self.last_resume_frame = frame
        self.resume_attempts += 1
      if (
          self.resume_attempts >= AUTO_LAUNCH_RESUME_ATTEMPTS
          and elapsed_frames >= AUTO_LAUNCH_SETTLE_FRAMES
          and requested_accel_mps2 > 0.05
      ):
        self.handshake_pending = False
        self.handshake_frame = -1
        self.last_resume_frame = -AUTO_LAUNCH_RESUME_INTERVAL_FRAMES
        self.resume_attempts = 0
        self.armed = True
        self.arm_frame = frame
        self.reason = "auto_lead_departure_armed"
        return self._result()
      if elapsed_frames >= AUTO_LAUNCH_HANDSHAKE_TIMEOUT_FRAMES:
        return self._disarm("auto_resume_handshake_timeout")
      self.reason = (
        "auto_resume_settling"
        if self.resume_attempts >= AUTO_LAUNCH_RESUME_ATTEMPTS
        else "auto_resume_handshake"
      )
      return self._result(send_resume=send_resume)

    if self.armed:
      self.reason = "auto_lead_departure_armed"
    else:
      self.reason = "awaiting_lead_departure"

    return self._result()

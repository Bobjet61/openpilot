from dataclasses import dataclass
import math


WP_LONG_DIAGNOSTIC_SIGNATURE = 0xB0
WP_LONG_DIAGNOSTIC_SIGNATURE_MASK = 0xF0
WP_LONG_DIAGNOSTIC_FAILURES = {
  0: "actuation_disabled",
  1: "host_not_requested",
  2: "brake_not_fresh",
  3: "dash_not_fresh",
  4: "torque_not_fresh",
  5: "speed_not_fresh",
  6: "gas_not_fresh",
  7: "brake_pedal_not_fresh",
  8: "stock_acc_not_fresh",
  9: "counters_misaligned",
  10: "private_integrity",
  11: "speed_too_low",
  12: "driver_brake",
  13: "driver_gas",
  14: "collision",
  15: "command_envelope",
}
WP_LONG_OWNER_SIGNATURE = 0xD0
WP_LONG_OWNER_SIGNATURE_MASK = 0xF0
WP_LONG_OWNER_STATES = {
  0: "off",
  1: "canceling",
  2: "openpilot",
  3: "failed",
}


# b6q single-owner stop/go actuation build. Runtime output still requires the host,
# embedded Panda, and external White Panda to independently accept the same
# fresh, counter-matched, pedal-free, collision-free command cycle.
JEEP_LONG_SHADOW_TRANSPORT_COMPILED = True

# Tags Panda-rejected private frames in their USB rejection receipts. The tag
# never reaches a vehicle CAN transmit queue and does not change acceptance.
JEEP_LONG_REJECT_DIAGNOSTICS_COMPILED = True

# Independent actuation gate. The embedded Panda and the external White Panda
# each retain an independent fail-closed gate and validate the complete command
# envelope before vehicle CAN is modified.
JEEP_LONG_ACTUATION_COMPILED = True

if JEEP_LONG_ACTUATION_COMPILED and not JEEP_LONG_SHADOW_TRANSPORT_COMPILED:
  raise RuntimeError("Jeep longitudinal actuation requires shadow transport")
if (
    JEEP_LONG_REJECT_DIAGNOSTICS_COMPILED
    and not JEEP_LONG_SHADOW_TRANSPORT_COMPILED
):
  raise RuntimeError("Jeep longitudinal diagnostics require shadow transport")

# Stock-log calibration on this EcoDiesel found a DAS_3 braking p01 of
# -3.001015 m/s^2. Keep the shadow envelope just inside that value.
ACCEL_MIN = -3.0
ACCEL_MAX = 1.25
# Telemetry-only planner classification threshold. b6o actuator mode selection
# uses the blended hysteresis thresholds below, not this legacy deadband.
ACCEL_DEADBAND = 0.05
COMMAND_DT = 0.02
JERK_UP = 1.0
JERK_DOWN = 2.0

# The factory two-sided capture established the low-speed sequence used here:
# braking continues through zero, the stopped state holds -2.0 m/s^2, brake
# release completes before a short GO pulse, and engine torque does not begin
# until the Jeep is already rolling. A failed launch attempt re-applies hold
# and will not retry until the controller first withdraws the launch request.
LOW_SPEED_HOLD_ENTRY_MPS = 0.15
LOW_SPEED_ENGINE_MIN_MPS = 0.78
LOW_SPEED_HOLD_ACCEL_MPS2 = -2.0
LOW_SPEED_LAUNCH_REQUEST_ACCEL = 0.10
LOW_SPEED_LAUNCH_RESET_ACCEL = 0.02
LOW_SPEED_LAUNCH_CONFIRM_CYCLES = 10  # 200 ms at the 50 Hz envelope update
LOW_SPEED_GO_PULSE_CYCLES = 5         # 100 ms, matching the OEM capture
LOW_SPEED_CREEP_TIMEOUT_CYCLES = 50   # 1.0 s; never add torque from standstill

# The embedded Panda rejects private cycles closer than 15 ms. Even a 20 ms
# sender interval occasionally arrived below that threshold after USB/CAN
# scheduling jitter. The controller offers a cycle every 20 ms, so this 25 ms
# gate deliberately selects every other opportunity (nominally 25 Hz), leaving
# enough arrival-time margin while the White Panda safely holds a complete
# command snapshot between the stock 50 Hz DAS_3 frames.
# Advance the counter only for cycles actually sent.
TRANSPORT_MIN_SEND_INTERVAL_NS = 25_000_000

# b6m's synchronized factory-ACC capture established both command paths. The
# braking fit uses 681 same-direction samples (R^2 0.740). Propulsion uses a
# deliberately simple speed-aware feed-forward fit over 1,007 same-direction
# samples; remaining error is handled by openpilot's normal feedback loop.
BRAKE_ACCEL_INTERCEPT_MPS2 = -0.2176
BRAKE_ACCEL_GAIN = 0.8012
ENGINE_TORQUE_INTERCEPT_NM = -44.2
ENGINE_TORQUE_ACCEL_GAIN = 163.5
# b6q's successful route showed weak response during some positive requests,
# while reaching the guarded 425 Nm ceiling in only 4.2% of relevant engine
# samples. Add a modest 10% positive-request term below that unchanged ceiling
# without altering zero/negative-acceleration calibration.
ENGINE_TORQUE_POSITIVE_ACCEL_GAIN = 16.5
ENGINE_TORQUE_SPEED_GAIN = 11.5
# Do not extrapolate the speed term beyond the 25.58 m/s calibration drive.
ENGINE_TORQUE_CALIBRATION_SPEED_MAX_MPS = 26.0
# Matched planner/factory samples reached 422.25 Nm. Keep the host and both
# Panda guards at 425 Nm, below the unrelated 547 Nm factory outlier.
ENGINE_TORQUE_MAX_NM = 425.0

# b6q's route recorded 47 engine -> coast and 40 coast -> engine changes,
# concentrated where the planner moved around its -0.05 m/s^2 propulsion
# boundary. b6r uses separate entry and exit thresholds: once active, torque
# tapers smoothly to zero through mild negative corrections, but it cannot
# restart until the planner has made a positive request. The wide coast region
# before BRAKE_ENTER_ACCEL and the engine/brake interlock remain unchanged.
PROPULSION_ENTER_ACCEL = 0.02
PROPULSION_EXIT_ACCEL = -0.15
TORQUE_BLEND_ZERO_ACCEL = PROPULSION_EXIT_ACCEL
TORQUE_BLEND_FULL_ACCEL = 0.0
BRAKE_ENTER_ACCEL = -0.40
BRAKE_IMMEDIATE_ACCEL = -0.75
BRAKE_ENTRY_CONFIRM_CYCLES = 10
BRAKE_EXIT_ACCEL = -0.08
BRAKE_BLEND_FULL_ACCEL = -0.80
ENGINE_TORQUE_RATE_UP_NM_PER_S = 300.0
ENGINE_TORQUE_RATE_DOWN_NM_PER_S = 600.0
# A confirmed brake request must retire even the 425 Nm ceiling before the
# coast interlock can admit braking. The faster brake-transition release is
# only a withdrawal of requested engine torque; propulsion increases retain
# the ordinary 300 Nm/s limit and normal coasting retains 600 Nm/s.
BRAKE_TRANSITION_TORQUE_RATE_DOWN_NM_PER_S = 1800.0
BRAKE_APPLY_RATE_MPS3 = 1.5
BRAKE_RELEASE_RATE_MPS3 = 2.0
ENGINE_TORQUE_ZERO_EPSILON_NM = 0.5
BRAKE_ZERO_EPSILON_MPS2 = 0.005
COAST_INTERLOCK_CYCLES = 5


def fca_checksum(dat: bytes) -> int:
  """FCA CRC over every payload byte except the final checksum byte."""
  checksum = 0xFF
  for current in dat[:-1]:
    shift = 0x80
    for _ in range(8):
      bit_sum = current & shift
      temp_checksum = checksum & 0x80
      if bit_sum:
        bit_sum = 0x1C
        if temp_checksum:
          bit_sum = 1
        checksum = (checksum << 1) & 0xFF
        temp_checksum = checksum | 1
        bit_sum ^= temp_checksum
      else:
        if temp_checksum:
          bit_sum = 0x1D
        checksum = (checksum << 1) & 0xFF
        bit_sum ^= checksum
      checksum = bit_sum & 0xFF
      shift >>= 1
  return (~checksum) & 0xFF


@dataclass(frozen=True)
class JeepLongitudinalEnvelope:
  requested_accel: float
  limited_accel: float
  brake_accel_mps2: float
  brake_active: bool
  engine_active: bool
  engine_torque_nm: float
  command_mode: str
  brake_latched: bool
  stop_request: bool
  go_request: bool
  low_speed_state: str
  transport_enabled: bool
  host_enabled: bool
  eligible: bool


@dataclass(frozen=True)
class JeepLongitudinalDiagnostic:
  valid: bool
  applied: bool
  host_requested: bool
  brake_requested: bool
  engine_requested: bool
  failure_mask: int
  failure_reasons: tuple[str, ...]
  private_counter: int
  stock_counter: int


@dataclass(frozen=True)
class JeepLongitudinalCommandDiagnostic:
  valid: bool
  version: int
  stock_engine_active: bool
  stock_engine_torque_nm: float
  output_engine_active: bool
  output_engine_torque_nm: float
  stock_acc_available: bool
  stock_acc_active: bool
  stock_accel_mps2: float


@dataclass(frozen=True)
class JeepLongitudinalOwnerDiagnostic:
  valid: bool
  owner_state: int
  owner_name: str
  stock_valid: bool
  stock_available: bool
  stock_active: bool
  stock_fault: int
  stock_collision: bool
  cancel_injected: bool


def decode_wp_long_diagnostic(
    status: int,
    failure_low: int,
    failure_high: int,
    counters: int,
) -> JeepLongitudinalDiagnostic:
  status &= 0xFF
  failure_mask = (failure_low & 0xFF) | ((failure_high & 0xFF) << 8)
  valid = (
    status & WP_LONG_DIAGNOSTIC_SIGNATURE_MASK
  ) == WP_LONG_DIAGNOSTIC_SIGNATURE
  failure_reasons = tuple(
    reason
    for bit, reason in WP_LONG_DIAGNOSTIC_FAILURES.items()
    if failure_mask & (1 << bit)
  ) if valid else ("unsupported_beacon",)
  return JeepLongitudinalDiagnostic(
    valid=valid,
    applied=valid and bool(status & (1 << 0)),
    host_requested=valid and bool(status & (1 << 1)),
    brake_requested=valid and bool(status & (1 << 2)),
    engine_requested=valid and bool(status & (1 << 3)),
    failure_mask=failure_mask if valid else 0,
    failure_reasons=failure_reasons,
    private_counter=(counters >> 4) & 0xF,
    stock_counter=counters & 0xF,
  )


def decode_wp_long_command_diagnostic(
    signature: int,
    version: int,
    stock_engine_active: int,
    stock_engine_torque_nm: float,
    output_engine_active: int,
    output_engine_torque_nm: float,
    stock_acc_available: int,
    stock_acc_active: int,
    stock_accel_mps2: float,
) -> JeepLongitudinalCommandDiagnostic:
  version = int(version) & 0xFF
  return JeepLongitudinalCommandDiagnostic(
    valid=(int(signature) & 0xFF) == 0xC1 and version == 1,
    version=version,
    stock_engine_active=bool(stock_engine_active),
    stock_engine_torque_nm=float(stock_engine_torque_nm),
    output_engine_active=bool(output_engine_active),
    output_engine_torque_nm=float(output_engine_torque_nm),
    stock_acc_available=bool(stock_acc_available),
    stock_acc_active=bool(stock_acc_active),
    stock_accel_mps2=float(stock_accel_mps2),
  )


def decode_wp_long_owner_diagnostic(
    owner_status: int,
    stock_status: int,
    stock_fault_byte: int,
) -> JeepLongitudinalOwnerDiagnostic:
  owner_status &= 0xFF
  stock_status &= 0xFF
  owner_state = owner_status & 0xF
  valid = (
    owner_status & WP_LONG_OWNER_SIGNATURE_MASK
  ) == WP_LONG_OWNER_SIGNATURE
  return JeepLongitudinalOwnerDiagnostic(
    valid=valid,
    owner_state=owner_state if valid else 0,
    owner_name=(
      WP_LONG_OWNER_STATES.get(owner_state, "unknown")
      if valid else "unsupported"
    ),
    stock_valid=valid and bool(stock_status & (1 << 4)),
    stock_available=valid and bool(stock_status & (1 << 0)),
    stock_active=valid and bool(stock_status & (1 << 1)),
    stock_fault=(stock_fault_byte & 0x3) if valid else 0,
    stock_collision=valid and bool(stock_status & (1 << 2)),
    cancel_injected=valid and bool(stock_status & (1 << 3)),
  )


def clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


def move_toward(
    current: float,
    target: float,
    increasing_rate: float,
    decreasing_rate: float,
    dt: float = COMMAND_DT,
) -> float:
  if target > current:
    return min(target, current + increasing_rate * dt)
  return max(target, current - decreasing_rate * dt)


def jeep_long_shadow_safety_param(
  safety_param: int,
  shadow_flag: int,
  diagnostic_flag: int = 0,
  actuation_flag: int = 0,
) -> int:
  if JEEP_LONG_SHADOW_TRANSPORT_COMPILED:
    safety_param |= shadow_flag
    if JEEP_LONG_REJECT_DIAGNOSTICS_COMPILED:
      safety_param |= diagnostic_flag
    if JEEP_LONG_ACTUATION_COMPILED:
      safety_param |= actuation_flag
  return safety_param


class JeepLongitudinalShadow:
  """Rate-limited command generator behind independent transport/safety gates."""

  def __init__(self):
    self.accel_last = 0.0
    self.engine_torque_last_nm = 0.0
    self.brake_accel_last_mps2 = 0.0
    self.brake_latched = False
    self.brake_immediate = False
    self.brake_entry_confirm_cycles = 0
    self.propulsion_latched = False
    self.last_nonzero_mode = "coast"
    self.coast_interlock_remaining = 0
    self.low_speed_state = "drive"
    self.launch_confirm_cycles = 0
    self.release_transport_confirmed = False
    self.go_pulse_remaining = 0
    self.creep_wait_remaining = 0

  def update(
      self,
      requested_accel: float,
      eligible: bool,
      speed_mps: float = 0.0,
  ) -> JeepLongitudinalEnvelope:
    if not math.isfinite(requested_accel) or not math.isfinite(speed_mps):
      requested_accel = 0.0
      speed_mps = 0.0
      eligible = False
    requested_accel = clip(requested_accel, ACCEL_MIN, ACCEL_MAX)

    if not eligible:
      limited_accel = 0.0
      self.engine_torque_last_nm = 0.0
      self.brake_accel_last_mps2 = 0.0
      self.brake_latched = False
      self.brake_immediate = False
      self.brake_entry_confirm_cycles = 0
      self.propulsion_latched = False
      self.last_nonzero_mode = "coast"
      self.coast_interlock_remaining = 0
      self.low_speed_state = "drive"
      self.launch_confirm_cycles = 0
      self.release_transport_confirmed = False
      self.go_pulse_remaining = 0
      self.creep_wait_remaining = 0
    else:
      lower = self.accel_last - JERK_DOWN * COMMAND_DT
      upper = self.accel_last + JERK_UP * COMMAND_DT
      limited_accel = clip(requested_accel, lower, upper)

    self.accel_last = limited_accel
    if eligible:
      if self.brake_latched:
        self.brake_entry_confirm_cycles = 0
        if (
            limited_accel >= BRAKE_EXIT_ACCEL
            and requested_accel > BRAKE_ENTER_ACCEL
        ):
          self.brake_latched = False
          self.brake_immediate = False
      elif requested_accel <= BRAKE_IMMEDIATE_ACCEL:
        self.brake_entry_confirm_cycles = 0
        self.brake_latched = True
        self.brake_immediate = True
      elif requested_accel <= BRAKE_ENTER_ACCEL:
        self.brake_entry_confirm_cycles += 1
        if self.brake_entry_confirm_cycles >= BRAKE_ENTRY_CONFIRM_CYCLES:
          self.brake_entry_confirm_cycles = 0
          self.brake_latched = True
          self.brake_immediate = False
      else:
        self.brake_entry_confirm_cycles = 0

      if self.brake_latched:
        self.propulsion_latched = False
      elif self.propulsion_latched:
        if requested_accel <= PROPULSION_EXIT_ACCEL:
          self.propulsion_latched = False
      elif requested_accel >= PROPULSION_ENTER_ACCEL:
        self.propulsion_latched = True

    # Enter the bounded low-speed state machine only after a braking stop or
    # when controls are first enabled at true standstill. Merely creeping
    # slowly with a positive request must not create an unsolicited hold.
    stopped = speed_mps <= 0.001
    stop_entry = (
      speed_mps <= LOW_SPEED_HOLD_ENTRY_MPS
      and (
        stopped
        or self.brake_latched
        or self.brake_accel_last_mps2 < -BRAKE_ZERO_EPSILON_MPS2
      )
    )
    if eligible and self.low_speed_state == "drive" and stop_entry:
      self.low_speed_state = "hold"
      self.launch_confirm_cycles = 0
      self.release_transport_confirmed = False

    if eligible and self.low_speed_state == "hold":
      if limited_accel >= LOW_SPEED_LAUNCH_REQUEST_ACCEL:
        self.launch_confirm_cycles += 1
      else:
        self.launch_confirm_cycles = 0
      if self.launch_confirm_cycles >= LOW_SPEED_LAUNCH_CONFIRM_CYCLES:
        self.low_speed_state = "release"
    elif eligible and self.low_speed_state in ("release", "go", "creep"):
      if limited_accel <= LOW_SPEED_LAUNCH_RESET_ACCEL:
        self.low_speed_state = "hold"
        self.launch_confirm_cycles = 0
        self.release_transport_confirmed = False
        self.go_pulse_remaining = 0
        self.creep_wait_remaining = 0
    elif eligible and self.low_speed_state == "blocked":
      if limited_accel <= LOW_SPEED_LAUNCH_RESET_ACCEL:
        self.low_speed_state = "hold"
        self.launch_confirm_cycles = 0
        self.release_transport_confirmed = False

    calibration_speed_mps = clip(
      speed_mps, 0.0, ENGINE_TORQUE_CALIBRATION_SPEED_MAX_MPS,
    )
    desired_engine_torque_nm = 0.0
    if eligible and not self.brake_latched and self.propulsion_latched:
      base_engine_torque_nm = clip(
        ENGINE_TORQUE_INTERCEPT_NM
        + ENGINE_TORQUE_ACCEL_GAIN * limited_accel
        + ENGINE_TORQUE_POSITIVE_ACCEL_GAIN * max(limited_accel, 0.0)
        + ENGINE_TORQUE_SPEED_GAIN * calibration_speed_mps,
        0.0,
        ENGINE_TORQUE_MAX_NM,
      )
      # The input slew limiter must never create or sustain propulsion after
      # the production controller has already requested deceleration. Use the
      # more conservative of raw and limited acceleration for the propulsion
      # blend; the 600 Nm/s output release still smooths an existing request.
      torque_blend_accel = min(limited_accel, requested_accel)
      torque_blend = clip(
        (torque_blend_accel - TORQUE_BLEND_ZERO_ACCEL)
        / (TORQUE_BLEND_FULL_ACCEL - TORQUE_BLEND_ZERO_ACCEL),
        0.0,
        1.0,
      )
      desired_engine_torque_nm = base_engine_torque_nm * torque_blend

    desired_brake_accel_mps2 = 0.0
    if eligible and self.brake_latched:
      # A strong request bypasses only the duplicate input jerk limiter. The
      # engine/brake interlock and physical brake slew limit still apply.
      brake_control_accel = (
        min(limited_accel, requested_accel)
        if self.brake_immediate else limited_accel
      )
      calibrated_brake_accel_mps2 = clip(
        BRAKE_ACCEL_INTERCEPT_MPS2 + BRAKE_ACCEL_GAIN * brake_control_accel,
        ACCEL_MIN,
        0.0,
      )
      brake_blend = clip(
        (BRAKE_EXIT_ACCEL - brake_control_accel)
        / (BRAKE_EXIT_ACCEL - BRAKE_BLEND_FULL_ACCEL),
        0.0,
        1.0,
      )
      desired_brake_accel_mps2 = (
        calibrated_brake_accel_mps2 * brake_blend
      )

    # Low-speed overrides are deliberately asymmetric: braking is allowed all
    # the way to zero, while propulsion remains impossible until wheel speed
    # independently proves that the Jeep is already rolling.
    if eligible and self.low_speed_state in ("hold", "blocked"):
      desired_engine_torque_nm = 0.0
      desired_brake_accel_mps2 = LOW_SPEED_HOLD_ACCEL_MPS2
      self.brake_latched = True
      self.brake_immediate = False
    elif eligible and self.low_speed_state in ("release", "go", "creep"):
      desired_engine_torque_nm = 0.0
      desired_brake_accel_mps2 = 0.0
      self.brake_latched = False
      self.brake_immediate = False
    elif eligible and speed_mps < LOW_SPEED_ENGINE_MIN_MPS:
      desired_engine_torque_nm = 0.0

    if not eligible:
      engine_torque_nm = 0.0
      brake_accel_mps2 = 0.0
    elif desired_brake_accel_mps2 < 0.0:
      engine_torque_nm = move_toward(
        self.engine_torque_last_nm,
        0.0,
        ENGINE_TORQUE_RATE_UP_NM_PER_S,
        BRAKE_TRANSITION_TORQUE_RATE_DOWN_NM_PER_S,
      )
      if engine_torque_nm <= ENGINE_TORQUE_ZERO_EPSILON_NM:
        engine_torque_nm = 0.0
        if self.last_nonzero_mode == "engine":
          if self.coast_interlock_remaining == 0:
            self.coast_interlock_remaining = COAST_INTERLOCK_CYCLES
          self.coast_interlock_remaining -= 1
          brake_accel_mps2 = 0.0
          if self.coast_interlock_remaining == 0:
            self.last_nonzero_mode = "coast"
        else:
          brake_accel_mps2 = move_toward(
            self.brake_accel_last_mps2,
            desired_brake_accel_mps2,
            BRAKE_RELEASE_RATE_MPS3,
            BRAKE_APPLY_RATE_MPS3,
          )
      else:
        brake_accel_mps2 = move_toward(
          self.brake_accel_last_mps2,
          0.0,
          BRAKE_RELEASE_RATE_MPS3,
          BRAKE_APPLY_RATE_MPS3,
        )
    else:
      brake_accel_mps2 = move_toward(
        self.brake_accel_last_mps2,
        0.0,
        BRAKE_RELEASE_RATE_MPS3,
        BRAKE_APPLY_RATE_MPS3,
      )
      if brake_accel_mps2 >= -BRAKE_ZERO_EPSILON_MPS2:
        brake_accel_mps2 = 0.0
        if desired_engine_torque_nm > 0.0 and self.last_nonzero_mode == "brake":
          if self.coast_interlock_remaining == 0:
            self.coast_interlock_remaining = COAST_INTERLOCK_CYCLES
          self.coast_interlock_remaining -= 1
          engine_torque_nm = 0.0
          if self.coast_interlock_remaining == 0:
            self.last_nonzero_mode = "coast"
        else:
          engine_torque_nm = move_toward(
            self.engine_torque_last_nm,
            desired_engine_torque_nm,
            ENGINE_TORQUE_RATE_UP_NM_PER_S,
            ENGINE_TORQUE_RATE_DOWN_NM_PER_S,
          )
      else:
        engine_torque_nm = move_toward(
          self.engine_torque_last_nm,
          0.0,
          ENGINE_TORQUE_RATE_UP_NM_PER_S,
          ENGINE_TORQUE_RATE_DOWN_NM_PER_S,
        )

    self.engine_torque_last_nm = engine_torque_nm
    self.brake_accel_last_mps2 = brake_accel_mps2
    brake_active = brake_accel_mps2 < 0.0
    engine_active = engine_torque_nm > 0.0

    # A GO pulse cannot overlap any residual brake request. After the captured
    # 100 ms pulse, wait for vehicle creep. If rolling speed is not established
    # within one second, reapply hold and latch the attempt blocked.
    go_request = False
    if eligible and self.low_speed_state == "release":
      if brake_active:
        self.release_transport_confirmed = False
      elif self.release_transport_confirmed:
        self.low_speed_state = "go"
        self.release_transport_confirmed = False
        self.go_pulse_remaining = LOW_SPEED_GO_PULSE_CYCLES
    if eligible and self.low_speed_state == "go":
      if brake_active:
        self.low_speed_state = "hold"
        self.go_pulse_remaining = 0
      else:
        go_request = self.go_pulse_remaining > 0
        self.go_pulse_remaining = max(0, self.go_pulse_remaining - 1)
        if self.go_pulse_remaining == 0:
          self.low_speed_state = "creep"
          self.creep_wait_remaining = LOW_SPEED_CREEP_TIMEOUT_CYCLES
    elif eligible and self.low_speed_state == "creep":
      if speed_mps >= LOW_SPEED_ENGINE_MIN_MPS:
        self.low_speed_state = "drive"
        self.creep_wait_remaining = 0
      else:
        self.creep_wait_remaining = max(0, self.creep_wait_remaining - 1)
        if self.creep_wait_remaining == 0:
          self.low_speed_state = "blocked"

    # The single-owner handoff must never replace a factory standstill hold
    # with a slowly ramping brake request. At independently measured true
    # standstill (and after a failed creep attempt), use the exact captured OEM
    # -2.0 m/s^2 hold immediately. Braking while the Jeep is still moving keeps
    # the normal slew limits above.
    if (
        eligible
        and stopped
        and self.low_speed_state in ("hold", "blocked")
    ):
      engine_torque_nm = 0.0
      brake_accel_mps2 = LOW_SPEED_HOLD_ACCEL_MPS2
      self.engine_torque_last_nm = engine_torque_nm
      self.brake_accel_last_mps2 = brake_accel_mps2
      brake_active = True
      engine_active = False

    if brake_active and engine_active:
      raise RuntimeError("Jeep longitudinal torque/brake interlock violated")
    command_mode = (
      "hold" if self.low_speed_state in ("hold", "blocked") and brake_active
      else "go" if go_request
      else "brake" if brake_active else "engine" if engine_active
      else "coast" if eligible else "inactive"
    )
    if engine_active:
      self.last_nonzero_mode = "engine"
      self.coast_interlock_remaining = 0
    elif brake_active:
      self.last_nonzero_mode = "brake"
      self.coast_interlock_remaining = 0

    transport_enabled = JEEP_LONG_SHADOW_TRANSPORT_COMPILED and eligible
    return JeepLongitudinalEnvelope(
      requested_accel=requested_accel,
      limited_accel=limited_accel,
      brake_accel_mps2=brake_accel_mps2,
      brake_active=brake_active,
      engine_active=engine_active,
      engine_torque_nm=engine_torque_nm,
      command_mode=command_mode,
      brake_latched=self.brake_latched,
      stop_request=(
        eligible
        and self.low_speed_state in ("hold", "blocked")
        and brake_active
      ),
      go_request=eligible and go_request and not brake_active and not engine_active,
      low_speed_state=self.low_speed_state,
      transport_enabled=transport_enabled,
      host_enabled=JEEP_LONG_ACTUATION_COMPILED and transport_enabled,
      eligible=eligible,
    )

  def note_transport_sent(self, envelope: JeepLongitudinalEnvelope) -> None:
    """Confirm that the Pandas received a complete neutral release cycle."""
    if (
        self.low_speed_state == "release"
        and envelope.low_speed_state == "release"
        and envelope.eligible
        and not envelope.brake_active
        and not envelope.engine_active
        and not envelope.stop_request
        and not envelope.go_request
    ):
      self.release_transport_confirmed = True


class JeepLongitudinalTransportScheduler:
  """Rate gate and counter for the neutral private transport probe."""

  def __init__(self):
    self.counter = 0
    self.last_send_nanos: int | None = None

  def next_counter(self, now_nanos: int, enabled: bool) -> int | None:
    if not enabled:
      return None
    if (
        self.last_send_nanos is not None
        and now_nanos - self.last_send_nanos < TRANSPORT_MIN_SEND_INTERVAL_NS
    ):
      return None

    counter = self.counter
    self.counter = (self.counter + 1) & 0xF
    self.last_send_nanos = now_nanos
    return counter

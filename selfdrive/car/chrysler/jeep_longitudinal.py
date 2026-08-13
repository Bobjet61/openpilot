from __future__ import annotations

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

FACTORY_SNG_LEAD_MIN_DISTANCE_M = 2.0
FACTORY_SNG_LEAD_MIN_VREL_MPS = 0.5
FACTORY_SNG_LEAD_MAX_SPEED_DISAGREEMENT_MPS = 1.5
FACTORY_SNG_VISION_MIN_PROB = 0.90
FACTORY_SNG_VISION_MAX_DISTANCE_M = 25.0
FACTORY_SNG_VISION_MIN_VREL_MPS = 0.6
FACTORY_SNG_VISION_MIN_DISTANCE_GAIN_M = 0.5


def jeep_factory_sng_lead_moving(
    vision_status, vision_d_rel, vision_v_rel, vision_prob,
    radar_d_rel, radar_v_rel,
):
  """Require independent vision/radar agreement before automatic RESUME."""
  values = (vision_d_rel, vision_v_rel, vision_prob, radar_d_rel, radar_v_rel)
  return (
    bool(vision_status)
    and all(math.isfinite(float(v)) for v in values)
    and vision_prob >= 0.70
    and vision_d_rel >= FACTORY_SNG_LEAD_MIN_DISTANCE_M
    and radar_d_rel >= FACTORY_SNG_LEAD_MIN_DISTANCE_M
    and vision_v_rel >= FACTORY_SNG_LEAD_MIN_VREL_MPS
    and radar_v_rel >= FACTORY_SNG_LEAD_MIN_VREL_MPS
    and abs(vision_v_rel - radar_v_rel) <=
        FACTORY_SNG_LEAD_MAX_SPEED_DISAGREEMENT_MPS
    and abs(vision_d_rel - radar_d_rel) <= max(5.0, vision_d_rel * 0.25)
  )


def jeep_factory_sng_vision_lead_moving(
    vision_status, vision_d_rel, vision_v_rel, vision_prob,
    minimum_held_distance,
):
  """Conservative fallback when the raw-radar association drops at launch."""
  values = (
    vision_d_rel, vision_v_rel, vision_prob, minimum_held_distance,
  )
  return (
    bool(vision_status)
    and all(math.isfinite(float(v)) for v in values)
    and vision_prob >= FACTORY_SNG_VISION_MIN_PROB
    and FACTORY_SNG_LEAD_MIN_DISTANCE_M <= vision_d_rel <=
        FACTORY_SNG_VISION_MAX_DISTANCE_M
    and vision_v_rel >= FACTORY_SNG_VISION_MIN_VREL_MPS
    and vision_d_rel - minimum_held_distance >=
        FACTORY_SNG_VISION_MIN_DISTANCE_GAIN_M
  )


def jeep_acc_faulted(das_3_fault, das_4_fault):
  """Combine both FCA ACC fault sources used by the EcoDiesel."""
  return das_3_fault != 0 or das_4_fault != 0


# b6u single-owner stop/go actuation build. Runtime output still requires the host,
# embedded Panda, and external White Panda to independently accept the same
# fresh, counter-matched, pedal-free, collision-free command cycle.
JEEP_LONG_SHADOW_TRANSPORT_COMPILED = True

# Tags Panda-rejected private frames in their USB rejection receipts. The tag
# never reaches a vehicle CAN transmit queue and does not change acceptance.
JEEP_LONG_REJECT_DIAGNOSTICS_COMPILED = True

# Independent actuation gate. The embedded Panda and the external White Panda
# each retain an independent fail-closed gate and validate the complete command
# envelope before vehicle CAN is modified.
# b7o development branch: offline/shadow ownership only. This must remain
# false until replay evidence and a separate reviewed activation commit exist.
JEEP_LONG_ACTUATION_COMPILED = False

if JEEP_LONG_ACTUATION_COMPILED and not JEEP_LONG_SHADOW_TRANSPORT_COMPILED:
  raise RuntimeError("Jeep longitudinal actuation requires shadow transport")
if (
    JEEP_LONG_REJECT_DIAGNOSTICS_COMPILED
    and not JEEP_LONG_SHADOW_TRANSPORT_COMPILED
):
  raise RuntimeError("Jeep longitudinal diagnostics require shadow transport")


@dataclass(frozen=True)
class JeepLongitudinalModeSelection:
  name: str
  factory_acc: bool
  factory_stop_and_go: bool
  openpilot_long: bool
  experimental_shadow: bool
  conflicting_toggles: bool


def select_jeep_longitudinal_mode(
    experimental_long,
    custom_stock_long,
    actuation_compiled=None,
):
  """Resolve the restart-gated Jeep longitudinal mode in one place.

  Experimental takes precedence when both offroad toggles are selected, but a
  non-actuating build never hands the vehicle to openpilot. It instead records
  the experimental request in shadow while factory ACC remains the sole moving
  longitudinal owner. Factory stop/go is deliberately disabled in that state
  so two augmentation paths can never be armed together.
  """
  if actuation_compiled is None:
    actuation_compiled = JEEP_LONG_ACTUATION_COMPILED
  experimental_requested = bool(experimental_long)
  factory_sng_requested = bool(custom_stock_long)
  conflicting_toggles = experimental_requested and factory_sng_requested
  if experimental_requested:
    if actuation_compiled:
      return JeepLongitudinalModeSelection(
        name="experimental",
        factory_acc=False,
        factory_stop_and_go=False,
        openpilot_long=True,
        experimental_shadow=False,
        conflicting_toggles=conflicting_toggles,
      )
    return JeepLongitudinalModeSelection(
      name="experimental_shadow",
      factory_acc=True,
      factory_stop_and_go=False,
      openpilot_long=False,
      experimental_shadow=True,
      conflicting_toggles=conflicting_toggles,
    )
  if factory_sng_requested:
    return JeepLongitudinalModeSelection(
      name="factory_stop_and_go",
      factory_acc=True,
      factory_stop_and_go=True,
      openpilot_long=False,
      experimental_shadow=False,
      conflicting_toggles=False,
    )
  return JeepLongitudinalModeSelection(
    name="factory",
    factory_acc=True,
    factory_stop_and_go=False,
    openpilot_long=False,
    experimental_shadow=False,
    conflicting_toggles=False,
  )


def jeep_long_actuation_enabled(experimental_long):
  """Enable Jeep openpilot-long only from the offroad alpha toggle.

  The toggle is sampled while card fingerprints the vehicle. It is not a
  live onroad ownership switch: changing it requires a comma restart before
  CarParams and both Panda safety configurations can change together.
  """
  return select_jeep_longitudinal_mode(
    experimental_long,
    custom_stock_long=False,
  ).openpilot_long


def jeep_long_mode_safety_param(
    current_safety_param,
    experimental_long,
    shadow_flag,
    diagnostic_flag,
    actuation_flag,
):
  """Apply the complete Jeep-long safety group only in openpilot-long mode."""
  if not jeep_long_actuation_enabled(experimental_long):
    return current_safety_param
  return jeep_long_shadow_safety_param(
    current_safety_param,
    shadow_flag,
    diagnostic_flag,
    actuation_flag,
  )

# Stock-log calibration on this EcoDiesel found a DAS_3 braking p01 of
# -3.001015 m/s^2. Keep the shadow envelope just inside that value.
ACCEL_MIN = -3.0
# b6r's 5.38% uphill interval stayed at the old 1.25 m/s^2 mapper ceiling
# while losing 8.24 km/h. A separate stock-ACC uphill capture reached
# 503.25 Nm median and 535.5 Nm maximum. The active b6x route asserted the
# dashboard ACC fault after its output reached 440 Nm, but the fault-free b6w
# route had both p95 and p99 at the same 440 Nm ceiling. Preserve that proven
# ceiling while reverting the b6x gain/rise regression below.
ACCEL_MAX = 1.5
# Telemetry-only planner classification threshold. b6o actuator mode selection
# uses the blended hysteresis thresholds below, not this legacy deadband.
ACCEL_DEADBAND = 0.05
COMMAND_DT = 0.02
JERK_UP = 1.0
JERK_DOWN = 2.0
# This changes only the internal request while the fixed standstill brake hold
# remains applied. It lets a confirmed launch reach the separate release state
# without waiting more than two seconds for the ordinary driving jerk ramp.
LOW_SPEED_HOLD_LAUNCH_JERK_UP = 4.0

# The complete factory two-sided capture established the low-speed sequence:
# braking continues through zero, the stopped state holds -2.0 m/s^2, brake
# release completes before a short GO pulse, and a bounded engine request can
# begin immediately after GO. A failed launch attempt re-applies hold and will
# not retry until the controller first withdraws the launch request.
LOW_SPEED_HOLD_ENTRY_MPS = 0.15
LOW_SPEED_ENGINE_MIN_MPS = 0.78
LOW_SPEED_HOLD_ACCEL_MPS2 = -2.0
LOW_SPEED_LAUNCH_REQUEST_ACCEL = 0.10
LOW_SPEED_LAUNCH_RESET_ACCEL = 0.02
LOW_SPEED_LAUNCH_CONFIRM_CYCLES = 10  # 200 ms at the 50 Hz envelope update
LOW_SPEED_GO_PULSE_CYCLES = 5         # 100 ms, matching the OEM capture
# The complete August 5 capture showed that factory engine torque normally
# begins immediately after GO, before the Jeep reaches 0.78 m/s. Permit only a
# separately capped, rate-limited launch request after HOLD -> RELEASE -> GO.
LOW_SPEED_LAUNCH_TORQUE_MAX_NM = 200.0
LOW_SPEED_LAUNCH_GRADE_MIN_NM = -25.0
LOW_SPEED_LAUNCH_GRADE_MAX_NM = 50.0
LOW_SPEED_CREEP_TIMEOUT_CYCLES = 75   # 1.5 s before fail-closed re-hold

# The embedded Panda rejects private cycles closer than 15 ms. Even a 20 ms
# sender interval occasionally arrived below that threshold after USB/CAN
# scheduling jitter. The controller offers the latest 50 Hz envelope to this
# gate every 10 ms; the 25 ms floor therefore produces a nominal 30 ms/33 Hz
# complete triplet. That preserves a 10 ms arrival-time margin while reducing
# exposure to the White Panda's unchanged 100 ms freshness watchdog.
# Advance the counter only for cycles actually sent.
TRANSPORT_MIN_SEND_INTERVAL_NS = 25_000_000

# b6m's synchronized factory-ACC capture established both command paths. The
# braking fit uses 681 same-direction samples (R^2 0.740). Propulsion uses a
# deliberately simple speed-aware feed-forward fit over 1,007 same-direction
# samples; remaining error is handled by openpilot's normal feedback loop.
BRAKE_ACCEL_INTERCEPT_MPS2 = -0.2176
BRAKE_ACCEL_GAIN = 0.8012
ENGINE_TORQUE_INTERCEPT_NM = 0.0
ENGINE_TORQUE_ACCEL_GAIN = 163.5
# b6x raised this term to 40 Nm/(m/s^2) and raised the torque slew rate at the
# same time. Its first new vehicle fault occurred at the resulting 440 Nm
# plateau. Restore the last fault-free b6w feed-forward while isolating the
# effect of the newly mirrored independent Panda command envelope.
ENGINE_TORQUE_POSITIVE_ACCEL_GAIN = 16.5
ENGINE_TORQUE_SPEED_GAIN = 4.5
# Do not extrapolate the speed term beyond the 25.58 m/s calibration drive.
ENGINE_TORQUE_CALIBRATION_SPEED_MAX_MPS = 26.0
# The long LAX factory-ACC capture separated the old speed coefficient from
# road grade: planner acceleration plus speed explained almost none of the
# factory torque variance. The complete August 5 capture measured about
# 3,327 Nm/rad after speed and acceleration, so b6u uses a slightly reduced,
# filtered coefficient and a bounded contribution. Low-speed launch torque is
# separately capped by the host and both Pandas.
ENGINE_TORQUE_GRADE_GAIN_NM_PER_RAD = 3200.0
ENGINE_TORQUE_GRADE_MIN_NM = -50.0
ENGINE_TORQUE_GRADE_MAX_NM = 150.0
ENGINE_TORQUE_GRADE_MIN_SPEED_MPS = 5.0
ENGINE_TORQUE_GRADE_PITCH_LIMIT_RAD = math.radians(4.0)
# Raw body pitch includes short suspension/bump motion as well as road grade.
# Two-drive replay showed that 1.5 s cuts grade-command variation about 22%
# and the largest qlog-scale step about 47%, while sustained hills still reach
# the full 150 Nm feed-forward range.
ENGINE_TORQUE_GRADE_FILTER_TAU_S = 1.5
ENGINE_TORQUE_LOW_SPEED_BASE_MAX_NM = 250.0
ENGINE_TORQUE_LOW_SPEED_MAX_GAIN_NM_PER_MPS = 20.0
# b6w sustained the 440 Nm ceiling without a dashboard fault. Keep that proven
# authority and separately mirror the speed-shaped low-speed envelope in both
# Pandas so a corrupt host cannot jump directly to this ceiling at low speed.
ENGINE_TORQUE_MAX_NM = 440.0

# b6r cut normalized switching 48.8%, but its route still cycled at planner
# requests near -0.14 to -0.19 m/s^2. Offline same-input screening showed that
# extending the taper to -0.32 m/s^2 reduces modeled transitions from 67 to 32
# while preserving all brake entries and the zero-overlap interlock. The
# remaining -0.32 to -0.40 coast band still separates propulsion from braking.
PROPULSION_ENTER_ACCEL = 0.02
PROPULSION_EXIT_ACCEL = -0.32
TORQUE_BLEND_ZERO_ACCEL = PROPULSION_EXIT_ACCEL
TORQUE_BLEND_FULL_ACCEL = 0.0
BRAKE_ENTER_ACCEL = -0.40
BRAKE_IMMEDIATE_ACCEL = -0.75
BRAKE_ENTRY_CONFIRM_CYCLES = 10
BRAKE_EXIT_ACCEL = -0.08
BRAKE_BLEND_FULL_ACCEL = -0.80
ENGINE_TORQUE_RATE_UP_NM_PER_S = 300.0
ENGINE_TORQUE_RATE_DOWN_NM_PER_S = 600.0
# A confirmed brake request must retire even the 440 Nm ceiling before the
# coast interlock can admit braking. The faster brake-transition release is
# only a withdrawal of requested engine torque; propulsion increases retain
# the last fault-free 300 Nm/s rate and normal coasting retains 600 Nm/s.
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
  filtered_pitch_rad: float
  grade_torque_nm: float
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


def engine_torque_max_for_speed(speed_mps: float) -> float:
  """Return the host ceiling mirrored by both independent Panda guards."""
  calibration_speed_mps = clip(
    speed_mps, 0.0, ENGINE_TORQUE_CALIBRATION_SPEED_MAX_MPS,
  )
  return clip(
    ENGINE_TORQUE_LOW_SPEED_BASE_MAX_NM
    + ENGINE_TORQUE_LOW_SPEED_MAX_GAIN_NM_PER_MPS * calibration_speed_mps,
    ENGINE_TORQUE_LOW_SPEED_BASE_MAX_NM,
    ENGINE_TORQUE_MAX_NM,
  )


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
    self.filtered_pitch_rad = 0.0

  def update(
      self,
      requested_accel: float,
      eligible: bool,
      speed_mps: float = 0.0,
      pitch_rad: float = 0.0,
  ) -> JeepLongitudinalEnvelope:
    if not math.isfinite(requested_accel) or not math.isfinite(speed_mps):
      requested_accel = 0.0
      speed_mps = 0.0
      eligible = False
    if not math.isfinite(pitch_rad):
      pitch_rad = 0.0
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
      self.filtered_pitch_rad = 0.0
    else:
      lower = self.accel_last - JERK_DOWN * COMMAND_DT
      jerk_up = (
        LOW_SPEED_HOLD_LAUNCH_JERK_UP
        if (
          self.low_speed_state == "hold"
          and requested_accel >= LOW_SPEED_LAUNCH_REQUEST_ACCEL
        )
        else JERK_UP
      )
      upper = self.accel_last + jerk_up * COMMAND_DT
      limited_accel = clip(requested_accel, lower, upper)
      bounded_pitch_rad = clip(
        pitch_rad,
        -ENGINE_TORQUE_GRADE_PITCH_LIMIT_RAD,
        ENGINE_TORQUE_GRADE_PITCH_LIMIT_RAD,
      )
      pitch_alpha = COMMAND_DT / (
        ENGINE_TORQUE_GRADE_FILTER_TAU_S + COMMAND_DT
      )
      self.filtered_pitch_rad += pitch_alpha * (
        bounded_pitch_rad - self.filtered_pitch_rad
      )

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
    grade_torque_nm = 0.0
    if eligible and not self.brake_latched and self.propulsion_latched:
      if speed_mps >= ENGINE_TORQUE_GRADE_MIN_SPEED_MPS:
        grade_torque_nm = clip(
          ENGINE_TORQUE_GRADE_GAIN_NM_PER_RAD * self.filtered_pitch_rad,
          ENGINE_TORQUE_GRADE_MIN_NM,
          ENGINE_TORQUE_GRADE_MAX_NM,
        )
      speed_limited_torque_max_nm = engine_torque_max_for_speed(
        calibration_speed_mps,
      )
      base_engine_torque_nm = clip(
        ENGINE_TORQUE_INTERCEPT_NM
        + ENGINE_TORQUE_ACCEL_GAIN * limited_accel
        + ENGINE_TORQUE_POSITIVE_ACCEL_GAIN * max(limited_accel, 0.0)
        + ENGINE_TORQUE_SPEED_GAIN * calibration_speed_mps,
        0.0,
        speed_limited_torque_max_nm,
      )
      base_engine_torque_nm = clip(
        base_engine_torque_nm + grade_torque_nm,
        0.0,
        speed_limited_torque_max_nm,
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

    # Low-speed overrides require the complete stopped sequence. RELEASE and
    # GO remain neutral. Only the subsequent CREEP state can apply the small
    # launch envelope observed in the full factory capture; ordinary low-speed
    # DRIVE still requires independently measured rolling speed.
    if eligible and self.low_speed_state in ("hold", "blocked"):
      desired_engine_torque_nm = 0.0
      desired_brake_accel_mps2 = LOW_SPEED_HOLD_ACCEL_MPS2
      self.brake_latched = True
      self.brake_immediate = False
    elif eligible and self.low_speed_state in ("release", "go"):
      desired_engine_torque_nm = 0.0
      desired_brake_accel_mps2 = 0.0
      self.brake_latched = False
      self.brake_immediate = False
    elif eligible and self.low_speed_state == "creep":
      desired_brake_accel_mps2 = 0.0
      self.brake_latched = False
      self.brake_immediate = False
      if self.propulsion_latched and limited_accel >= LOW_SPEED_LAUNCH_REQUEST_ACCEL:
        launch_grade_torque_nm = clip(
          ENGINE_TORQUE_GRADE_GAIN_NM_PER_RAD * self.filtered_pitch_rad,
          LOW_SPEED_LAUNCH_GRADE_MIN_NM,
          LOW_SPEED_LAUNCH_GRADE_MAX_NM,
        )
        grade_torque_nm = launch_grade_torque_nm
        desired_engine_torque_nm = clip(
          desired_engine_torque_nm + launch_grade_torque_nm,
          0.0,
          LOW_SPEED_LAUNCH_TORQUE_MAX_NM,
        )
      else:
        desired_engine_torque_nm = 0.0
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

    # A GO pulse cannot overlap any residual brake or engine request. After the
    # captured 100 ms pulse, permit the separately bounded launch envelope. If
    # rolling speed is not established within 1.5 seconds, reapply hold and
    # latch the attempt blocked.
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
      filtered_pitch_rad=self.filtered_pitch_rad,
      grade_torque_nm=grade_torque_nm,
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

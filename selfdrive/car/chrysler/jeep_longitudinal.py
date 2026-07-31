from dataclasses import dataclass


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


# Independent host-to-Panda transport gate.
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
ACCEL_MAX = 1.0
ACCEL_DEADBAND = 0.05
COMMAND_DT = 0.02
JERK_UP = 1.0
JERK_DOWN = 2.0

# Match the White Panda's moving-only gate (raw wheel speed 29, approximately
# 2.06 m/s). Stop, go, brake preparation, and hold remain unavailable.
MIN_ACTIVE_SPEED_MPS = 2.1

# The embedded Panda rejects private cycles closer than 15 ms. Even a 20 ms
# sender interval occasionally arrived below that threshold after USB/CAN
# scheduling jitter. The controller offers a cycle every 20 ms, so this 25 ms
# gate deliberately selects every other opportunity (nominally 25 Hz), leaving
# enough arrival-time margin while the White Panda safely holds a complete
# command snapshot between the stock 50 Hz DAS_3 frames.
# Advance the counter only for cycles actually sent.
TRANSPORT_MIN_SEND_INTERVAL_NS = 25_000_000

# Recovered from the Chrysler Advanced implementation and used only to map the
# bounded acceleration request into the existing guarded torque envelope.
# Make the production controller's bounded +1.0 m/s^2 command reach the
# already-enforced 100 Nm host and White Panda ceiling. b6g used 1200/15.5,
# which topped out at only 77.4 Nm and could not maintain speed in the Jeep.
VEHICLE_MASS_SCALE_KG = 1550.0
NON_HYBRID_GEAR_RATIO = 15.5
ENGINE_TORQUE_MAX_NM = 100.0


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
  brake_active: bool
  engine_active: bool
  engine_torque_nm: float
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


def clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


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

  def update(self, requested_accel: float, eligible: bool) -> JeepLongitudinalEnvelope:
    requested_accel = clip(requested_accel, ACCEL_MIN, ACCEL_MAX)

    if not eligible:
      limited_accel = 0.0
    else:
      lower = self.accel_last - JERK_DOWN * COMMAND_DT
      upper = self.accel_last + JERK_UP * COMMAND_DT
      limited_accel = clip(requested_accel, lower, upper)

    self.accel_last = limited_accel
    brake_active = eligible and limited_accel < -ACCEL_DEADBAND
    engine_active = eligible and limited_accel > ACCEL_DEADBAND
    engine_torque_nm = (
      clip(
        limited_accel * VEHICLE_MASS_SCALE_KG / NON_HYBRID_GEAR_RATIO,
        0.0,
        ENGINE_TORQUE_MAX_NM,
      )
      if engine_active else 0.0
    )

    transport_enabled = JEEP_LONG_SHADOW_TRANSPORT_COMPILED and eligible
    return JeepLongitudinalEnvelope(
      requested_accel=requested_accel,
      limited_accel=limited_accel,
      brake_active=brake_active,
      engine_active=engine_active,
      engine_torque_nm=engine_torque_nm,
      transport_enabled=transport_enabled,
      host_enabled=JEEP_LONG_ACTUATION_COMPILED and transport_enabled,
      eligible=eligible,
    )


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

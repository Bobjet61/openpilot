from dataclasses import dataclass


# Independent host-to-Panda transport gate. The b8t branch enables only the
# private transport path; every transmitted actuator field is hard-coded
# neutral and OP_LONG_ENABLE remains false.
JEEP_LONG_SHADOW_TRANSPORT_COMPILED = True

# Independent actuation gate. Enabling this without transport is invalid, and
# both Panda layers retain their own hard-off actuation gates.
JEEP_LONG_ACTUATION_COMPILED = False

if JEEP_LONG_ACTUATION_COMPILED and not JEEP_LONG_SHADOW_TRANSPORT_COMPILED:
  raise RuntimeError("Jeep longitudinal actuation requires shadow transport")

# Stock-log calibration on this EcoDiesel found a DAS_3 braking p01 of
# -3.001015 m/s^2. Keep the shadow envelope just inside that value.
ACCEL_MIN = -3.0
ACCEL_MAX = 1.0
ACCEL_DEADBAND = 0.05
COMMAND_DT = 0.02
JERK_UP = 1.0
JERK_DOWN = 2.0

# Recovered from the Chrysler Advanced implementation. This is logged for
# calibration only; it is not transmitted by this branch.
VEHICLE_MASS_SCALE_KG = 1200.0
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


def clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


def jeep_long_shadow_safety_param(
  safety_param: int,
  shadow_flag: int,
) -> int:
  if JEEP_LONG_SHADOW_TRANSPORT_COMPILED:
    return safety_param | shadow_flag
  return safety_param


class JeepLongitudinalShadow:
  """Rate-limited candidate command generator with a hard compile-off gate."""

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

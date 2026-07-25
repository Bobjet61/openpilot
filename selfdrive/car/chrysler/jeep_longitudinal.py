from dataclasses import dataclass


# This remains false until both Panda layers have independent message-level
# safety policies and replay plus isolated bench validation are complete.
JEEP_LONG_ACTUATION_COMPILED = False

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


@dataclass(frozen=True)
class JeepLongitudinalEnvelope:
  requested_accel: float
  limited_accel: float
  brake_active: bool
  engine_active: bool
  engine_torque_nm: float
  host_enabled: bool
  eligible: bool


def clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


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
    engine_torque_nm = clip(
      max(0.0, limited_accel) * VEHICLE_MASS_SCALE_KG / NON_HYBRID_GEAR_RATIO,
      0.0,
      ENGINE_TORQUE_MAX_NM,
    )

    return JeepLongitudinalEnvelope(
      requested_accel=requested_accel,
      limited_accel=limited_accel,
      brake_active=brake_active,
      engine_active=engine_active,
      engine_torque_nm=engine_torque_nm,
      host_enabled=JEEP_LONG_ACTUATION_COMPILED and eligible,
      eligible=eligible,
    )

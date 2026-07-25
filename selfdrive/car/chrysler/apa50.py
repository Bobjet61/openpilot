from dataclasses import dataclass

from common.conversions import Conversions as CV


# This branch is intentionally non-actuating. Changing this to True is not a
# deployment procedure: the paired White Panda firmware has an independent
# compile-time gate and must be reviewed and validated separately.
APA50_ACTUATION_COMPILED = False

STEER_TYPE_NONE = 0
STEER_TYPE_LKAS = 1
STEER_TYPE_APA = 2

APA_FULL_AUTHORITY_MPH = 5.0
APA_REENTRY_MPH = 48.0
APA_CUTOFF_MPH = 50.0

APA_BASE_MULTIPLIER = 1.0
APA_MAX_MULTIPLIER = 4.0


@dataclass(frozen=True)
class Apa50Envelope:
  speed_mph: float
  extra_authority: float
  torque_multiplier: float
  shadow_steer_type: int
  commanded_steer_type: int


def extra_apa_authority(speed_mph: float) -> float:
  """Fraction of authority above LKAS-equivalent torque.

  Full additional APA authority is available through 5 mph. It then falls
  linearly to zero at 50 mph. Ordinary LKAS-equivalent authority is not part
  of this fraction and remains at 50 mph.
  """
  if speed_mph <= APA_FULL_AUTHORITY_MPH:
    return 1.0
  if speed_mph >= APA_CUTOFF_MPH:
    return 0.0
  return (APA_CUTOFF_MPH - speed_mph) / (APA_CUTOFF_MPH - APA_FULL_AUTHORITY_MPH)


def apa_torque_multiplier(speed_mph: float) -> float:
  return APA_BASE_MULTIPLIER + (APA_MAX_MULTIPLIER - APA_BASE_MULTIPLIER) * extra_apa_authority(speed_mph)


class Apa50ModeController:
  """Stateful APA/LKAS selector with a 2 mph re-entry hysteresis."""

  def __init__(self):
    self.apa_latched = False

  def update(self, speed_mps: float, eligible: bool) -> Apa50Envelope:
    speed_mph = max(0.0, speed_mps * CV.MS_TO_MPH)

    if not eligible or speed_mph >= (APA_CUTOFF_MPH - 1e-6):
      self.apa_latched = False
    elif speed_mph <= (APA_REENTRY_MPH + 1e-6):
      self.apa_latched = True

    shadow_steer_type = STEER_TYPE_APA if self.apa_latched else STEER_TYPE_LKAS
    commanded_steer_type = shadow_steer_type if APA50_ACTUATION_COMPILED else STEER_TYPE_NONE

    return Apa50Envelope(
      speed_mph=speed_mph,
      extra_authority=extra_apa_authority(speed_mph),
      torque_multiplier=apa_torque_multiplier(speed_mph),
      shadow_steer_type=shadow_steer_type,
      commanded_steer_type=commanded_steer_type,
    )

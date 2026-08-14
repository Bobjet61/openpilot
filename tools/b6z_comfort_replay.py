#!/usr/bin/env python3
"""Offline replay of Jeep-specific longitudinal feedback candidates.

The tool reads recorded logs and never publishes CAN. It re-runs openpilot's
production LongControl against the recorded vehicle trajectory, then feeds the
result through the Jeep torque/brake mapper. This can compare command chatter
on identical inputs, but it cannot predict the vehicle trajectory that a new
tune would produce.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import importlib.util
import json
import math
from pathlib import Path
import statistics
import sys
import types


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
  sys.path.append(str(REPO_ROOT))

try:
  import openpilot
  if str(REPO_ROOT) not in openpilot.__path__:
    openpilot.__path__.insert(1, str(REPO_ROOT))
  import openpilot.common
  if str(REPO_ROOT / "common") not in openpilot.common.__path__:
    openpilot.common.__path__.insert(1, str(REPO_ROOT / "common"))
  import openpilot.tools
  import openpilot.tools.lib
  if str(REPO_ROOT / "tools") not in openpilot.tools.__path__:
    openpilot.tools.__path__.append(str(REPO_ROOT / "tools"))
  if str(REPO_ROOT / "tools" / "lib") not in openpilot.tools.lib.__path__:
    openpilot.tools.lib.__path__.append(str(REPO_ROOT / "tools" / "lib"))
except ModuleNotFoundError:
  pass

# LogReader only uses cloudlog for source-fallback notices. Importing the
# device logger on Windows otherwise pulls in Linux I2C hardware modules that
# an offline replay neither needs nor should touch.
if sys.platform == "win32" and "openpilot.common.swaglog" not in sys.modules:
  class _OfflineCloudlog:
    def __getattr__(self, _name):
      return lambda *_args, **_kwargs: None

  swaglog_stub = types.ModuleType("openpilot.common.swaglog")
  swaglog_stub.cloudlog = _OfflineCloudlog()
  sys.modules["openpilot.common.swaglog"] = swaglog_stub

if sys.platform == "win32" and "openpilot.system.hardware.hw" not in sys.modules:
  class _OfflinePaths:
    @staticmethod
    def download_cache_root():
      return str(REPO_ROOT / ".b6z-replay-cache")

  hardware_stub = types.ModuleType("openpilot.system.hardware")
  hardware_stub.__path__ = []
  hardware_stub.PC = True
  hardware_hw_stub = types.ModuleType("openpilot.system.hardware.hw")
  hardware_hw_stub.Paths = _OfflinePaths
  sys.modules["openpilot.system.hardware"] = hardware_stub
  sys.modules["openpilot.system.hardware.hw"] = hardware_hw_stub

if sys.platform == "win32" and "openpilot.tools.lib.filereader" not in sys.modules:
  filereader_stub = types.ModuleType("openpilot.tools.lib.filereader")
  filereader_stub.FileReader = lambda path, debug=False: open(path, "rb")
  filereader_stub.file_exists = lambda path: Path(path).exists()
  filereader_stub.internal_source_available = lambda: False
  sys.modules["openpilot.tools.lib.filereader"] = filereader_stub

# The replay only needs the two control-period constants. Avoid importing the
# device process-priority helpers (and their Linux-only runtime dependencies)
# when this read-only tool is run on Windows.
if "openpilot.common.realtime" not in sys.modules:
  realtime_stub = types.ModuleType("openpilot.common.realtime")
  realtime_stub.DT_CTRL = 0.01
  realtime_stub.DT_MDL = 0.05
  sys.modules["openpilot.common.realtime"] = realtime_stub

from cereal import car  # noqa: E402
from openpilot.common.numpy_fast import interp  # noqa: E402
from openpilot.selfdrive.controls.lib.longcontrol import LongControl  # noqa: E402
from openpilot.selfdrive.modeld.constants import ModelConstants  # noqa: E402

try:
  from openpilot.tools.lib.logreader import LogReader  # noqa: E402
except ModuleNotFoundError:
  from tools.lib.logreader import LogReader  # noqa: E402


LONG_PATH = REPO_ROOT / "selfdrive" / "car" / "chrysler" / "jeep_longitudinal.py"
LONG_SPEC = importlib.util.spec_from_file_location(
  "jeep_longitudinal_b6z_comfort", LONG_PATH,
)
assert LONG_SPEC is not None and LONG_SPEC.loader is not None
LONG = importlib.util.module_from_spec(LONG_SPEC)
sys.modules[LONG_SPEC.name] = LONG
LONG_SPEC.loader.exec_module(LONG)

CONTROL_T = ModelConstants.T_IDXS[:17]


@dataclass(frozen=True)
class Tune:
  name: str
  kp: float
  ki: float
  deadzone: float
  delay_s: float


TUNES = (
  Tune("recorded_defaults", 1.0, 1.0, 0.0, 0.15),
  Tune("jeep_a_delay15", 0.8, 0.30, 0.05, 0.15),
  Tune("jeep_a", 0.8, 0.30, 0.05, 0.30),
  Tune("jeep_b_delay15", 0.6, 0.20, 0.10, 0.15),
  Tune("jeep_b_delay30", 0.6, 0.20, 0.10, 0.30),
  Tune("jeep_b", 0.6, 0.20, 0.10, 0.40),
  Tune("jeep_c_delay15", 0.5, 0.10, 0.10, 0.15),
  Tune("jeep_c", 0.5, 0.10, 0.10, 0.50),
  Tune("jeep_d", 0.4, 0.05, 0.15, 0.60),
)


def make_cp(tune: Tune):
  cp = car.CarParams.new_message()
  cp.enableGasInterceptorDEPRECATED = False
  cp.stopAccel = -2.0
  cp.stoppingDecelRate = 0.8
  cp.vEgoStopping = 0.5
  cp.vEgoStarting = 0.5
  cp.stoppingControl = True
  cp.startingState = False
  cp.startAccel = 0.0
  cp.longitudinalTuning.deadzoneBP = [0.0]
  cp.longitudinalTuning.deadzoneV = [tune.deadzone]
  cp.longitudinalTuning.kf = 1.0
  cp.longitudinalTuning.kpBP = [0.0]
  cp.longitudinalTuning.kpV = [tune.kp]
  cp.longitudinalTuning.kiBP = [0.0]
  cp.longitudinalTuning.kiV = [tune.ki]
  cp.longitudinalActuatorDelayLowerBound = tune.delay_s
  cp.longitudinalActuatorDelayUpperBound = tune.delay_s
  return cp


def finite(value: object, default: float = 0.0) -> float:
  try:
    result = float(value)
  except (TypeError, ValueError, OverflowError):
    return default
  return result if math.isfinite(result) else default


def percentile(values: list[float], fraction: float) -> float | None:
  if not values:
    return None
  ordered = sorted(values)
  index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * fraction)))
  return ordered[index]


def correlation(xs: list[float], ys: list[float]) -> float | None:
  if len(xs) < 3 or len(xs) != len(ys):
    return None
  x_mean = statistics.fmean(xs)
  y_mean = statistics.fmean(ys)
  covariance = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))
  x_variance = sum((x - x_mean) ** 2 for x in xs)
  y_variance = sum((y - y_mean) ** 2 for y in ys)
  if x_variance <= 0.0 or y_variance <= 0.0:
    return None
  return covariance / math.sqrt(x_variance * y_variance)


def response_lag_screen(samples: list[dict[str, object]]) -> list[dict[str, object]]:
  output = []
  difference_span = 5  # 0.5 s at qlog's 10 Hz carControl rate
  for lag_steps in range(0, 11):
    torque_levels: list[float] = []
    accel_levels: list[float] = []
    torque_deltas: list[float] = []
    accel_deltas: list[float] = []
    for index in range(difference_span, len(samples) - lag_steps):
      response_index = index + lag_steps
      command_before = samples[index - difference_span]
      command_now = samples[index]
      response_before = samples[response_index - difference_span]
      response_now = samples[response_index]
      expected_lag = lag_steps * 0.1
      actual_lag = float(response_now["time_s"]) - float(command_now["time_s"])
      command_span = float(command_now["time_s"]) - float(command_before["time_s"])
      response_span = float(response_now["time_s"]) - float(response_before["time_s"])
      if (
          abs(actual_lag - expected_lag) > 0.06
          or not 0.40 <= command_span <= 0.60
          or not 0.40 <= response_span <= 0.60
          or not all(bool(sample["active"]) for sample in (
            command_before, command_now, response_before, response_now,
          ))
          or any(sample["recorded_torque_nm"] is None for sample in (
            command_before, command_now,
          ))
      ):
        continue
      torque_before = float(command_before["recorded_torque_nm"])
      torque_now = float(command_now["recorded_torque_nm"])
      accel_before = float(response_before["actual_accel_mps2"])
      accel_now = float(response_now["actual_accel_mps2"])
      torque_levels.append(torque_now)
      accel_levels.append(accel_now)
      torque_deltas.append(torque_now - torque_before)
      accel_deltas.append(accel_now - accel_before)
    output.append({
      "lag_s": lag_steps * 0.1,
      "pairs": len(torque_levels),
      "level_correlation": correlation(torque_levels, accel_levels),
      "half_second_change_correlation": correlation(torque_deltas, accel_deltas),
    })
  return output


def mode_direction_changes(samples: list[dict[str, object]]) -> int:
  compressed: list[tuple[float, str]] = []
  for sample in samples:
    mode = str(sample["mode"])
    if mode not in ("engine", "brake"):
      continue
    if not compressed or compressed[-1][1] != mode:
      compressed.append((float(sample["time_s"]), mode))
  return sum(
    before[1] != after[1] and after[0] - before[0] <= 3.0
    for before, after in zip(compressed, compressed[1:])
  )


def grade_filter_screen(
    samples: list[dict[str, object]], tau_s: float,
) -> dict[str, float | int | None]:
  filtered_pitch = 0.0
  previous_time: float | None = None
  values: list[float] = []
  steps: list[float] = []
  active_seconds = 0.0
  for sample in samples:
    timestamp = float(sample["time_s"])
    elapsed = 0.02 if previous_time is None else timestamp - previous_time
    previous_time = timestamp
    continuous = 0.0 < elapsed <= 0.25
    if not continuous:
      filtered_pitch = 0.0
      elapsed = 0.02
    active_seconds += elapsed
    substeps = max(1, round(elapsed / 0.02))
    bounded_pitch = min(max(
      float(sample["pitch_rad"]),
      -LONG.ENGINE_TORQUE_GRADE_PITCH_LIMIT_RAD,
    ), LONG.ENGINE_TORQUE_GRADE_PITCH_LIMIT_RAD)
    alpha = 0.02 / (tau_s + 0.02)
    for _ in range(substeps):
      filtered_pitch += alpha * (bounded_pitch - filtered_pitch)
    grade = min(max(
      LONG.ENGINE_TORQUE_GRADE_GAIN_NM_PER_RAD * filtered_pitch,
      LONG.ENGINE_TORQUE_GRADE_MIN_NM,
    ), LONG.ENGINE_TORQUE_GRADE_MAX_NM)
    if values and continuous:
      steps.append(abs(grade - values[-1]))
    values.append(grade)
  return {
    "tau_s": tau_s,
    "samples": len(values),
    "p05_nm": percentile(values, 0.05),
    "p95_nm": percentile(values, 0.95),
    "positive_saturation_fraction": (
      sum(value >= LONG.ENGINE_TORQUE_GRADE_MAX_NM - 0.01 for value in values) / len(values)
      if values else None
    ),
    "total_variation_per_s": (
      sum(steps) / active_seconds if active_seconds > 0.0 else None
    ),
    "max_step_nm": max(steps) if steps else None,
  }


def summarize(tune: Tune, samples: list[dict[str, object]]) -> dict[str, object]:
  active = [sample for sample in samples if bool(sample["active"])]
  pid = [sample for sample in active if str(sample["long_state"]) == "pid"]
  if not active:
    return {"tune": tune.__dict__, "active_samples": 0}

  intervals = [
    float(after["time_s"]) - float(before["time_s"])
    for before, after in zip(active, active[1:])
    if 0.0 < float(after["time_s"]) - float(before["time_s"]) <= 0.15
  ]
  output_steps = [
    abs(float(after["output_accel"]) - float(before["output_accel"]))
    for before, after in zip(active, active[1:])
    if 0.0 < float(after["time_s"]) - float(before["time_s"]) <= 0.15
  ]
  active_seconds = sum(intervals)

  steady = [
    sample for sample in pid
    if str(sample["source"]) == "cruise"
    and not bool(sample["has_lead"])
    and float(sample["speed_mps"]) >= 8.0
    and abs(float(sample["speed_error_mps"])) <= 0.75
  ]
  steady_steps = [
    abs(float(after["output_accel"]) - float(before["output_accel"]))
    for before, after in zip(steady, steady[1:])
    if 0.0 < float(after["time_s"]) - float(before["time_s"]) <= 0.15
  ]
  large_deficit = [
    sample for sample in pid
    if str(sample["source"]) == "cruise"
    and not bool(sample["has_lead"])
    and float(sample["speed_mps"]) >= 5.0
    and float(sample["speed_error_mps"]) >= 2.0
  ]
  planned_deceleration = [
    sample for sample in pid
    if float(sample["target_accel_mps2"]) <= -0.5
  ]
  recorded_errors = [
    abs(float(sample["output_accel"]) - float(sample["recorded_accel"]))
    for sample in pid
  ]
  grade_values = [float(sample["grade_torque_nm"]) for sample in active]
  pitch_samples = [
    sample for sample in active if float(sample["speed_mps"]) >= 8.0
  ]
  pitch_values = [float(sample["pitch_rad"]) for sample in pitch_samples]
  measured_accels = [float(sample["actual_accel_mps2"]) for sample in pitch_samples]
  pitch_accel_slope = None
  pitch_accel_correlation = None
  if len(pitch_values) >= 2:
    pitch_mean = statistics.fmean(pitch_values)
    accel_mean = statistics.fmean(measured_accels)
    covariance = sum(
      (pitch - pitch_mean) * (accel - accel_mean)
      for pitch, accel in zip(pitch_values, measured_accels)
    )
    pitch_variance = sum((pitch - pitch_mean) ** 2 for pitch in pitch_values)
    accel_variance = sum((accel - accel_mean) ** 2 for accel in measured_accels)
    if accel_variance > 0.0:
      pitch_accel_slope = covariance / accel_variance
    if pitch_variance > 0.0 and accel_variance > 0.0:
      pitch_accel_correlation = covariance / math.sqrt(
        pitch_variance * accel_variance,
      )

  return {
    "tune": tune.__dict__,
    "active_samples": len(active),
    "active_seconds": round(active_seconds, 3),
    "pid_samples": len(pid),
    "recorded_accel_mae_mps2": (
      round(statistics.fmean(recorded_errors), 4) if recorded_errors else None
    ),
    "output_total_variation_per_s": (
      round(sum(output_steps) / active_seconds, 4) if active_seconds else None
    ),
    "output_step_mps2": {
      "p95": percentile(output_steps, 0.95),
      "p99": percentile(output_steps, 0.99),
      "max": max(output_steps) if output_steps else None,
    },
    "mapped_mode_direction_changes_within_3s": mode_direction_changes(active),
    "mapped_mode_counts": {
      mode: sum(str(sample["mode"]) == mode for sample in active)
      for mode in ("engine", "coast", "brake")
    },
    "steady_cruise": {
      "samples": len(steady),
      "mean_abs_output_accel_mps2": (
        round(statistics.fmean(abs(float(sample["output_accel"])) for sample in steady), 4)
        if steady else None
      ),
      "output_step_p99_mps2": percentile(steady_steps, 0.99),
      "output_total_variation": round(sum(steady_steps), 4),
    },
    "large_speed_deficit": {
      "samples": len(large_deficit),
      "mean_output_accel_mps2": (
        round(statistics.fmean(float(sample["output_accel"]) for sample in large_deficit), 4)
        if large_deficit else None
      ),
      "p10_output_accel_mps2": percentile(
        [float(sample["output_accel"]) for sample in large_deficit], 0.10,
      ),
      "fraction_at_or_above_1_5": (
        round(sum(float(sample["output_accel"]) >= 1.5 for sample in large_deficit) / len(large_deficit), 4)
        if large_deficit else None
      ),
    },
    "planned_deceleration": {
      "samples": len(planned_deceleration),
      "mean_target_accel_mps2": (
        round(statistics.fmean(
          float(sample["target_accel_mps2"]) for sample in planned_deceleration
        ), 4) if planned_deceleration else None
      ),
      "mean_output_accel_mps2": (
        round(statistics.fmean(
          float(sample["output_accel"]) for sample in planned_deceleration
        ), 4) if planned_deceleration else None
      ),
      "p10_output_accel_mps2": percentile(
        [float(sample["output_accel"]) for sample in planned_deceleration], 0.10,
      ),
      "p90_output_accel_mps2": percentile(
        [float(sample["output_accel"]) for sample in planned_deceleration], 0.90,
      ),
      "fraction_at_or_below_minus_0_4": (
        round(sum(
          float(sample["output_accel"]) <= -0.4
          for sample in planned_deceleration
        ) / len(planned_deceleration), 4) if planned_deceleration else None
      ),
    },
    "grade_torque_nm": {
      "p05": percentile(grade_values, 0.05),
      "p95": percentile(grade_values, 0.95),
      "max_abs": max((abs(value) for value in grade_values), default=None),
    },
    "raw_pitch": {
      "samples": len(pitch_values),
      "p05_rad": percentile(pitch_values, 0.05),
      "p95_rad": percentile(pitch_values, 0.95),
      "pitch_per_measured_accel_rad_per_mps2": pitch_accel_slope,
      "measured_accel_correlation": pitch_accel_correlation,
    },
    "grade_filter_screen": [
      grade_filter_screen(active, tau_s) for tau_s in (0.75, 1.5, 2.5)
    ],
    "recorded_propulsion_response_lag_screen": response_lag_screen(active),
  }


def replay_file(path: Path) -> dict[str, list[dict[str, object]]]:
  controllers = {tune.name: LongControl(make_cp(tune)) for tune in TUNES}
  shadows = {tune.name: LONG.JeepLongitudinalShadow() for tune in TUNES}
  output = {tune.name: [] for tune in TUNES}
  car_state = None
  plan = None
  plan_time_ns: int | None = None
  first_ns: int | None = None
  control_index = 0
  last_control_ns: int | None = None
  recorded_torque_nm: float | None = None

  for msg in LogReader(str(path), sort_by_time=True):
    mono_ns = int(msg.logMonoTime)
    if first_ns is None:
      first_ns = mono_ns
    which = msg.which()
    if which == "carState":
      car_state = msg.carState
      continue
    if which == "longitudinalPlan":
      plan = msg.longitudinalPlan
      plan_time_ns = mono_ns
      continue
    if which == "sendcan":
      for frame in msg.sendcan:
        if int(frame.address) != 0x272:
          continue
        dat = bytes(frame.dat)
        if len(dat) != 8:
          continue
        torque_raw = ((dat[4] & 0x1F) << 8) | dat[5]
        recorded_torque_nm = (
          torque_raw * 0.25 - 500.0 if dat[4] & 0x80 else 0.0
        )
      continue
    if which != "carControl" or car_state is None or plan is None or plan_time_ns is None:
      continue

    control = msg.carControl
    time_s = mono_ns / 1e9
    plan_age_s = max(0.0, (mono_ns - plan_time_ns) / 1e9)
    speeds = list(plan.speeds)
    accels = list(plan.accels)
    target_speed = (
      finite(interp(plan_age_s, CONTROL_T, speeds), finite(car_state.vEgo))
      if len(speeds) == len(CONTROL_T) else finite(car_state.vEgo)
    )
    target_accel = (
      finite(interp(plan_age_s, CONTROL_T, accels))
      if len(accels) == len(CONTROL_T) else 0.0
    )
    pitch_rad = (
      finite(control.orientationNED[1]) if len(control.orientationNED) > 1 else 0.0
    )
    active = (
      bool(control.longActive)
      and bool(car_state.cruiseState.available)
      and not bool(car_state.gasPressed)
      and not bool(car_state.brakePressed)
    )
    elapsed_s = (
      0.01 if last_control_ns is None else max(0.01, (mono_ns - last_control_ns) / 1e9)
    )
    last_control_ns = mono_ns
    # qlogs retain carControl at 10 Hz. Recreate the missing 100 Hz controller
    # ticks with the latest recorded state/plan so PID integration and the
    # 50 Hz Jeep mapper run at their production rates.
    substeps = min(25, max(1, round(elapsed_s / 0.01)))
    requested_by_tune: dict[str, float] = {}
    envelope_by_tune = {}
    for substep in range(substeps):
      substep_plan_age_s = max(
        0.0, plan_age_s - (substeps - 1 - substep) * 0.01,
      )
      mapper_cycle = control_index % 2 == 0
      for tune in TUNES:
        requested = controllers[tune.name].update(
          bool(control.longActive), car_state, plan, [-3.5, 2.0],
          substep_plan_age_s,
        )
        requested_by_tune[tune.name] = requested
        if mapper_cycle:
          envelope_by_tune[tune.name] = shadows[tune.name].update(
            requested,
            eligible=active,
            speed_mps=finite(car_state.vEgo),
            pitch_rad=pitch_rad,
          )
      control_index += 1

    for tune in TUNES:
      requested = requested_by_tune[tune.name]
      envelope = envelope_by_tune[tune.name]
      output[tune.name].append({
        "time_s": time_s,
        "active": active,
        "long_state": str(control.actuators.longControlState),
        "recorded_accel": finite(control.actuators.accel),
        "output_accel": requested,
        "speed_mps": finite(car_state.vEgo),
        "target_speed_mps": target_speed,
        "target_accel_mps2": target_accel,
        "speed_error_mps": target_speed - finite(car_state.vEgo),
        "source": str(plan.longitudinalPlanSource),
        "has_lead": bool(plan.hasLead),
        "mode": envelope.command_mode,
        "torque_nm": envelope.engine_torque_nm,
        "brake_accel_mps2": envelope.brake_accel_mps2,
        "grade_torque_nm": envelope.grade_torque_nm,
        "pitch_rad": pitch_rad,
        "actual_accel_mps2": finite(car_state.aEgo),
        "recorded_torque_nm": recorded_torque_nm,
      })
  return output


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("rlogs", nargs="+", type=Path)
  parser.add_argument("--json", dest="json_path", required=True, type=Path)
  args = parser.parse_args()

  combined = {tune.name: [] for tune in TUNES}
  for path in args.rlogs:
    result = replay_file(path)
    for tune in TUNES:
      combined[tune.name].extend(result[tune.name])

  report = {
    "inputs": [str(path.resolve()) for path in args.rlogs],
    "limitations": (
      "Recorded-trajectory replay compares command behavior on identical "
      "inputs; a vehicle test remains necessary to validate closed-loop response."
    ),
    "results": [summarize(tune, combined[tune.name]) for tune in TUNES],
  }
  args.json_path.parent.mkdir(parents=True, exist_ok=True)
  args.json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(report, indent=2))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())

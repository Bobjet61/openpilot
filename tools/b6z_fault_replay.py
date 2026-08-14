#!/usr/bin/env python3
"""Replay the b6z Jeep torque mapper against a recorded rlog.

This is an offline analysis tool. It reads recorded messages, never publishes
CAN, and cannot command a vehicle.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import statistics
import sys
import types
import os

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
  sys.path.append(str(REPO_ROOT))

if sys.platform == "win32" and "openpilot" not in sys.modules:
  openpilot_package = types.ModuleType("openpilot")
  openpilot_package.__path__ = [str(REPO_ROOT)]
  sys.modules["openpilot"] = openpilot_package
  if not hasattr(os, "register_at_fork"):
    os.register_at_fork = lambda **_kwargs: None

try:
  import openpilot
  import openpilot.tools
  import openpilot.tools.lib
  if str(REPO_ROOT / "tools") not in openpilot.tools.__path__:
    openpilot.tools.__path__.append(str(REPO_ROOT / "tools"))
  if str(REPO_ROOT / "tools" / "lib") not in openpilot.tools.lib.__path__:
    openpilot.tools.lib.__path__.append(str(REPO_ROOT / "tools" / "lib"))
except ModuleNotFoundError:
  pass

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

try:
  from openpilot.tools.lib.logreader import LogReader
except ModuleNotFoundError:
  from tools.lib.logreader import LogReader


LONG_PATH = REPO_ROOT / "selfdrive" / "car" / "chrysler" / "jeep_longitudinal.py"
LONG_SPEC = importlib.util.spec_from_file_location(
  "jeep_longitudinal_b6z_replay", LONG_PATH,
)
assert LONG_SPEC is not None and LONG_SPEC.loader is not None
LONG = importlib.util.module_from_spec(LONG_SPEC)
sys.modules[LONG_SPEC.name] = LONG
LONG_SPEC.loader.exec_module(LONG)


def finite(value: object, default: float = 0.0) -> float:
  try:
    result = float(value)
  except (TypeError, ValueError, OverflowError):
    return default
  return result if math.isfinite(result) else default


def command_diag(dat: bytes) -> dict[str, float | bool] | None:
  if len(dat) != 8 or dat[0] != 0xC1 or dat[1] != 1:
    return None
  stock_raw = ((dat[2] & 0x1F) << 8) | dat[3]
  output_raw = ((dat[4] & 0x1F) << 8) | dat[5]
  return {
    "stock_active": bool(dat[2] >> 7),
    "stock_torque_nm": stock_raw * 0.25 - 500.0,
    "output_active": bool(dat[4] >> 7),
    "output_torque_nm": output_raw * 0.25 - 500.0,
  }


def percentile(values: list[float], fraction: float) -> float | None:
  if not values:
    return None
  ordered = sorted(values)
  index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * fraction)))
  return ordered[index]


def contiguous_duration_before(
    samples: list[tuple[float, float]],
    end_time: float,
    threshold: float,
) -> float:
  retained = [sample for sample in samples if sample[0] < end_time]
  if not retained:
    return 0.0
  index = len(retained) - 1
  while index >= 0 and retained[index][1] >= threshold:
    index -= 1
  start_index = index + 1
  if start_index >= len(retained):
    return 0.0
  return max(0.0, retained[-1][0] - retained[start_index][0])


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("rlog", type=Path)
  parser.add_argument("--json", dest="json_path", type=Path, required=True)
  args = parser.parse_args()

  shadow = LONG.JeepLongitudinalShadow()
  scheduler = LONG.JeepLongitudinalTransportScheduler()
  first_mono_ns: int | None = None
  state: dict[str, object] | None = None
  fault_time_s: float | None = None
  prior_fault = False
  control_times: list[float] = []
  mapper_times: list[float] = []
  candidate_samples: list[dict[str, float | bool | str]] = []
  candidate_transmitted_torque: list[tuple[float, float]] = []
  actual_samples: list[tuple[float, float]] = []
  stock_samples: list[tuple[float, float]] = []
  diagnostic_logs: list[dict[str, float | str]] = []

  for msg in LogReader(str(args.rlog), sort_by_time=True):
    mono_ns = int(msg.logMonoTime)
    if first_mono_ns is None:
      first_mono_ns = mono_ns
    time_s = (mono_ns - first_mono_ns) / 1e9
    which = msg.which()

    if which == "logMessage":
      message_text = str(msg.logMessage)
      if (
          "Jeep radar assist:" in message_text
          or "Jeep long shadow:" in message_text
      ):
        diagnostic_logs.append({"time_s": time_s, "text": message_text})
    elif which == "carState":
      car_state = msg.carState
      faulted = bool(car_state.accFaulted)
      if faulted and not prior_fault and fault_time_s is None:
        fault_time_s = time_s
      prior_fault = faulted
      state = {
        "v_ego": finite(car_state.vEgo),
        "a_ego": finite(car_state.aEgo),
        "faulted": faulted,
        "cruise_available": bool(car_state.cruiseState.available),
        "gear_drive": str(car_state.gearShifter) == "drive",
        "door_open": bool(car_state.doorOpen),
        "seatbelt_unlatched": bool(car_state.seatbeltUnlatched),
        "gas_pressed": bool(car_state.gasPressed),
        "brake_pressed": bool(car_state.brakePressed),
        "stock_aeb": bool(car_state.stockAeb),
      }
    elif which == "can":
      for can_msg in msg.can:
        if int(can_msg.address) != 0x4FE:
          continue
        decoded = command_diag(bytes(can_msg.dat))
        if decoded is None:
          continue
        if decoded["output_active"]:
          actual_samples.append((time_s, float(decoded["output_torque_nm"])))
        if decoded["stock_active"]:
          stock_samples.append((time_s, float(decoded["stock_torque_nm"])))
    elif which == "carControl" and state is not None:
      control = msg.carControl
      control_times.append(time_s)
      # CarController receives carControl at 100 Hz but updates the Jeep
      # longitudinal mapper only on even frames (50 Hz).
      if (len(control_times) - 1) % 2 != 0:
        continue
      mapper_times.append(time_s)
      pitch_rad = 0.0
      try:
        if len(control.orientationNED) > 1:
          pitch_rad = finite(control.orientationNED[1])
      except Exception:
        pitch_rad = 0.0

      eligible = (
        bool(control.enabled)
        and bool(control.longActive)
        and bool(state["cruise_available"])
        and bool(state["gear_drive"])
        and not bool(state["faulted"])
        and not bool(state["door_open"])
        and not bool(state["seatbelt_unlatched"])
        and not bool(state["gas_pressed"])
        and not bool(state["brake_pressed"])
        and not bool(state["stock_aeb"])
      )
      requested_accel = finite(control.actuators.accel)
      envelope = shadow.update(
        requested_accel,
        eligible=eligible,
        speed_mps=float(state["v_ego"]),
        pitch_rad=pitch_rad,
      )
      transport_counter = scheduler.next_counter(
        mono_ns, envelope.transport_enabled,
      )
      if transport_counter is not None:
        if envelope.engine_active:
          candidate_transmitted_torque.append(
            (time_s, envelope.engine_torque_nm),
          )
        shadow.note_transport_sent(envelope)
      candidate_samples.append({
        "time_s": time_s,
        "eligible": envelope.eligible,
        "requested_accel_mps2": requested_accel,
        "openpilot_set_speed_mps": finite(control.hudControl.setSpeed),
        "limited_accel_mps2": envelope.limited_accel,
        "v_ego_mps": float(state["v_ego"]),
        "a_ego_mps2": float(state["a_ego"]),
        "pitch_rad": pitch_rad,
        "torque_nm": envelope.engine_torque_nm,
        "brake_accel_mps2": envelope.brake_accel_mps2,
        "mode": envelope.command_mode,
      })

  if first_mono_ns is None:
    raise RuntimeError("rlog contained no messages")

  if fault_time_s is None:
    fault_time_s = math.inf
  pre_fault_candidate = [
    sample for sample in candidate_samples
    if float(sample["time_s"]) < fault_time_s and bool(sample["eligible"])
  ]
  pre_fault_actual = [sample for sample in actual_samples if sample[0] < fault_time_s]
  control_intervals = [
    later - earlier
    for earlier, later in zip(control_times, control_times[1:])
    if later > earlier
  ]
  mapper_intervals = [
    later - earlier
    for earlier, later in zip(mapper_times, mapper_times[1:])
    if later > earlier
  ]
  torque_values = [float(sample["torque_nm"]) for sample in pre_fault_candidate]
  actual_torque_values = [sample[1] for sample in pre_fault_actual]
  actual_positive_steps = [
    later[1] - earlier[1]
    for earlier, later in zip(pre_fault_actual, pre_fault_actual[1:])
    if 0.0 < later[0] - earlier[0] <= 0.05 and later[1] > earlier[1]
  ]
  candidate_positive_steps = [
    later[1] - earlier[1]
    for earlier, later in zip(
      candidate_transmitted_torque, candidate_transmitted_torque[1:],
    )
    if 0.0 < later[0] - earlier[0] <= 0.06 and later[1] > earlier[1]
  ]

  candidate_at_fault = pre_fault_candidate[-1] if pre_fault_candidate else None
  actual_at_fault = pre_fault_actual[-1] if pre_fault_actual else None
  summary = {
    "input": str(args.rlog.resolve()),
    # logMonoTime retains the route clock in this extracted segment, so this
    # timestamp is route-relative rather than time since this file starts.
    "fault_time_from_route_start_s": (
      None if not math.isfinite(fault_time_s) else fault_time_s
    ),
    "car_control": {
      "samples": len(control_times),
      "median_interval_s": (
        statistics.median(control_intervals) if control_intervals else None
      ),
      "p05_interval_s": percentile(control_intervals, 0.05),
      "p95_interval_s": percentile(control_intervals, 0.95),
      "mapper_samples": len(mapper_times),
      "mapper_median_interval_s": (
        statistics.median(mapper_intervals) if mapper_intervals else None
      ),
    },
    "recorded_output": {
      "active_samples_before_fault": len(pre_fault_actual),
      "max_torque_nm": max(actual_torque_values) if actual_torque_values else None,
      "last_active_sample_before_fault": (
        {"time_s": actual_at_fault[0], "torque_nm": actual_at_fault[1]}
        if actual_at_fault is not None else None
      ),
      "continuous_time_at_or_above_439_5_nm_before_fault_s": (
        contiguous_duration_before(actual_samples, fault_time_s, 439.5)
      ),
      "positive_torque_step_nm": {
        "p95": percentile(actual_positive_steps, 0.95),
        "p99": percentile(actual_positive_steps, 0.99),
        "max": max(actual_positive_steps) if actual_positive_steps else None,
      },
    },
    "candidate_b6z": {
      "eligible_samples_before_fault": len(pre_fault_candidate),
      "max_torque_nm": max(torque_values) if torque_values else None,
      "p95_torque_nm": percentile(torque_values, 0.95),
      "last_sample_before_fault": candidate_at_fault,
      "continuous_time_at_or_above_439_5_nm_before_fault_s": (
        contiguous_duration_before(
          [(float(sample["time_s"]), float(sample["torque_nm"]))
           for sample in pre_fault_candidate],
          fault_time_s,
          439.5,
        )
      ),
      "configured_absolute_ceiling_nm": LONG.ENGINE_TORQUE_MAX_NM,
      "configured_rate_up_nm_per_s": LONG.ENGINE_TORQUE_RATE_UP_NM_PER_S,
      "configured_positive_accel_gain": LONG.ENGINE_TORQUE_POSITIVE_ACCEL_GAIN,
      "transmitted_active_samples": len(candidate_transmitted_torque),
      "transmitted_positive_torque_step_nm": {
        "p95": percentile(candidate_positive_steps, 0.95),
        "p99": percentile(candidate_positive_steps, 0.99),
        "max": (
          max(candidate_positive_steps) if candidate_positive_steps else None
        ),
      },
    },
    "recorded_factory_active_torque": {
      "samples": len(stock_samples),
      "max_torque_nm": (
        max(sample[1] for sample in stock_samples) if stock_samples else None
      ),
    },
    "last_controller_diagnostic_logs_before_fault": [
      entry for entry in diagnostic_logs
      if float(entry["time_s"]) < fault_time_s
    ][-12:],
  }

  args.json_path.parent.mkdir(parents=True, exist_ok=True)
  args.json_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(summary, indent=2))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())

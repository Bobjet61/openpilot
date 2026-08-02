#!/usr/bin/env python3
import unittest
from panda import Panda
from panda.tests.libpanda import libpanda_py
import panda.tests.safety.common as common
from panda.tests.safety.common import CANPackerPanda


class TestChryslerSafety(common.PandaCarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [[0x23B, 0], [0x292, 0], [0x2A6, 0], [0x1F4, 0]]
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS = {0: (0x292,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x292, 0x2A6]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 3
  MAX_TORQUE = 261
  MAX_RT_DELTA = 112
  RT_INTERVAL = 250000
  MAX_TORQUE_ERROR = 80

  LKAS_ACTIVE_VALUE = 1

  DAS_BUS = 0
  SAFETY_PARAM = 0

  def setUp(self):
    self.packer = CANPackerPanda("chrysler_pacifica_2017_hybrid_generated")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_CHRYSLER, self.SAFETY_PARAM)
    self.safety.init_tests()

  def _button_msg(self, cancel=False, resume=False):
    values = {"ACC_Cancel": cancel, "ACC_Resume": resume}
    return self.packer.make_can_msg_panda("CRUISE_BUTTONS", self.DAS_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_ACTIVE": enable}
    return self.packer.make_can_msg_panda("DAS_3", self.DAS_BUS, values)

  def _das_3_msg(self, counter=1, **changes):
    values = {"COUNTER": counter}
    values.update(changes)
    return self.packer.make_can_msg_panda("DAS_3", self.DAS_BUS, values)

  def _speed_msg(self, speed):
    values = {"SPEED_LEFT": speed, "SPEED_RIGHT": speed}
    return self.packer.make_can_msg_panda("SPEED_1", 0, values)

  def _user_gas_msg(self, gas):
    values = {"Accelerator_Position": gas}
    return self.packer.make_can_msg_panda("ECM_5", 0, values)

  def _user_brake_msg(self, brake):
    values = {"Brake_Pedal_State": 1 if brake else 0}
    return self.packer.make_can_msg_panda("ESP_1", 0, values)

  def _torque_meas_msg(self, torque):
    values = {"EPS_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_panda("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEERING_TORQUE": torque, "LKAS_CONTROL_BIT": self.LKAS_ACTIVE_VALUE if steer_req else 0}
    return self.packer.make_can_msg_panda("LKAS_COMMAND", 0, values)

  def test_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      # resume only while controls allowed
      self.assertEqual(controls_allowed, self._tx(self._button_msg(resume=True)))

      # can always cancel
      self.assertTrue(self._tx(self._button_msg(cancel=True)))

      # only one button at a time
      self.assertFalse(self._tx(self._button_msg(cancel=True, resume=True)))
      self.assertFalse(self._tx(self._button_msg(cancel=False, resume=False)))


  def test_auto_resume_at_standstill(self):
    if self.DAS_BUS != 0:
      self.skipTest("Jeep/Pacifica only")

    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    self.assertTrue(self._rx(self._das_3_msg(counter=1, ACC_AVAILABLE=1)))
    self._rx(self._speed_msg(0))
    self.assertFalse(self.safety.get_longitudinal_allowed())
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    self._rx(self._speed_msg(1))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    self._rx(self._speed_msg(0))
    self._rx(self._das_3_msg(counter=2, ACC_AVAILABLE=0))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

  def test_das_3_brake_hold(self):
    if self.DAS_BUS != 0:
      self.skipTest("Jeep/Pacifica only")

    hold = {"ACC_AVAILABLE": 1, "ACC_ACTIVE": 1, "ACC_DECEL_REQ": 1,
            "ACC_DECEL": -2.0, "GR_MAX_REQ": 2}

    self.assertTrue(self._rx(self._das_3_msg(counter=1, ACC_AVAILABLE=1)))
    self.assertFalse(self._tx(self._das_3_msg(counter=3, **hold)))


class TestChryslerB6yHoldSafety(common.PandaSafetyTestBase):
  # This focused class tests a payload-restricted subset of the same Chrysler
  # safety mode; it is not a separate cross-mode TX allowlist.
  TX_MSGS = None
  SOURCE_TIMEOUT_US = 100_000
  HOLD_PARAM = (
    Panda.FLAG_CHRYSLER_JEEP_LONG_SHADOW |
    Panda.FLAG_CHRYSLER_JEEP_LONG_ACTUATION
  )

  def setUp(self):
    self.packer = CANPackerPanda("chrysler_pacifica_2017_hybrid_generated")
    self.safety = libpanda_py.libpanda
    self._reset_hold_safety()

  def _reset_hold_safety(self):
    self.safety.set_safety_hooks(Panda.SAFETY_CHRYSLER, self.HOLD_PARAM)
    self.safety.init_tests()
    self.safety.set_controls_allowed(False)

  def _button_msg(self, cancel=False, resume=False):
    values = {"ACC_Cancel": cancel, "ACC_Resume": resume}
    return self.packer.make_can_msg_panda("CRUISE_BUTTONS", 0, values)

  def _das_3_msg(self, counter=1, **changes):
    values = {"COUNTER": counter}
    values.update(changes)
    return self.packer.make_can_msg_panda("DAS_3", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED_LEFT": speed, "SPEED_RIGHT": speed}
    return self.packer.make_can_msg_panda("SPEED_1", 0, values)

  def _user_gas_msg(self, gas):
    values = {"Accelerator_Position": gas}
    return self.packer.make_can_msg_panda("ECM_5", 0, values)

  def _user_brake_msg(self, brake):
    values = {"Brake_Pedal_State": 1 if brake else 0}
    return self.packer.make_can_msg_panda("ESP_1", 0, values)

  def _hold_msg(self, counter=3, **changes):
    values = {
      "ACC_AVAILABLE": 1,
      "ACC_ACTIVE": 1,
      "ACC_DECEL_REQ": 1,
      "ACC_DECEL": -2.0,
      "GR_MAX_REQ": 2,
    }
    values.update(changes)
    return self._das_3_msg(counter=counter, **values)

  def _enable_hold_sources(self, counter=1, time_us=1_000_000):
    self.safety.set_timer(time_us)
    self.assertTrue(self._rx(self._das_3_msg(
      counter=counter, ACC_AVAILABLE=1, ACC_ACTIVE=0,
    )))
    self.assertTrue(self._rx(self._speed_msg(0)))
    self.assertTrue(self._rx(self._user_gas_msg(0)))
    self.assertTrue(self._rx(self._user_brake_msg(False)))

  def test_exact_hold_frame_unlocks_only_recent_resume(self):
    self.assertFalse(self._tx(self._button_msg(resume=True)))
    self._enable_hold_sources()
    self.assertTrue(self._tx(self._hold_msg()))
    self.assertTrue(self._tx(self._button_msg(resume=True)))

    self.safety.set_timer(1_000_000 + self.SOURCE_TIMEOUT_US + 1)
    self.assertFalse(self._tx(self._button_msg(resume=True)))

  def test_hold_requires_exact_proven_brake_only_payload(self):
    self._enable_hold_sources()
    self.assertTrue(self._tx(self._hold_msg()))

    for changes in (
      {"ACC_DECEL": -1.9},
      {"ACC_DECEL": -2.1},
      {"ACC_GO": 1},
      {"ACC_STANDSTILL": 1},
      {"ENGINE_TORQUE_REQUEST_MAX": 1},
      {"GR_MAX_REQ": 3},
      {"ACC_BRK_PREP": 1},
      {"ACC_DECEL_REQ": 0},
    ):
      self._reset_hold_safety()
      self._enable_hold_sources()
      self.assertFalse(self._tx(self._hold_msg(**changes)))

  def test_hold_rejects_bad_counter_and_bad_checksum(self):
    self._enable_hold_sources(counter=1)
    self.assertFalse(self._tx(self._hold_msg(counter=2)))
    self.assertFalse(self._tx(self._hold_msg(counter=5)))

    msg = self._hold_msg(counter=3)
    dat = bytearray(msg.data)
    dat[7] ^= 1
    self.assertFalse(self._tx(common.make_msg(0, 0x1F4, dat=bytes(dat))))

  def test_hold_fails_closed_on_motion_pedals_collision_and_stale_sources(self):
    self._enable_hold_sources()
    self.assertTrue(self._rx(self._speed_msg(1)))
    self.assertFalse(self._tx(self._hold_msg()))

    self._reset_hold_safety()
    self._enable_hold_sources()
    self.assertTrue(self._rx(self._user_gas_msg(1)))
    self.assertFalse(self._tx(self._hold_msg()))

    self._reset_hold_safety()
    self._enable_hold_sources()
    self.assertTrue(self._rx(self._user_brake_msg(True)))
    self.assertFalse(self._tx(self._hold_msg()))

    self._reset_hold_safety()
    self._enable_hold_sources()
    self.assertTrue(self._rx(self._das_3_msg(
      counter=2, ACC_AVAILABLE=1, COLLISION_BRK_PREP=1,
    )))
    self.assertFalse(self._tx(self._hold_msg(counter=4)))

    self._reset_hold_safety()
    self._enable_hold_sources(time_us=2_000_000)
    self.safety.set_timer(2_000_000 + self.SOURCE_TIMEOUT_US + 1)
    self.assertFalse(self._tx(self._hold_msg()))

  def test_cancel_clears_special_resume_authority(self):
    self._enable_hold_sources()
    self.assertTrue(self._tx(self._hold_msg()))
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))


class TestJeepRate4Limits(unittest.TestCase):
  """Focused boundary tests for the opt-in Jeep steering envelope."""

  TX_MSGS = None

  def setUp(self):
    self.packer = CANPackerPanda("chrysler_pacifica_2017_hybrid_generated")
    self.safety = libpanda_py.libpanda

  def _reset(self, param):
    self.safety.set_safety_hooks(Panda.SAFETY_CHRYSLER, param)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)

  def _torque_cmd_msg(self, torque):
    values = {"STEERING_TORQUE": torque, "LKAS_CONTROL_BIT": 1}
    return self.packer.make_can_msg_panda("LKAS_COMMAND", 0, values)

  def _set_torque_state(self, desired_last, rt_last, measured):
    self.safety.set_desired_torque_last(desired_last)
    self.safety.set_rt_torque_last(rt_last)
    self.safety.set_torque_meas(measured, measured)

  def _tx_from_zero(self, param, torque):
    self._reset(param)
    self._set_torque_state(0, 0, 0)
    return self.safety.safety_tx_hook(self._torque_cmd_msg(torque))

  def test_legacy_rate3_is_unchanged(self):
    self.assertTrue(self._tx_from_zero(0, 3))
    self.assertFalse(self._tx_from_zero(0, 4))
    self.assertTrue(self._tx_from_zero(0, -3))
    self.assertFalse(self._tx_from_zero(0, -4))

  def test_jeep_rate4_boundary(self):
    param = Panda.FLAG_CHRYSLER_JEEP_RATE4
    self.assertTrue(self._tx_from_zero(param, 4))
    self.assertFalse(self._tx_from_zero(param, 5))
    self.assertTrue(self._tx_from_zero(param, -4))
    self.assertFalse(self._tx_from_zero(param, -5))

  def test_jeep_rate5_boundary(self):
    param = Panda.FLAG_CHRYSLER_JEEP_RATE5
    self.assertTrue(self._tx_from_zero(param, 5))
    self.assertFalse(self._tx_from_zero(param, 6))
    self.assertTrue(self._tx_from_zero(param, -5))
    self.assertFalse(self._tx_from_zero(param, -6))

  def test_conflicting_rate_flags_fall_closed_to_stock_rate3(self):
    param = (
      Panda.FLAG_CHRYSLER_JEEP_RATE4 |
      Panda.FLAG_CHRYSLER_JEEP_RATE5
    )
    self.assertTrue(self._tx_from_zero(param, 3))
    self.assertFalse(self._tx_from_zero(param, 4))

  def test_rate4_composes_with_long_shadow_flag(self):
    param = (
      Panda.FLAG_CHRYSLER_JEEP_RATE4 |
      Panda.FLAG_CHRYSLER_JEEP_LONG_SHADOW
    )
    self.assertTrue(self._tx_from_zero(param, 4))
    self.assertFalse(self._tx_from_zero(param, 5))

  def test_max_torque_remains_261(self):
    for param in (
      Panda.FLAG_CHRYSLER_JEEP_RATE4,
      Panda.FLAG_CHRYSLER_JEEP_RATE5,
    ):
      self._reset(param)
      self._set_torque_state(261, 261, 261)
      self.assertTrue(self.safety.safety_tx_hook(self._torque_cmd_msg(261)))

      self._reset(param)
      self._set_torque_state(262, 262, 262)
      self.assertFalse(self.safety.safety_tx_hook(self._torque_cmd_msg(262)))

  def test_realtime_delta_remains_112(self):
    for param in (
      Panda.FLAG_CHRYSLER_JEEP_RATE4,
      Panda.FLAG_CHRYSLER_JEEP_RATE5,
    ):
      self._reset(param)
      self._set_torque_state(108, 0, 112)
      self.assertTrue(self.safety.safety_tx_hook(self._torque_cmd_msg(112)))

      self._reset(param)
      self._set_torque_state(109, 0, 113)
      self.assertFalse(self.safety.safety_tx_hook(self._torque_cmd_msg(113)))


class TestChryslerLongShadowSafety(common.PandaSafetyTestBase):
  TX_MSGS = [[0x1F6, 0], [0x1F7, 0], [0x272, 0]]

  PRIVATE_BRAKE = 0x1F6
  PRIVATE_DASH = 0x1F7
  PRIVATE_TORQUE = 0x272
  SOURCE_TIMEOUT_US = 100_000
  MIN_CYCLE_INTERVAL_US = 15_000

  def setUp(self):
    self.packer = CANPackerPanda("chrysler_pacifica_2017_hybrid_generated")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(
      Panda.SAFETY_CHRYSLER,
      Panda.FLAG_CHRYSLER_JEEP_LONG_SHADOW,
    )
    self.safety.init_tests()

  def _das_3_msg(self, counter=1, **changes):
    values = {"COUNTER": counter}
    values.update(changes)
    return self.packer.make_can_msg_panda("DAS_3", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED_LEFT": speed, "SPEED_RIGHT": speed}
    return self.packer.make_can_msg_panda("SPEED_1", 0, values)

  def _user_gas_msg(self, gas):
    values = {"Accelerator_Position": gas}
    return self.packer.make_can_msg_panda("ECM_5", 0, values)

  def _user_brake_msg(self, brake):
    values = {"Brake_Pedal_State": 1 if brake else 0}
    return self.packer.make_can_msg_panda("ESP_1", 0, values)

  @staticmethod
  def _fca_checksum(dat):
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
          bit_sum ^= checksum | 1
        else:
          if temp_checksum:
            bit_sum = 0x1D
          checksum = (checksum << 1) & 0xFF
          bit_sum ^= checksum
        checksum = bit_sum & 0xFF
        shift >>= 1
    return (~checksum) & 0xFF

  def _private_msg(self, address, dat, corrupt_checksum=False, bus=0):
    dat = bytearray(dat)
    dat[7] = self._fca_checksum(dat)
    if corrupt_checksum:
      dat[7] ^= 1
    return common.make_msg(bus, address, dat=bytes(dat))

  def _private_brake_msg(self, counter, decel_raw=4094, command_type=0,
                         available=True, enabled=True, stop=False, go=False,
                         brake_prep=False, corrupt_checksum=False, bus=0):
    dat = bytearray(8)
    dat[0] = (int(stop) << 5) | (int(go) << 6)
    dat[2] = (
      ((decel_raw >> 8) & 0xF)
      | (int(available) << 4)
      | (int(enabled) << 5)
    )
    dat[3] = decel_raw & 0xFF
    dat[4] = (command_type & 0x7) << 4
    dat[6] = ((counter & 0xF) << 4) | (int(brake_prep) << 1)
    return self._private_msg(
      self.PRIVATE_BRAKE, dat, corrupt_checksum=corrupt_checksum, bus=bus,
    )

  def _private_dash_msg(self, counter, enable=False,
                        corrupt_checksum=False, unused_byte=0, bus=0):
    dat = bytearray(8)
    dat[0] = unused_byte
    dat[3] = int(enable)
    dat[6] = (counter & 0xF) << 4
    return self._private_msg(
      self.PRIVATE_DASH, dat, corrupt_checksum=corrupt_checksum, bus=bus,
    )

  def _private_torque_msg(self, counter, torque_raw=2000,
                          engine_request=False, corrupt_checksum=False,
                          unused_byte=0):
    dat = bytearray(8)
    dat[0] = unused_byte
    dat[4] = (
      (int(engine_request) << 7)
      | ((torque_raw >> 8) & 0x7F)
    )
    dat[5] = torque_raw & 0xFF
    dat[6] = (counter & 0xF) << 4
    return self._private_msg(
      self.PRIVATE_TORQUE, dat, corrupt_checksum=corrupt_checksum,
    )

  def _enable_safe_source(self, counter=1):
    self.assertTrue(self._rx(self._das_3_msg(
      counter=counter, ACC_AVAILABLE=1, ACC_ACTIVE=1,
    )))
    self.assertTrue(self._rx(self._speed_msg(1)))
    self.assertTrue(self._rx(self._user_gas_msg(0)))
    self.assertTrue(self._rx(self._user_brake_msg(False)))

  def _tx_private_cycle(self, counter, time_us, decel_raw=4094,
                        command_type=0, torque_raw=2000,
                        engine_request=False, enable=False):
    self.safety.set_timer(time_us)
    return (
      self._tx(self._private_brake_msg(
        counter, decel_raw=decel_raw, command_type=command_type,
      )),
      self._tx(self._private_dash_msg(counter, enable=enable)),
      self._tx(self._private_torque_msg(
        counter, torque_raw=torque_raw,
        engine_request=engine_request,
      )),
    )

  def _reset_long_shadow(self, diagnostic=False, actuation=False):
    param = Panda.FLAG_CHRYSLER_JEEP_LONG_SHADOW
    if diagnostic:
      param |= Panda.FLAG_CHRYSLER_JEEP_LONG_DIAGNOSTIC
    if actuation:
      param |= Panda.FLAG_CHRYSLER_JEEP_LONG_ACTUATION
    self.safety.set_safety_hooks(
      Panda.SAFETY_CHRYSLER,
      param,
    )
    self.safety.init_tests()

  @staticmethod
  def _reject_diagnostic(msg):
    detail = (
      int(msg.data[2])
      | (int(msg.data[3]) << 8)
      | (int(msg.data[4]) << 16)
      | (int(msg.data[5]) << 24)
    )
    return int(msg.data[0]), int(msg.data[1]), detail

  def test_reject_diagnostics_are_opt_in_and_do_not_change_acceptance(self):
    self._reset_long_shadow()
    self.safety.set_timer(1_000_000)
    self._enable_safe_source()
    self.assertEqual(self._tx_private_cycle(0, 1_000_000), (True, True, True))
    untagged = self._private_brake_msg(1)
    self.safety.set_timer(1_010_000)
    self.assertFalse(self._tx(untagged))
    self.assertEqual((int(untagged.data[0]), int(untagged.data[1])), (0, 0))

    self._reset_long_shadow(diagnostic=True)
    self.safety.set_timer(2_000_000)
    self._enable_safe_source()
    accepted = self._private_brake_msg(0)
    self.assertTrue(self._tx(accepted))
    self.assertEqual(int(accepted.data[0]), 0)

  def test_reject_diagnostic_exposes_hidden_longitudinal_state(self):
    self._reset_long_shadow(diagnostic=True)
    self.safety.set_timer(1_000_000)
    self._enable_safe_source()
    self.assertTrue(self._rx(self._das_3_msg(
      counter=2, ACC_AVAILABLE=1, ACC_ACTIVE=0,
    )))
    self.safety.set_controls_allowed(True)

    msg = self._private_brake_msg(0)
    self.assertFalse(self._tx(msg))
    self.assertEqual(self._reject_diagnostic(msg), (0xD7, 5, 0))

  def test_reject_diagnostic_exposes_panda_interval(self):
    self._reset_long_shadow(diagnostic=True)
    self.safety.set_timer(1_000_000)
    self._enable_safe_source()
    self.assertEqual(self._tx_private_cycle(0, 1_000_000), (True, True, True))

    msg = self._private_brake_msg(1)
    self.safety.set_timer(1_010_000)
    self.assertFalse(self._tx(msg))
    self.assertEqual(self._reject_diagnostic(msg), (0xD7, 22, 10_000))

  def test_rate_rejection_tracks_valid_counter_without_transmitting(self):
    self._reset_long_shadow(diagnostic=True)
    self.safety.set_timer(1_000_000)
    self._enable_safe_source()
    self.assertEqual(self._tx_private_cycle(0, 1_000_000), (True, True, True))

    # Counter 1 is well-formed but too early, so the complete cycle is blocked.
    self.assertEqual(
      self._tx_private_cycle(1, 1_012_000),
      (False, False, False),
    )

    # Counter 2 is accepted because the rate-rejected input counter was
    # observed, while accepted-frame spacing still uses the original timestamp.
    self.assertEqual(
      self._tx_private_cycle(2, 1_032_000),
      (True, True, True),
    )

  def test_repeated_rate_rejections_preserve_accepted_frame_floor(self):
    self._reset_long_shadow()
    self.safety.set_timer(1_000_000)
    self._enable_safe_source()
    self.assertEqual(self._tx_private_cycle(0, 1_000_000), (True, True, True))

    for counter, time_us in ((1, 1_012_000), (2, 1_013_000), (3, 1_014_000)):
      self.assertEqual(
        self._tx_private_cycle(counter, time_us),
        (False, False, False),
      )

    # No frame can pass until 15 ms after the last accepted frame.
    self.assertEqual(self._tx_private_cycle(4, 1_015_000), (True, True, True))

  def test_bad_early_counter_cannot_use_rate_recovery(self):
    self._reset_long_shadow()
    self.safety.set_timer(1_000_000)
    self._enable_safe_source()
    self.assertEqual(self._tx_private_cycle(0, 1_000_000), (True, True, True))

    # Skipping counter 1 is invalid even though the frame is also too early.
    self.assertFalse(self._tx(self._private_brake_msg(2)))
    self.safety.set_timer(1_020_000)
    self.assertFalse(self._tx(self._private_brake_msg(3)))

  def test_reject_diagnostic_exposes_counter_mismatch(self):
    self._reset_long_shadow(diagnostic=True)
    self.safety.set_timer(1_000_000)
    self._enable_safe_source()
    self.assertEqual(self._tx_private_cycle(0, 1_000_000), (True, True, True))

    msg = self._private_brake_msg(2)
    self.safety.set_timer(1_020_000)
    self.assertFalse(self._tx(msg))
    self.assertEqual(self._reject_diagnostic(msg), (0xD7, 23, 0x102))

  def test_private_frames_blocked_without_shadow_param(self):
    self.safety.set_safety_hooks(Panda.SAFETY_CHRYSLER, 0)
    self.safety.init_tests()
    self.assertFalse(self._tx(self._private_brake_msg(0)))
    self.assertFalse(self._tx(self._private_dash_msg(0)))
    self.assertFalse(self._tx(self._private_torque_msg(0)))

  def test_private_frames_require_bus_zero_and_eight_bytes(self):
    self._enable_safe_source()
    self.assertFalse(self._tx(self._private_brake_msg(0, bus=1)))
    self.assertFalse(self._tx(common.make_msg(
      0, self.PRIVATE_BRAKE, length=7,
    )))

  def test_private_valid_neutral_engine_and_brake_cycles(self):
    self._enable_safe_source()
    self.assertEqual(
      self._tx_private_cycle(14, 0),
      (True, True, True),
    )
    self.assertEqual(
      self._tx_private_cycle(
        15, 20_000, torque_raw=3700, engine_request=True,
      ),
      (True, True, True),
    )
    self.assertEqual(
      self._tx_private_cycle(
        0, 40_000, decel_raw=2661, command_type=1,
      ),
      (True, True, True),
    )

  def test_private_enable_requires_dedicated_actuation_flag(self):
    self._enable_safe_source()
    self.assertTrue(self._tx(self._private_brake_msg(0)))
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertFalse(self._tx(self._private_dash_msg(0, enable=True)))
      self.assertFalse(self._tx(self._private_torque_msg(0)))

  def test_private_actuation_flag_accepts_only_enabled_complete_cycles(self):
    self._reset_long_shadow(actuation=True)
    self._enable_safe_source()
    self.assertEqual(
      self._tx_private_cycle(
        0, 0, decel_raw=2866, command_type=1, enable=True,
      ),
      (True, True, True),
    )
    self.assertEqual(
      self._tx_private_cycle(
        1, 20_000, torque_raw=2310,
        engine_request=True, enable=True,
      ),
      (True, True, True),
    )

    self._reset_long_shadow(actuation=True)
    self._enable_safe_source()
    self.assertTrue(self._tx(self._private_brake_msg(0)))
    self.assertFalse(self._tx(self._private_dash_msg(0, enable=False)))
    self.assertFalse(self._tx(self._private_torque_msg(0)))

  def test_private_actuation_flag_without_shadow_cannot_transmit(self):
    self.safety.set_safety_hooks(
      Panda.SAFETY_CHRYSLER,
      Panda.FLAG_CHRYSLER_JEEP_LONG_ACTUATION,
    )
    self.safety.init_tests()
    self._enable_safe_source()
    self.assertFalse(self._tx(self._private_brake_msg(0)))
    self.assertFalse(self._tx(self._private_dash_msg(0, enable=True)))
    self.assertFalse(self._tx(self._private_torque_msg(0)))

  def test_source_change_between_private_stages_aborts_cycle(self):
    self._reset_long_shadow(actuation=True)
    self._enable_safe_source()
    self.assertTrue(self._tx(self._private_brake_msg(
      0, decel_raw=2866, command_type=1,
    )))
    self.assertTrue(self._rx(self._user_gas_msg(1)))
    self.assertFalse(self._tx(self._private_dash_msg(0, enable=True)))
    self.assertFalse(self._tx(self._private_torque_msg(0)))

  def test_private_source_gates_and_freshness(self):
    self.assertFalse(self._tx(self._private_brake_msg(0)))

    self._reset_long_shadow()
    self._enable_safe_source()
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._private_brake_msg(0)))

    self._reset_long_shadow()
    self._enable_safe_source()
    self.assertTrue(self._rx(self._speed_msg(0)))
    self.assertFalse(self._tx(self._private_brake_msg(0)))

    self._reset_long_shadow()
    self._enable_safe_source()
    self.assertTrue(self._rx(self._user_gas_msg(1)))
    self.assertFalse(self._tx(self._private_brake_msg(0)))

    self._reset_long_shadow()
    self._enable_safe_source()
    self.assertTrue(self._rx(self._user_brake_msg(True)))
    self.assertFalse(self._tx(self._private_brake_msg(0)))

    self._reset_long_shadow()
    self._enable_safe_source()
    self.assertTrue(self._rx(self._das_3_msg(
      counter=2, ACC_AVAILABLE=1, ACC_ACTIVE=1, ACC_DECEL_REQ=2,
    )))
    self.assertFalse(self._tx(self._private_brake_msg(0)))

    self._reset_long_shadow()
    self._enable_safe_source()
    self.safety.set_timer(self.SOURCE_TIMEOUT_US + 1)
    self.assertFalse(self._tx(self._private_brake_msg(0)))

  def test_private_brake_payload_rejection(self):
    invalid_messages = (
      self._private_brake_msg(0, decel_raw=2660, command_type=1),
      self._private_brake_msg(0, decel_raw=3276, command_type=1),
      self._private_brake_msg(0, decel_raw=4093, command_type=0),
      self._private_brake_msg(0, command_type=2),
      self._private_brake_msg(0, available=False),
      self._private_brake_msg(0, enabled=False),
      self._private_brake_msg(0, stop=True),
      self._private_brake_msg(0, go=True),
      self._private_brake_msg(0, brake_prep=True),
      self._private_brake_msg(0, corrupt_checksum=True),
    )
    for message in invalid_messages:
      self._reset_long_shadow()
      self._enable_safe_source()
      self.assertFalse(self._tx(message))

  def test_private_torque_payload_and_exclusivity(self):
    invalid_torque = (
      self._private_torque_msg(
        0, torque_raw=3701, engine_request=True,
      ),
      self._private_torque_msg(
        0, torque_raw=2001, engine_request=False,
      ),
      self._private_torque_msg(0, corrupt_checksum=True),
      self._private_torque_msg(0, unused_byte=1),
    )
    for message in invalid_torque:
      self._reset_long_shadow()
      self._enable_safe_source()
      self.assertTrue(self._tx(self._private_brake_msg(0)))
      self.assertTrue(self._tx(self._private_dash_msg(0)))
      self.assertFalse(self._tx(message))

    self._reset_long_shadow()
    self._enable_safe_source()
    self.assertTrue(self._tx(self._private_brake_msg(
      0, decel_raw=2661, command_type=1,
    )))
    self.assertTrue(self._tx(self._private_dash_msg(0)))
    self.assertFalse(self._tx(self._private_torque_msg(
      0, torque_raw=2100, engine_request=True,
    )))

  def test_private_dashboard_unused_payload_rejected(self):
    self._enable_safe_source()
    self.assertTrue(self._tx(self._private_brake_msg(0)))
    self.assertFalse(self._tx(self._private_dash_msg(
      0, unused_byte=1,
    )))
    self.assertFalse(self._tx(self._private_torque_msg(0)))

  def test_private_counter_order_checksum_and_rate(self):
    self._enable_safe_source()
    self.assertEqual(self._tx_private_cycle(0, 0), (True, True, True))

    self.safety.set_timer(20_000)
    self.assertFalse(self._tx(self._private_brake_msg(0)))

    self._reset_long_shadow()
    self._enable_safe_source()
    self.assertEqual(self._tx_private_cycle(0, 0), (True, True, True))
    self.safety.set_timer(20_000)
    self.assertFalse(self._tx(self._private_brake_msg(2)))

    self._reset_long_shadow()
    self._enable_safe_source()
    self.assertEqual(self._tx_private_cycle(0, 0), (True, True, True))
    self.safety.set_timer(self.MIN_CYCLE_INTERVAL_US - 1)
    self.assertFalse(self._tx(self._private_brake_msg(1)))

    self._reset_long_shadow()
    self._enable_safe_source()
    self.assertFalse(self._tx(self._private_dash_msg(0)))
    self.assertFalse(self._tx(self._private_torque_msg(0)))
    self.assertTrue(self._tx(self._private_brake_msg(0)))
    self.assertFalse(self._tx(self._private_dash_msg(1)))
    self.assertFalse(self._tx(self._private_torque_msg(0)))

    self._reset_long_shadow()
    self._enable_safe_source()
    self.assertTrue(self._tx(self._private_brake_msg(0)))
    self.assertFalse(self._tx(self._private_dash_msg(
      0, corrupt_checksum=True,
    )))
    self.assertFalse(self._tx(self._private_torque_msg(0)))


class TestChryslerRamDTSafety(TestChryslerSafety):
  TX_MSGS = [[0xB1, 2], [0xA6, 0], [0xFA, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0xA6,)}
  FWD_BLACKLISTED_ADDRS = {2: [0xA6, 0xFA]}

  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 6
  MAX_TORQUE = 350

  DAS_BUS = 2

  LKAS_ACTIVE_VALUE = 2

  def setUp(self):
    self.packer = CANPackerPanda("chrysler_ram_dt_generated")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_CHRYSLER, Panda.FLAG_CHRYSLER_RAM_DT)
    self.safety.init_tests()

  def _speed_msg(self, speed):
    values = {"Vehicle_Speed": speed}
    return self.packer.make_can_msg_panda("ESP_8", 0, values)

class TestChryslerRamHDSafety(TestChryslerSafety):
  TX_MSGS = [[0x275, 0], [0x276, 0], [0x23A, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x276,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x275, 0x276]}

  MAX_TORQUE = 361
  MAX_RATE_UP = 14
  MAX_RATE_DOWN = 14
  MAX_RT_DELTA = 182

  DAS_BUS = 2

  LKAS_ACTIVE_VALUE = 2

  def setUp(self):
    self.packer = CANPackerPanda("chrysler_ram_hd_generated")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_CHRYSLER, Panda.FLAG_CHRYSLER_RAM_HD)
    self.safety.init_tests()

  def _speed_msg(self, speed):
    values = {"Vehicle_Speed": speed}
    return self.packer.make_can_msg_panda("ESP_8", 0, values)


if __name__ == "__main__":
  unittest.main()

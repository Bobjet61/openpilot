from types import SimpleNamespace
import unittest

from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car.chrysler import chryslercan


def fca_checksum(dat):
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


class TestJeepLongitudinalCanPacking(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("chrysler_pacifica_2017_hybrid_generated")

  def test_private_white_panda_addresses(self):
    brake = self.packer.make_can_msg(
      "WP_ACC_BRAKE_CMD", 0,
      {"ACC_DECEL_CMD": -1.0, "COUNTER": 9, "CHECKSUM": 0},
    )
    dash = self.packer.make_can_msg(
      "WP_ACC_DASH_CMD", 0,
      {"OP_LONG_ENABLE": 0, "COUNTER": 9, "CHECKSUM": 0},
    )
    torque = self.packer.make_can_msg(
      "WP_ACC_TORQUE_CMD", 0,
      {"ENGINE_TORQUE_REQUEST": 0, "COUNTER": 9, "CHECKSUM": 0},
    )
    self.assertEqual((brake[0], dash[0], torque[0]), (0x1F6, 0x1F7, 0x272))
    self.assertEqual((len(brake[2]), len(dash[2]), len(torque[2])), (8, 8, 8))
    self.assertEqual(tuple(msg[2][6] >> 4 for msg in (brake, dash, torque)), (9, 9, 9))
    self.assertTrue(all(msg[2][7] == fca_checksum(msg[2]) for msg in (brake, dash, torque)))

  def test_private_engine_torque_uses_das3_scaling(self):
    _, _, dat, _ = self.packer.make_can_msg(
      "WP_ACC_TORQUE_CMD", 0,
      {"ENGINE_TORQUE_REQUEST_MAX": 1, "ENGINE_TORQUE_REQUEST": 100},
    )
    raw = ((dat[4] & 0x7F) << 8) | dat[5]
    self.assertEqual(raw, 2400)
    self.assertEqual(dat[4] >> 7, 1)

  def test_private_brake_range_and_inactive_sentinel(self):
    _, _, inactive, _ = self.packer.make_can_msg(
      "WP_ACC_BRAKE_CMD", 0,
      {"ACC_DECEL_CMD": 4.0, "COMMAND_TYPE": 0, "ACC_BRK_PREP": 0},
    )
    _, _, braking, _ = self.packer.make_can_msg(
      "WP_ACC_BRAKE_CMD", 0,
      {"ACC_DECEL_CMD": -3.0, "COMMAND_TYPE": 1, "ACC_BRK_PREP": 0},
    )
    inactive_raw = ((inactive[2] & 0xF) << 8) | inactive[3]
    braking_raw = ((braking[2] & 0xF) << 8) | braking[3]
    self.assertEqual(inactive_raw, 4094)
    self.assertEqual(braking_raw, 2661)
    self.assertEqual((inactive[4] >> 4) & 0x7, 0)
    self.assertEqual((braking[4] >> 4) & 0x7, 1)
    self.assertEqual((braking[6] >> 1) & 0x1, 0)

  def test_shadow_dashboard_tracks_independent_host_gate(self):
    envelope = SimpleNamespace(
      limited_accel=0.0,
      brake_active=False,
      engine_active=False,
      engine_torque_nm=0.0,
      eligible=True,
      host_enabled=False,
    )
    _, disabled_dash, _ = chryslercan.create_wp_long_shadow_messages(
      self.packer, envelope, 0,
    )
    self.assertEqual(disabled_dash[2][3] & 0x1, 0)

    envelope.host_enabled = True
    _, hypothetical_enabled_dash, _ = (
      chryslercan.create_wp_long_shadow_messages(
        self.packer, envelope, 1,
      )
    )
    self.assertEqual(hypothetical_enabled_dash[2][3] & 0x1, 1)
    self.assertEqual(hypothetical_enabled_dash[2][6] >> 4, 1)
    self.assertEqual(
      hypothetical_enabled_dash[2][7],
      fca_checksum(hypothetical_enabled_dash[2]),
    )

  def test_transport_probe_is_strictly_neutral(self):
    brake, dash, torque = chryslercan.create_wp_long_transport_messages(
      self.packer, 13,
    )

    brake_raw = ((brake[2][2] & 0xF) << 8) | brake[2][3]
    torque_raw = ((torque[2][4] & 0x7F) << 8) | torque[2][5]
    self.assertEqual(brake_raw, 4094)
    self.assertEqual((brake[2][4] >> 4) & 0x7, 0)
    self.assertEqual((brake[2][6] >> 1) & 0x1, 0)
    self.assertEqual(dash[2][3] & 0x1, 0)
    self.assertEqual(torque[2][4] >> 7, 0)
    self.assertEqual(torque_raw, 2000)
    self.assertEqual(
      tuple(msg[2][6] >> 4 for msg in (brake, dash, torque)),
      (13, 13, 13),
    )
    self.assertTrue(
      all(msg[2][7] == fca_checksum(msg[2])
          for msg in (brake, dash, torque)),
    )


if __name__ == "__main__":
  unittest.main()

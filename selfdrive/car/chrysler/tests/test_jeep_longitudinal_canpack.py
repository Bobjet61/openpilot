import unittest

from opendbc.can.packer import CANPacker


class TestJeepLongitudinalCanPacking(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("chrysler_pacifica_2017_hybrid_generated")

  def test_private_white_panda_addresses(self):
    brake = self.packer.make_can_msg("WP_ACC_BRAKE_CMD", 0, {"ACC_DECEL_CMD": -1.0})
    dash = self.packer.make_can_msg("WP_ACC_DASH_CMD", 0, {"OP_LONG_ENABLE": 0})
    torque = self.packer.make_can_msg("WP_ACC_TORQUE_CMD", 0, {"ENGINE_TORQUE_REQUEST": 0})
    self.assertEqual((brake[0], dash[0], torque[0]), (0x1F6, 0x1F7, 0x272))
    self.assertEqual((len(brake[2]), len(dash[2]), len(torque[2])), (8, 8, 8))

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

  def test_shadow_dashboard_never_enables_wp_long(self):
    _, _, dat, _ = self.packer.make_can_msg("WP_ACC_DASH_CMD", 0, {"OP_LONG_ENABLE": 0})
    self.assertEqual(dat[3] & 0x1, 0)


if __name__ == "__main__":
  unittest.main()

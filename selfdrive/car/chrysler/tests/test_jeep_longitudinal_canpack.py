import unittest

from opendbc.can.packer import CANPacker


class TestJeepLongitudinalCanPacking(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("chrysler_pacifica_2017_hybrid_generated")

  def test_private_white_panda_addresses(self):
    brake = self.packer.make_can_msg("WP_ACC_BRAKE_CMD", 0, {"ACC_DECEL_CMD": -1.0})
    dash = self.packer.make_can_msg("WP_ACC_DASH_CMD", 0, {"OP_LONG_ENABLE": 0})
    torque = self.packer.make_can_msg("WP_ACC_TORQUE_CMD", 0, {"ACC_TORQ": 0})
    self.assertEqual((brake[0], dash[0], torque[0]), (0x1F6, 0x1F7, 0x272))
    self.assertEqual((len(brake[2]), len(dash[2]), len(torque[2])), (8, 8, 8))

  def test_shadow_dashboard_never_enables_wp_long(self):
    _, _, dat, _ = self.packer.make_can_msg("WP_ACC_DASH_CMD", 0, {"OP_LONG_ENABLE": 0})
    self.assertEqual(dat[3] & 0x1, 0)


if __name__ == "__main__":
  unittest.main()

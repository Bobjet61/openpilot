import unittest

from opendbc.can.packer import CANPacker


class TestApa50CanPacking(unittest.TestCase):
  def test_steer_type_is_byte_six_low_three_bits(self):
    packer = CANPacker("chrysler_pacifica_2017_hybrid_generated")
    address, _, dat, bus = packer.make_can_msg("DAS_6", 0, {"STEER_TYPE": 2})
    self.assertEqual(address, 0x2A6)
    self.assertEqual(bus, 0)
    self.assertEqual(dat[6] & 0x7, 2)


if __name__ == "__main__":
  unittest.main()

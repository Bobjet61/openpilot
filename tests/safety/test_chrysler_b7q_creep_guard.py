#!/usr/bin/env python3
"""Focused b7q hold-creep test without an opendbc packer dependency."""

import unittest

from panda import Panda
from panda.tests.libpanda import libpanda_py


def fca_checksum(dat: bytes) -> int:
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


def packet(address: int, dat: bytearray, checksum: bool = True):
  if checksum:
    dat[-1] = fca_checksum(dat)
  return libpanda_py.make_CANPacket(address, 0, bytes(dat))


class TestChryslerB7qCreepGuard(unittest.TestCase):
  def setUp(self):
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(
      Panda.SAFETY_CHRYSLER,
      Panda.FLAG_CHRYSLER_JEEP_FACTORY_SNG,
    )
    self.safety.init_tests()
    self.safety.set_timer(1_000_000)

  def rx(self, address, dat, checksum=True):
    self.assertTrue(self.safety.safety_rx_hook(
      packet(address, bytearray(dat), checksum=checksum),
    ))

  def enable_sources(self, speed_raw: int):
    # Stock DAS_3: ACC available, inactive, counter 1.
    self.rx(0x1F4, [0, 0, 0x10, 0, 0, 0, 0x10, 0])
    speed = bytearray(8)
    speed[0] = (speed_raw >> 4) & 0xFF
    speed[1] = (speed_raw & 0xF) << 4
    speed[2] = (speed_raw >> 4) & 0xFF
    speed[3] = (speed_raw & 0xF) << 4
    self.rx(0x202, speed, checksum=False)
    self.rx(0x22F, bytearray(8))
    self.rx(0x140, bytearray(8))
    self.rx(0x1F5, bytearray(8), checksum=False)

  @staticmethod
  def hold_packet():
    # Exact -2.0 m/s^2 braking-only hold, counter 3 (stock + 2).
    return packet(0x1F4, bytearray([0, 0, 0x3B, 0x32, 0x12, 0, 0x30, 0]))

  def test_raw_two_creep_remains_braking_only_hold_eligible(self):
    self.enable_sources(2)
    self.assertTrue(self.safety.safety_tx_hook(self.hold_packet()))

  def test_raw_three_motion_clears_hold_authority(self):
    self.enable_sources(3)
    self.assertFalse(self.safety.safety_tx_hook(self.hold_packet()))


if __name__ == "__main__":
  unittest.main()

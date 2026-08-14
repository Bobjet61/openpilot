#!/usr/bin/env python3
"""Focused b6z torque-envelope test without an opendbc packer dependency."""

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


def packet(address: int, dat: bytearray, bus: int = 0, checksum: bool = True):
  if checksum:
    dat[-1] = fca_checksum(dat)
  return libpanda_py.make_CANPacket(address, bus, bytes(dat))


class TestChryslerB6zTorqueGuard(unittest.TestCase):
  def setUp(self):
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(
      Panda.SAFETY_CHRYSLER,
      Panda.FLAG_CHRYSLER_JEEP_LONG_SHADOW,
    )
    self.safety.init_tests()

  def enable_sources(self, speed_mps: float) -> None:
    das_3 = bytearray(8)
    das_3[2] = 0x30  # ACC available and active
    das_3[6] = 0x10  # counter 1
    self.assertTrue(self.safety.safety_rx_hook(packet(0x1F4, das_3)))

    speed_raw = round(speed_mps / 0.071028)
    speed = bytearray(8)
    speed[0] = (speed_raw >> 4) & 0xFF
    speed[1] = (speed_raw & 0xF) << 4
    speed[2] = (speed_raw >> 4) & 0xFF
    speed[3] = (speed_raw & 0xF) << 4
    self.assertTrue(self.safety.safety_rx_hook(
      packet(0x202, speed, checksum=False),
    ))

    gas = bytearray(8)
    self.assertTrue(self.safety.safety_rx_hook(packet(0x22F, gas)))
    brake = bytearray(8)
    self.assertTrue(self.safety.safety_rx_hook(packet(0x140, brake)))
    dashboard = bytearray(8)
    self.assertTrue(self.safety.safety_rx_hook(
      packet(0x1F5, dashboard, checksum=False),
    ))

  def transmit_engine_cycle(self, torque_raw: int) -> tuple[bool, bool, bool]:
    brake = bytearray(8)
    brake[2] = 0x3F  # available, active, inactive decel high nibble
    brake[3] = 0xFE  # inactive decel low byte (4094)

    dash = bytearray(8)

    torque = bytearray(8)
    torque[4] = 0x80 | ((torque_raw >> 8) & 0x7F)
    torque[5] = torque_raw & 0xFF

    return (
      bool(self.safety.safety_tx_hook(packet(0x1F6, brake))),
      bool(self.safety.safety_tx_hook(packet(0x1F7, dash))),
      bool(self.safety.safety_tx_hook(packet(0x272, torque))),
    )

  def test_speed_shaped_low_running_ceiling(self):
    self.enable_sources(1.0)
    self.assertEqual(
      self.transmit_engine_cycle(3075),
      (True, True, True),
    )

    self.setUp()
    self.enable_sources(1.0)
    self.assertEqual(
      self.transmit_engine_cycle(3085),
      (True, True, False),
    )

  def test_absolute_500_nm_ceiling(self):
    # Use a comfortably saturated point of the speed-shaped envelope. The
    # production host reaches the 500 Nm cap at 12.5 m/s.
    self.enable_sources(16.0)
    self.assertEqual(
      self.transmit_engine_cycle(4000),
      (True, True, True),
    )

    self.setUp()
    self.enable_sources(16.0)
    self.assertEqual(
      self.transmit_engine_cycle(4001),
      (True, True, False),
    )


if __name__ == "__main__":
  unittest.main()

# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_bh1750`
================================================================================

CircuitPython library for use with the Adafruit BH1750 breakout


* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

* `Adafruit BH1750 Breakout <https://www.adafruit.com/products/46XX>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

 * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
 * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

# imports

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_BH1750.git"

from struct import unpack_from
from micropython import const
import adafruit_bus_device.i2c_device as i2c_device


# pylint: disable=bad-whitespace
_BH1750_DEVICE_ID = 0xE1  # Correct content of WHO_AM_I register

# I2C addresses (without R/W bit)
_BH1750_DEFAULT_ADDRESS = const(0x23)  # I2C address with ADDR pin low
_BH1750_ALT_ADDRESS = const(0x5C)  # I2C address with ADDR pin high

# Instructions
_BH1750_POWER_DOWN = const(0x00)  # Power down instruction
_BH1750_POWER_ON = const(0x01)  # Power on instruction
_BH1750_RESET = const(0x07)  # Reset instruction

# Bitfields
_BH1750_MODE_MASK = const(0x30)  # Mode mask bits
_BH1750_RES_MASK = const(0x03)  # Mode resolution mask bits

# Worst case conversion timing in ms
_BH1750_CONV_TIME_L = const(24)  # Worst case conversion timing low res
_BH1750_CONV_TIME_H = const(180)  # Worst case conversion timing high res

# Change measurement time.
#  ※ Please refer "adjust measurement result for influence of optical window."",
# 0b01000MT[7-5] # set measurement time high nibble
# 0b011MT[4-0] # set measurement time low nibble


class CV:
    """struct helper"""

    @classmethod
    def add_values(cls, value_tuples):
        """Add CV values to the class"""
        cls.string = {}
        cls.lsb = {}

        for value_tuple in value_tuples:
            name, value, string, lsb = value_tuple
            setattr(cls, name, value)
            cls.string[value] = string
            cls.lsb[value] = lsb

    @classmethod
    def is_valid(cls, value):
        """Validate that a given value is a member"""
        return value in cls.string


class RWBitfields:
    """
    Multibit register (less than a full byte) that is readable and writeable.
    This must be within a byte register.

    Values are `int` between 0 and 2 ** ``num_bits`` - 1.

    :param int num_bits: The number of bits in the field.
    :param int register_address: The register address to read the bit from
    :param type lowest_bit: The lowest bits index within the byte at ``register_address``
    :param int register_width: The number of bytes in the register. Defaults to 1.
    :param bool lsb_first: Is the first byte we read from I2C the LSB? Defaults to true
    """

    def __init__(self, num_bits, lowest_bit):
        self._bit_mask = ((1 << num_bits) - 1) << lowest_bit
        self._lowest_bit = lowest_bit

    def __get__(self, obj, objtype=None):

        return (obj._settings & self._bit_mask) >> self._lowest_bit

    def __set__(self, obj, value):
        # shift the value over to the right spot
        value <<= self._lowest_bit
        settings = obj._settings

        # mask off the bits we're about to change
        settings &= ~self._bit_mask
        settings |= value  # then or in our new value
        obj._settings = settings


class Mode(CV):
    """Options for ``mode``"""

    pass  # pylint: disable=unnecessary-pass


Mode.add_values(
    (
        ("SHUTDOWN", 0, "Shutdown", None),
        ("CONTINUOUS", 1, "Continuous", None),
        ("ONE_SHOT", 2, "One Shot", None),
    )
)


class Resolution(CV):
    """Options for ``resolution``"""

    pass  # pylint: disable=unnecessary-pass


Resolution.add_values(
    (
        ("LOW", 3, "Low", None),  # 4 lx resolution
        ("MID", 0, "Mid", None),  # 1 lx resolution
        ("HIGH", 1, "High", None),  # 0.5 lx resolution
    )
)


class BH1750:  # pylint:disable=too-many-instance-attributes
    """Library for the BH1750 Sensor


        :param ~busio.I2C i2c_bus: The I2C bus the BH1750 is connected to.
        :param address: The I2C slave address of the sensor

    """

    mode = RWBitfields(2, 4)
    resolution = RWBitfields(2, 0)

    def __init__(self, i2c_bus, address=_BH1750_DEFAULT_ADDRESS):

        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        self._buffer = bytearray(2)
        self._settings_byte = 0

        self.initialize()

    def initialize(self):
        """Configure the sensors with the default settings. For use after calling `reset()`"""
        self.mode = Mode.CONTINUOUS  # pylint:disable=no-member

    @property
    def _settings(self):
        return self._settings_byte

    @_settings.setter
    def _settings(self, value):
        self._settings_byte = value
        self._write(self._settings_byte)

    @property
    def _raw_reading(self):

        self._buffer[0] = 0
        self._buffer[1] = 0

        with self.i2c_device as i2c:
            i2c.readinto(self._buffer)

        return unpack_from(">H", self._buffer)[0]

    @property
    def lux(self):
        """Light value in lux.

        This example prints the light data in lux. Cover the sensor to see the values change.

        .. code-block:: python

            import time
            import board
            import busio
            import adafruit_bh1750

            i2c = busio.I2C(board.SCL, board.SDA)
            sensor = adafruit_bh1750.BH1750(i2c)

            while True:
                print("Lux:", sensor.lux)
                time.sleep(0.1)

        """
        raw_lux = self._raw_reading

        return self._convert_to_lux(raw_lux)

    @staticmethod
    def _convert_to_lux(raw_lux):
        return ((raw_lux * 10) + 5) / 12

    def _write(self, cmd_byte):
        self._buffer[0] = cmd_byte
        with self.i2c_device as i2c:
            i2c.write(self._buffer, end=1)

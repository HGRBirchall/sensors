"""
`adxl355`
MIT Licence
====================================================

A driver for the ADXL355 3-axis accelerometer

* Author(s): Hugh Birchall, based on original ADXL345 and ADXL355 drivers by Adafruit and oaslananka.

"""

from struct import unpack
from micropython import const
from adafruit_bus_device import i2c_device

try:
    from typing import Tuple, Dict

    import busio
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_ADXL355.git"

_ADXL355_DEFAULT_ADDRESS: int = const(0x1D)

# Device registers
_REG_DEVID = const(0x00)
_REG_RANGE = const(0x2C)
_REG_FILTER = const(0x28)
_REG_POWER_CTL = const(0x2D)
_REG_INT_MAP = const(0x2F)
_REG_XDATA3 = const(0x08)
_REG_YDATA3 = const(0x0B)
_REG_ZDATA3 = const(0x0E)

# Conversion factor
_ADXL355_G_PER_LSB = 0.0000039

class DataRate:
    """An enum-like class for ADXL355 data rates."""
    ODR_4000HZ = const(0b0000)
    ODR_2000HZ = const(0b0001)
    ODR_1000HZ = const(0b0010)
    ODR_500HZ = const(0b0011)
    ODR_250HZ = const(0b0100)
    ODR_125HZ = const(0b0101)
    ODR_62_5HZ = const(0b0110)
    ODR_31_25HZ = const(0b0111)

class Range:
    """An enum-like class for ADXL355 measurement ranges."""
    RANGE_2G = const(0b01)
    RANGE_4G = const(0b10)
    RANGE_8G = const(0b11)

class ADXL355:
    """Driver for the ADXL355 3-axis accelerometer."""

    def __init__(self, i2c: busio.I2C, address: int = _ADXL355_DEFAULT_ADDRESS):
        self._i2c = i2c_device.I2CDevice(i2c, address)
        self._buffer = bytearray(7)
        # set the 'measure' bit to enable measurement
        self._write_register_byte(_REG_POWER_CTL, 0x00)
        self._write_register_byte(_REG_INT_MAP, 0x00)

    def _write_register_byte(self, register: int, value: int) -> None:
        """Writes an 8-bit value to the specified register."""
        self._buffer[0] = register & 0xFF
        self._buffer[1] = value & 0xFF
        with self._i2c as i2c:
            i2c.write(self._buffer, start=0, end=2)

    def _read_register(self, register: int, length: int) -> bytearray:
        """Reads from the specified register."""
        self._buffer[0] = register & 0xFF
        with self._i2c as i2c:
            i2c.write(self._buffer, start=0, end=1)
            i2c.readinto(self._buffer, start=0, end=length)
            return self._buffer[0:length]

    def _read_register_unpacked(self, register: int) -> int:
        """Reads a single byte value from the specified register."""
        return unpack("<B", self._read_register(register, 1))[0]

    def _convert_to_signed(self, value):
        """Converts a raw 20-bit value to a signed value."""
        if value & 0x80000:
            return value - 0x100000
        return value

    @property
    def acceleration(self) -> Tuple[float, float, float]:
        """The x, y, z acceleration values returned in a 3-tuple."""
        x = self._read_axis_data(_REG_XDATA3)
        y = self._read_axis_data(_REG_YDATA3)
        z = self._read_axis_data(_REG_ZDATA3)
        return (
            x * _ADXL355_G_PER_LSB,
            y * _ADXL355_G_PER_LSB,
            z * _ADXL355_G_PER_LSB,
        )

    def _read_axis_data(self, axis_register: int) -> int:
        """Reads the raw axis data."""
        data = self._read_register(axis_register, 3)
        raw_value = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        return self._convert_to_signed(raw_value)

    def set_range(self, range_g: int) -> None:
        """Sets the measurement range."""
        reg_value = self._read_register_unpacked(_REG_RANGE)
        reg_value &= 0b11111100  # Clear the two range bits
        reg_value |= range_g
        self._write_register_byte(_REG_RANGE, reg_value)

    def set_data_rate(self, odr: int) -> None:
        """Sets the output data rate (ODR)."""
        reg_value = self._read_register_unpacked(_REG_FILTER)
        reg_value &= 0b11110000  # Clear the four ODR bits
        reg_value |= odr
        self._write_register_byte(_REG_FILTER, reg_value)

    @property
    def raw_x(self) -> int:
        """The raw x value."""
        return self._read_axis_data(_REG_XDATA3)

    @property
    def raw_y(self) -> int:
        """The raw y value."""
        return self._read_axis_data(_REG_YDATA3)

    @property
    def raw_z(self) -> int:
        """The raw z value."""
        return self._read_axis_data(_REG_ZDATA3)

# High-level usage example:
# import board
# import adafruit_adxl355
# i2c = board.I2C()  # uses board.SCL and board.SDA
# accelerometer = adafruit_adxl355.ADXL355(i2c)
# print(accelerometer.acceleration)

"""MAX17048 fuel gauge driver (I2C).

Pure driver — opens the I2C bus, reads registers, converts to physical units.
No CLI here; see `ross.commands.fuel` for the user-facing command.
"""

from smbus2 import SMBus

I2C_ADDR = 0x36
I2C_BUS = 1  # /dev/i2c-1 on Raspberry Pi

REG_VCELL = 0x02
REG_SOC = 0x04
REG_MODE = 0x06
REG_VERSION = 0x08
REG_CRATE = 0x16


def open_bus() -> SMBus:
    """Open the default I2C bus. Raises FileNotFoundError if I2C is disabled."""
    return SMBus(I2C_BUS)


def read_word(bus: SMBus, reg: int) -> int:
    """Read a 16-bit big-endian word from a register."""
    try:
        data = bus.read_i2c_block_data(I2C_ADDR, reg, 2)
    except OSError as e:
        raise OSError(
            f"Failed to read register 0x{reg:02X} from MAX17048 at 0x{I2C_ADDR:02X}: {e}. "
            "Is a battery connected to the fuel gauge?"
        ) from e
    return (data[0] << 8) | data[1]


def read_voltage(bus: SMBus) -> float:
    """Return battery voltage in volts."""
    return read_word(bus, REG_VCELL) * 78.125e-6


def read_soc(bus: SMBus) -> float:
    """Return state of charge in percent."""
    return read_word(bus, REG_SOC) / 256.0


def read_charge_rate(bus: SMBus) -> float:
    """Return charge/discharge rate in % per hour (signed)."""
    raw = read_word(bus, REG_CRATE)
    if raw >= 0x8000:
        raw -= 0x10000
    return raw * 0.208


def read_version(bus: SMBus) -> int:
    """Return the chip version register."""
    return read_word(bus, REG_VERSION)

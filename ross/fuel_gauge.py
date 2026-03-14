"""Monitor the MAX17048 fuel gauge over I2C.

Reads battery voltage and state of charge from the fuel gauge
connected to the Raspberry Pi's I2C bus (address 0x36).

Usage:
    uv run python ross/fuel_gauge.py              # print once
    uv run python ross/fuel_gauge.py --watch      # poll every 5s
    uv run python ross/fuel_gauge.py --watch -i 2 # poll every 2s
"""

import argparse
import sys
import time

from loguru import logger
from smbus2 import SMBus

# MAX17048 I2C address and register map
I2C_ADDR = 0x36
I2C_BUS = 1  # /dev/i2c-1 on Raspberry Pi

REG_VCELL = 0x02  # Battery voltage
REG_SOC = 0x04  # State of charge (%)
REG_MODE = 0x06
REG_VERSION = 0x08
REG_CRATE = 0x16  # Charge/discharge rate


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
    raw = read_word(bus, REG_VCELL)
    return raw * 78.125e-6  # 78.125 µV per LSB


def read_soc(bus: SMBus) -> float:
    """Return state of charge in percent."""
    raw = read_word(bus, REG_SOC)
    return raw / 256.0  # upper byte = integer %, lower byte = 1/256 %


def read_charge_rate(bus: SMBus) -> float:
    """Return charge/discharge rate in % per hour."""
    raw = read_word(bus, REG_CRATE)
    # Signed 16-bit value, 0.208% per hour per LSB
    if raw >= 0x8000:
        raw -= 0x10000
    return raw * 0.208


def read_version(bus: SMBus) -> int:
    """Return the chip version register."""
    return read_word(bus, REG_VERSION)


def print_status(bus: SMBus) -> None:
    """Print a one-line battery status."""
    voltage = read_voltage(bus)
    soc = read_soc(bus)
    rate = read_charge_rate(bus)
    logger.info(f"Battery: {voltage:.3f} V | {soc:.1f}% | rate: {rate:+.1f} %/hr")


def main() -> None:
    parser = argparse.ArgumentParser(description="Monitor MAX17048 fuel gauge")
    parser.add_argument("--watch", action="store_true", help="poll continuously")
    parser.add_argument("-i", "--interval", type=float, default=5, help="poll interval in seconds")
    args = parser.parse_args()

    try:
        bus = SMBus(I2C_BUS)
    except FileNotFoundError:
        logger.error("I2C bus /dev/i2c-1 not found. Enable I2C via raspi-config.")
        sys.exit(1)

    version = read_version(bus)
    logger.info(f"MAX17048 version: 0x{version:04X}")

    if args.watch:
        try:
            while True:
                print_status(bus)
                time.sleep(args.interval)
        except KeyboardInterrupt:
            logger.info("Stopped.")
    else:
        print_status(bus)

    bus.close()


if __name__ == "__main__":
    main()

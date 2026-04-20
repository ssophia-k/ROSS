"""`ross fuel` — battery fuel-gauge monitor."""

import time

from loguru import logger
import typer

from ross.drivers import fuel_gauge


def fuel(
    watch: bool = typer.Option(False, "--watch", help="poll continuously"),
    interval: float = typer.Option(
        5.0, "-i", "--interval", help="poll interval in seconds (with --watch)"
    ),
) -> None:
    """Monitor battery voltage, SoC, and charge rate via MAX17048."""
    try:
        bus = fuel_gauge.open_bus()
    except FileNotFoundError:
        logger.error("I2C bus /dev/i2c-1 not found. Enable I2C via raspi-config.")
        raise typer.Exit(code=1)

    version = fuel_gauge.read_version(bus)
    logger.info(f"MAX17048 version: 0x{version:04X}")

    def one_shot() -> None:
        v = fuel_gauge.read_voltage(bus)
        s = fuel_gauge.read_soc(bus)
        r = fuel_gauge.read_charge_rate(bus)
        logger.info(f"Battery: {v:.3f} V | {s:.1f}% | rate: {r:+.1f} %/hr")

    try:
        if watch:
            while True:
                one_shot()
                time.sleep(interval)
        else:
            one_shot()
    except KeyboardInterrupt:
        logger.info("Stopped.")
    finally:
        bus.close()

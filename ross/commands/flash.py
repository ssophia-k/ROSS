"""`ross flash` — flash firmware onto ESP32-CAM over UART from Raspberry Pi.

Automates GPIO 0 control and runs esptool. The ESP32-CAM does not expose RST
on its header, so you press the RST button when prompted (twice total: once to
enter flash mode, once to boot the new firmware).

Wiring (see README § UART Connections for flashing):
    RPi GPIO 17 → ESP32 GPIO 0   (boot mode select)
    RPi UART TX → ESP32 RX       (GPIO 14 → GPIO 3)
    RPi UART RX → ESP32 TX       (GPIO 15 → GPIO 1)
    RPi 5V      → ESP32 5V
    RPi GND     → ESP32 GND
"""

from pathlib import Path
import subprocess
import time

import typer

from ross.drivers.gpio import output_low as gpio_output_low
from ross.drivers.gpio import release as gpio_release

DEFAULT_BOOT_GPIO = 17
DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = "460800"
FLASH_MODE = "dio"
FLASH_FREQ = "40m"


def run_esptool(port: str, extra_args: list[str]) -> int:
    """Run an esptool command, return exit code."""
    cmd = ["uv", "run", "esptool", "--port", port] + extra_args
    print(f"→ {' '.join(cmd)}")
    return subprocess.run(cmd).returncode


def parse_images(raw: list[str]) -> list[str]:
    """Flatten [ADDR:]FILE args into esptool's alternating [addr, path] form."""
    result = []
    for arg in raw:
        if ":" in arg:
            addr, path = arg.split(":", 1)
            result.extend([addr, str(Path(path).resolve())])
        else:
            result.extend(["0x10000", str(Path(arg).resolve())])
    return result


def flash(
    images: list[str] = typer.Argument(
        None,
        metavar="[ADDR:]FILE...",
        help=(
            "Firmware images. `file.bin` writes at 0x10000; "
            "`0x1000:file.bin` for explicit addresses."
        ),
    ),
    chip_id: bool = typer.Option(
        False, "--chip-id", help="Only check chip connectivity (don't flash)."
    ),
    erase: bool = typer.Option(
        False, "--erase", help="Erase entire flash before writing."
    ),
    port: str = typer.Option(DEFAULT_PORT, "--port", help="Serial port."),
    baud: str = typer.Option(DEFAULT_BAUD, "--baud", help="Baud rate for flashing."),
) -> None:
    """Flash firmware onto ESP32-CAM over UART (RPi drives GPIO 0)."""
    if not chip_id and not images:
        raise typer.BadParameter(
            "Provide at least one firmware image, or use --chip-id."
        )

    gpio = DEFAULT_BOOT_GPIO
    print(f"→ Driving GPIO {gpio} LOW (selecting flash mode)")
    gpio_output_low(gpio)
    time.sleep(0.1)
    print()
    input("  Press RST on the ESP32-CAM, then press Enter here... ")
    print()

    try:
        if chip_id:
            rc = run_esptool(port, ["--baud", "115200", "chip_id"])
            if rc != 0:
                raise typer.Exit(code=rc)

        if erase:
            print()
            rc = run_esptool(port, ["--baud", baud, "erase_flash"])
            if rc != 0:
                raise typer.Exit(code=rc)
            if images:
                print()
                input(
                    "  Press RST on the ESP32-CAM again, then press Enter here... "
                )
                print()

        if images:
            print()
            image_args = parse_images(images)
            rc = run_esptool(
                port,
                [
                    "--baud",
                    baud,
                    "--chip",
                    "esp32",
                    "write-flash",
                    "--flash-mode",
                    FLASH_MODE,
                    "--flash-freq",
                    FLASH_FREQ,
                    "--flash-size",
                    "detect",
                    *image_args,
                ],
            )
            if rc != 0:
                raise typer.Exit(code=rc)
    finally:
        print()
        print(f"→ Releasing GPIO {gpio} (selecting normal boot)")
        gpio_release(gpio)

    print()
    print("Done! Press RST on the ESP32-CAM to boot the new firmware.")

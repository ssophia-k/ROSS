"""Flash firmware onto ESP32-CAM from a Raspberry Pi over UART.

Automates GPIO 0 control and runs esptool. The ESP32-CAM does not expose an RST
pin on its header, so you must press the RST button when prompted (twice total:
once to enter flash mode, once to boot the new firmware).

Wiring assumed (see README § UART Connections for flashing):
    RPi GPIO 17 → ESP32 GPIO 0   (boot mode select)
    RPi UART TX → ESP32 RX       (GPIO 14 → GPIO 3)
    RPi UART RX → ESP32 TX       (GPIO 15 → GPIO 1)
    RPi 5V      → ESP32 5V       (powers the board)
    RPi GND     → ESP32 GND

Usage:
    uv run python ross/flash.py firmware.bin              # writes to 0x10000
    uv run python ross/flash.py --erase firmware.bin
    uv run python ross/flash.py 0x1000:bootloader.bin 0x8000:partitions.bin 0x10000:firmware.bin
    uv run python ross/flash.py --chip-id
"""

import argparse
from pathlib import Path
import subprocess
import sys
import time

from ross.gpio import output_low as gpio_output_low
from ross.gpio import release as gpio_release

# ── Defaults ───────────────────────────────────────────────────────────────────

DEFAULT_BOOT_GPIO = 17
DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = "460800"
FLASH_MODE = "dio"
FLASH_FREQ = "40m"


# ── esptool wrappers ──────────────────────────────────────────────────────────


def run_esptool(port: str, extra_args: list[str]) -> int:
    """Run an esptool command, return exit code."""
    cmd = ["uv", "run", "esptool", "--port", port] + extra_args
    print(f"→ {' '.join(cmd)}")
    result = subprocess.run(cmd)
    return result.returncode


# ── Image argument parsing ────────────────────────────────────────────────────


def parse_images(raw: list[str]) -> list[str]:
    """Parse image arguments. Accepts two formats:

    Positional:   firmware.bin            → writes to 0x10000 (default app address)
    Addr:file:    0x10000:firmware.bin     → writes to 0x10000

    Returns a flat list of [addr, file, addr, file, ...] for esptool.
    """
    result = []
    for arg in raw:
        if ":" in arg:
            addr, path = arg.split(":", 1)
            result.extend([addr, str(Path(path).resolve())])
        else:
            result.extend(["0x10000", str(Path(arg).resolve())])
    return result


# ── Main ──────────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Flash firmware onto ESP32-CAM from a Raspberry Pi.",
        epilog="Examples:\n"
               "  %(prog)s firmware.bin\n"
               "  %(prog)s --erase firmware.bin\n"
               "  %(prog)s 0x1000:bootloader.bin 0x8000:partitions.bin 0x10000:firmware.bin\n"
               "  %(prog)s --chip-id\n",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "images", nargs="*", metavar="[ADDR:]FILE",
        help="Firmware images to flash. Use ADDR:FILE for explicit addresses "
             "(e.g. 0x1000:bootloader.bin) or just FILE to flash at 0x10000.",
    )
    parser.add_argument(
        "--chip-id", action="store_true",
        help="Only check chip connectivity (don't flash).",
    )
    parser.add_argument(
        "--erase", action="store_true",
        help="Erase entire flash before writing.",
    )
    parser.add_argument(
        "--port", default=DEFAULT_PORT,
        help=f"Serial port (default: {DEFAULT_PORT}).",
    )
    parser.add_argument(
        "--baud", default=DEFAULT_BAUD,
        help=f"Baud rate for flashing (default: {DEFAULT_BAUD}).",
    )
    args = parser.parse_args()

    if not args.chip_id and not args.images:
        parser.error("Provide at least one firmware image, or use --chip-id.")

    port = args.port
    baud = args.baud
    gpio = DEFAULT_BOOT_GPIO

    # ── Enter flash mode ──────────────────────────────────────────────────
    print(f"→ Driving GPIO {gpio} LOW (selecting flash mode)")
    gpio_output_low(gpio)
    time.sleep(0.1)
    print()
    input("  Press RST on the ESP32-CAM, then press Enter here... ")
    print()

    try:
        if args.chip_id:
            rc = run_esptool(port, ["--baud", "115200", "chip_id"])
            if rc != 0:
                sys.exit(rc)

        if args.erase:
            print()
            rc = run_esptool(port, ["--baud", baud, "erase_flash"])
            if rc != 0:
                sys.exit(rc)

            if args.images:
                # erase_flash resets the chip, so we need to re-enter flash mode
                print()
                input("  Press RST on the ESP32-CAM again, then press Enter here... ")
                print()

        if args.images:
            print()
            image_args = parse_images(args.images)
            rc = run_esptool(port, [
                "--baud", baud,
                "--chip", "esp32",
                "write-flash",
                "--flash-mode", FLASH_MODE,
                "--flash-freq", FLASH_FREQ,
                "--flash-size", "detect",
                *image_args,
            ])
            if rc != 0:
                sys.exit(rc)

    finally:
        # Always release GPIO 0, even on failure
        print()
        print(f"→ Releasing GPIO {gpio} (selecting normal boot)")
        gpio_release(gpio)

    print()
    print("Done! Press RST on the ESP32-CAM to boot the new firmware.")


if __name__ == "__main__":
    main()

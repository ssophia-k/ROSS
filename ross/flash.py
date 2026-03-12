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
    uv run python ross/flash.py firmware.bin
    uv run python ross/flash.py --erase firmware.bin
    uv run python ross/flash.py 0x1000:bootloader.bin 0x8000:partitions.bin 0x10000:firmware.bin
    uv run python ross/flash.py --chip-id
"""

import argparse
import subprocess
import sys
import time

# ── Configuration ──────────────────────────────────────────────────────────────

BOOT_GPIO = 17  # RPi GPIO pin wired to ESP32 GPIO 0
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = "460800"
FLASH_MODE = "dio"
FLASH_FREQ = "40m"

# ── GPIO helpers (uses pinctrl, no extra libraries) ────────────────────────────


def gpio_output_low(pin: int) -> None:
    """Drive a GPIO pin LOW (output mode, drive low)."""
    subprocess.run(["pinctrl", "set", str(pin), "op", "dl"], check=True)


def gpio_release(pin: int) -> None:
    """Release a GPIO pin back to input (high-impedance). Internal pull-up on
    ESP32 GPIO 0 will restore HIGH."""
    subprocess.run(["pinctrl", "set", str(pin), "ip"], check=True)


def enter_flash_mode() -> None:
    """Pull GPIO 0 LOW and prompt user to press RST."""
    print(f"→ Driving GPIO {BOOT_GPIO} LOW (selecting flash mode)")
    gpio_output_low(BOOT_GPIO)
    time.sleep(0.1)
    print()
    input("  Press RST on the ESP32-CAM, then press Enter here... ")
    print()


def exit_flash_mode() -> None:
    """Release GPIO 0 and prompt user to press RST for normal boot."""
    print(f"→ Releasing GPIO {BOOT_GPIO} (selecting normal boot)")
    gpio_release(BOOT_GPIO)


# ── esptool wrappers ──────────────────────────────────────────────────────────


def run_esptool(args: list[str]) -> int:
    """Run an esptool command, return exit code."""
    cmd = ["uv", "run", "esptool", "--port", SERIAL_PORT] + args
    print(f"→ {' '.join(cmd)}")
    result = subprocess.run(cmd)
    return result.returncode


def chip_id() -> int:
    return run_esptool(["--baud", "115200", "chip_id"])


def erase_flash() -> int:
    return run_esptool(["--baud", BAUD_RATE, "erase_flash"])


def write_flash(image_args: list[str]) -> int:
    return run_esptool([
        "--baud", BAUD_RATE,
        "--chip", "esp32",
        "write_flash",
        "--flash_mode", FLASH_MODE,
        "--flash_freq", FLASH_FREQ,
        "--flash_size", "detect",
        *image_args,
    ])


# ── Argument parsing ──────────────────────────────────────────────────────────


def parse_images(raw: list[str]) -> list[str]:
    """Parse image arguments. Accepts two formats:

    Positional:   firmware.bin            → writes to 0x0
    Addr:file:    0x10000:firmware.bin     → writes to 0x10000

    Returns a flat list of [addr, file, addr, file, ...] for esptool.
    """
    result = []
    for arg in raw:
        if ":" in arg:
            addr, path = arg.split(":", 1)
            result.extend([addr, path])
        else:
            # Single binary — default to address 0x0
            result.extend(["0x0", arg])
    return result


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
             "(e.g. 0x10000:firmware.bin) or just FILE to flash at 0x0.",
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
        "--port", default=SERIAL_PORT,
        help=f"Serial port (default: {SERIAL_PORT}).",
    )
    parser.add_argument(
        "--baud", default=BAUD_RATE,
        help=f"Baud rate for flashing (default: {BAUD_RATE}).",
    )
    args = parser.parse_args()

    # Apply overrides
    global SERIAL_PORT, BAUD_RATE
    SERIAL_PORT = args.port
    BAUD_RATE = args.baud

    if not args.chip_id and not args.images:
        parser.error("Provide at least one firmware image, or use --chip-id.")

    # ── Enter flash mode ──────────────────────────────────────────────────
    enter_flash_mode()

    try:
        if args.chip_id:
            rc = chip_id()
            if rc != 0:
                sys.exit(rc)

        if args.erase:
            print()
            rc = erase_flash()
            if rc != 0:
                sys.exit(rc)

        if args.images:
            print()
            image_args = parse_images(args.images)
            rc = write_flash(image_args)
            if rc != 0:
                sys.exit(rc)

    finally:
        # Always release GPIO 0, even on failure
        print()
        exit_flash_mode()

    print()
    print("Done! Press RST on the ESP32-CAM to boot the new firmware.")


if __name__ == "__main__":
    main()

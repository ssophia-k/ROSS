"""Basic serial test: verify the Pi can receive data from the ESP32 over UART.

Pulls GPIO0 low, waits for you to press RST, then listens for any bytes
on the serial port. This confirms wiring and UART connectivity independent
of esptool.

Usage:
    uv run python ross/serial_test.py
    uv run python ross/serial_test.py --port /dev/ttyAMA10 --baud 115200
"""

import argparse
import subprocess
import sys
import time

from ross.gpio import output_low as gpio_output_low
from ross.gpio import release as gpio_release

DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 115200
DEFAULT_GPIO = 17
LISTEN_SECONDS = 3


def main() -> None:
    parser = argparse.ArgumentParser(description="Test UART connectivity to ESP32-CAM.")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--gpio", type=int, default=DEFAULT_GPIO)
    parser.add_argument("--timeout", type=int, default=LISTEN_SECONDS,
                        help="Seconds to listen for data.")
    args = parser.parse_args()

    gpio = args.gpio

    print(f"→ Driving GPIO {gpio} LOW (boot mode select)")
    gpio_output_low(gpio)
    time.sleep(0.1)

    print()
    input("  Press RST on the ESP32-CAM, then press Enter here... ")
    print()

    try:
        # Configure port to raw mode at the specified baud rate
        subprocess.run(
            ["stty", "-F", args.port, str(args.baud), "raw", "-echo"],
            check=True,
        )

        print(f"→ Listening on {args.port} at {args.baud} baud for {args.timeout}s...")
        print()

        result = subprocess.run(
            ["timeout", str(args.timeout), "cat", args.port],
            capture_output=True,
        )

        data = result.stdout
        if data:
            print(f"✓ Received {len(data)} bytes — UART link is working!")
            print()
            # Show hex dump of first 128 bytes
            hexdump = subprocess.run(
                ["xxd", "-l", "128"],
                input=data,
                capture_output=True,
                text=True,
            )
            print(hexdump.stdout)
        else:
            print("✗ No data received. Check:")
            print("  - TX/RX wires are crossed (Pi TX → ESP RX, Pi RX → ESP TX)")
            print("  - RST button was pressed while GPIO0 was held low")
            print("  - ESP32 has adequate power (5V, ≥500mA)")
            print("  - Wires are on GPIO 14 (TX) and GPIO 15 (RX)")
            sys.exit(1)

    finally:
        print()
        print(f"→ Releasing GPIO {gpio}")
        gpio_release(gpio)


if __name__ == "__main__":
    main()

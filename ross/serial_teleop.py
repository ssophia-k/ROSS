"""Teleoperate ROSS motors over serial (UART).

Sends a handshake at boot to put the ESP32 into serial mode,
then accepts WASD keyboard input for motor control. No WiFi,
camera, or IMU — just motors over the UART link.

GPIO 3 is shared between UART RX and I2C SCL, so serial mode
skips IMU initialization to keep the UART available.

Usage:
    uv run ross/serial_teleop.py
    uv run ross/serial_teleop.py --speed 200
    uv run ross/serial_teleop.py --port /dev/ttyAMA10
"""

import argparse
import sys
import termios
import time
import tty

from loguru import logger
import serial

DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 115200
DEFAULT_SPEED = 180

HELP_TEXT = """
ROSS Serial Teleop
───────────────────
  W / ↑    forward
  S / ↓    reverse
  A / ←    spin left
  D / →    spin right
  Space    stop
  Q / Esc  quit
───────────────────
"""


def send(ser: serial.Serial, cmd: str) -> str | None:
    """Send a command and return the response line."""
    ser.write(f"{cmd}\n".encode())
    ser.flush()
    try:
        return ser.readline().decode().strip()
    except Exception:
        return None


def get_key() -> str | None:
    """Read a single keypress (blocking). Returns key name or None."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            seq = sys.stdin.read(2)
            return {"[A": "up", "[B": "down", "[C": "right", "[D": "left"}.get(seq)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def handshake(ser: serial.Serial) -> bool:
    """Send the serial mode handshake within the firmware's 2s boot window.

    Continuously sends 'S\\n' so it's already in the UART buffer when the
    ESP32 reboots and enters the handshake window.
    """
    logger.info("Waiting for ESP32 boot (press RST now)...")

    # Drain any stale data
    ser.reset_input_buffer()

    # Spam 'S\n' every 200ms while watching for the confirmation.
    # The ESP32 checks Serial.available() during its 2s window, so
    # having bytes waiting in the buffer guarantees we catch it.
    deadline = time.monotonic() + 15
    last_send = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_send > 0.2:
            ser.write(b"S\n")
            ser.flush()
            last_send = now

        if ser.in_waiting:
            line = ser.readline().decode(errors="replace").strip()
            if not line:
                continue
            logger.debug(f"< {line}")
            if "Serial mode" in line or "UART teleop" in line:
                # Drain any remaining boot output
                time.sleep(0.1)
                while ser.in_waiting:
                    ser.readline()
                return True

        time.sleep(0.05)

    return False


def run(ser: serial.Serial, speed: int) -> None:
    """Main teleop loop."""
    print(HELP_TEXT)

    prev_left, prev_right = 0, 0
    last_key_time = time.monotonic()
    deadman_fired = False

    try:
        while True:
            key = get_key()
            left, right = 0, 0
            moved = False

            if key in ("w", "W", "up"):
                left, right = speed, speed
                moved = True
            elif key in ("s", "S", "down"):
                left, right = -speed, -speed
                moved = True
            elif key in ("a", "A", "left"):
                left, right = -speed, speed
                moved = True
            elif key in ("d", "D", "right"):
                left, right = speed, -speed
                moved = True
            elif key == " ":
                left, right = 0, 0
                moved = True
            elif key in ("q", "Q", "\x1b", "\x03"):  # q, Esc, Ctrl-C
                break

            if moved:
                last_key_time = time.monotonic()
                deadman_fired = False

            # Deadman: stop if no key for 500ms
            if time.monotonic() - last_key_time > 0.5 and not deadman_fired:
                left, right = 0, 0
                deadman_fired = True
                moved = True

            if (left, right) != (prev_left, prev_right):
                if left == 0 and right == 0:
                    resp = send(ser, "S")
                else:
                    resp = send(ser, f"M {left} {right}")
                if resp:
                    logger.debug(resp)
                prev_left, prev_right = left, right

    except KeyboardInterrupt:
        pass
    finally:
        send(ser, "S")
        logger.info("Stopped.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Teleoperate ROSS over serial")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="motor speed 0-255 (default: 180)")
    parser.add_argument("--no-handshake", action="store_true", help="skip boot handshake (ESP32 already in serial mode)")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        logger.error(f"Cannot open {args.port}: {e}")
        sys.exit(1)

    try:
        if not args.no_handshake:
            logger.info("Press RST on the ESP32 to trigger boot handshake...")
            if not handshake(ser):
                logger.error("Handshake failed — ESP32 did not enter serial mode")
                logger.info("Tip: press RST right before running this script, or use --no-handshake")
                sys.exit(1)

        logger.info(f"Serial teleop: port={args.port} speed={args.speed}")
        run(ser, args.speed)
    finally:
        ser.close()


if __name__ == "__main__":
    main()

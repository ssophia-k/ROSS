"""`ross serial ...` — UART-level debugging (GPIO 0 + serial link).

Two subcommands:
  `ross serial test`   — verify UART wiring; listen for bytes after RST
  `ross serial teleop` — drive motors over UART (no WiFi)
"""

import subprocess
import sys
import termios
import time
import tty

from loguru import logger
import serial as pyserial
import typer

from ross.drivers.gpio import output_low as gpio_output_low
from ross.drivers.gpio import release as gpio_release

DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 115200
DEFAULT_SPEED = 180
DEFAULT_GPIO = 17
LISTEN_SECONDS = 3

app = typer.Typer(
    help="UART-level debugging (GPIO 0 + serial).", no_args_is_help=True
)

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


@app.command("test")
def serial_test(
    port: str = typer.Option(DEFAULT_PORT, "--port", help="serial port"),
    baud: int = typer.Option(DEFAULT_BAUD, "--baud", help="baud rate"),
    gpio: int = typer.Option(
        DEFAULT_GPIO, "--gpio", help="RPi GPIO pin wired to ESP32 GPIO 0"
    ),
    timeout: int = typer.Option(
        LISTEN_SECONDS, "--timeout", help="seconds to listen for data"
    ),
) -> None:
    """Verify UART connectivity — hold GPIO 0 low, wait for RST, listen for bytes."""
    print(f"→ Driving GPIO {gpio} LOW (boot mode select)")
    gpio_output_low(gpio)
    time.sleep(0.1)

    print()
    input("  Press RST on the ESP32-CAM, then press Enter here... ")
    print()

    try:
        subprocess.run(
            ["stty", "-F", port, str(baud), "raw", "-echo"], check=True
        )
        print(f"→ Listening on {port} at {baud} baud for {timeout}s...")
        print()
        result = subprocess.run(
            ["timeout", str(timeout), "cat", port], capture_output=True
        )
        data = result.stdout
        if data:
            print(f"✓ Received {len(data)} bytes — UART link is working!")
            print()
            hexdump = subprocess.run(
                ["xxd", "-l", "128"], input=data, capture_output=True, text=True
            )
            print(hexdump.stdout)
        else:
            print("✗ No data received. Check:")
            print("  - TX/RX wires are crossed (Pi TX → ESP RX, Pi RX → ESP TX)")
            print("  - RST button was pressed while GPIO0 was held low")
            print("  - ESP32 has adequate power (5V, ≥500mA)")
            print("  - Wires are on GPIO 14 (TX) and GPIO 15 (RX)")
            raise typer.Exit(code=1)
    finally:
        print()
        print(f"→ Releasing GPIO {gpio}")
        gpio_release(gpio)


@app.command("teleop")
def serial_teleop(
    port: str = typer.Option(DEFAULT_PORT, "--port", help="serial port"),
    baud: int = typer.Option(DEFAULT_BAUD, "--baud", help="baud rate"),
    speed: int = typer.Option(
        DEFAULT_SPEED, "--speed", min=0, max=255, help="motor speed 0-255"
    ),
    no_handshake: bool = typer.Option(
        False, "--no-handshake", help="skip the boot handshake"
    ),
) -> None:
    """Drive ROSS motors over UART (no WiFi). Requires ESP32 in serial mode."""
    try:
        ser = pyserial.Serial(port, baud, timeout=0.1)
    except pyserial.SerialException as e:
        logger.error(f"Cannot open {port}: {e}")
        raise typer.Exit(code=1)

    try:
        if not no_handshake:
            logger.info("Press RST on the ESP32 to trigger boot handshake...")
            if not _handshake(ser):
                logger.error("Handshake failed — ESP32 did not enter serial mode")
                logger.info(
                    "Tip: press RST right before running this command, "
                    "or pass --no-handshake"
                )
                raise typer.Exit(code=1)

        logger.info(f"Serial teleop: port={port} speed={speed}")
        _run(ser, speed)
    finally:
        ser.close()


def _send(ser: pyserial.Serial, cmd: str) -> str | None:
    ser.write(f"{cmd}\n".encode())
    ser.flush()
    try:
        return ser.readline().decode().strip()
    except Exception:
        return None


def _get_key() -> str | None:
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


def _handshake(ser: pyserial.Serial) -> bool:
    """Send `S\\n` in a tight loop within the firmware's 2 s boot window."""
    logger.info("Waiting for ESP32 boot (press RST now)...")
    ser.reset_input_buffer()
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
                time.sleep(0.1)
                while ser.in_waiting:
                    ser.readline()
                return True
        time.sleep(0.05)
    return False


def _run(ser: pyserial.Serial, speed: int) -> None:
    print(HELP_TEXT)
    prev_left, prev_right = 0, 0
    last_key_time = time.monotonic()
    deadman_fired = False

    try:
        while True:
            key = _get_key()
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
            elif key in ("q", "Q", "\x1b", "\x03"):
                break

            if moved:
                last_key_time = time.monotonic()
                deadman_fired = False

            if time.monotonic() - last_key_time > 0.5 and not deadman_fired:
                left, right = 0, 0
                deadman_fired = True
                moved = True

            if (left, right) != (prev_left, prev_right):
                if left == 0 and right == 0:
                    resp = _send(ser, "S")
                else:
                    resp = _send(ser, f"M {left} {right}")
                if resp:
                    logger.debug(resp)
                prev_left, prev_right = left, right
    except KeyboardInterrupt:
        pass
    finally:
        _send(ser, "S")
        logger.info("Stopped.")

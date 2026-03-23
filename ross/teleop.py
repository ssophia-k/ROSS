"""Teleoperate ROSS via keyboard over WiFi.

Displays the MJPEG video stream and sends motor commands
based on WASD / arrow key input.

Usage:
    uv run python ross/teleop.py                # auto-discover via mDNS
    uv run python ross/teleop.py --host 10.0.0.5  # explicit IP
    uv run python ross/teleop.py --speed 200      # 0-255, default 180
"""

import argparse
import socket
import sys
import time
import urllib.request

import cv2
from loguru import logger
import numpy as np


def discover_host() -> str | None:
    """Try to resolve ross.local via system mDNS, then zeroconf."""
    # System mDNS (works if Avahi is running)
    try:
        results = socket.getaddrinfo("ross.local", 80, socket.AF_INET)
        addr = results[0][4][0]
        logger.info(f"Found ross.local via mDNS: {addr}")
        return addr
    except socket.gaierror:
        logger.debug("System mDNS failed, trying zeroconf")

    # Zeroconf fallback
    try:
        from zeroconf import ServiceBrowser, Zeroconf

        found = {}

        class Listener:
            def add_service(self, zc, type_, name):
                info = zc.get_service_info(type_, name)
                if info and info.server and "ross" in info.server.lower():
                    found["addr"] = socket.inet_ntoa(info.addresses[0])

            def remove_service(self, *_):
                pass

            def update_service(self, *_):
                pass

        zc = Zeroconf()
        ServiceBrowser(zc, "_http._tcp.local.", Listener())
        deadline = time.monotonic() + 5
        while not found and time.monotonic() < deadline:
            time.sleep(0.1)
        zc.close()

        if found:
            logger.info(f"Found ROSS via zeroconf: {found['addr']}")
            return found["addr"]
    except ImportError:
        logger.debug("zeroconf not installed")

    return None


def send_motor(host: str, left: int, right: int) -> None:
    """Send a motor command. Failures are logged and swallowed."""
    try:
        urllib.request.urlopen(f"http://{host}/motor?l={left}&r={right}", timeout=1)
    except Exception as e:
        logger.debug(f"Motor command failed: {e}")


def send_stop(host: str) -> None:
    """Send the stop endpoint."""
    try:
        urllib.request.urlopen(f"http://{host}/stop", timeout=1)
    except Exception:
        pass


def run(host: str, speed: int) -> None:
    """Main video + control loop."""
    cap = None
    left, right = 0, 0
    prev_left, prev_right = 0, 0
    last_key_time = time.monotonic()
    deadman_fired = False

    try:
        while True:
            # (Re)connect to stream
            if cap is None or not cap.isOpened():
                logger.info(f"Connecting to http://{host}:81/stream ...")
                cap = cv2.VideoCapture(f"http://{host}:81/stream")
                if not cap.isOpened():
                    img = np.zeros((240, 320, 3), dtype="uint8")
                    cv2.putText(img, "Connecting...", (60, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.imshow("ROSS", img)
                    cv2.waitKey(2000)
                    continue

            ok, frame = cap.read()
            if not ok:
                logger.warning("Stream dropped, reconnecting...")
                cap.release()
                cap = None
                continue

            cv2.imshow("ROSS", frame)
            key = cv2.waitKeyEx(1)

            left, right = 0, 0
            moved = False

            if key in (ord("w"), ord("W"), 0x10000 + 82, 65362):  # W / Up
                left, right = speed, speed
                moved = True
            elif key in (ord("s"), ord("S"), 0x10000 + 84, 65364):  # S / Down
                left, right = -speed, -speed
                moved = True
            elif key in (ord("a"), ord("A"), 0x10000 + 81, 65361):  # A / Left
                left, right = -speed, speed
                moved = True
            elif key in (ord("d"), ord("D"), 0x10000 + 83, 65363):  # D / Right
                left, right = speed, -speed
                moved = True
            elif key in (ord(" "),):
                left, right = 0, 0
                moved = True
            elif key in (ord("q"), ord("Q"), 27):  # Q / Esc
                break

            if moved:
                last_key_time = time.monotonic()
                deadman_fired = False

            # Deadman switch: stop if no key for 500ms
            if time.monotonic() - last_key_time > 0.5 and not deadman_fired:
                left, right = 0, 0
                deadman_fired = True
                moved = True  # force send

            # Only send when values change
            if (left, right) != (prev_left, prev_right):
                if left == 0 and right == 0:
                    send_stop(host)
                else:
                    send_motor(host, left, right)
                prev_left, prev_right = left, right

    except KeyboardInterrupt:
        pass
    finally:
        send_stop(host)
        if cap:
            cap.release()
        cv2.destroyAllWindows()
        logger.info("Stopped.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Teleoperate ROSS")
    parser.add_argument("--host", help="robot IP or hostname (default: auto-discover)")
    parser.add_argument("--speed", type=int, default=180, help="motor speed 0-255 (default: 180)")
    args = parser.parse_args()

    host = args.host
    if not host:
        host = discover_host()
        if not host:
            logger.error("Could not find ROSS on the network. Use --host to specify manually.")
            sys.exit(1)

    logger.info(f"Teleop: host={host} speed={args.speed}")
    run(host, args.speed)


if __name__ == "__main__":
    main()

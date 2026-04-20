"""`ross teleop` — WiFi teleop (WASD/arrows + MJPEG stream)."""

import time

import cv2
from loguru import logger
import numpy as np
import typer

from ross.net.discovery import discover_host
from ross.net.robot import send_motor, send_stop, stream_url


def teleop(
    host: str = typer.Option(
        None, "--host", help="robot IP or hostname (default: auto-discover via mDNS)"
    ),
    speed: int = typer.Option(
        180, "--speed", min=0, max=255, help="motor speed 0-255"
    ),
) -> None:
    """Drive ROSS over WiFi with WASD/arrow keys; shows the MJPEG stream."""
    if not host:
        host = discover_host()
        if not host:
            logger.error(
                "Could not find ROSS on the network. Use --host to specify manually."
            )
            raise typer.Exit(code=1)

    logger.info(f"Teleop: host={host} speed={speed}")
    _run(host, speed)


def _run(host: str, speed: int) -> None:
    cap = None
    prev_left, prev_right = 0, 0
    last_key_time = time.monotonic()
    deadman_fired = False

    try:
        while True:
            if cap is None or not cap.isOpened():
                logger.info(f"Connecting to {stream_url(host)} ...")
                cap = cv2.VideoCapture(stream_url(host))
                if not cap.isOpened():
                    img = np.zeros((240, 320, 3), dtype="uint8")
                    cv2.putText(
                        img,
                        "Connecting...",
                        (60, 130),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 255, 255),
                        2,
                    )
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

            if key in (ord("w"), ord("W"), 0x10000 + 82, 65362):
                left, right = speed, speed
                moved = True
            elif key in (ord("s"), ord("S"), 0x10000 + 84, 65364):
                left, right = -speed, -speed
                moved = True
            elif key in (ord("a"), ord("A"), 0x10000 + 81, 65361):
                left, right = -speed, speed
                moved = True
            elif key in (ord("d"), ord("D"), 0x10000 + 83, 65363):
                left, right = speed, -speed
                moved = True
            elif key == ord(" "):
                left, right = 0, 0
                moved = True
            elif key in (ord("q"), ord("Q"), 27):
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

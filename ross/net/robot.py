"""HTTP client for the robot's on-board command server (port 80).

Failures are logged and swallowed — teleop loops should never crash
because of a missed packet.
"""

import json
import urllib.request

from loguru import logger


def send_motor(host: str, left: int, right: int, timeout: float = 1.0) -> None:
    """Send a motor command. Failures are logged and swallowed."""
    try:
        urllib.request.urlopen(
            f"http://{host}/motor?l={left}&r={right}", timeout=timeout
        )
    except Exception as e:
        logger.debug(f"Motor command failed: {e}")


def send_stop(host: str, timeout: float = 1.0) -> None:
    """Send the stop endpoint. Failures are silently ignored."""
    try:
        urllib.request.urlopen(f"http://{host}/stop", timeout=timeout)
    except Exception:
        pass


def fetch_imu(host: str, timeout: float = 2.0) -> dict | None:
    """Fetch one IMU reading from the ESP32. Returns dict or None."""
    try:
        resp = urllib.request.urlopen(f"http://{host}/imu", timeout=timeout)
        return json.loads(resp.read())
    except Exception:
        return None


def stream_url(host: str) -> str:
    """Return the MJPEG stream URL (port 81)."""
    return f"http://{host}:81/stream"

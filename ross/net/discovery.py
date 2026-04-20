"""Discover a running ROSS robot on the local network.

Tries system mDNS (Avahi-backed `getaddrinfo`) first, falls back to
the `zeroconf` Python library if available.
"""

import socket
import time

from loguru import logger


def discover_host() -> str | None:
    """Return the IP of a ROSS robot on the local network, or None."""
    try:
        results = socket.getaddrinfo("ross.local", 80, socket.AF_INET)
        addr = results[0][4][0]
        logger.info(f"Found ross.local via mDNS: {addr}")
        return addr
    except socket.gaierror:
        logger.debug("System mDNS failed, trying zeroconf")

    try:
        from zeroconf import ServiceBrowser, Zeroconf

        found: dict[str, str] = {}

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

"""Raspberry Pi GPIO helpers using pinctrl (no external libraries)."""

import subprocess


def output_low(pin: int) -> None:
    """Drive a GPIO pin LOW (output mode)."""
    subprocess.run(["pinctrl", "set", str(pin), "op", "dl"], check=True)


def release(pin: int) -> None:
    """Release a GPIO pin back to input (high-impedance)."""
    subprocess.run(["pinctrl", "set", str(pin), "ip"], check=True)

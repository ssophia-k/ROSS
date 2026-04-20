"""Top-level Typer app exposed as the `ross` entry point."""

import typer

from ross.commands import flash as flash_cmd
from ross.commands import fuel as fuel_cmd
from ross.commands import serial as serial_cmd
from ross.commands import slam as slam_cmd
from ross.commands import teleop as teleop_cmd

app = typer.Typer(
    name="ross",
    help="ROSS — Remote Observation Scout Suite.",
    no_args_is_help=True,
    context_settings={"help_option_names": ["-h", "--help"]},
)

app.command(name="flash", help="Flash firmware over UART (RPi → ESP32-CAM).")(
    flash_cmd.flash
)
app.command(name="teleop", help="WiFi teleop: WASD drive + MJPEG view.")(
    teleop_cmd.teleop
)
app.command(name="slam", help="Room scan via ESP32-CAM stream + SLAM.")(slam_cmd.slam)
app.command(name="fuel", help="Monitor battery via MAX17048 fuel gauge.")(fuel_cmd.fuel)
app.add_typer(serial_cmd.app, name="serial")


if __name__ == "__main__":
    app()

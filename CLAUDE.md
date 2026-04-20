# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with
code in this repository.

## Architecture

- **Robot (ESP32-CAM)**: Streams MJPEG video + IMU over WiFi. OV2640 camera,
  DRV8833 motor driver, LSM6DS3TR-C IMU.
- **Station (Raspberry Pi)**: Flashes firmware over UART (GPIO 17 drives
  ESP32 boot mode via `pinctrl`), monitors battery via MAX17048 over I2C,
  runs teleop and SLAM against the robot's HTTP endpoints.

## Two CLIs, one boundary

- **`make`** — dev infra only (firmware build, serial monitor, env, lint,
  clean). No Python invocations.
- **`ross`** (Typer) — every Python feature. Entry: `uv run ross --help`.
  Subcommands: `flash`, `teleop`, `slam`, `fuel`, `serial {test,teleop}`.

Adding a new Python feature: put logic under `ross/{drivers,net,slam}/`,
write a thin Typer wrapper in `ross/commands/<name>.py`, register it in
`ross/cli.py`. Don't add a Makefile target for it.

## Build & run

```bash
make setup-env   # interactive WiFi → .env
make build       # pio run (firmware)
uv run ross flash firmware/.pio/build/esp32cam/firmware.bin
uv run ross teleop
```

WiFi credentials are injected at firmware build time from `.env` via
`firmware/load_env.py` (PlatformIO pre-build script).

## Key design constraints

- **`camera.cpp` is separate from `main.cpp`** to avoid `sensor_t` type
  redefinition between `esp_camera.h` and the Adafruit sensor headers. Do
  not merge them.
- **GPIO 3 (UART RX) is reused as I2C SCL** at runtime. The Pi must release
  the UART during normal robot operation.
- **GPIO 0, 2, 12, 15 are strapping pins** — their state at reset determines
  boot mode and flash voltage. See README § Strapping pin notes.
- **No SD card** — all SD-interface GPIOs (2, 12, 13, 14, 15) are allocated
  to motors and I2C.

## Firmware HTTP endpoints

- `GET /` — status page
- `GET /stream` — MJPEG video (port 81)
- `GET /motor?l=<-255..255>&r=<-255..255>` — motor command
- `GET /stop` — motor stop
- `GET /imu` — JSON `{accel, gyro, temp}`

## Repository layout

Run `tree -L 2 -I '.venv|.git|__pycache__'` for the current shape. Key dirs:

- `firmware/` — ESP32-CAM PlatformIO project (unchanged from the early
  repo; don't refactor).
- `ross/` — Python package, subpackaged by concern:
  `commands/` (Typer wrappers), `drivers/` (RPi hardware), `net/` (mDNS +
  robot HTTP client), `slam/` (depth, detection, reconstruction, I/O).
- `notebooks/` — experimental prototypes; not supported.
- `outputs/`, `models/` — gitignored (scan outputs and cached weights).

## Gotchas when editing

- `ross slam` writes to `outputs/` by default; don't hardcode output paths.
- YOLO weights are cached at `models/yolov8n.pt` — first run downloads them.
- `discover_host()` lives in `ross/net/discovery.py`; reuse it from any
  command that needs to find the robot on the network. Don't duplicate.
- `send_motor` / `send_stop` / `fetch_imu` / `stream_url` all live in
  `ross/net/robot.py`.

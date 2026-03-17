# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROSS (Robotic Operating Swarm System) is a teleoperated mobile robot built around the ESP32-CAM. The robot streams MJPEG video and IMU data over WiFi while accepting motor commands from a Linux workstation. A Raspberry Pi serves as a flashing and charging station.

## Architecture

Three subsystems communicate over WiFi and UART:

- **Robot (ESP32-CAM)**: Runs firmware that serves HTTP endpoints for video streaming, IMU data, and motor control. Camera (OV2640), IMU (LSM6DS3TR-C on I2C), and dual motors (via DRV8833 PWM driver).
- **Station (Raspberry Pi)**: Flashes firmware over UART (GPIO 17 controls ESP32 boot mode via pinctrl), monitors battery via MAX17048 fuel gauge over I2C.
- **Workstation (Linux)**: Teleoperation client (consumes /stream and /imu, sends /motor commands).

## Build & Run Commands

### Firmware (PlatformIO, C++/Arduino)

```bash
make setup-env                # Interactive WiFi credential setup → .env
cd firmware && pio run        # Build firmware
```

WiFi credentials are injected at build time from `.env` via `firmware/load_env.py` (PlatformIO pre-build script).

### Python Utilities (uv + flit)

```bash
uv sync                                          # Install dependencies
uv run python ross/flash.py firmware/.pio/build/esp32cam/firmware.bin  # Flash ESP32
uv run python ross/flash.py --chip-id            # Detect chip
uv run python ross/fuel_gauge.py --watch          # Monitor battery
uv run python ross/serial_test.py                 # UART connectivity test
```

### Serial Monitoring

```bash
screen /dev/ttyAMA0 115200
```

## Key Design Constraints

- **camera.cpp is separate from main.cpp** to avoid `sensor_t` type redefinition between `esp_camera.h` and Adafruit sensor headers. Do not merge them.
- **GPIO 3 (UART RX) is reused as I2C SCL** at runtime. The Pi must release the UART during normal robot operation.
- **GPIO 0, 2, 12, 15 are strapping pins** — their state at reset determines boot mode. Motor/SD-related code must account for this.
- **No SD card** — all SD-interface GPIOs (2, 12, 13, 14, 15) are allocated to motors and I2C.
- **DRV8833 current limit** — 1.2A continuous; motors stall at 1.5A. Avoid sustained stall conditions.

## Firmware HTTP Endpoints

- `GET /` — Status page
- `GET /stream` — MJPEG video
- `GET /imu` — JSON accelerometer/gyroscope/temperature
- `GET /motor?l=<-255..255>&r=<-255..255>` — Set motor speeds
- `GET /stop` — Stop motors

## Repository Layout

- `firmware/` — ESP32-CAM PlatformIO project (main.cpp, config.h, motors.h, camera.h/.cpp)
- `ross/` — Python package: flash.py (UART flashing), fuel_gauge.py (MAX17048 I2C), serial_test.py, config.py
- `docs/` — Hardware reference images
- `.env` / `.env.sample` — WiFi credentials (gitignored / template)

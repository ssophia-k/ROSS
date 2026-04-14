# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Architecture

- **Robot (ESP32-CAM)**: Streams MJPEG video over WiFi. Camera (OV2640). IMU and motors are wired but disabled.
- **Station (Raspberry Pi)**: Flashes firmware over UART (GPIO 17 controls ESP32 boot mode via pinctrl), monitors battery via MAX17048 fuel gauge over I2C.

## Build & Run Commands

```bash
make setup-env     # Interactive WiFi credential setup → .env
make build         # Build firmware
make deploy        # Build + flash
make serial        # Monitor serial output
```

WiFi credentials are injected at build time from `.env` via `firmware/load_env.py` (PlatformIO pre-build script).

## Key Design Constraints

- **camera.cpp is separate from main.cpp** to avoid `sensor_t` type redefinition between `esp_camera.h` and Adafruit sensor headers. Do not merge them.
- **GPIO 3 (UART RX) is reused as I2C SCL** at runtime. The Pi must release the UART during normal robot operation.
- **GPIO 0, 2, 12, 15 are strapping pins** — their state at reset determines boot mode.
- **No SD card** — all SD-interface GPIOs (2, 12, 13, 14, 15) are allocated to motors and I2C.

## Firmware HTTP Endpoints

- `GET /` — Status page
- `GET /stream` — MJPEG video

## Repository Layout

- `firmware/` — ESP32-CAM PlatformIO project (main.cpp, config.h, camera.h/.cpp, motors.h)
- `ross/` — Python package: flash.py (UART flashing), config.py, and other utilities
- `docs/` — Hardware reference images
- `.env` / `.env.sample` — WiFi credentials (gitignored / template)

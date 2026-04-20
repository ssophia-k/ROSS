# ROSS — Remote Observation Scout Suite

An ultra-low-cost scout robot for exploratory applications. An ESP32-CAM
streams MJPEG video and IMU data over WiFi; a Raspberry Pi station flashes
firmware, monitors the battery, and runs teleop and SLAM from a single CLI.

| Subsystem   | Hardware                                       | Role                                              |
|-------------|------------------------------------------------|---------------------------------------------------|
| **Robot**   | ESP32-CAM, IMU, DRV8833 motor driver, battery  | Streams video, drives motors, reports IMU         |
| **Station** | Raspberry Pi, MAX17048 fuel gauge, USB-C charger | Flashes firmware over UART, monitors battery, teleoperates |

---

## Quickstart

```bash
# 1. Install dependencies (uv + PlatformIO)
uv sync
uv tool install platformio

# 2. Configure WiFi credentials
make setup-env

# 3. Build firmware and flash it over UART
make build
uv run ross flash firmware/.pio/build/esp32cam/firmware.bin

# 4. Drive the robot once it reboots and joins WiFi
uv run ross teleop
```

`uv run ross --help` lists every subcommand and its flags.

---

## CLI reference

Python features live under one Typer entry point. Run `uv run ross --help`
(or `ross --help` when the venv is on `PATH`) to see the current surface.
For per-command options, use `ross <command> --help`.

| Command                | What it does                                                      |
|------------------------|-------------------------------------------------------------------|
| `ross flash`           | Flash firmware onto ESP32-CAM over UART (drives GPIO 0, runs esptool) |
| `ross teleop`          | WiFi teleop: WASD/arrow keys + MJPEG stream, deadman switch       |
| `ross slam`            | Room scan via MiDaS depth + ICP + Poisson mesh + YOLO humans      |
| `ross fuel`            | Poll battery voltage / state of charge / charge rate              |
| `ross serial test`     | Verify UART wiring — listen for bytes after RST                   |
| `ross serial teleop`   | Drive motors over UART (no WiFi)                                  |

SLAM outputs land in `outputs/` by default; YOLO weights are cached in
`models/`. Both directories are gitignored.

---

## Make targets

The Makefile is kept short on purpose — it handles firmware and dev infra
only. Python features are in the `ross` CLI above.

| Target          | Purpose                                          |
|-----------------|--------------------------------------------------|
| `make help`     | List targets and point to `ross --help`          |
| `make setup-env`| Interactive WiFi credentials → `.env`            |
| `make build`    | Build firmware (`pio run`)                       |
| `make serial`   | Monitor serial output (`screen /dev/ttyAMA0`)    |
| `make lint`     | `ruff check` on Python sources                   |
| `make format`   | `ruff format` on Python sources                  |
| `make clean`    | Remove `firmware/.pio/`                          |

---

## Repository layout

Run `tree -L 2 -I '.venv|.git|__pycache__'` for the current shape. At a
glance:

- `firmware/` — ESP32-CAM PlatformIO project (C++). `src/main.cpp` runs the
  HTTP + MJPEG servers; `src/camera.cpp` is isolated to avoid `sensor_t`
  conflicts between Adafruit sensor headers and `esp_camera.h`.
  `load_env.py` injects WiFi credentials from `.env` at build time.
- `ross/` — Python package:
  - `ross/cli.py`, `ross/commands/` — Typer surface (thin wrappers).
  - `ross/drivers/` — RPi-side hardware (GPIO via `pinctrl`, MAX17048 I2C).
  - `ross/net/` — shared networking (mDNS discovery, robot HTTP client).
  - `ross/slam/` — MiDaS depth, YOLO + Re-ID, ICP registration, Poisson mesh,
    PLY I/O.
- `notebooks/` — experimental prototypes (ORB-SLAM, MiDaS variants, etc.)
  kept for reference; not supported by the CLI.
- `outputs/`, `models/` — gitignored. SLAM scans and cached model weights.
- `docs/` — hardware reference images.

---

## Firmware

The firmware serves two HTTP servers:

| Port | Endpoint       | Description                     |
|------|----------------|---------------------------------|
| 80   | `GET /`        | HTML status page                |
| 80   | `GET /motor?l=<-255..255>&r=<-255..255>` | Motor command |
| 80   | `GET /stop`    | Stop motors                     |
| 80   | `GET /imu`     | JSON: `{accel, gyro, temp}`     |
| 81   | `GET /stream`  | MJPEG camera stream             |

A 500 ms deadman timer stops the motors if no command arrives.

### Build & flash

```bash
make setup-env   # first time: fill in .env
make build
uv run ross flash firmware/.pio/build/esp32cam/firmware.bin
make serial      # watch the boot log (Ctrl-A k to exit)
```

WiFi credentials are injected at compile time from `.env` via
`firmware/load_env.py`.

---

## Wiring

### Robot

#### Power

Battery → boost converter (3.7 V → 6 V) → ESP32-CAM 5 V pin.

| From                   | To                          | Wire                      |
|------------------------|-----------------------------|---------------------------|
| Battery JST-PH **+**   | Boost converter **VIN**     | Red                       |
| Battery JST-PH **–**   | Common ground bus           | Black                     |
| Boost converter **VOUT** | ESP32-CAM **5V pin**      | Red — 6 V rail            |
| ESP32-CAM **GND**      | Common ground bus           | Black                     |

> The boost converter has no reverse-voltage protection. Double-check
> polarity before applying power.

#### ESP32-CAM pinout

![ESP32-CAM Pinout](docs/esp32-cam-pinout.png)

| GPIO             | Assigned to              | Notes                                       |
|------------------|--------------------------|---------------------------------------------|
| **GPIO 2**       | IMU SDA                  | I2C data · strapping pin                    |
| **GPIO 3** (RX)  | IMU SCL                  | Repurposed after boot                       |
| **GPIO 12**      | DRV8833 AIN1             | Left motor forward · strapping pin          |
| **GPIO 13**      | DRV8833 AIN2             | Left motor reverse                          |
| **GPIO 14**      | DRV8833 BIN1             | Right motor forward                         |
| **GPIO 15**      | DRV8833 BIN2             | Right motor reverse · strapping pin         |
| **GPIO 1** (TX)  | Station RX               | Flashing only                               |
| **GPIO 0**       | Station GPIO             | Boot mode control                           |

#### Strapping pin notes

GPIO 0, 2, 12, and 15 are sampled at reset to configure boot mode and flash
voltage.

| GPIO      | Strapping function   | Required state at boot   | This design                                    |
|-----------|----------------------|--------------------------|-----------------------------------------------|
| **GPIO 0**  | Boot mode select    | HIGH = run, LOW = flash  | Controlled by RPi GPIO 17                     |
| **GPIO 2**  | Download mode       | LOW or floating          | IMU SDA pull-up is weak enough                |
| **GPIO 12** | Flash voltage select| LOW = 3.3 V              | DRV8833 input floats LOW when unpowered       |
| **GPIO 15** | Boot log output     | HIGH = print boot msgs   | DRV8833 BIN2 may suppress boot msgs if LOW    |

### Station (Raspberry Pi)

#### UART connections (for flashing)

| RPi Pin | Signal                | ESP32-CAM Pin | Notes                       |
|---------|-----------------------|---------------|-----------------------------|
| Pin 2   | 5 V                   | **5V**        | Powers ESP32 during flashing |
| Pin 6   | GND                   | **GND**       | Common ground               |
| Pin 8   | GPIO 14 — UART TX     | **GPIO 3 (RX)** | RPi TX → ESP RX           |
| Pin 10  | GPIO 15 — UART RX     | **GPIO 1 (TX)** | ESP TX → RPi RX           |
| Pin 11  | GPIO 17 — output      | **GPIO 0**    | LOW = flash mode            |

#### Battery charging

Fuel gauge (MAX17048) sits in-line between battery and charger. The Pi
monitors state of charge over I2C.

| RPi Pin | Signal          | MAX17048 Pin |
|---------|-----------------|--------------|
| Pin 1   | 3.3 V           | **VIN**      |
| Pin 3   | GPIO 2 (SDA)    | **SDA**      |
| Pin 5   | GPIO 3 (SCL)    | **SCL**      |
| Pin 9   | GND             | **GND**      |

---

## Flashing

### Semi-automated

```bash
uv run ross flash firmware/.pio/build/esp32cam/firmware.bin
```

`ross flash` pulls GPIO 17 low, asks you to press RST, runs esptool, then
releases GPIO 17. Pass `--chip-id` to only verify connectivity, or `--erase`
to wipe the chip before writing.

### Manual

```bash
pinctrl set 17 op dl           # GPIO 0 LOW → flash mode
# Press RST on ESP32-CAM
uv run esptool --port /dev/ttyAMA0 --baud 460800 --chip esp32 \
  write_flash --flash_mode dio --flash_freq 40m --flash_size detect \
  0x10000 firmware/.pio/build/esp32cam/firmware.bin
pinctrl set 17 ip              # Release GPIO 0
# Press RST → ESP32 boots normally
```

### Setup (first time only)

1. Enable hardware UART: `sudo raspi-config` → Interface Options → Serial Port
   → login shell **No**, hardware **Yes**.
2. Grant serial access: `sudo usermod -aG dialout $USER && sudo reboot`.
3. Verify: `ls -l /dev/ttyAMA0`.

---

## Development

- **Package manager**: `uv`. Use `uv sync` for base deps; `uv sync --extra slam`
  pulls in the ML stack (torch, open3d, ultralytics) needed by `ross slam`.
- **Style**: ruff. `make lint` and `make format`.
- **Adding a CLI command**: implement the logic under `ross/drivers/`,
  `ross/net/`, or `ross/slam/`; add a thin Typer wrapper to
  `ross/commands/<name>.py`; register it in `ross/cli.py`.
- **Prototypes**: park experimental code under `notebooks/`. It's not picked
  up by the Typer CLI.

---

## License

MIT License. See [LICENSE](LICENSE) for details.

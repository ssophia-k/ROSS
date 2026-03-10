# External Integrations

**Analysis Date:** 2026-03-09

## Hardware Integrations

**Camera (Webcam / USB):**
- Used for live visual SLAM and color-based distance estimation
- Access: `cv2.VideoCapture(0)` — device index 0 (default camera)
- Used in: `notebooks/cv_distance.py`, `notebooks/distance_calibration.py`, `ross/slam/main.py` (git history)

**BLE Peripheral (Arduino):**
- Firmware: `notebooks/ble_distance.ino` (git history)
- Library: `ArduinoBLE.h` (Arduino BLE library)
- Role: Scans for a BLE device named "Proximity", reads RSSI at 100ms intervals
- Output: Serial tab-delimited `time_ms\tRSSI_dBm` stream for distance estimation
- No Python-side BLE integration detected in current source

## APIs & External Services

**None detected.**

No HTTP API clients, cloud SDKs, or web service integrations are present in the current codebase. This is a local/embedded robotics research project.

## ML Model Integrations

**YOLO (Ultralytics):**
- Used in slam testing scripts (`testing-m/3Dslam.py`, `testing-m/anchor-slam.py`) — present in git history on `hardware` branch
- Purpose: Real-time object detection to anchor SLAM map to known objects
- Not currently in `pyproject.toml` dependencies (removed in "clean" commit)

## Data Storage

**Databases:**
- None detected. No database clients or ORMs present.

**File Storage:**
- Local filesystem only
- Structured paths defined in `ross/config.py`:
  - `data/raw/` — raw input data
  - `data/interim/` — intermediate processed data
  - `data/processed/` — final processed datasets
  - `data/external/` — third-party datasets
  - `models/` — saved model artifacts
  - `reports/figures/` — generated plots
- `data/` directory is gitignored (`/data/` in `.gitignore`)
- Test video: `data/test_video.mp4` (local, not committed)
- Parts list: `parts.csv` (untracked, present on `hardware` branch)

**Caching:**
- None detected beyond Python's standard `__pycache__`

## Authentication & Identity

**None detected.**

No auth providers, OAuth, API keys, or identity services are present.

## Environment Configuration

**`.env` file:**
- Present at project root (gitignored per `.gitignore`)
- Loaded at import time via `python-dotenv` in `ross/config.py`
- No specific required variables documented in source code
- Existence noted; contents not inspected

**No other secrets management** detected (no AWS Secrets Manager, Vault, etc.)

## Monitoring & Observability

**Logging:**
- `loguru` — structured logging throughout the package
- Configured in `ross/config.py` to route through `tqdm.write` when tqdm is installed (prevents progress bar corruption)
- No remote logging or error tracking services detected

**Error Tracking:**
- None (no Sentry, Datadog, etc.)

## Notebooks & Reproducibility

**Jupytext:**
- Syncs Jupyter notebooks (`.ipynb`) with percent-format Python scripts (`.py`)
- Enables git-diffable notebook source while preserving `.ipynb` for interactive use
- Config: implied by `jupytext>=1.18.1` dependency and `.py` counterparts in `notebooks/`

**nbdime:**
- Git diff/merge tool for notebooks
- Version: `>=4.0.2`

## CI/CD & Deployment

**CI Pipeline:**
- Not detected. No GitHub Actions, CircleCI, or other CI config files present.

**Hosting:**
- Not applicable. Local research codebase with no deployment target.

## Webhooks & Callbacks

**Incoming:** None
**Outgoing:** None

---

*Integration audit: 2026-03-09*

# Technology Stack

**Analysis Date:** 2026-03-09

## Languages

**Primary:**
- Python 3.12 - All application code, notebooks, scripts

**Secondary:**
- C/C++ (Arduino/INO) - Embedded BLE firmware (`notebooks/ble_distance.ino` in git history)

## Runtime

**Environment:**
- CPython 3.12.3

**Package Manager:**
- uv 0.9.5 (replaces pip for environment management)
- Lockfile: `uv.lock` (committed to version control)

## Build System

**Core:**
- flit_core >=3.2,<4 - Lightweight Python package build backend
- Config: `pyproject.toml`

## Frameworks

**Notebooks / Interactive:**
- Jupyter >=1.1.1 - Interactive notebook runner
- JupyterLab - Browser-based notebook UI
- Jupytext >=1.18.1 - Sync `.ipynb` ↔ `.py` percent-format notebooks
- IPython - Interactive Python kernel
- nbdime >=4.0.2 - Notebook diff/merge for git

**CLI:**
- Typer - CLI application framework (used in `src/plots.py`, `src/utils.py`)

**Build/Dev:**
- Ruff - Linter and formatter (config in `pyproject.toml`, line-length 99)

## Key Dependencies

**Computer Vision (used in slam module and notebooks):**
- opencv-contrib-python - Camera calibration, ORB feature extraction, BFMatcher, video capture, HSV color detection
- scikit-image - RANSAC-based fundamental matrix estimation (`FundamentalMatrixTransform`)

**3D Visualization:**
- open3d - Non-blocking 3D point cloud viewer for SLAM map (used via multiprocessing in `ross/slam/pointmap.py`)

**Object Detection (used in testing scripts):**
- ultralytics (YOLO) - Real-time object detection (referenced in `testing-m/3Dslam.py`, `testing-m/anchor-slam.py` in git history)

**Data / Science:**
- numpy - Array math, matrix operations throughout
- pandas - Data manipulation
- scikit-learn - ML utilities
- matplotlib - Plotting and animation (used in simulation notebooks with `FuncAnimation`)
- scipy - Voronoi computation (`scipy.spatial.Voronoi`) in simulation notebooks
- shapely - Geometric operations (`shapely.geometry`, `shapely.ops.voronoi_diagram`) in simulation notebooks

**Utilities:**
- loguru - Structured logging (configured in `ross/config.py` with tqdm integration)
- tqdm - Progress bars
- python-dotenv - `.env` file loading (used in `ross/config.py`)

**Note on current vs. historical dependencies:**
The current `pyproject.toml` (branch `hardware`, commit `ddbe9a3`) lists only the base dependencies (no opencv, open3d, ultralytics, scipy, shapely). Those were present during the `hardware` branch slam work but were removed in the "clean" commit. The slam `.pyc` files in `ross/slam/__pycache__/` reflect the slam-era dependency set.

## Configuration

**Environment:**
- `.env` file at project root (gitignored) — loaded via `python-dotenv` in `ross/config.py`
- No specific env vars are documented in source; file presence is required for `load_dotenv()` to succeed

**Build:**
- `pyproject.toml` — single source of truth for project metadata, dependencies, and tool config
- `uv.lock` — pinned dependency tree committed to version control

**Path constants (defined in `ross/config.py`):**
- `PROJ_ROOT` — repository root
- `DATA_DIR` / `RAW_DATA_DIR` / `INTERIM_DATA_DIR` / `PROCESSED_DATA_DIR` / `EXTERNAL_DATA_DIR`
- `MODELS_DIR`
- `REPORTS_DIR` / `FIGURES_DIR`

## Linting / Formatting

**Tool:** Ruff (replaces flake8, isort, black)

**Settings (`pyproject.toml`):**
- `line-length = 99`
- `src = ["ross"]`
- `extend-select = ["I"]` (import sorting via isort rules)
- `known-first-party = ["ross"]`
- `force-sort-within-sections = true`

## Platform Requirements

**Development:**
- Python 3.12 exactly (`requires-python = "~=3.12.0"`)
- uv installed for environment management
- Webcam access required for live CV/SLAM scripts
- Arduino IDE or compatible toolchain for BLE firmware

**Production:**
- No web server or deployment target detected — project is a research/robotics codebase
- Runs locally on hardware with attached camera

---

*Stack analysis: 2026-03-09*

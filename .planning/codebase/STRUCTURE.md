# Codebase Structure

**Analysis Date:** 2026-03-09

## Directory Layout

```
ross/                       # Project root
├── ross/                   # Python package (same name as project)
│   ├── __init__.py         # Re-exports config; triggers logging setup
│   ├── config.py           # Path constants + loguru configuration
│   └── slam/               # Monocular Visual SLAM sub-package
│       ├── __init__.py     # Public API; exports Frame, Map, Point, helpers
│       ├── display.py      # 2-D OpenCV window viewer
│       ├── extractor.py    # Feature extraction, matching, pose estimation, Frame
│       ├── main.py         # Pipeline loop + CLI entry-point
│       └── pointmap.py     # Map, Point, Open3D 3-D viewer process
├── data/                   # Video inputs (git-ignored; only test_video.mp4 present locally)
│   └── test_video.mp4
├── .planning/              # GSD planning documents (not shipped)
│   └── codebase/
├── .venv/                  # Local virtual environment (git-ignored)
├── .env                    # Environment variables (git-ignored)
├── .gitignore
├── LICENSE
├── Makefile                # Dev helper targets
├── pyproject.toml          # Project metadata, dependencies, ruff config
├── README.md
└── uv.lock                 # Locked dependency graph (committed)
```

## Directory Purposes

**`ross/` (package):**
- Purpose: All application source code; installable as the `ross` package
- Contains: Top-level bootstrap (`__init__.py`, `config.py`) plus sub-packages for each system
- Key files: `ross/__init__.py`, `ross/config.py`

**`ross/slam/`:**
- Purpose: Complete monocular Visual SLAM implementation
- Contains: Five Python modules covering every pipeline stage plus the public `__init__.py` with explicit `__all__`
- Key files: `ross/slam/main.py` (entry point), `ross/slam/extractor.py` (algorithms), `ross/slam/pointmap.py` (state)

**`data/`:**
- Purpose: Input video files for SLAM processing
- Contains: `.mp4` video files (gitignored by `/data/` rule)
- Key files: `data/test_video.mp4` (local test asset)
- Note: `config.py` defines `DATA_DIR` and sub-directories (`raw/`, `interim/`, `processed/`, `external/`) though they are not created automatically

**`.planning/codebase/`:**
- Purpose: GSD codebase analysis documents consumed by `/gsd:plan-phase` and `/gsd:execute-phase`
- Generated: Yes (by `/gsd:map-codebase`)
- Committed: Yes

## Key File Locations

**Entry Points:**
- `ross/slam/main.py`: CLI entry-point; run with `python -m ross.slam.main`

**Configuration:**
- `ross/config.py`: Project-wide path constants and logging setup
- `pyproject.toml`: Build system, dependencies, ruff lint/format settings
- `.env`: Runtime environment variables (loaded via `python-dotenv` in `config.py`)

**Core Logic:**
- `ross/slam/extractor.py`: Feature extraction (`extract`), matching (`match_frames`), pose (`extract_pose`), triangulation (`triangulate`), `Frame` class
- `ross/slam/pointmap.py`: `Map` and `Point` classes; Open3D viewer subprocess (`_viewer_process`)
- `ross/slam/display.py`: `Display` class wrapping `cv2.imshow`

**Public API:**
- `ross/slam/__init__.py`: Explicit re-exports of all public symbols; the single import surface for external users of the slam sub-package

**Dependency Lock:**
- `uv.lock`: Full dependency tree; committed for reproducibility

## Naming Conventions

**Files:**
- `snake_case.py` for all modules (e.g., `extractor.py`, `pointmap.py`, `generate_test_video.py`)
- `__init__.py` used at both package levels to define public interfaces

**Directories:**
- `snake_case` for all directories (e.g., `ross/`, `slam/`, `.planning/`)

**Classes:**
- `PascalCase`: `Frame`, `Map`, `Point`, `Display`

**Functions and variables:**
- `snake_case`: `match_frames`, `extract_pose`, `add_ones`, `process_frame`
- Module-level constants in `UPPER_SNAKE_CASE`: `W`, `H`, `F`, `K`, `Kinv`, `IRt`, `PROJ_ROOT`, `DATA_DIR`

**Private helpers:**
- Prefixed with `_` for module-internal functions: `_viewer_process`, `_make_frustum`

## Where to Add New Code

**New SLAM algorithm or pipeline stage:**
- Implementation: `ross/slam/<module_name>.py`
- Export public symbols from: `ross/slam/__init__.py` (add to `__all__`)

**New sub-system (e.g., `navigation`, `control`, `sensor_fusion`):**
- Create: `ross/<subsystem>/` directory with `__init__.py`
- Follow the slam sub-package pattern: one file per concern, explicit `__all__` in `__init__.py`

**New CLI entry-point:**
- Add a `main()` function to the relevant module with `if __name__ == "__main__"` guard
- Use `argparse` (existing pattern in `ross/slam/main.py`)
- Invoke with `python -m ross.<subsystem>.main`

**Shared utilities:**
- Cross-cutting helpers: `ross/utils.py` (file existed in earlier history; currently absent — recreate if needed)
- Visualisation helpers: extend `ross/slam/display.py` or add `ross/slam/viz.py`

**Configuration constants:**
- Project paths: add to `ross/config.py`
- Algorithm-specific constants: define as module-level `UPPER_SNAKE_CASE` in the owning module (e.g., `W`, `H`, `F` in `ross/slam/main.py`)

**Test data / input videos:**
- Place in `data/` directory at project root
- `main()` searches `data/` for known filenames as fallback; add new candidates to the `candidates` list in `ross/slam/main.py`

## Special Directories

**`.venv/`:**
- Purpose: Python virtual environment managed by `uv`
- Generated: Yes
- Committed: No (gitignored)

**`.planning/`:**
- Purpose: GSD workflow planning and codebase analysis documents
- Generated: Yes (by GSD commands)
- Committed: Yes

**`data/`:**
- Purpose: Input video and dataset files
- Generated: No (user-supplied)
- Committed: No (gitignored by `/data/` rule)

**`ross/__pycache__/` and `ross/slam/__pycache__/`:**
- Purpose: Python bytecode cache
- Generated: Yes (by Python interpreter)
- Committed: No (gitignored by `__pycache__/` rule)

---

*Structure analysis: 2026-03-09*

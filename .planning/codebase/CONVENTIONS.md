# Coding Conventions

**Analysis Date:** 2026-03-09

## Naming Patterns

**Files:**
- Modules use `snake_case.py` (e.g., `config.py`)
- Package directories use `snake_case/` with `__init__.py`

**Functions:**
- Use `snake_case` for all function and method names (PEP 8, enforced by ruff)

**Variables:**
- Use `snake_case` for local variables and module-level names
- Module-level constants use `UPPER_SNAKE_CASE` (e.g., `PROJ_ROOT`, `DATA_DIR`, `MODELS_DIR`)

**Types:**
- Use `PascalCase` for class names (PEP 8)

## Code Style

**Formatting:**
- Tool: `ruff` (configured in `pyproject.toml`)
- Line length: 99 characters
- Source root: `ross/`
- Applies to: `pyproject.toml`, `ross/**/*.py`

**Linting:**
- Tool: `ruff` with `extend-select = ["I"]` (import sorting enabled)
- Import sorting follows isort rules via ruff's `isort` integration

**Type Annotations:**
- Not observed in current source files; not enforced by tooling

## Import Organization

**Order (ruff isort):**
1. Standard library imports
2. Third-party imports
3. First-party imports (`ross` package — marked as `known-first-party`)

**Path Aliases:**
- None configured; use absolute imports from `ross` package root

**Example from `ross/config.py`:**
```python
from pathlib import Path          # stdlib

from dotenv import load_dotenv    # third-party
from loguru import logger         # third-party
```

**`noqa` suppression:**
- Use `# noqa: F401` on re-exported imports that are intentionally unused at the import site (e.g., `ross/__init__.py`)

## Error Handling

**Patterns:**
- Use bare `try/except ModuleNotFoundError` for optional dependency handling (as in `config.py`)
- Do not silence broad exceptions; catch specific exception types
- Log errors via `loguru` logger rather than `print`

**Example from `ross/config.py`:**
```python
try:
    from tqdm import tqdm
    logger.remove(0)
    logger.add(lambda msg: tqdm.write(msg, end=""), colorize=True)
except ModuleNotFoundError:
    pass
```

## Logging

**Framework:** `loguru` (imported as `logger` from `loguru`)

**Patterns:**
- Import: `from loguru import logger`
- Use `logger.info(...)` for informational startup messages (e.g., path confirmation)
- Configure tqdm integration in `ross/config.py` when tqdm is available
- Do not use `print()` for runtime output — use `logger` methods

**Example:**
```python
logger.info(f"PROJ_ROOT path is: {PROJ_ROOT}")
```

## Comments

**When to Comment:**
- Explain non-obvious configuration choices (e.g., the loguru/tqdm integration comment links to the upstream issue)
- Group related constants with a section comment (e.g., `# Paths`)

**Docstrings:**
- Not present in current source files; no enforced standard yet
- Recommend NumPy-style or Google-style docstrings for public functions

## Function Design

**Size:** Not established; current codebase has no functions
**Parameters:** Not established
**Return Values:** Not established

## Module Design

**Exports:**
- `ross/__init__.py` imports `config` to ensure side effects (path setup, logger config) run on package import
- Use `# noqa: F401` on re-export lines to suppress "imported but unused" warnings

**Barrel Files:**
- `ross/__init__.py` serves as a minimal entry point that triggers config initialization

## Environment & Paths

**Pattern from `ross/config.py`:**
- Resolve project root via `Path(__file__).resolve().parents[N]`
- Define all data/model/report directory paths as module-level `Path` constants
- Load `.env` at module import time via `python-dotenv`

```python
PROJ_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = PROJ_ROOT / "data"
RAW_DATA_DIR = DATA_DIR / "raw"
INTERIM_DATA_DIR = DATA_DIR / "interim"
PROCESSED_DATA_DIR = DATA_DIR / "processed"
EXTERNAL_DATA_DIR = DATA_DIR / "external"
MODELS_DIR = PROJ_ROOT / "models"
REPORTS_DIR = PROJ_ROOT / "reports"
FIGURES_DIR = REPORTS_DIR / "figures"
```

---

*Convention analysis: 2026-03-09*

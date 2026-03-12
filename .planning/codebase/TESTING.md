# Testing Patterns

**Analysis Date:** 2026-03-09

## Test Framework

**Runner:**
- Not configured. No `pytest`, `unittest`, or other test runner is present in `pyproject.toml` dependencies or as a config file.
- `pytest` is the standard choice for Python 3.12 projects and should be added.

**Assertion Library:**
- Not applicable (no tests exist)

**Run Commands:**
```bash
# Recommended once pytest is added:
uv run pytest                  # Run all tests
uv run pytest --tb=short       # Concise tracebacks
uv run pytest --cov=ross       # Coverage (requires pytest-cov)
```

## Test File Organization

**Location:**
- No test files exist in the repository (only `data/test_video.mp4`, which is a binary fixture)
- Recommended: co-locate a `tests/` directory at the project root

**Naming:**
- Use `test_<module>.py` convention (e.g., `tests/test_config.py`)
- Use `test_<behavior>` for function names

**Recommended Structure:**
```
ross/
tests/
├── __init__.py
├── test_config.py
└── ...
```

## Test Structure

**Suite Organization:**
- No suites exist yet
- Recommended pattern using pytest:

```python
import pytest
from ross import config


def test_proj_root_is_directory():
    assert config.PROJ_ROOT.is_dir()


def test_data_dir_path_resolves_under_proj_root():
    assert config.DATA_DIR == config.PROJ_ROOT / "data"
```

**Patterns:**
- Setup: use `pytest` fixtures (`@pytest.fixture`) for shared state
- Teardown: use fixture `yield` pattern for cleanup
- Assertions: use plain `assert` statements (pytest rewrites them for readable output)

## Mocking

**Framework:** `unittest.mock` (stdlib) or `pytest-mock` (recommended addition)

**Patterns:**
- No mocking patterns established yet
- Recommended pattern for testing loguru output or dotenv loading:

```python
from unittest.mock import patch

def test_load_dotenv_called(monkeypatch):
    with patch("ross.config.load_dotenv") as mock_load:
        import importlib
        import ross.config
        importlib.reload(ross.config)
        mock_load.assert_called_once()
```

**What to Mock:**
- File system operations when testing path logic in isolation
- External service calls (none currently exist)
- Environment variables: use `monkeypatch.setenv` (pytest built-in)

**What NOT to Mock:**
- `pathlib.Path` resolution — test against real filesystem layout
- `loguru` logger in most cases — it's a side effect, not a return value

## Fixtures and Factories

**Test Data:**
- `data/test_video.mp4` exists as a binary video fixture for SLAM/CV testing
- Access via the `config.DATA_DIR` constant:

```python
import pytest
from ross.config import DATA_DIR

@pytest.fixture
def test_video_path():
    path = DATA_DIR / "test_video.mp4"
    assert path.exists(), f"Test video not found: {path}"
    return path
```

**Location:**
- Binary fixtures: `data/` (gitignored)
- Python fixtures: `tests/conftest.py` (to be created)

## Coverage

**Requirements:** None enforced

**Recommended setup in `pyproject.toml`:**
```toml
[tool.pytest.ini_options]
testpaths = ["tests"]

[tool.coverage.run]
source = ["ross"]

[tool.coverage.report]
show_missing = true
```

**View Coverage:**
```bash
uv run pytest --cov=ross --cov-report=term-missing
```

## Test Types

**Unit Tests:**
- Scope: individual functions and module-level constants in `ross/`
- Approach: test path resolution, config values, logger setup

**Integration Tests:**
- Scope: not applicable yet; will be relevant when SLAM/swarm modules are added
- Approach: use real filesystem, real video data from `data/`

**E2E Tests:**
- Not applicable at current project stage

## Common Patterns

**Async Testing:**
- Not applicable (no async code present)

**Error Testing:**
```python
import pytest

def test_missing_optional_dependency_does_not_raise():
    # config.py uses try/except ModuleNotFoundError for tqdm — verify graceful handling
    import ross.config  # should not raise
```

**Environment Variable Testing:**
```python
def test_env_var_loaded(monkeypatch):
    monkeypatch.setenv("SOME_VAR", "value")
    # re-import or call function that reads env var
```

---

*Testing analysis: 2026-03-09*

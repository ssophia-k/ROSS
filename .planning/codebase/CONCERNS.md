# Codebase Concerns

**Analysis Date:** 2026-03-09

---

## Tech Debt

**SLAM source code missing from `hardware` branch (main working branch):**
- Issue: The `ross/slam/` subpackage exists only as compiled `.pyc` bytecode files in `ross/slam/__pycache__/`. No `.py` source files are present in the working tree or tracked by git on the `hardware` branch.
- Files: `ross/slam/__pycache__/extractor.cpython-312.pyc`, `ross/slam/__pycache__/pointmap.cpython-312.pyc`, `ross/slam/__pycache__/display.cpython-312.pyc`, `ross/slam/__pycache__/generate_test_video.cpython-312.pyc`
- Impact: The SLAM module cannot be modified, reviewed, or executed from source on the active branch. Source only exists on the `LearnOpenCV` branch. The `hardware` branch (labeled the current sprint branch) is missing the core vision component.
- Fix approach: Cherry-pick or merge `ross/slam/` source files from the `LearnOpenCV` branch into `hardware`. Track `ross/slam/*.py` in git.

**SLAM dependencies absent from `hardware` branch `pyproject.toml`:**
- Issue: The `hardware` branch `pyproject.toml` lists no vision dependencies (`opencv-contrib-python`, `open3d`, `scikit-image` are all absent). These appear only in the `LearnOpenCV` branch `pyproject.toml`. The `uv.lock` on `hardware` also has no entries for these packages.
- Files: `pyproject.toml`, `uv.lock`
- Impact: Running any SLAM code on the `hardware` branch will fail at import time with `ModuleNotFoundError` for `cv2`, `open3d`, and `skimage`. The installed environment is incomplete for the stated project goal.
- Fix approach: Add `opencv-contrib-python`, `open3d`, and `scikit-image` back to `pyproject.toml` dependencies and re-lock with `uv lock`.

**`generate_test_video` source removed but `.pyc` remains:**
- Issue: Commit `2658dfb` ("rm generate test video; use actual benchmark test videos") deleted the source for `ross/slam/generate_test_video.py`, but the compiled `.pyc` file is still present on disk in `ross/slam/__pycache__/`.
- Files: `ross/slam/__pycache__/generate_test_video.cpython-312.pyc`
- Impact: Stale bytecode clutters the repository and misleads contributors into thinking a working generator is available. There is no replacement benchmark video path tracked in the repo.
- Fix approach: Delete the stale `.pyc` file. Ensure `data/` test videos are documented (they are gitignored).

**Duplicate `config.py` across package renames:**
- Issue: The repo has been renamed from `src/` → `ross/` across multiple refactor commits. On the `morsalin` branch, both `src/config.py` and `ross/config.py` exist with identical contents. The `hardware` branch only has `ross/config.py`, but the rename history leaves confusion about the canonical location.
- Files: `ross/config.py`, `ross/__init__.py`
- Impact: Low on `hardware` branch directly, but future merges from `morsalin` risk re-introducing the stale `src/` package.
- Fix approach: Confirm `src/` package remnants are fully removed across all branches before merging.

**`parts.csv` is untracked:**
- Issue: `parts.csv` (hardware bill of materials for Phase 3) is present in the working directory but not tracked by git.
- Files: `parts.csv`
- Impact: Hardware design decisions (component choices, suppliers, order status) are not version-controlled and could be lost or diverge from the codebase state.
- Fix approach: Either add `parts.csv` to git tracking or document where the BOM is maintained (e.g., a shared document link in `README.md`).

**`data/` directory is entirely gitignored:**
- Issue: `.gitignore` excludes `/data/`. The sole test video `data/test_video.mp4` (14 MB) is tracked in the working directory but not in git.
- Files: `data/test_video.mp4`
- Impact: Test videos cannot be reproduced by new contributors cloning the repo. The SLAM pipeline will fail unless the user separately provides a video file.
- Fix approach: Document in `README.md` how to obtain test videos, or use `git lfs` to track binary test assets.

---

## Known Bugs

**`match_frames` uses bare `assert` for control flow:**
- Symptoms: If fewer than 8 feature matches are found between frames, or if RANSAC fails, an `AssertionError` is raised inside `match_frames`. This is caught with `except (AssertionError, Exception)` in `process_frame`, which silently drops the frame.
- Files: `ross/slam/extractor.py` (lines 211, 227 in LearnOpenCV branch)
- Trigger: Any low-texture scene, fast camera motion, or motion blur can reduce matches below 8.
- Workaround: The broad `except` in `main.py::process_frame` prevents crashes, but the pipeline produces no map points for the skipped frame.

**`Point.pt` stores raw homogeneous coordinates without guaranteed normalization:**
- Symptoms: `Map.display()` calls `p.pt[:3]` directly, but `Point.__init__` stores `loc` as-is (could be `[X, Y, Z, W]` with W ≠ 1). For points where `good_pts4d` filtering passes but `W` is small, the 3D positions sent to the viewer can be at incorrect distances.
- Files: `ross/slam/pointmap.py` (`Point.__init__`), `ross/slam/main.py` (`process_frame`)
- Trigger: Triangulated points with non-unit W that slip through the `abs(orig_w) > 0.005` filter.
- Workaround: None; points are silently displayed at wrong positions.

**`global F, K, Kinv` mutation in `main.py`:**
- Symptoms: The `main()` function mutates module-level globals `F`, `K`, `Kinv` when `--focal` is passed. This is flagged with `# noqa: PLW0603` (suppressed ruff warning). If `main()` is called more than once in a process (e.g., in tests or scripts), the globals retain the previous call's values.
- Files: `ross/slam/main.py` (line 161)
- Trigger: Re-calling `main()` with different focal lengths in one Python session.
- Workaround: None; the `noqa` comment suppresses the lint warning without fixing the underlying issue.

---

## Security Considerations

**`.env` file present in working directory:**
- Risk: `.env` is listed in `.gitignore` and should not be committed. However, it is present on disk. If it contains API keys or credentials (e.g., for cloud services or IoT backends), accidental `git add .` could expose secrets.
- Files: `.env` (existence confirmed; contents not read)
- Current mitigation: `.gitignore` rule prevents git tracking.
- Recommendations: Verify `.env` is never staged. Consider using `git-secrets` or a pre-commit hook to block accidental commits of secret-bearing files.

**No input validation on video path CLI argument:**
- Risk: `main.py` passes `video_path` directly to `cv2.VideoCapture()`. A path with special characters or a symlink to a sensitive file could cause unexpected behavior.
- Files: `ross/slam/main.py`
- Current mitigation: OpenCV handles the path internally; failure to open returns a log error and exits.
- Recommendations: Low severity for a research tool, but add a `Path(video_path).exists()` check before calling `VideoCapture`.

---

## Performance Bottlenecks

**Per-point SVD triangulation loop (O(N) SVD operations):**
- Problem: `triangulate()` in `extractor.py` runs a full 4×4 SVD for every matched point pair in a `for` loop. With up to 3000 corners per frame, this is up to 3000 individual `np.linalg.svd` calls per frame.
- Files: `ross/slam/extractor.py` (`triangulate` function)
- Cause: Linear SVD (DLT) implemented point-by-point instead of as a batched operation. OpenCV's `cv2.triangulatePoints()` performs this in optimized C++ and would be significantly faster.
- Improvement path: Replace the Python loop with `cv2.triangulatePoints(proj1[:3], proj2[:3], pts1.T, pts2.T)` for a vectorized solution.

**Unbounded `Map.points` growth:**
- Problem: Every triangulated point that passes the quality filter is appended to `mapp.points` and never pruned. On long videos the list grows without bound, causing increasing memory usage and slower `Map.display()` calls.
- Files: `ross/slam/pointmap.py` (`Map` class), `ross/slam/main.py` (`process_frame`)
- Cause: No local/global map management; all points are kept for the lifetime of the run.
- Improvement path: Implement a sliding window or keyframe-based point culling strategy.

**Open3D viewer receives full point cloud every frame:**
- Problem: `Map.display()` serializes the entire `mapp.points` array and sends it through a `multiprocessing.Queue` on every call. For large maps this creates large IPC payloads and can cause the queue to back up.
- Files: `ross/slam/pointmap.py` (`Map.display`)
- Cause: No incremental update mechanism; full state is pushed every frame.
- Improvement path: Send only newly added points as deltas, or throttle display updates to every N frames.

---

## Fragile Areas

**SLAM pipeline is tightly coupled to module-level globals in `main.py`:**
- Files: `ross/slam/main.py`
- Why fragile: `W`, `H`, `F`, `K`, `Kinv` are declared at module scope. `Frame.__init__` (in `extractor.py`) takes `K` as a parameter, so the extractor is properly parameterized, but `main.py`'s `process_frame` closes over the module-level `K`. Any refactoring that moves `process_frame` to another module or calls it from outside `main.py` silently uses stale globals.
- Safe modification: Pass `K`, `W`, `H` explicitly as function arguments rather than reading from module scope.
- Test coverage: No tests exist for this module.

**`_viewer_process` drain loop has no timeout or sentinel:**
- Files: `ross/slam/pointmap.py` (`_viewer_process`)
- Why fragile: The daemon process loops forever calling `q.get()` when Open3D is absent. If the main process exits ungracefully (e.g., keyboard interrupt before `daemon=True` cleanup), the background process may not terminate promptly on some platforms.
- Safe modification: Replace the infinite drain loop with a sentinel value (e.g., `None`) that causes the viewer process to exit cleanly.
- Test coverage: None.

**`match_frames` distance threshold `0.1` is hardcoded in normalized coordinate space:**
- Files: `ross/slam/extractor.py` (`match_frames`)
- Why fragile: The `np.linalg.norm(p1 - p2) < 0.1` filter is a magic constant that is appropriate for a specific focal length and resolution. Changing the camera intrinsics (e.g., using a fisheye lens or different resolution) without adjusting this threshold will silently over- or under-reject matches.
- Safe modification: Make the threshold a configurable parameter or derive it from the intrinsic matrix.
- Test coverage: None.

---

## Scaling Limits

**Single-camera monocular SLAM (scale ambiguity):**
- Current capacity: The SLAM implementation is purely monocular with no IMU fusion or stereo baseline. Recovered translations are up-to-scale (relative only).
- Limit: Cannot provide metric-scale localization needed for robot navigation without external scale reference (e.g., known object size, IMU, wheel odometry).
- Scaling path: Integrate IMU data (LSM6DS3TR-C is listed in `parts.csv`) via sensor fusion for metric scale recovery, or use ArUco markers of known size as scale references.

**No loop closure detection:**
- Current capacity: Map points accumulate without any revisit detection. Pose drift compounds over time with no correction.
- Limit: Accuracy degrades unboundedly over long trajectories. Unsuitable for returning to a previously visited area.
- Scaling path: Implement a bag-of-words or DBoW-style loop closure detector, or adopt ORB-SLAM3 as the backend.

---

## Dependencies at Risk

**`open3d` is not in `hardware` branch `pyproject.toml`:**
- Risk: `open3d` was removed during the "clean" commits but is required by `ross/slam/pointmap.py`. The package has irregular release cadence and no Python 3.12 wheel was available on some platforms as of late 2025.
- Impact: `Map.create_viewer()` and `Map.display()` will silently fail (the headless stub drains the queue) without warning if `open3d` is not installed.
- Migration plan: Re-add to dependencies. The headless fallback in `_viewer_process` provides graceful degradation but should log a warning rather than fail silently.

**`scikit-image` used only for `FundamentalMatrixTransform`:**
- Risk: `scikit-image` is a large dependency (~100MB) used exclusively for one class in RANSAC estimation. It was removed from `hardware` branch `pyproject.toml`.
- Impact: `extractor.py` will fail to import with `ImportError` on the `hardware` branch environment.
- Migration plan: Either restore the dependency or replace `FundamentalMatrixTransform` + `skimage.measure.ransac` with `cv2.findFundamentalMat(..., method=cv2.FM_RANSAC)` to eliminate the dependency.

---

## Missing Critical Features

**No automated tests of any kind:**
- Problem: There are no test files, no test configuration (`pytest.ini`, `pytest` in dependencies), and no CI pipeline. The `morsalin` branch `testing-m/` directory contains exploratory scripts, not automated tests.
- Blocks: Cannot safely refactor SLAM components, validate hardware integration changes, or run regression checks.

**No hardware integration code on `hardware` branch:**
- Problem: The `hardware` branch is named for hardware integration but contains only the same boilerplate files as `main` (just `ross/config.py` and `ross/__init__.py`). No motor driver, IMU, BLE, or ESP32-CAM interface code exists.
- Blocks: Actual robot deployment. The `parts.csv` BOM references components (DRV8833, LSM6DS3TR-C, ESP32-CAM) with no corresponding firmware or driver code.

**No camera calibration pipeline integrated into package:**
- Problem: `checkerboard_calibration.py` was moved to `notebooks/` and then removed in the "clean" commit on `hardware`. Camera intrinsic matrix values in SLAM files are hardcoded defaults (`F = 270`, `FOCAL_LENGTH = 718.8560`) not derived from actual hardware calibration.
- Blocks: Accurate SLAM on physical robot hardware. The IMU-camera extrinsic calibration is also absent.

---

## Test Coverage Gaps

**Entire SLAM pipeline is untested:**
- What's not tested: Feature extraction (`extract`), pose estimation (`match_frames`, `extract_pose`), triangulation (`triangulate`), map management (`Map`, `Point`), viewer process communication.
- Files: `ross/slam/extractor.py`, `ross/slam/pointmap.py`, `ross/slam/display.py`, `ross/slam/main.py` (all on `LearnOpenCV` branch only)
- Risk: Any change to SLAM math or feature matching could silently degrade tracking without detection.
- Priority: High

**`config.py` path logic is untested:**
- What's not tested: `PROJ_ROOT` resolution, loguru/tqdm integration at import time.
- Files: `ross/config.py`
- Risk: Low — path computation is straightforward, but `logger.info` at import time means every `import ross` emits a log line, which is unexpected behavior in library code.
- Priority: Low

---

*Concerns audit: 2026-03-09*

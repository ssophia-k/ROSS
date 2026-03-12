# Architecture

**Analysis Date:** 2026-03-09

## Pattern Overview

**Overall:** Single-package Python library with a pipeline-oriented SLAM (Simultaneous Localization and Mapping) module.

**Key Characteristics:**
- Modular pipeline: each stage (extraction, matching, pose estimation, triangulation, display, mapping) is a separate concern in its own file
- Stateful objects (`Frame`, `Map`, `Point`) accumulate and link data across the pipeline lifetime
- Visualisation decoupled from computation: 3-D viewer runs in a separate OS process via `multiprocessing`, communicating over a `Queue`
- No web server, no REST API, no database — purely an offline video-processing and robotics research tool

## Layers

**Configuration / Bootstrap:**
- Purpose: Resolve project root, define data directory constants, configure logging
- Location: `ross/config.py`
- Contains: Path constants (`PROJ_ROOT`, `DATA_DIR`, `MODELS_DIR`, etc.), loguru setup with tqdm integration
- Depends on: `python-dotenv`, `loguru`, `tqdm`
- Used by: Everything inside the `ross` package via `from ross import config`

**Feature Front-End:**
- Purpose: Convert raw video frames into matched keypoint pairs and relative camera pose
- Location: `ross/slam/extractor.py`
- Contains: `Frame` class, `extract()`, `normalize()`, `denormalize()`, `add_ones()`, `match_frames()`, `extract_pose()`, `triangulate()`
- Depends on: `cv2` (ORB + BFMatcher), `numpy`, `skimage.measure.ransac`, `skimage.transform.FundamentalMatrixTransform`
- Used by: `ross/slam/main.py`

**Global Map:**
- Purpose: Hold all camera poses and triangulated 3-D points; own the viewer IPC channel
- Location: `ross/slam/pointmap.py`
- Contains: `Map` class, `Point` class, `_viewer_process()` function, `_make_frustum()` helper
- Depends on: `numpy`, `multiprocessing`, optionally `open3d`
- Used by: `ross/slam/main.py`; the viewer process is spawned internally by `Map.create_viewer()`

**2-D Display:**
- Purpose: Render annotated frames in an OpenCV window during processing
- Location: `ross/slam/display.py`
- Contains: `Display` class (`paint()`, `__init__()`)
- Depends on: `cv2` (highgui)
- Used by: `ross/slam/main.py`

**Pipeline Orchestration (Entry Point):**
- Purpose: Drive the per-frame SLAM loop; wire together all layers
- Location: `ross/slam/main.py`
- Contains: `process_frame()`, `main()`, CLI argument parsing (`argparse`)
- Depends on: All three layers above plus `cv2.VideoCapture`
- Used by: Direct execution (`python -m ross.slam.main`) or import

## Data Flow

**Per-Frame SLAM Pipeline:**

1. `cv2.VideoCapture.read()` yields a raw BGR frame in `main()` (`ross/slam/main.py`)
2. `process_frame()` resizes the frame to `(W, H)` and constructs a `Frame` object (`ross/slam/extractor.py:Frame.__init__`), which runs `extract()` internally — detecting Shi-Tomasi corners and computing ORB descriptors — then normalises keypoint pixel coordinates to camera coordinates using `K_inv`; the `Frame` self-registers into `mapp.frames`
3. `match_frames(f1, f2)` runs BFMatcher + Lowe's ratio test + a normalised-coordinate distance filter, then RANSAC-fits a `FundamentalMatrixTransform`; `extract_pose()` decomposes it via SVD into a 4×4 `[R|t]` matrix
4. The new frame's pose is accumulated: `f1.pose = Rt @ f2.pose`
5. `triangulate()` uses the DLT linear SVD method to recover homogeneous 3-D points from the two-view correspondences
6. Depth and parallax filters discard degenerate points; surviving points become `Point` objects (`ross/slam/pointmap.py:Point`) that record which frames observed them
7. Feature tracks are drawn onto the frame image; `Display.paint()` calls `cv2.imshow`
8. `Map.display()` serialises all current poses and 3-D point positions and puts them on the IPC `Queue` for the background Open3D viewer process

**3-D Viewer IPC:**

1. `Map.create_viewer()` spawns `_viewer_process` in a separate process (spawn context) with a shared `multiprocessing.Queue`
2. The main process pushes `(poses_array, pts_array)` tuples via `Map.display()` after each frame
3. `_viewer_process` drains the queue, updates `open3d.geometry.PointCloud` and `LineSet` geometries, and redraws the window via `vis.poll_events()` / `vis.update_renderer()`

**State Management:**
- `Map` holds the canonical lists `frames: list[Frame]` and `points: list[Point]`
- `Frame` objects are mutable (`.pose` is updated in-place during processing)
- `Point` objects grow their `.frames` and `.idxs` lists as new observations are added
- No external state persistence; all data lives in memory for the duration of one run

## Key Abstractions

**Frame:**
- Purpose: Encapsulates one camera view — its intrinsics, pose, keypoints, and descriptors
- Examples: `ross/slam/extractor.py` (`class Frame`)
- Pattern: Registers itself into `Map.frames` on construction; pose starts as identity and is updated by the pipeline

**Map:**
- Purpose: Global scene representation — camera trajectory + sparse 3-D point cloud
- Examples: `ross/slam/pointmap.py` (`class Map`)
- Pattern: Singleton-style object created once in `main()` and threaded through `process_frame()`; owns the viewer subprocess

**Point:**
- Purpose: A single triangulated world point with multi-frame observation records
- Examples: `ross/slam/pointmap.py` (`class Point`)
- Pattern: Self-registers into `Map.points` on construction; accumulates observations via `add_observation()`

## Entry Points

**CLI / Module Execution:**
- Location: `ross/slam/main.py` (`main()` function, `if __name__ == "__main__"` block)
- Triggers: `python -m ross.slam.main [video] [--focal F] [--width W] [--height H]`
- Responsibilities: Parse CLI args, resolve video path from `data/` fallback candidates, instantiate `Display` and `Map`, run the frame loop, release resources

## Error Handling

**Strategy:** Fail-fast with logged warnings; degenerate frames are skipped, not fatal.

**Patterns:**
- `match_frames()` raises `AssertionError` if fewer than 8 matches are found; caught in `process_frame()` with `logger.warning` and a graceful skip (display-only for that frame)
- Open3D import failure in `_viewer_process` is silently handled — the process drains the queue forever, making the viewer optional
- Video open failure and missing video file both call `sys.exit(1)` after `logger.error`
- Divide-by-zero in triangulation prevented by `np.abs(orig_w) > 1e-6` guard before normalisation

## Cross-Cutting Concerns

**Logging:** `loguru` throughout; configured in `ross/config.py` to use `tqdm.write` so log output does not corrupt progress bars. Import `from ross.slam import ...` triggers config load via `ross/__init__.py`.

**Validation:** Inline assertions and numpy boolean masks (no schema validation library). Camera intrinsics are hardcoded constants in `main.py` with CLI override support.

**Authentication:** Not applicable — no network calls, no auth layer.

---

*Architecture analysis: 2026-03-09*

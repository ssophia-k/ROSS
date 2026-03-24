"""
Simple Monocular Visual-Inertial SLAM (RUN THIS SCRIPT)

Usage
-----
    python -m ross.slam.main <data_dir> [--focal <F>] [--width <W>] [--height <H>]

Expected data_dir layout (Sensor Logger export, unzipped):
    data_dir/
        Frames/               ← folder of images from Sensor Logger
            frame_0.jpg       ← (any image extension)
            frame_1.jpg
            ...
        frames.csv            ← columns: timestamp_iso, seconds_elapsed, path
        Gyroscope.csv         ← Sensor Logger gyro export
        Accelerometer.csv     ← Sensor Logger accel export

Pipeline (per frame)
--------------------
1. Load image + timestamp from the Sensor Logger frames folder.
2. Extract ORB features, create a Frame with its timestamp.
3. Pre-integrate IMU samples between the last two frame timestamps gives us Delta R, p
4. Use ΔR as the initial pose prior for the new frame.
5. Attempt visual feature matching + Fundamental-matrix pose refinement.
6. If vision succeeds, use visual pose (IMU prior helped initialise it).
   If vision fails, keep IMU-only pose estimate (graceful fallback).
7. Triangulate 3-D points, add to map, update displays.
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
from loguru import logger

from ross.slam.display import Display
from ross.slam.extractor import Frame, denormalize, match_frames, triangulate
from ross.slam.imu import IMUData
from ross.slam.pointmap import Map, Point


# CAMERA INTRINSICS FROM CALIBRATION

W = 1920 // 2   # processing width  (960)
H = 1080 // 2   # processing height (540)
F = 270         # focal length in pixels (override with --focal)

K = np.array([[F, 0, W // 2], [0, F, H // 2], [0, 0, 1]], dtype=float)
Kinv = np.linalg.inv(K)


# Sensor Logger data loader

def load_image_sequence(data_dir: Path) -> list[tuple[float, Path]]:
    """Return a time-sorted list of (seconds_elapsed, image_path) tuples.

    Sensor Logger writes a ``frames.csv`` alongside the image folder.
    Columns (lowercased): ``seconds_elapsed``, and a path/filename column.

    Falls back to lexicographic sort by filename if no CSV is found.
    """
    csv_candidates = list(data_dir.glob("frames.csv")) + list(data_dir.glob("Frames.csv"))

    # ── with CSV ────────────────────────────────────────────────────────────
    if csv_candidates:
        import pandas as pd
        df = pd.read_csv(csv_candidates[0])
        df.columns = [c.strip().lower() for c in df.columns]

        if "seconds_elapsed" not in df.columns:
            raise ValueError(f"frames.csv missing 'seconds_elapsed' column. Got: {list(df.columns)}")

        # Find the column that contains the image filenames / paths
        path_col = next(
            (c for c in df.columns if c not in {"seconds_elapsed", "time", "timestamp"}),
            None,
        )
        if path_col is None:
            raise ValueError("Cannot identify image path column in frames.csv")

        rows = []
        frames_dir = data_dir / "Frames"
        for _, row in df.iterrows():
            img_name = Path(row[path_col]).name
            # Search the Frames sub-folder first, then data_dir itself
            for search_dir in [frames_dir, data_dir]:
                img_path = search_dir / img_name
                if img_path.exists():
                    rows.append((float(row["seconds_elapsed"]), img_path))
                    break

        if rows:
            return sorted(rows, key=lambda x: x[0])
        logger.warning("frames.csv found but no matching images located – falling back to glob")

    # ── without CSV (lexicographic fallback) ────────────────────────────────
    extensions = ("*.jpg", "*.jpeg", "*.png")
    all_imgs: list[Path] = []
    for ext in extensions:
        all_imgs.extend((data_dir / "Frames").glob(ext))
        all_imgs.extend(data_dir.glob(ext))

    if not all_imgs:
        raise FileNotFoundError(f"No images found in {data_dir}")

    all_imgs = sorted(set(all_imgs))
    logger.warning("No frames.csv found – using uniform 20 fps timestamps")
    return [(i / 20.0, p) for i, p in enumerate(all_imgs)]


# Processing per frame

def process_frame(
    img: np.ndarray,
    timestamp: float,
    mapp: Map,
    display: Display,
    imu: IMUData | None,
) -> None:
    """Process one image frame through the visual-inertial SLAM pipeline."""
    img = cv2.resize(img, (W, H))
    frame = Frame(mapp, img, K, timestamp=timestamp)

    if frame.id == 0:
        display.paint(img)
        return

    f1 = mapp.frames[-1]   # current
    f2 = mapp.frames[-2]   # previous

    # 1. IMU pre-integration gives pose prior
    imu_pose_used = False

    if imu is not None and f2.timestamp is not None and f1.timestamp is not None:
        delta = imu.preintegrate(f2.timestamp, f1.timestamp)

        if delta.n_samples > 0:
            # Build a 4×4 incremental pose from IMU ΔR and Δp
            imu_Rt = np.eye(4)
            imu_Rt[:3, :3] = delta.dR
            imu_Rt[:3,  3] = delta.dp          # small in loosely-coupled mode

            # Initialise f1's pose with the IMU prior
            f1.pose = imu_Rt @ f2.pose
            imu_pose_used = True


    # 2. Visual feature matching + pose refinement
    visual_ok = False
    try:
        idx1, idx2, Rt = match_frames(f1, f2)

        if imu_pose_used:
            # IMU rotation is more reliable than fundamental matrix for pure rotation.
            # Use IMU rotation, but take translation from vision.
            visual_pose = Rt @ f2.pose
            imu_pose    = imu_Rt @ f2.pose

            f1.pose = visual_pose.copy()
            f1.pose[:3, :3] = imu_pose[:3, :3]   # ← swap in IMU rotation
        else:
            f1.pose = Rt @ f2.pose

        visual_ok = True
    except (AssertionError, Exception) as exc:
        if imu_pose_used:
            logger.warning(
                f"Frame {frame.id}: visual matching failed ({exc}) – "
                "keeping IMU-only pose"
            )
        else:
            logger.warning(f"Frame {frame.id}: match_frames failed – {exc}")
            display.paint(img)
            return

    # 3. Triangulation (only when vision succeeded)
    if visual_ok:
        pts4d = triangulate(f1.pose, f2.pose, f1.pts[idx1], f2.pts[idx2])

        orig_w   = pts4d[:, 3].copy()
        good_w   = np.abs(orig_w) > 1e-6
        pts4d[good_w] /= pts4d[good_w, 3:4]
        good_pts = good_w & (np.abs(orig_w) > 0.005) & (pts4d[:, 2] > 0)

        for i, p in enumerate(pts4d):
            if not good_pts[i]:
                continue
            pt = Point(mapp, p)
            pt.add_observation(f1, idx1[i])
            pt.add_observation(f2, idx2[i])

        # Draw feature tracks
        for pt1, pt2 in zip(f1.pts[idx1], f2.pts[idx2]):
            u1, v1 = denormalize(K, pt1)
            u2, v2 = denormalize(K, pt2)
            cv2.circle(img, (u1, v1), color=(0, 255, 0), radius=3)
            cv2.line(img, (u1, v1), (u2, v2), color=(255, 0, 0), thickness=1)

    # 4. HUD overlay
    src_label = "V+I" if (visual_ok and imu_pose_used) else ("V" if visual_ok else "IMU")
    cv2.putText(
        img,
        f"Frame {frame.id}  |  Map pts: {len(mapp.points)}  |  Pose: {src_label}",
        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
    )

    display.paint(img)
    mapp.display()


# Main pipeline

def main(data_dir: str | None = None, focal: float | None = None) -> None:
    """Run the VI-SLAM pipeline on a Sensor Logger dataset directory."""
    global F, K, Kinv  # noqa: PLW0603

    if focal is not None:
        F = focal
        K    = np.array([[F, 0, W // 2], [0, F, H // 2], [0, 0, 1]], dtype=float)
        Kinv = np.linalg.inv(K)

    # Resolve data directory
    if data_dir is None:
        proj_root = Path(__file__).resolve().parents[2]
        data_dir  = str(proj_root / "data" / "sensor_logger")
    data_path = Path(data_dir)

    if not data_path.exists():
        logger.error(f"Data directory not found: {data_path}")
        sys.exit(1)

    # Load image sequence
    logger.info(f"Loading image sequence from: {data_path}")
    sequence = load_image_sequence(data_path)
    logger.info(f"Found {len(sequence)} frames spanning "
                f"{sequence[-1][0] - sequence[0][0]:.1f} s")

    # Load IMU (optional – pipeline degrades gracefully without it)
    imu: IMUData | None = None
    gyro_path  = data_path / "Gyroscope.csv"
    accel_path = data_path / "Accelerometer.csv"

    if gyro_path.exists() and accel_path.exists():
        try:
            imu = IMUData(gyro_path, accel_path)
            logger.info("IMU data loaded – running visual-inertial mode")
        except Exception as exc:
            logger.warning(f"Could not load IMU data: {exc} – running visual-only")
    else:
        logger.warning("Gyroscope.csv / Accelerometer.csv not found – running visual-only")

    display = Display(W, H)
    mapp    = Map()
    mapp.create_viewer()

    logger.info("SLAM started – press 'q' to quit")

    for ts, img_path in sequence:
        img = cv2.imread(str(img_path))
        if img is None:
            logger.warning(f"Could not read image: {img_path}")
            continue
        process_frame(img, ts, mapp, display, imu)

    cv2.destroyAllWindows()
    logger.info(
        f"Done – {len(mapp.frames)} frames, {len(mapp.points)} map points."
    )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="ROSS – Visual-Inertial SLAM")
    parser.add_argument("data_dir", nargs="?", default=None,
                        help="Path to Sensor Logger export directory")
    parser.add_argument("--focal",  type=float, default=None, help="Focal length in pixels")
    parser.add_argument("--width",  type=int,   default=None)
    parser.add_argument("--height", type=int,   default=None)
    args = parser.parse_args()

    if args.width:  W = args.width
    if args.height: H = args.height

    main(data_dir=args.data_dir, focal=args.focal)
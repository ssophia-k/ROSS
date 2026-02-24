#!/usr/bin/env python3
"""
Simple Monocular Visual SLAM – main entry-point.

Usage
-----
    python -m ross.slam.main <video_path> [--focal <F>] [--width <W>] [--height <H>]

Pipeline (per frame)
--------------------
1. Resize the incoming frame to (W, H).
2. Extract ORB features and create a Frame object.
3. Match features between the current and previous frame.
4. Estimate the relative camera pose via the Fundamental matrix + SVD.
5. Triangulate 3-D points from the two-view correspondences.
6. Filter out points with insufficient parallax or behind the camera.
7. Add surviving points to the global map.
8. Draw feature tracks on the 2-D display and update the 3-D viewer.

Based on https://learnopencv.com/monocular-slam-in-python/
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
from loguru import logger

from ross.slam.display import Display
from ross.slam.extractor import Frame, denormalize, match_frames, triangulate
from ross.slam.pointmap import Map, Point


# ---------------------------------------------------------------------------
# Default camera intrinsics (override via CLI args or the constants below)
# ---------------------------------------------------------------------------

# Display / processing resolution
W = 1920 // 2  # 960
H = 1080 // 2  # 540

# Focal length (pixels) – a reasonable guess for dashcam footage
F = 270

# Build the 3×3 intrinsic matrix  K  and its inverse
K = np.array([
    [F, 0, W // 2],
    [0, F, H // 2],
    [0, 0, 1],
], dtype=float)
Kinv = np.linalg.inv(K)


def process_frame(img: np.ndarray, mapp: Map, display: Display) -> None:
    """Process a single video frame through the SLAM pipeline.

    Parameters
    ----------
    img : np.ndarray
        Raw BGR frame from the video.
    mapp : Map
        Global SLAM map (frames + points).
    display : Display
        2-D visualisation window.
    """
    img = cv2.resize(img, (W, H))

    # Create a new Frame (automatically registers with the map)
    frame = Frame(mapp, img, K)

    # Need at least two frames to do anything
    if frame.id == 0:
        display.paint(img)
        return

    # Current and previous frame
    f1 = mapp.frames[-1]
    f2 = mapp.frames[-2]

    # ------------------------------------------------------------------
    # Feature matching + relative-pose estimation
    # ------------------------------------------------------------------
    try:
        idx1, idx2, Rt = match_frames(f1, f2)
    except (AssertionError, Exception) as exc:
        logger.warning(f"Frame {frame.id}: match_frames failed – {exc}")
        display.paint(img)
        return

    # Accumulate the pose:  X_f1 = Rt · X_f2
    f1.pose = Rt @ f2.pose

    # ------------------------------------------------------------------
    # Triangulation  →  3-D map points (homogeneous)
    # ------------------------------------------------------------------
    pts4d = triangulate(f1.pose, f2.pose, f1.pts[idx1], f2.pts[idx2])

    # Check W *before* normalising so we can use it as a parallax filter.
    # W≈0 means the rays are nearly parallel (degenerate triangulation).
    orig_w = pts4d[:, 3].copy()
    good_w = np.abs(orig_w) > 1e-6

    # Normalise only well-conditioned points to avoid divide-by-zero.
    pts4d[good_w] /= pts4d[good_w, 3:4]

    # Reject degenerate (small |W|), low-parallax, or behind-camera points
    good_pts4d = good_w & (np.abs(orig_w) > 0.005) & (pts4d[:, 2] > 0)

    for i, p in enumerate(pts4d):
        if not good_pts4d[i]:
            continue
        pt = Point(mapp, p)
        pt.add_observation(f1, idx1[i])
        pt.add_observation(f2, idx2[i])

    # ------------------------------------------------------------------
    # Draw feature tracks on the 2-D image
    # ------------------------------------------------------------------
    for pt1, pt2 in zip(f1.pts[idx1], f2.pts[idx2]):
        u1, v1 = denormalize(K, pt1)
        u2, v2 = denormalize(K, pt2)
        cv2.circle(img, (u1, v1), color=(0, 255, 0), radius=3)
        cv2.line(img, (u1, v1), (u2, v2), color=(255, 0, 0), thickness=1)

    # Show stats overlay
    cv2.putText(
        img,
        f"Frame {frame.id}  |  Map pts: {len(mapp.points)}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )

    # ------------------------------------------------------------------
    # Update displays
    # ------------------------------------------------------------------
    display.paint(img)
    mapp.display()


# ---------------------------------------------------------------------------
# CLI entry-point
# ---------------------------------------------------------------------------

def main(video_path: str | None = None, focal: float | None = None) -> None:
    """Run the SLAM pipeline on a video file.

    Parameters
    ----------
    video_path : str | None
        Path to the input video.  If *None*, looks for
        ``data/test_countryroad.mp4`` relative to project root.
    focal : float | None
        Override focal length.
    """
    global F, K, Kinv  # noqa: PLW0603

    if focal is not None:
        F = focal
        K = np.array([[F, 0, W // 2], [0, F, H // 2], [0, 0, 1]], dtype=float)
        Kinv = np.linalg.inv(K)

    # Resolve video path
    if video_path is None:
        proj_root = Path(__file__).resolve().parents[2]
        candidates = [
            proj_root / "data" / "test_countryroad.mp4",
            proj_root / "data" / "car.mp4",
            proj_root / "data" / "video.mp4",
        ]
        for c in candidates:
            if c.exists():
                video_path = str(c)
                break
        if video_path is None:
            logger.error(
                "No video file found.  Place a video in data/ or pass the path as argument.\n"
                f"  Tried: {[str(c) for c in candidates]}"
            )
            sys.exit(1)

    logger.info(f"Opening video: {video_path}")
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        logger.error(f"Cannot open video: {video_path}")
        sys.exit(1)

    display = Display(W, H)
    mapp = Map()
    mapp.create_viewer()

    logger.info("SLAM started – press 'q' in the display window to quit.")

    frame_count = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        process_frame(frame, mapp, display)
        frame_count += 1

    cap.release()
    cv2.destroyAllWindows()
    logger.info(
        f"Done – processed {frame_count} frames, "
        f"{len(mapp.points)} map points, "
        f"{len(mapp.frames)} keyframes."
    )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="ROSS – Monocular Visual SLAM")
    parser.add_argument("video", nargs="?", default=None, help="Path to input video")
    parser.add_argument("--focal", type=float, default=None, help="Focal length in pixels")
    parser.add_argument("--width", type=int, default=None, help="Processing width")
    parser.add_argument("--height", type=int, default=None, help="Processing height")

    args = parser.parse_args()

    if args.width:
        W = args.width
    if args.height:
        H = args.height

    main(video_path=args.video, focal=args.focal)

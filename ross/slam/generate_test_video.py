#!/usr/bin/env python3
"""Generate a synthetic test video with a moving camera for SLAM testing.

Creates a video of a virtual 3-D scene viewed from a camera following a
curved trajectory. The scene contains randomly placed textured squares that
give the feature extractor something to track.

Usage
-----
    python -m ross.slam.generate_test_video [--output data/test_video.mp4]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def generate_test_video(
    output_path: str = "data/test_video.mp4",
    n_frames: int = 600,
    width: int = 960,
    height: int = 540,
    fps: int = 30,
) -> None:
    """Generate a synthetic dashcam-style video for SLAM testing.

    The scene consists of random 3-D points projected onto each frame as small
    filled circles / random textures, with the virtual camera moving forward
    along a gentle curve.
    """
    out_path = Path(output_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_path), fourcc, fps, (width, height))

    rng = np.random.default_rng(42)

    # Generate random 3-D points in the world
    n_points = 2000
    # Points scattered in a wide corridor ahead of the camera
    world_pts = np.column_stack([
        rng.uniform(-30, 30, n_points),   # X
        rng.uniform(-10, 10, n_points),   # Y
        rng.uniform(5, 200, n_points),    # Z (ahead of camera)
    ])

    # Give each point a random colour
    colours = rng.integers(50, 255, size=(n_points, 3)).tolist()

    # Camera intrinsics
    fx = fy = 270.0
    cx, cy = width / 2, height / 2
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    for i in range(n_frames):
        t = i / n_frames

        # Camera moves forward with slight lateral oscillation
        tx = 5.0 * np.sin(2 * np.pi * t * 2)
        ty = 0.5 * np.sin(2 * np.pi * t * 3)
        tz = t * 150.0  # move forward

        # Small yaw rotation following the lateral motion
        yaw = 0.05 * np.sin(2 * np.pi * t * 2)
        R = np.array([
            [np.cos(yaw), 0, np.sin(yaw)],
            [0, 1, 0],
            [-np.sin(yaw), 0, np.cos(yaw)],
        ])
        tvec = np.array([tx, ty, tz])

        # Project world points into this camera
        pts_cam = (R @ (world_pts - tvec).T).T  # (N, 3)

        frame = np.zeros((height, width, 3), dtype=np.uint8)

        # Background gradient (road-like)
        for row in range(height):
            val = int(80 + 40 * (row / height))
            frame[row, :] = [val, val + 10, val + 5]

        for j in range(n_points):
            z = pts_cam[j, 2]
            if z < 1.0:
                continue
            u = int(fx * pts_cam[j, 0] / z + cx)
            v = int(fy * pts_cam[j, 1] / z + cy)
            if 0 <= u < width and 0 <= v < height:
                radius = max(1, int(6.0 / (z * 0.05)))
                cv2.circle(frame, (u, v), radius, colours[j], -1)

        # Add some noise for realism
        noise = rng.integers(0, 8, frame.shape, dtype=np.uint8)
        frame = cv2.add(frame, noise)

        writer.write(frame)

    writer.release()
    print(f"Wrote {n_frames} frames to {out_path}  ({out_path.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate synthetic SLAM test video")
    parser.add_argument("-o", "--output", default="data/test_video.mp4")
    parser.add_argument("-n", "--frames", type=int, default=600)
    args = parser.parse_args()

    generate_test_video(output_path=args.output, n_frames=args.frames)

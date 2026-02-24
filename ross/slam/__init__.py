"""
Monocular Visual SLAM implementation.

Based on the LearnOpenCV tutorial:
https://learnopencv.com/monocular-slam-in-python/

This package implements a simple monocular Visual SLAM pipeline consisting of:
  - Feature extraction (ORB + goodFeaturesToTrack)
  - Feature matching (BFMatcher + Lowe's ratio test)
  - Pose estimation (Fundamental matrix via RANSAC → Essential matrix → R, t)
  - Triangulation (linear SVD method)
  - Local mapping (accumulation of 3D points)
  - 2-D display (OpenCV imshow)
  - 3-D visualization (Open3D)

Files:
  extractor.py  – Feature extraction, matching, pose estimation, Frame class
  display.py    – 2-D image display via OpenCV
  pointmap.py   – 3-D map + Point class, Open3D viewer
  main.py       – Main entry-point / video processing loop
"""

from ross.slam.extractor import Frame, add_ones, denormalize, match_frames, normalize, triangulate
from ross.slam.pointmap import Map, Point

__all__ = [
    "Frame",
    "Map",
    "Point",
    "add_ones",
    "denormalize",
    "match_frames",
    "normalize",
    "triangulate",
]

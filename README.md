# ROSS 

## Monocular Visual SLAM

A simple monocular Visual SLAM implementation in Python, based on the
[LearnOpenCV tutorial](https://learnopencv.com/monocular-slam-in-python/).

### Pipeline

| Stage | Description |
|---|---|
| **Feature Extraction** | ORB descriptors on Shi-Tomasi corners (`goodFeaturesToTrack`) |
| **Feature Matching** | BFMatcher + Lowe's ratio test + distance filter |
| **Pose Estimation** | Fundamental matrix via RANSAC → SVD → R, t |
| **Triangulation** | Linear DLT (SVD) for 3-D point estimation |
| **Local Mapping** | Accumulate good 3-D points into a global map |
| **2-D Display** | OpenCV `imshow` with feature tracks overlay |
| **3-D Viewer** | Open3D visualiser (camera trajectory + point cloud) |

### Quick Start

```bash
# Install dependencies
pip install -e .

# Generate a synthetic test video (optional)
python -m ross.slam.generate_test_video -o data/test_video.mp4

# Run SLAM on a video
python -m ross.slam.main data/test_video.mp4

# Or use your own dashcam video with a custom focal length
python -m ross.slam.main path/to/video.mp4 --focal 500
```

### File Structure

```
ross/slam/
├── __init__.py               # Package exports
├── extractor.py              # Feature extraction, matching, pose estimation, Frame class
├── display.py                # 2-D display (OpenCV imshow)
├── pointmap.py               # Map + Point classes, Open3D 3-D viewer
├── main.py                   # Main entry-point / video processing loop
└── generate_test_video.py    # Synthetic test video generator
```

### Controls

- Press **q** in the 2-D window to quit
- The 3-D Open3D window supports mouse rotation/zoom/pan

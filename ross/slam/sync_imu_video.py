"""
sync_imu_video.py  –  Align iOS Camera video frames with Sensor Logger IMU data.

Usage
-----
    python sync_imu_video.py <recording_dir> [--fps 20] [--offset <seconds>]

What it does
------------
1. Reads the video file's creation timestamp via ffprobe (absolute UTC).
2. Reads the IMU Gyroscope.csv 'time' column (Unix nanoseconds).
3. Computes the offset between the two clocks.
4. Extracts frames from the video at the target fps using ffmpeg.
5. Writes frames.csv with absolute Unix seconds for each frame,
   aligned to the same time reference as the IMU CSVs.

Requirements
------------
    pip install pandas
    brew install ffmpeg        # or: conda install -c conda-forge ffmpeg

Expected input layout
---------------------
    recording/
        video.MOV             (or .mp4, .mov – any ffmpeg-readable format)
        Gyroscope.csv         (Sensor Logger export)
        Accelerometer.csv     (Sensor Logger export)

Output
------
    recording/
        Frames/               (extracted jpg frames)
        frames.csv            (seconds_elapsed, filename – aligned to IMU)
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import pandas as pd



SUPPORTED_EXTENSIONS = (".mov", ".mp4", ".MOV", ".MP4", ".m4v")


def find_video(directory: Path) -> Path:
    for ext in SUPPORTED_EXTENSIONS:
        matches = list(directory.glob(f"*{ext}"))
        if matches:
            return matches[0]
    raise FileNotFoundError(
        f"No video file found in {directory}. "
        f"Supported formats: {SUPPORTED_EXTENSIONS}"
    )


def get_video_creation_time(video_path: Path) -> float:
    """Return the video's creation time as a Unix timestamp (seconds).

    Uses ffprobe to read the 'creation_time' metadata tag, which iOS Camera
    writes as a UTC ISO-8601 string.
    """
    cmd = [
        "ffprobe", "-v", "quiet",
        "-print_format", "json",
        "-show_entries", "format_tags=creation_time",
        str(video_path),
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
    except FileNotFoundError:
        raise RuntimeError(
            "ffprobe not found. Install ffmpeg: brew install ffmpeg"
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"ffprobe failed: {e.stderr}")

    data = json.loads(result.stdout)
    try:
        ts_str = data["format"]["tags"]["creation_time"]
    except KeyError:
        raise RuntimeError(
            "No creation_time tag found in video metadata.\n"
            "Try passing --offset manually (see --help)."
        )

    # Parse ISO-8601 UTC string, e.g. "2026-03-24T02:35:00.000000Z"
    ts_str = ts_str.replace("Z", "+00:00")
    dt = datetime.fromisoformat(ts_str)
    return dt.timestamp()


def get_imu_start_time(gyro_path: Path) -> float:
    """Return the first IMU sample time as Unix seconds.

    Sensor Logger stores the 'time' column as Unix nanoseconds.
    """
    df = pd.read_csv(gyro_path, nrows=5)
    df.columns = [c.strip().lower() for c in df.columns]

    if "time" not in df.columns:
        raise ValueError(
            f"'time' column not found in {gyro_path.name}. "
            f"Got: {list(df.columns)}"
        )

    return float(df["time"].iloc[0]) / 1e9


def extract_frames(video_path: Path, output_dir: Path, fps: int) -> int:
    """Extract frames from video at *fps* using ffmpeg. Returns frame count."""
    output_dir.mkdir(parents=True, exist_ok=True)

    pattern = str(output_dir / "%06d.jpg")
    cmd = [
        "ffmpeg", "-y",
        "-i", str(video_path),
        "-vf", f"fps={fps}",
        "-q:v", "2",          # high quality JPEG (1=best, 31=worst)
        pattern,
    ]
    print(f"  Extracting frames at {fps}fps → {output_dir} ...")
    try:
        subprocess.run(cmd, check=True, capture_output=True)
    except FileNotFoundError:
        raise RuntimeError("ffmpeg not found. Install: brew install ffmpeg")
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"ffmpeg failed:\n{e.stderr.decode()}")

    frames = sorted(output_dir.glob("*.jpg"))
    return len(frames)


def build_frames_csv(
    output_dir: Path,
    fps: int,
    video_start_unix: float,
    imu_start_unix: float,
    out_csv: Path,
) -> None:
    """Write frames.csv with one row per frame.

    The 'seconds_elapsed' value for each frame matches the IMU time reference:
        t_frame = (video_start - imu_start) + frame_index / fps
    """
    frames = sorted(output_dir.glob("*.jpg"))
    if not frames:
        raise RuntimeError(f"No frames found in {output_dir}")

    # Offset: how many seconds after IMU t=0 does the video start?
    clock_offset = video_start_unix - imu_start_unix
    print(f"  Clock offset (video - IMU): {clock_offset:.3f} s")

    rows = []
    for i, f in enumerate(frames):
        t = clock_offset + i / fps
        rows.append({"seconds_elapsed": t, "filename": f.name})

    df = pd.DataFrame(rows)
    df.to_csv(out_csv, index=False)
    print(f"  Wrote {len(rows)} rows to {out_csv}")


# Main pipeline

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Sync iOS video frames with Sensor Logger IMU data."
    )
    parser.add_argument(
        "recording_dir",
        help="Path to directory containing video.MOV + Gyroscope.csv",
    )
    parser.add_argument(
        "--fps", type=int, default=20,
        help="Frame extraction rate (default: 20)",
    )
    parser.add_argument(
        "--offset", type=float, default=None,
        help=(
            "Manual clock offset in seconds (video_start - imu_start). "
            "Use if ffprobe cannot read creation_time from the video."
        ),
    )
    args = parser.parse_args()

    rec_dir = Path(args.recording_dir).resolve()
    if not rec_dir.exists():
        print(f"ERROR: directory not found: {rec_dir}")
        sys.exit(1)

    gyro_path  = rec_dir / "Gyroscope.csv"
    accel_path = rec_dir / "Accelerometer.csv"

    for p in [gyro_path, accel_path]:
        if not p.exists():
            print(f"ERROR: missing {p.name} in {rec_dir}")
            sys.exit(1)

    print(f"\n── Sync: {rec_dir.name} ──")

    # 1. Find video
    video_path = find_video(rec_dir)
    print(f"  Video:  {video_path.name}")

    # 2. Get timestamps
    imu_start = get_imu_start_time(gyro_path)
    print(f"  IMU start (Unix s):   {imu_start:.3f}")

    if args.offset is not None:
        clock_offset = args.offset
        video_start  = imu_start + clock_offset
        print(f"  Using manual offset: {clock_offset:.3f} s")
    else:
        video_start = get_video_creation_time(video_path)
        clock_offset = video_start - imu_start
        print(f"  Video start (Unix s): {video_start:.3f}")

    # 3. Extract frames
    frames_dir = rec_dir / "Frames"
    n = extract_frames(video_path, frames_dir, args.fps)
    print(f"  Extracted {n} frames")

    # 4. Write frames.csv
    csv_path = rec_dir / "frames.csv"
    build_frames_csv(frames_dir, args.fps, video_start, imu_start, csv_path)

    print(f"\n✓ Done. Run SLAM with:")
    print(f"    python -m ross.slam.main {rec_dir} --focal 600\n")


if __name__ == "__main__":
    main()
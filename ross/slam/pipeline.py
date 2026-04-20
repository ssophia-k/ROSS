"""Room scan orchestration: stream → MiDaS → detect → register → save."""

from pathlib import Path
import sys
import time

import cv2

from ross.net.discovery import discover_host
from ross.net.robot import fetch_imu, stream_url
from ross.slam.io import save_scan
from ross.slam.reconstruct import (
    TOTAL_YAW_DEGREES,
    build_floor,
    build_scene,
    view_scene,
)
from ross.slam.vision import estimate_depth, load_midas, load_yolo

MAX_FRAMES = 20


def capture_frames(host: str, model, transform, device, use_imu: bool = True):
    """Capture frames from the ESP32-CAM MJPEG stream. SPACE=capture, ENTER=build."""
    url = stream_url(host)
    print(f"[Stream] Connecting to {url} ...")

    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print("[Stream] Failed to connect. Is ROSS running?")
        sys.exit(1)

    print("[Stream] Connected.")

    frames = []
    imu_readings = []
    step = TOTAL_YAW_DEGREES / MAX_FRAMES

    print(f"\n{'=' * 60}")
    print(f"  ROSS Room Scan  ({MAX_FRAMES} frames x {step:.0f} deg each)")
    print("  Rotate the robot between captures.")
    print("  SPACE=capture   ENTER=build   Q=quit")
    print(f"{'=' * 60}\n")

    win = "ROSS Scan  |  SPACE=capture  ENTER=build  Q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 800, 520)

    last_depth_vis = None

    while True:
        ok, frame = cap.read()
        if not ok:
            cap.release()
            cap = cv2.VideoCapture(url)
            continue

        h, w = frame.shape[:2]
        disp = frame.copy()

        next_angle = len(frames) * step
        cv2.rectangle(disp, (0, 0), (w, 56), (10, 13, 20), -1)
        cv2.putText(
            disp,
            f"Frame {len(frames)}/{MAX_FRAMES}   aim at {next_angle:.0f} deg",
            (12, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (60, 210, 150),
            2,
        )
        cv2.putText(
            disp,
            "SPACE=capture  ENTER=build  Q=quit",
            (12, h - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (150, 150, 150),
            1,
        )

        if last_depth_vis is not None:
            sw = w // 3
            disp[56:, w - sw :] = cv2.resize(last_depth_vis, (sw, h - 56))

        cv2.imshow(win, disp)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(" "):
            print(
                f"[{len(frames) + 1}/{MAX_FRAMES}] {next_angle:.0f} deg — MiDaS ...",
                end=" ",
                flush=True,
            )
            t0 = time.time()
            depth = estimate_depth(frame, model, transform, device)
            print(f"{time.time() - t0:.1f}s")

            frames.append((frame.copy(), depth))

            if use_imu:
                imu = fetch_imu(host)
                if imu:
                    imu_readings.append(imu)
                    print(f"    [IMU] accel={imu['accel']}  gyro={imu['gyro']}")
                else:
                    imu_readings.append(None)
                    if len(frames) == 1:
                        print("    [IMU] Not available — using yaw sweep only")

            last_depth_vis = cv2.applyColorMap(
                (depth * 255).astype("uint8"), cv2.COLORMAP_MAGMA
            )

            if len(frames) >= MAX_FRAMES:
                print("[Capture] Max frames reached.")
                break

        elif key in (13, 10):
            if len(frames) < 2:
                print("[!] Need at least 2 frames.")
            else:
                break
        elif key == ord("q"):
            cap.release()
            cv2.destroyAllWindows()
            return [], []

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n[Capture] {len(frames)} frames ready.\n")

    if use_imu and imu_readings and any(r is not None for r in imu_readings):
        return frames, imu_readings
    return frames, None


def run(
    output: Path,
    host: str | None = None,
    model: str = "DPT_Hybrid",
    use_imu: bool = True,
    use_yolo: bool = True,
    yolo_weights: str | None = None,
) -> None:
    """Entry point for the scanner. Called by `ross slam`."""
    print(f"\n{'=' * 60}")
    print("  ROSS Room Scanner")
    print(f"{'=' * 60}")

    if not host:
        host = discover_host()
        if not host:
            print("[!] Could not find ROSS. Use --host to specify.")
            sys.exit(1)

    midas_model, midas_transform, device = load_midas(model)
    yolo_model = load_yolo(yolo_weights) if use_yolo else None

    frames, imu_readings = capture_frames(
        host, midas_model, midas_transform, device, use_imu=use_imu
    )
    if not frames:
        return

    pcd, mesh, people = build_scene(frames, yolo_model, imu_readings)

    mesh = mesh + build_floor()
    mesh.compute_vertex_normals()

    mesh_path, cloud_path = save_scan(output, mesh, pcd)
    print(f"\n[Saved] {mesh_path}\n        {cloud_path}")

    view_scene(mesh, pcd, people, mesh_path=str(mesh_path))

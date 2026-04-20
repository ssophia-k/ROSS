"""
MiDaS 360° → 3D Environment Builder
=====================================
Capture a 360° pan with your MacBook webcam, then explore the scene
as a navigable 3D point cloud using MiDaS depth estimation.

Dependencies (install once):
    pip install torch torchvision timm
    pip install opencv-python
    pip install open3d
    pip install numpy scipy

Usage:
    python midas_360_3d.py

Controls during capture:
    SPACE  → capture a frame
    ENTER  → finish capture & build 3D scene
    Q      → quit without building

Controls in the 3D viewer (Open3D window):
    Left-drag   → rotate
    Right-drag  → pan
    Scroll      → zoom
    Q / Ctrl+C  → close
"""

import cv2
import torch
import numpy as np
import open3d as o3d
from scipy.ndimage import median_filter
import time
import os

# ─────────────────────────────────────────────
#  1.  MiDaS setup
# ─────────────────────────────────────────────

def load_midas(model_type="DPT_Hybrid"):
    """
    Load MiDaS from torch hub.
    model_type options (accuracy vs speed):
        "DPT_Large"   – most accurate, slowest  (~700 MB)
        "DPT_Hybrid"  – good balance            (~470 MB)  ← default
        "MiDaS_small" – fastest, least accurate (~80 MB)
    """
    print(f"[MiDaS] Loading model: {model_type}  (first run downloads weights ~470 MB) …")
    model = torch.hub.load("intel-isl/MiDaS", model_type, trust_repo=True)
    transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)

    if model_type in ("DPT_Large", "DPT_Hybrid"):
        transform = transforms.dpt_transform
    else:
        transform = transforms.small_transform

    device = torch.device("mps") if torch.backends.mps.is_available() else \
             torch.device("cuda") if torch.cuda.is_available() else \
             torch.device("cpu")

    print(f"[MiDaS] Using device: {device}")
    model.to(device).eval()
    return model, transform, device


def estimate_depth(frame_bgr, model, transform, device):
    """Run MiDaS on a single BGR frame, return normalised depth (0-1, far=0, near=1)."""
    img_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    input_batch = transform(img_rgb).to(device)

    with torch.no_grad():
        prediction = model(input_batch)
        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=img_rgb.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()

    depth = prediction.cpu().numpy()
    # MiDaS outputs *inverse* depth (higher = closer).
    # Normalise to [0, 1] so near=1, far=0.
    d_min, d_max = depth.min(), depth.max()
    if d_max - d_min > 1e-6:
        depth = (depth - d_min) / (d_max - d_min)
    return depth.astype(np.float32)


# ─────────────────────────────────────────────
#  2.  Camera intrinsics (estimated for webcam)
# ─────────────────────────────────────────────

def get_intrinsics(width, height):
    """
    Estimate pinhole camera intrinsics for a typical MacBook webcam.
    Focal length ≈ 1.0× the image diagonal (a reasonable heuristic).
    Refine with a proper calibration board for best results.
    """
    focal = max(width, height) * 0.9   # rough estimate
    cx, cy = width / 2.0, height / 2.0
    return focal, cx, cy


# ─────────────────────────────────────────────
#  3.  Depth map → point cloud
# ─────────────────────────────────────────────

def depth_to_pointcloud(frame_bgr, depth_map, focal, cx, cy,
                         rotation_deg=0.0,
                         max_points=80_000,
                         depth_scale=5.0,
                         smooth=True):
    """
    Back-project pixels into 3D using the estimated depth map.

    Args:
        frame_bgr   : original captured frame
        depth_map   : normalised depth (0-1)
        focal       : estimated focal length in pixels
        cx, cy      : principal point
        rotation_deg: yaw angle of this frame in the 360° sweep (degrees)
        max_points  : subsample to this many points for performance
        depth_scale : stretches the depth axis so the scene has depth "feel"
        smooth      : apply light smoothing to reduce noise
    Returns:
        open3d PointCloud
    """
    h, w = depth_map.shape

    if smooth:
        depth_map = median_filter(depth_map, size=3)

    # Create pixel grid
    u = np.arange(w)
    v = np.arange(h)
    uu, vv = np.meshgrid(u, v)

    # Convert normalised depth to a metric-like "distance"
    # MiDaS gives inverse depth, so use 1/(depth+ε) → real distance proxy
    epsilon = 0.05
    Z = depth_scale / (depth_map + epsilon)   # larger Z = farther

    # Back-project
    X = (uu - cx) * Z / focal
    Y = (vv - cy) * Z / focal

    # Flatten
    pts = np.stack([X, Y, Z], axis=-1).reshape(-1, 3)
    colors_bgr = frame_bgr.reshape(-1, 3)
    colors_rgb = colors_bgr[:, ::-1] / 255.0   # RGB, float [0,1]

    # Rotate the point cloud around Y-axis by the sweep angle so frames
    # fan out into a panoramic arc instead of all overlapping.
    theta = np.radians(rotation_deg)
    Ry = np.array([
        [ np.cos(theta), 0, np.sin(theta)],
        [             0, 1,             0],
        [-np.sin(theta), 0, np.cos(theta)],
    ])
    pts = (Ry @ pts.T).T

    # Subsample
    if len(pts) > max_points:
        idx = np.random.choice(len(pts), max_points, replace=False)
        pts = pts[idx]
        colors_rgb = colors_rgb[idx]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colors_rgb.astype(np.float64))
    return pcd


# ─────────────────────────────────────────────
#  4.  Capture loop
# ─────────────────────────────────────────────

def capture_frames(model, transform, device):
    """
    Open webcam, let user press SPACE to capture frames while panning,
    ENTER to finish, Q to quit.
    Returns list of (frame_bgr, depth_map) tuples.
    """
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam. Check camera permissions in "
                           "System Settings → Privacy → Camera.")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    frames = []
    print("\n" + "="*60)
    print("  CAPTURE MODE")
    print("  • Slowly pan your MacBook through 360°")
    print("  • Press  SPACE  to capture each frame (~6-12 frames total)")
    print("  • Press  ENTER  when done")
    print("  • Press  Q      to quit")
    print("="*60 + "\n")

    window_name = "MiDaS 360° Capture  |  SPACE=capture  ENTER=done  Q=quit"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 800, 500)

    last_depth_vis = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Dropped frame from webcam.")
            continue

        # Mirror for natural selfie-cam feel
        frame = cv2.flip(frame, 1)

        # Overlay UI
        display = frame.copy()
        h, w = display.shape[:2]

        # Status bar
        cv2.rectangle(display, (0, 0), (w, 55), (30, 30, 30), -1)
        count_text = f"Captured: {len(frames)} frame(s)"
        cv2.putText(display, count_text, (12, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 230, 100), 2)

        hint = "SPACE=capture   ENTER=build 3D   Q=quit"
        cv2.putText(display, hint, (12, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Show last depth side-by-side if available
        if last_depth_vis is not None:
            side_w = w // 3
            depth_small = cv2.resize(last_depth_vis, (side_w, h - 55))
            display[55:, w - side_w:] = depth_small
            cv2.putText(display, "last depth", (w - side_w + 4, h - 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        cv2.imshow(window_name, display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):
            print(f"[Capture] Running MiDaS on frame {len(frames)+1} …", end=" ", flush=True)
            t0 = time.time()
            depth = estimate_depth(frame, model, transform, device)
            print(f"done in {time.time()-t0:.1f}s")
            frames.append((frame.copy(), depth))

            # Build colourised depth preview
            depth_uint8 = (depth * 255).astype(np.uint8)
            last_depth_vis = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_MAGMA)

        elif key in (13, ord('\r')):   # ENTER
            if len(frames) < 2:
                print("[!] Capture at least 2 frames before pressing ENTER.")
            else:
                break

        elif key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            return []

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n[Capture] {len(frames)} frames captured. Building 3D scene …")
    return frames


# ─────────────────────────────────────────────
#  5.  Build merged point cloud
# ─────────────────────────────────────────────

def build_scene(frames):
    """
    Merge all frames into one point cloud, evenly spacing them across 360°.
    """
    if not frames:
        return None

    n = len(frames)
    h, w = frames[0][0].shape[:2]
    focal, cx, cy = get_intrinsics(w, h)

    combined = o3d.geometry.PointCloud()
    angle_step = 360.0 / n

    print(f"[Build] Projecting {n} frames, each rotated {angle_step:.1f}° apart …")

    for i, (frame, depth) in enumerate(frames):
        angle = i * angle_step
        print(f"  Frame {i+1}/{n}  →  yaw {angle:.0f}°")
        pcd = depth_to_pointcloud(frame, depth, focal, cx, cy,
                                   rotation_deg=angle,
                                   max_points=60_000)
        combined += pcd

    # Light voxel downsampling to speed up rendering
    print("[Build] Downsampling merged cloud …")
    combined = combined.voxel_down_sample(voxel_size=0.02)

    # Remove statistical outliers (reduces MiDaS noise artefacts)
    print("[Build] Removing outliers …")
    combined, _ = combined.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    print(f"[Build] Final cloud: {len(combined.points):,} points")
    return combined


# ─────────────────────────────────────────────
#  6.  Interactive 3D viewer
# ─────────────────────────────────────────────

def view_scene(pcd):
    """
    Open an interactive Open3D window to explore the point cloud.
    Controls:
        Left-drag   rotate   |  Right-drag   pan
        Scroll      zoom     |  Q            close
        Ctrl+9      toggle background colour
    """
    print("\n[Viewer] Opening 3D scene …")
    print("  Controls:  Left-drag=rotate  Right-drag=pan  Scroll=zoom  Q=close\n")

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="MiDaS 360° Scene", width=1280, height=800)

    vis.add_geometry(pcd)

    # Render settings
    opt = vis.get_render_option()
    opt.background_color = np.array([0.08, 0.08, 0.12])   # dark background
    opt.point_size = 1.8
    opt.show_coordinate_frame = True

    # Set a nice initial viewpoint (slightly elevated, looking inward)
    ctr = vis.get_view_control()
    ctr.set_zoom(0.6)
    ctr.set_front([0, -0.3, -1])
    ctr.set_lookat([0, 0, 3])
    ctr.set_up([0, -1, 0])

    vis.run()
    vis.destroy_window()


# ─────────────────────────────────────────────
#  7.  Optional: save / load point cloud
# ─────────────────────────────────────────────

def save_pointcloud(pcd, path="scene.ply"):
    o3d.io.write_point_cloud(path, pcd)
    print(f"[Save] Point cloud saved to: {os.path.abspath(path)}")


def load_and_view(path="scene.ply"):
    """Reload a previously saved scene without recapturing."""
    pcd = o3d.io.read_point_cloud(path)
    view_scene(pcd)


# ─────────────────────────────────────────────
#  8.  Main entry point
# ─────────────────────────────────────────────

def main():
    # ── Choose model ──────────────────────────────────────────────────────────
    # Uncomment the line you want:
    MODEL_TYPE = "DPT_Hybrid"      # ← best balance for MacBook (recommended)
    # MODEL_TYPE = "DPT_Large"     # ← most accurate but ~2× slower
    # MODEL_TYPE = "MiDaS_small"   # ← real-time on CPU, less detail
    # ─────────────────────────────────────────────────────────────────────────

    print("\n" + "═"*60)
    print("  MiDaS 360° → 3D Environment Builder")
    print("═"*60)

    # Load model
    model, transform, device = load_midas(MODEL_TYPE)

    # Capture frames
    frames = capture_frames(model, transform, device)
    if not frames:
        print("[Exit] No frames captured.")
        return

    # Build 3D scene
    pcd = build_scene(frames)
    if pcd is None or len(pcd.points) == 0:
        print("[Error] Point cloud is empty.")
        return

    # Optionally save
    save_pointcloud(pcd, "my_scene.ply")

    # View
    view_scene(pcd)

    print("\n[Done] Scene saved as my_scene.ply — reload any time with load_and_view()")


if __name__ == "__main__":
    main()


# ─────────────────────────────────────────────
#  TIPS & EXTENSIONS
# ─────────────────────────────────────────────
#
#  1. BETTER QUALITY
#     • Use a tripod or steady surface; rotate the laptop in ~30° steps
#     • Capture 10-12 frames for a full 360° instead of 6
#     • Switch to MODEL_TYPE = "DPT_Large" if you can afford the wait
#
#  2. MESH RECONSTRUCTION
#     After building `pcd`, estimate normals and run Poisson reconstruction:
#
#       pcd.estimate_normals()
#       mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
#       o3d.visualization.draw_geometries([mesh])
#
#  3. RELOAD A SAVED SCENE (no camera needed):
#       from midas_360_3d import load_and_view
#       load_and_view("my_scene.ply")
#
#  4. REAL-TIME DEPTH VIDEO (no 3D, just live depth map):
#     See the `realtime_depth_preview()` helper below.
#
#  5. CAMERA CALIBRATION for metric accuracy:
#     Use cv2.calibrateCamera() with a chess-board printout.
#     Replace the `get_intrinsics()` estimates with your calibrated values.


def realtime_depth_preview():
    """
    Bonus: live MiDaS depth feed from your webcam.
    Press Q to quit.
    """
    model, transform, device = load_midas("MiDaS_small")  # use fast model for RT
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    print("[Live] Press Q to quit")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.flip(frame, 1)
        depth = estimate_depth(frame, model, transform, device)
        depth_vis = cv2.applyColorMap((depth * 255).astype(np.uint8), cv2.COLORMAP_MAGMA)
        both = np.hstack([cv2.resize(frame, (320, 240)),
                          cv2.resize(depth_vis, (320, 240))])
        cv2.imshow("Live  |  RGB (left)   Depth (right)   Q=quit", both)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
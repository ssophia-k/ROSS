"""
MiDaS Room Scanner — 360° + IMU-Ready
========================================


──────────────────────────────────────────────────────────────
  HOW TO PLUG IN REAL iPHONE IMU LATER
──────────────────────────────────────────────────────────────
  1. Record video on iPhone while logging ARKit poses.
     Use an app like "Motion Logger" or a custom Swift app
     that saves CMDeviceMotion quaternions to a JSON file.

  2. Export JSON alongside your video frames, e.g.:
       imu_data = [
         {"x": 0.01, "y": 0.12, "z": 0.02, "w": 0.99},
         ...one entry per captured frame...
       ]

  3. Pass it into main():
       pcd, mesh = build_scene(frames, imu_data=imu_data)

  4. That is it. get_frame_transform() handles the rest.
     quaternion_to_4x4() is already implemented below.
──────────────────────────────────────────────────────────────
"""

import cv2
import torch
import numpy as np
import open3d as o3d
from scipy.ndimage import median_filter
import time, os

# ═══════════════════════════════════════════════════════════════
#  SETTINGS
# ═══════════════════════════════════════════════════════════════

MODEL_TYPE           = "DPT_Hybrid"   # "MiDaS_small" for speed

MAX_FRAMES           = 30
TOTAL_YAW_DEGREES    = 360.0   # change to 180 if you only pan halfway

# Point cloud
MAX_PTS_PER_FRAME    = 100_000
DEPTH_SCALE          = 4.0
DEPTH_SMOOTH_K       = 5

# ICP (refines the initial yaw guess)
ICP_MAX_DIST         = 0.35
ICP_MAX_ITER         = 80
USE_ICP_REFINEMENT   = True    # set False for speed

# Noise removal
STAT_NB_NEIGHBORS    = 25
STAT_STD_RATIO       = 1.8
RADIUS_RADIUS        = 0.06
RADIUS_MIN_PTS       = 6

# Poisson mesh
POISSON_DEPTH        = 9
POISSON_DENSITY_PCT  = 5

# Voxel sizes
VOXEL_ICP            = 0.025
VOXEL_FINAL          = 0.018

# Floor
FLOOR_Y              = 2.5
FLOOR_SIZE           = 12.0


# ═══════════════════════════════════════════════════════════════
#  POSE MODULE  ←  only function you change for IMU support
# ═══════════════════════════════════════════════════════════════

def yaw_matrix(degrees):
    """4x4 rigid transform for a pure yaw rotation around Y-axis."""
    theta = np.radians(degrees)
    return np.array([
        [ np.cos(theta), 0, np.sin(theta), 0],
        [             0, 1,             0, 0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [             0, 0,             0, 1],
    ], dtype=np.float64)


def quaternion_to_4x4(qx, qy, qz, qw):
    """
    Unit quaternion to 4x4 rotation matrix.
    Already implemented and ready — just call it from get_frame_transform
    when you have real IMU data from iPhone ARKit.
    """
    n = qx*qx + qy*qy + qz*qz + qw*qw
    if n < 1e-10:
        return np.eye(4)
    s  = 2.0 / n
    wx = s*qw*qx; wy = s*qw*qy; wz = s*qw*qz
    xx = s*qx*qx; xy = s*qx*qy; xz = s*qx*qz
    yy = s*qy*qy; yz = s*qy*qz; zz = s*qz*qz
    return np.array([
        [1-(yy+zz),  xy-wz,      xz+wy,  0],
        [xy+wz,      1-(xx+zz),  yz-wx,  0],
        [xz-wy,      yz+wx,      1-(xx+yy), 0],
        [0,          0,          0,      1],
    ], dtype=np.float64)


def get_frame_transform(i, n, imu_data=None):
    """
    ╔══════════════════════════════════════════════════════════╗
    ║  THIS IS THE ONLY FUNCTION YOU CHANGE FOR IMU SUPPORT   ║
    ╚══════════════════════════════════════════════════════════╝

    Returns a 4x4 rigid transform placing frame i in world space.

    Default mode: equal-interval yaw sweep.
      Frame 0 = 0 deg, frame 1 = 360/N deg, frame 2 = 2x360/N deg ...

    iPhone IMU mode (when you are ready):
      Pass imu_data as a list of dicts with keys x, y, z, w.
      This function will call quaternion_to_4x4() automatically.
    """
    if imu_data is not None:
        q = imu_data[i]
        return quaternion_to_4x4(q['x'], q['y'], q['z'], q['w'])

    # Default: equal-interval yaw
    angle = (i / n) * TOTAL_YAW_DEGREES
    return yaw_matrix(angle)


# ═══════════════════════════════════════════════════════════════
#  MiDaS
# ═══════════════════════════════════════════════════════════════

def load_midas():
    print(f"[MiDaS] Loading {MODEL_TYPE} ...")
    model = torch.hub.load("intel-isl/MiDaS", MODEL_TYPE, trust_repo=True)
    transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
    transform = (transforms.dpt_transform if "DPT" in MODEL_TYPE
                 else transforms.small_transform)
    device = (torch.device("mps")  if torch.backends.mps.is_available() else
              torch.device("cuda") if torch.cuda.is_available()          else
              torch.device("cpu"))
    print(f"[MiDaS] Device: {device}")
    model.to(device).eval()
    return model, transform, device


def estimate_depth(frame_bgr, model, transform, device):
    img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    batch = transform(img).to(device)
    with torch.no_grad():
        pred = model(batch)
        pred = torch.nn.functional.interpolate(
            pred.unsqueeze(1), size=img.shape[:2],
            mode="bicubic", align_corners=False).squeeze()
    d = pred.cpu().numpy().astype(np.float32)
    lo, hi = d.min(), d.max()
    if hi - lo > 1e-6:
        d = (d - lo) / (hi - lo)
    return d


# ═══════════════════════════════════════════════════════════════
#  Frame to point cloud
# ═══════════════════════════════════════════════════════════════

def frame_to_pcd(frame_bgr, depth_map, global_depth_scale=None):
    h, w = depth_map.shape
    focal = max(w, h) * 0.9
    cx, cy = w / 2.0, h / 2.0

    d = median_filter(depth_map, size=DEPTH_SMOOTH_K).astype(np.float32)
    mean_d = d.mean()

    if global_depth_scale is not None and mean_d > 1e-4:
        d = d * (global_depth_scale / mean_d)

    eps = 0.05
    Z = DEPTH_SCALE / (d + eps)

    u = np.arange(w, dtype=np.float32)
    v = np.arange(h, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)
    X = (uu - cx) * Z / focal
    Y = (vv - cy) * Z / focal

    pts    = np.stack([X, Y, Z], axis=-1).reshape(-1, 3)
    colors = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB).reshape(-1, 3) / 255.0

    if len(pts) > MAX_PTS_PER_FRAME:
        idx    = np.random.choice(len(pts), MAX_PTS_PER_FRAME, replace=False)
        pts    = pts[idx]
        colors = colors[idx]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
    return pcd, float(mean_d)


# ═══════════════════════════════════════════════════════════════
#  Scene builder
# ═══════════════════════════════════════════════════════════════

def build_scene(frame_data, imu_data=None):
    n = len(frame_data)
    global_scale = float(np.mean([d.mean() for _, d in frame_data]))
    print(f"[Build] {n} frames | scale: {global_scale:.4f}")

    merged = o3d.geometry.PointCloud()

    for i, (frame, depth) in enumerate(frame_data):
        T = get_frame_transform(i, n, imu_data)
        angle_label = f"{(i/n)*TOTAL_YAW_DEGREES:.0f} deg"
        print(f"  Frame {i+1}/{n}  ({angle_label})", end=" ", flush=True)
        t0 = time.time()

        pcd, _ = frame_to_pcd(frame, depth, global_scale)
        pcd.transform(T)

        # ICP refinement against accumulated scene
        if USE_ICP_REFINEMENT and i > 0 and len(merged.points) > 100:
            pcd_ds    = pcd.voxel_down_sample(VOXEL_ICP)
            merged_ds = merged.voxel_down_sample(VOXEL_ICP)

            pcd_ds.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=30))
            merged_ds.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=30))

            result = o3d.pipelines.registration.registration_icp(
                pcd_ds, merged_ds, ICP_MAX_DIST, np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=ICP_MAX_ITER))

            if result.fitness > 0.25:
                pcd.transform(result.transformation)
                tag = f"ICP={result.fitness:.2f}"
            else:
                tag = f"no ICP ({result.fitness:.2f})"
        else:
            tag = "pose only"

        merged += pcd
        print(f" {tag}  {time.time()-t0:.1f}s")

    # Clean up
    print("[Build] Downsampling ...")
    merged = merged.voxel_down_sample(VOXEL_FINAL)

    print("[Build] Removing outliers ...")
    merged, _ = merged.remove_statistical_outlier(
        nb_neighbors=STAT_NB_NEIGHBORS, std_ratio=STAT_STD_RATIO)
    merged, _ = merged.remove_radius_outlier(
        nb_points=RADIUS_MIN_PTS, radius=RADIUS_RADIUS)

    print(f"[Build] Clean cloud: {len(merged.points):,} points")

    # Normals
    print("[Build] Estimating normals ...")
    merged.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=30))
    centre = np.asarray(merged.points).mean(axis=0)
    merged.orient_normals_towards_camera_location(centre)

    # Poisson
    print(f"[Build] Poisson mesh (depth={POISSON_DEPTH}) — ~30s ...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        merged, depth=POISSON_DEPTH)

    densities = np.asarray(densities)
    mesh.remove_vertices_by_mask(densities < np.percentile(densities, POISSON_DENSITY_PCT))
    mesh.compute_vertex_normals()

    print(f"[Build] Mesh: {len(mesh.vertices):,} verts  {len(mesh.triangles):,} tris")
    return merged, mesh


# ═══════════════════════════════════════════════════════════════
#  Floor
# ═══════════════════════════════════════════════════════════════

def build_floor():
    slab = o3d.geometry.TriangleMesh.create_box(
        width=FLOOR_SIZE*2, height=0.03, depth=FLOOR_SIZE*2)
    slab.translate([-FLOOR_SIZE, FLOOR_Y, -FLOOR_SIZE*0.15])
    slab.paint_uniform_color([0.14, 0.15, 0.19])
    slab.compute_vertex_normals()
    return slab


# ═══════════════════════════════════════════════════════════════
#  Webcam capture
# ═══════════════════════════════════════════════════════════════

def capture_frames(model, transform, device):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot open webcam.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    frames, last_depth_vis = [], None
    step = TOTAL_YAW_DEGREES / MAX_FRAMES

    print("\n" + "="*62)
    print(f"  360 Room Scan  —  {MAX_FRAMES} frames  x  {step:.0f} deg each")
    print("  SPACE=capture   ENTER=build   Q=quit")
    print("="*62 + "\n")

    win = "360 Room Scan  |  SPACE=capture  ENTER=build  Q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 900, 520)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        frame = cv2.flip(frame, 1)
        h, w  = frame.shape[:2]
        disp  = frame.copy()

        next_angle = len(frames) * step
        cv2.rectangle(disp, (0,0), (w,56), (10,13,20), -1)
        cv2.putText(disp,
                    f"Frame {len(frames)}/{MAX_FRAMES}   aim at {next_angle:.0f} deg",
                    (12,36), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (60,210,150), 2)
        cv2.putText(disp, "SPACE=capture  ENTER=build  Q=quit",
                    (12, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (150,150,150), 1)

        if last_depth_vis is not None:
            sw = w // 3
            disp[56:, w-sw:] = cv2.resize(last_depth_vis, (sw, h-56))

        cv2.imshow(win, disp)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):
            print(f"[{len(frames)+1}/{MAX_FRAMES}]  ~{next_angle:.0f} deg — MiDaS ...",
                  end=" ", flush=True)
            t0 = time.time()
            depth = estimate_depth(frame, model, transform, device)
            print(f"{time.time()-t0:.1f}s")
            frames.append((frame.copy(), depth))
            last_depth_vis = cv2.applyColorMap(
                (depth*255).astype(np.uint8), cv2.COLORMAP_MAGMA)
            if len(frames) >= MAX_FRAMES:
                print(f"[Capture] Auto-stopped.")
                break

        elif key in (13, 10):
            if len(frames) < 2:
                print("[!] Need at least 2 frames.")
            else:
                break
        elif key == ord('q'):
            cap.release(); cv2.destroyAllWindows(); return []

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n[Capture] {len(frames)} frames ready.\n")
    return frames


# ═══════════════════════════════════════════════════════════════
#  Viewer
# ═══════════════════════════════════════════════════════════════

def view_scene(mesh, pcd=None):
    print("\n[Viewer] Left-drag=rotate  Right-drag=pan  Scroll=zoom  Q=close\n")
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="360 Room Scan", width=1440, height=900)
    vis.add_geometry(mesh)
    if pcd is not None:
        vis.add_geometry(pcd)

    opt = vis.get_render_option()
    opt.background_color    = np.array([0.04, 0.05, 0.08])
    opt.mesh_show_back_face = True   # renders inside AND outside
    opt.light_on            = True
    opt.point_size          = 1.5

    ctr = vis.get_view_control()
    ctr.set_zoom(0.5)
    ctr.set_front([0.0, -0.3, -1.0])
    ctr.set_lookat([0.0, 0.5, 3.0])
    ctr.set_up([0.0, -1.0, 0.0])

    vis.run()
    vis.destroy_window()


# ═══════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════

def main():
    print("\n" + "="*62)
    print("  MiDaS 360 Room Scanner — IMU-Ready")
    print("="*62)

    model, transform, device = load_midas()
    frames = capture_frames(model, transform, device)
    if not frames:
        return

    # To use real iPhone IMU data later, load your JSON and pass:
    #   import json
    #   imu_data = json.load(open("imu_poses.json"))
    #   pcd, mesh = build_scene(frames, imu_data=imu_data)
    pcd, mesh = build_scene(frames, imu_data=None)

    mesh = mesh + build_floor()
    mesh.compute_vertex_normals()

    o3d.io.write_triangle_mesh("room_360.ply", mesh)
    o3d.io.write_point_cloud("room_360_cloud.ply", pcd)
    print(f"\n[Saved] room_360.ply  ->  {os.path.abspath('.')}")

    view_scene(mesh, pcd)


if __name__ == "__main__":
    main()


# ═══════════════════════════════════════════════════════════════
#  RELOAD:
#      import open3d as o3d
#      m = o3d.io.read_triangle_mesh("room_360.ply")
#      m.compute_vertex_normals()
#      o3d.visualization.draw_geometries([m])
#
#  TUNE:
#      TOTAL_YAW_DEGREES  ->  180 if only panning halfway
#      POISSON_DEPTH      ->  10 for more detail (slower)
#      USE_ICP_REFINEMENT ->  False to skip ICP (faster)
#      MAX_FRAMES         ->  12 for fewer shots, 30 for denser
# ═══════════════════════════════════════════════════════════════
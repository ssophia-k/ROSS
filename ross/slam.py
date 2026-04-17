"""SLAM room scanner using the ESP32-CAM stream.

Connects to ROSS's MJPEG stream and /imu endpoint, runs MiDaS depth
estimation + ICP registration + Poisson mesh reconstruction + YOLOv8
human detection with Re-ID tracking.

Based on testing-m/midasroom-detection.py, adapted to pull video from
the ESP32-CAM over WiFi instead of a local webcam.

Usage:
    uv run python ross/slam.py                   # auto-discover via mDNS
    uv run python ross/slam.py --host 10.0.0.5   # explicit IP
    uv run python ross/slam.py --no-imu           # skip IMU, use yaw sweep
    uv run python ross/slam.py --no-yolo           # skip human detection
    uv run python ross/slam.py --model MiDaS_small # faster, less accurate
"""

import argparse
import json
import os
import socket
import sys
import time
import urllib.request

import cv2
import numpy as np
import open3d as o3d
import torch
from scipy.ndimage import median_filter
from scipy.spatial.distance import cosine


# ═══════════════════════════════════════════════════════════════════════════════
#  SETTINGS
# ═══════════════════════════════════════════════════════════════════════════════

MAX_FRAMES           = 20
TOTAL_YAW_DEGREES    = 360.0

MAX_PTS_PER_FRAME    = 100_000
DEPTH_SCALE          = 4.0
DEPTH_SMOOTH_K       = 5

ICP_MAX_DIST         = 0.35
ICP_MAX_ITER         = 80
USE_ICP_REFINEMENT   = True

STAT_NB_NEIGHBORS    = 25
STAT_STD_RATIO       = 1.8
RADIUS_RADIUS        = 0.06
RADIUS_MIN_PTS       = 6

POISSON_DEPTH        = 9
POISSON_DENSITY_PCT  = 5

VOXEL_ICP            = 0.025
VOXEL_FINAL          = 0.018

FLOOR_Y              = 2.5
FLOOR_SIZE           = 12.0

# Human detection
YOLO_MODEL           = "yolov8n.pt"
YOLO_CONF            = 0.40
PERSON_CLASS_ID      = 0

REID_APPEARANCE_THRESH = 0.82
SAME_PERSON_DIST       = 1.2

LABEL_SPHERE_RADIUS  = 0.12
LABEL_SPHERE_COLOR   = [0.95, 0.35, 0.20]
LABEL_SPHERE_STEPS   = 12


# ═══════════════════════════════════════════════════════════════════════════════
#  ESP32 CONNECTION
# ═══════════════════════════════════════════════════════════════════════════════

def discover_host() -> str | None:
    """Try to resolve ross.local via system mDNS."""
    try:
        results = socket.getaddrinfo("ross.local", 80, socket.AF_INET)
        addr = results[0][4][0]
        print(f"[mDNS] Found ROSS at {addr}")
        return addr
    except socket.gaierror:
        pass

    try:
        from zeroconf import ServiceBrowser, Zeroconf

        found = {}

        class Listener:
            def add_service(self, zc, type_, name):
                info = zc.get_service_info(type_, name)
                if info and info.server and "ross" in info.server.lower():
                    found["addr"] = socket.inet_ntoa(info.addresses[0])
            def remove_service(self, *_): pass
            def update_service(self, *_): pass

        zc = Zeroconf()
        ServiceBrowser(zc, "_http._tcp.local.", Listener())
        deadline = time.monotonic() + 5
        while not found and time.monotonic() < deadline:
            time.sleep(0.1)
        zc.close()

        if found:
            print(f"[zeroconf] Found ROSS at {found['addr']}")
            return found["addr"]
    except ImportError:
        pass

    return None


def fetch_imu(host: str) -> dict | None:
    """Fetch one IMU reading from the ESP32. Returns dict or None."""
    try:
        resp = urllib.request.urlopen(f"http://{host}/imu", timeout=2)
        return json.loads(resp.read())
    except Exception:
        return None


# ═══════════════════════════════════════════════════════════════════════════════
#  POSE
# ═══════════════════════════════════════════════════════════════════════════════

def yaw_matrix(degrees):
    theta = np.radians(degrees)
    return np.array([
        [ np.cos(theta), 0, np.sin(theta), 0],
        [             0, 1,             0, 0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [             0, 0,             0, 1],
    ], dtype=np.float64)


def accel_to_gravity_rotation(accel):
    """Estimate a rotation matrix from accelerometer gravity vector.

    Assumes the robot is not accelerating significantly, so accel ≈ gravity.
    Returns a 4x4 transform that aligns the measured gravity with world -Y.
    """
    g = np.array(accel, dtype=np.float64)
    norm = np.linalg.norm(g)
    if norm < 1e-6:
        return np.eye(4)
    g = g / norm

    # World gravity direction (down = +Y in our coord system)
    world_down = np.array([0.0, 1.0, 0.0])

    # Rotation axis = cross(measured, world_down)
    axis = np.cross(g, world_down)
    sin_angle = np.linalg.norm(axis)
    cos_angle = np.dot(g, world_down)

    if sin_angle < 1e-6:
        return np.eye(4)

    axis = axis / sin_angle

    # Rodrigues' rotation formula
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ])
    R = np.eye(3) + sin_angle * K + (1 - cos_angle) * (K @ K)

    T = np.eye(4)
    T[:3, :3] = R
    return T


def get_frame_transform(i, n, imu_readings=None):
    """Return 4x4 pose for frame i.

    If IMU readings are available, combines yaw sweep with gravity-based
    tilt correction from the accelerometer. Otherwise pure yaw sweep.
    """
    yaw = yaw_matrix((i / n) * TOTAL_YAW_DEGREES)

    if imu_readings is not None and i < len(imu_readings):
        reading = imu_readings[i]
        tilt = accel_to_gravity_rotation(reading["accel"])
        return tilt @ yaw

    return yaw


# ═══════════════════════════════════════════════════════════════════════════════
#  MiDaS
# ═══════════════════════════════════════════════════════════════════════════════

def load_midas(model_type="DPT_Hybrid"):
    print(f"[MiDaS] Loading {model_type} ...")
    model = torch.hub.load("intel-isl/MiDaS", model_type, trust_repo=True)
    transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
    transform = (transforms.dpt_transform if "DPT" in model_type
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


# ═══════════════════════════════════════════════════════════════════════════════
#  YOLO
# ═══════════════════════════════════════════════════════════════════════════════

def load_yolo():
    try:
        from ultralytics import YOLO
        print(f"[YOLO] Loading {YOLO_MODEL} ...")
        model = YOLO(YOLO_MODEL)
        print("[YOLO] Ready.")
        return model
    except ImportError:
        print("[YOLO] ultralytics not installed — continuing without human detection")
        return None


def detect_people(frame_bgr, yolo_model):
    if yolo_model is None:
        return []
    results = yolo_model(frame_bgr, conf=YOLO_CONF, classes=[PERSON_CLASS_ID],
                         verbose=False)
    people = []
    for r in results:
        for box in r.boxes:
            if int(box.cls[0]) != PERSON_CLASS_ID:
                continue
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            people.append({
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "conf": float(box.conf[0]),
                "cx": (x1 + x2) // 2,
                "cy": (y1 + y2) // 2,
            })
    return people


# ═══════════════════════════════════════════════════════════════════════════════
#  Re-ID
# ═══════════════════════════════════════════════════════════════════════════════

def appearance_embedding(frame_bgr, person):
    x1, y1, x2, y2 = person["x1"], person["y1"], person["x2"], person["y2"]
    h_box, w_box = y2 - y1, x2 - x1

    y_top = y1 + h_box // 3
    y_bot = y2 - h_box // 3
    x_lft = x1 + w_box // 6
    x_rgt = x2 - w_box // 6

    crop = frame_bgr[max(0,y_top):max(0,y_bot), max(0,x_lft):max(0,x_rgt)]
    if crop.size == 0:
        crop = frame_bgr[max(0,y1):max(0,y2), max(0,x1):max(0,x2)]
    if crop.size == 0:
        return np.zeros(48, dtype=np.float32)

    hist = []
    for ch in range(3):
        h, _ = np.histogram(crop[:, :, ch], bins=16, range=(0, 256))
        hist.append(h.astype(np.float32))
    vec = np.concatenate(hist)
    norm = np.linalg.norm(vec)
    return vec / (norm + 1e-6)


def person_to_3d(person, depth_map, frame_w, frame_h,
                 global_depth_scale, transform_4x4):
    cx_px = np.clip(person["cx"], 0, frame_w - 1)
    cy_px = np.clip(person["cy"], 0, frame_h - 1)
    focal = max(frame_w, frame_h) * 0.9
    px_cx, px_cy = frame_w / 2.0, frame_h / 2.0

    r = 8
    patch = depth_map[
        max(0, cy_px-r):min(frame_h, cy_px+r),
        max(0, cx_px-r):min(frame_w, cx_px+r)
    ]
    d_val = float(np.median(patch)) if patch.size > 0 else float(depth_map[cy_px, cx_px])

    mean_d = depth_map.mean()
    if global_depth_scale is not None and mean_d > 1e-4:
        d_val = d_val * (global_depth_scale / mean_d)

    Z = DEPTH_SCALE / (d_val + 0.05)
    X = (cx_px - px_cx) * Z / focal

    y2_px = np.clip(person["y2"], 0, frame_h - 1)
    Y_foot = (y2_px - px_cy) * Z / focal

    local_pt = np.array([X, Y_foot, Z, 1.0])
    world_pt = transform_4x4 @ local_pt
    return tuple(world_pt[:3])


class PersonTracker:
    def __init__(self):
        self.people = []

    def update(self, pos3d, embedding):
        best_idx, best_sim = -1, -1.0

        for idx, p in enumerate(self.people):
            sim = 1.0 - cosine(embedding, p["embedding"])
            if sim < REID_APPEARANCE_THRESH:
                continue
            dist = np.linalg.norm(np.array(pos3d) - np.array(p["pos3d"]))
            if dist > SAME_PERSON_DIST:
                continue
            if sim > best_sim:
                best_sim, best_idx = sim, idx

        if best_idx >= 0:
            p = self.people[best_idx]
            n = p["count"]
            p["pos3d"] = tuple((np.array(p["pos3d"]) * n + np.array(pos3d)) / (n + 1))
            p["embedding"] = (p["embedding"] * n + embedding) / (n + 1)
            p["embedding"] /= np.linalg.norm(p["embedding"]) + 1e-6
            p["count"] += 1
            return p
        else:
            label = f"Person {len(self.people) + 1}"
            new_p = {"pos3d": pos3d, "embedding": embedding.copy(),
                     "count": 1, "label": label}
            self.people.append(new_p)
            print(f"    [Re-ID] New: {label}  pos=({pos3d[0]:.2f}, {pos3d[1]:.2f}, {pos3d[2]:.2f})")
            return new_p

    def all_people(self):
        return self.people


# ═══════════════════════════════════════════════════════════════════════════════
#  POINT CLOUD + SCENE BUILD
# ═══════════════════════════════════════════════════════════════════════════════

def frame_to_pcd(frame_bgr, depth_map, global_depth_scale=None):
    h, w = depth_map.shape
    focal = max(w, h) * 0.9
    cx, cy = w / 2.0, h / 2.0

    d = median_filter(depth_map, size=DEPTH_SMOOTH_K).astype(np.float32)
    mean_d = d.mean()
    if global_depth_scale is not None and mean_d > 1e-4:
        d = d * (global_depth_scale / mean_d)

    Z = DEPTH_SCALE / (d + 0.05)
    u = np.arange(w, dtype=np.float32)
    v = np.arange(h, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)
    X = (uu - cx) * Z / focal
    Y = (vv - cy) * Z / focal

    pts    = np.stack([X, Y, Z], axis=-1).reshape(-1, 3)
    colors = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB).reshape(-1, 3) / 255.0

    if len(pts) > MAX_PTS_PER_FRAME:
        idx = np.random.choice(len(pts), MAX_PTS_PER_FRAME, replace=False)
        pts, colors = pts[idx], colors[idx]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
    return pcd, float(mean_d)


def build_scene(frame_data, yolo_model, imu_readings=None):
    n = len(frame_data)
    global_scale = float(np.mean([d.mean() for _, d in frame_data]))
    print(f"[Build] {n} frames | scale: {global_scale:.4f}")

    merged  = o3d.geometry.PointCloud()
    tracker = PersonTracker()

    for i, (frame, depth) in enumerate(frame_data):
        T = get_frame_transform(i, n, imu_readings)
        angle = (i / n) * TOTAL_YAW_DEGREES
        print(f"  Frame {i+1}/{n} ({angle:.0f} deg)", end="  ", flush=True)
        t0 = time.time()

        h_f, w_f = frame.shape[:2]
        pcd, _ = frame_to_pcd(frame, depth, global_scale)
        pcd.transform(T)

        # ICP refinement
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
                T = result.transformation @ T
                tag = f"ICP={result.fitness:.2f}"
            else:
                tag = f"no ICP ({result.fitness:.2f})"
        else:
            tag = "pose only"

        merged += pcd

        # Human detection
        people = detect_people(frame, yolo_model)
        for p in people:
            pos3d = person_to_3d(p, depth, w_f, h_f, global_scale, T)
            emb   = appearance_embedding(frame, p)
            tracker.update(pos3d, emb)

        det_str = f"  {len(people)} person(s)" if people else ""
        print(f"{tag}{det_str}  {time.time()-t0:.1f}s")

    # Denoise
    print("[Build] Downsampling + denoising ...")
    merged = merged.voxel_down_sample(VOXEL_FINAL)
    merged, _ = merged.remove_statistical_outlier(
        nb_neighbors=STAT_NB_NEIGHBORS, std_ratio=STAT_STD_RATIO)
    merged, _ = merged.remove_radius_outlier(
        nb_points=RADIUS_MIN_PTS, radius=RADIUS_RADIUS)
    print(f"[Build] Clean cloud: {len(merged.points):,} points")

    # Normals + Poisson mesh
    print("[Build] Estimating normals ...")
    merged.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=30))
    centre = np.asarray(merged.points).mean(axis=0)
    merged.orient_normals_towards_camera_location(centre)

    print(f"[Build] Poisson mesh (depth={POISSON_DEPTH}) ...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        merged, depth=POISSON_DEPTH)
    densities = np.asarray(densities)
    mesh.remove_vertices_by_mask(
        densities < np.percentile(densities, POISSON_DENSITY_PCT))
    mesh.compute_vertex_normals()
    print(f"[Build] Mesh: {len(mesh.vertices):,} verts  {len(mesh.triangles):,} tris")

    people_found = tracker.all_people()
    if people_found:
        print(f"\n[People] {len(people_found)} unique person(s):")
        for p in people_found:
            pos = p["pos3d"]
            print(f"  {p['label']:12s}  ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
                  f"  seen {p['count']}x")

    return merged, mesh, people_found


# ═══════════════════════════════════════════════════════════════════════════════
#  FLOOR + LABELS + VIEWER
# ═══════════════════════════════════════════════════════════════════════════════

def build_floor():
    slab = o3d.geometry.TriangleMesh.create_box(
        width=FLOOR_SIZE*2, height=0.03, depth=FLOOR_SIZE*2)
    slab.translate([-FLOOR_SIZE, FLOOR_Y, -FLOOR_SIZE*0.15])
    slab.paint_uniform_color([0.14, 0.15, 0.19])
    slab.compute_vertex_normals()
    return slab


def make_person_label(person_dict):
    pos = np.array(person_dict["pos3d"])
    geometries = []

    sphere = o3d.geometry.TriangleMesh.create_sphere(
        radius=LABEL_SPHERE_RADIUS, resolution=LABEL_SPHERE_STEPS)
    sphere.translate(pos)
    sphere.paint_uniform_color(LABEL_SPHERE_COLOR)
    sphere.compute_vertex_normals()
    geometries.append(sphere)

    pole_bot = np.array([pos[0], FLOOR_Y - 0.03, pos[2]])
    if pole_bot[1] < pos[1]:
        pole = o3d.geometry.TriangleMesh.create_cylinder(
            radius=0.015, height=float(abs(pole_bot[1] - pos[1])))
        pole.translate((pos + pole_bot) / 2.0)
        pole.paint_uniform_color([0.8, 0.8, 0.8])
        pole.compute_vertex_normals()
        geometries.append(pole)

    return geometries


def view_scene(mesh, pcd, people):
    print("\n[Viewer] Left-drag=rotate  Right-drag=pan  Scroll=zoom  Q=close")
    if people:
        print("  Person legend (red spheres):")
        for p in people:
            pos = p["pos3d"]
            print(f"    {p['label']:12s}  ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})"
                  f"  seen {p['count']}x")
    print()

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="ROSS Room Scan", width=1440, height=900)
    vis.add_geometry(mesh)
    if pcd is not None:
        vis.add_geometry(pcd)
    for p in people:
        for geom in make_person_label(p):
            vis.add_geometry(geom)

    opt = vis.get_render_option()
    opt.background_color    = np.array([0.04, 0.05, 0.08])
    opt.mesh_show_back_face = True
    opt.light_on            = True
    opt.point_size          = 1.5

    ctr = vis.get_view_control()
    ctr.set_zoom(0.5)
    ctr.set_front([0.0, -0.3, -1.0])
    ctr.set_lookat([0.0, 0.5, 3.0])
    ctr.set_up([0.0, -1.0, 0.0])

    vis.run()
    vis.destroy_window()


# ═══════════════════════════════════════════════════════════════════════════════
#  CAPTURE FROM ESP32
# ═══════════════════════════════════════════════════════════════════════════════

def capture_frames(host, model, transform, device, use_imu=True):
    """Capture frames from the ESP32-CAM MJPEG stream."""
    stream_url = f"http://{host}:81/stream"
    print(f"[Stream] Connecting to {stream_url} ...")

    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        print(f"[Stream] Failed to connect. Is ROSS running?")
        sys.exit(1)

    print("[Stream] Connected.")

    frames = []
    imu_readings = []
    step = TOTAL_YAW_DEGREES / MAX_FRAMES

    print(f"\n{'='*60}")
    print(f"  ROSS Room Scan  ({MAX_FRAMES} frames x {step:.0f} deg each)")
    print(f"  Rotate the robot between captures.")
    print(f"  SPACE=capture   ENTER=build   Q=quit")
    print(f"{'='*60}\n")

    win = "ROSS Scan  |  SPACE=capture  ENTER=build  Q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 800, 520)

    last_depth_vis = None

    while True:
        ok, frame = cap.read()
        if not ok:
            # Reconnect on stream drop
            cap.release()
            cap = cv2.VideoCapture(stream_url)
            continue

        h, w = frame.shape[:2]
        disp = frame.copy()

        next_angle = len(frames) * step
        cv2.rectangle(disp, (0, 0), (w, 56), (10, 13, 20), -1)
        cv2.putText(disp,
                    f"Frame {len(frames)}/{MAX_FRAMES}   aim at {next_angle:.0f} deg",
                    (12, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (60, 210, 150), 2)
        cv2.putText(disp, "SPACE=capture  ENTER=build  Q=quit",
                    (12, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (150, 150, 150), 1)

        if last_depth_vis is not None:
            sw = w // 3
            disp[56:, w-sw:] = cv2.resize(last_depth_vis, (sw, h - 56))

        cv2.imshow(win, disp)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):
            print(f"[{len(frames)+1}/{MAX_FRAMES}] {next_angle:.0f} deg — MiDaS ...",
                  end=" ", flush=True)
            t0 = time.time()
            depth = estimate_depth(frame, model, transform, device)
            print(f"{time.time()-t0:.1f}s")

            frames.append((frame.copy(), depth))

            # Grab IMU reading at capture time
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
                (depth * 255).astype(np.uint8), cv2.COLORMAP_MAGMA)

            if len(frames) >= MAX_FRAMES:
                print("[Capture] Max frames reached.")
                break

        elif key in (13, 10):
            if len(frames) < 2:
                print("[!] Need at least 2 frames.")
            else:
                break
        elif key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            return [], []

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n[Capture] {len(frames)} frames ready.\n")

    # Only pass IMU readings if we actually got valid data
    if use_imu and imu_readings and any(r is not None for r in imu_readings):
        return frames, imu_readings
    return frames, None


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="ROSS room scanner with SLAM")
    parser.add_argument("--host", help="robot IP or hostname (default: auto-discover)")
    parser.add_argument("--model", default="DPT_Hybrid",
                        choices=["DPT_Large", "DPT_Hybrid", "MiDaS_small"],
                        help="MiDaS model (default: DPT_Hybrid)")
    parser.add_argument("--no-imu", action="store_true",
                        help="skip IMU, use pure yaw sweep for pose")
    parser.add_argument("--no-yolo", action="store_true",
                        help="skip human detection")
    parser.add_argument("--output", default="ross_scan",
                        help="output filename prefix (default: ross_scan)")
    args = parser.parse_args()

    print(f"\n{'='*60}")
    print("  ROSS Room Scanner")
    print(f"{'='*60}")

    # Discover robot
    host = args.host
    if not host:
        host = discover_host()
        if not host:
            print("[!] Could not find ROSS. Use --host to specify.")
            sys.exit(1)

    # Load models
    midas_model, midas_transform, device = load_midas(args.model)
    yolo_model = None if args.no_yolo else load_yolo()

    # Capture
    frames, imu_readings = capture_frames(
        host, midas_model, midas_transform, device, use_imu=not args.no_imu)
    if not frames:
        return

    # Build scene
    pcd, mesh, people = build_scene(frames, yolo_model, imu_readings)

    mesh = mesh + build_floor()
    mesh.compute_vertex_normals()

    # Save
    mesh_path = f"{args.output}.ply"
    cloud_path = f"{args.output}_cloud.ply"
    o3d.io.write_triangle_mesh(mesh_path, mesh)
    o3d.io.write_point_cloud(cloud_path, pcd)
    print(f"\n[Saved] {mesh_path}, {cloud_path} -> {os.path.abspath('.')}")

    # View
    view_scene(mesh, pcd, people)


if __name__ == "__main__":
    main()

"""
MiDaS Room Scanner — with Human Detection & 3D Labels
=======================================================
Everything from midas_room_360.py is preserved.
New additions:
  • YOLOv5  — detects people in each frame (runs on CPU, no GPU needed)
  • Re-ID   — compares each detected person's appearance (colour histogram
               + position) across frames so the same stationary person is
               only labelled ONCE in the 3D scene, not once per frame.
  • 3D sphere + text label — each unique person gets a coloured sphere
               placed at their real-world 3D position (estimated from
               MiDaS depth at their bounding-box centre).

Install (add to existing deps):
    pip install torch torchvision timm opencv-python open3d scipy numpy
    pip install ultralytics          ← YOLOv5/v8 via ultralytics package


Capture controls:
    SPACE   capture frame
    ENTER   build scene + detect people
    Q       quit

3D viewer:
    Left-drag=rotate  Right-drag=pan  Scroll=zoom  Q=close
"""

import cv2
import torch
import numpy as np
import open3d as o3d
from scipy.ndimage import median_filter
from scipy.spatial.distance import cosine
import time, os

# ═══════════════════════════════════════════════════════════════
#  SETTINGS
# ═══════════════════════════════════════════════════════════════

MODEL_TYPE           = "DPT_Hybrid"
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

# ── Human detection settings ──────────────────────────────────
YOLO_MODEL           = "yolov8n.pt"   # nano = fast; swap to yolov8m.pt for accuracy
YOLO_CONF            = 0.40           # minimum detection confidence
PERSON_CLASS_ID      = 0              # COCO class 0 = person

# Re-ID: two detections are the "same person" if their appearance
# similarity is above this threshold AND their 3D positions are
# within SAME_PERSON_DIST world units of each other.
REID_APPEARANCE_THRESH = 0.82         # cosine similarity (0-1, higher = more similar)
SAME_PERSON_DIST       = 1.2          # world-space distance threshold

# 3D label appearance
LABEL_SPHERE_RADIUS  = 0.12
LABEL_SPHERE_COLOR   = [0.95, 0.35, 0.20]   # warm red
LABEL_SPHERE_STEPS   = 12


# ═══════════════════════════════════════════════════════════════
#  POSE MODULE (unchanged from midas_room_360.py)
# ═══════════════════════════════════════════════════════════════

def yaw_matrix(degrees):
    theta = np.radians(degrees)
    return np.array([
        [ np.cos(theta), 0, np.sin(theta), 0],
        [             0, 1,             0, 0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [             0, 0,             0, 1],
    ], dtype=np.float64)


def quaternion_to_4x4(qx, qy, qz, qw):
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
        [0, 0, 0, 1],
    ], dtype=np.float64)


def get_frame_transform(i, n, imu_data=None):
    if imu_data is not None:
        q = imu_data[i]
        return quaternion_to_4x4(q['x'], q['y'], q['z'], q['w'])
    return yaw_matrix((i / n) * TOTAL_YAW_DEGREES)


# ═══════════════════════════════════════════════════════════════
#  MiDaS (unchanged)
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
#  YOLO human detector
# ═══════════════════════════════════════════════════════════════

def load_yolo():
    """Load YOLOv8 nano. Downloads ~6 MB on first run."""
    try:
        from ultralytics import YOLO
        print(f"[YOLO] Loading {YOLO_MODEL} ...")
        model = YOLO(YOLO_MODEL)
        print("[YOLO] Ready.")
        return model
    except ImportError:
        print("[YOLO] ultralytics not installed. Run:  pip install ultralytics")
        print("[YOLO] Continuing without human detection.")
        return None


def detect_people(frame_bgr, yolo_model):
    """
    Returns list of dicts, one per detected person:
        x1, y1, x2, y2  : bounding box in pixels
        conf             : detection confidence
        cx, cy           : centre of bounding box
    """
    if yolo_model is None:
        return []

    results = yolo_model(frame_bgr, conf=YOLO_CONF, classes=[PERSON_CLASS_ID],
                         verbose=False)
    people = []
    for r in results:
        for box in r.boxes:
            cls = int(box.cls[0])
            if cls != PERSON_CLASS_ID:
                continue
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            conf = float(box.conf[0])
            people.append({
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "conf": conf,
                "cx": (x1 + x2) // 2,
                "cy": (y1 + y2) // 2,
            })
    return people


# ═══════════════════════════════════════════════════════════════
#  Appearance embedding for Re-ID
# ═══════════════════════════════════════════════════════════════

def appearance_embedding(frame_bgr, person):
    """
    Extracts a compact appearance descriptor for a detected person.
    Uses an RGB colour histogram over the torso region (middle 60% of
    the bounding box) — fast, no extra model needed.

    For production you'd swap this for a proper Re-ID model (e.g. OSNet),
    but colour histograms handle same-clothing, same-room cases well enough.
    """
    x1, y1, x2, y2 = person["x1"], person["y1"], person["x2"], person["y2"]
    h_box = y2 - y1
    w_box = x2 - x1

    # Torso crop: middle vertical third, full width
    y_top = y1 + h_box // 3
    y_bot = y2 - h_box // 3
    x_lft = x1 + w_box // 6
    x_rgt = x2 - w_box // 6

    crop = frame_bgr[max(0,y_top):max(0,y_bot), max(0,x_lft):max(0,x_rgt)]
    if crop.size == 0:
        crop = frame_bgr[max(0,y1):max(0,y2), max(0,x1):max(0,x2)]
    if crop.size == 0:
        return np.zeros(48, dtype=np.float32)

    # 16-bin histogram per channel → 48-D vector
    hist = []
    for ch in range(3):
        h, _ = np.histogram(crop[:, :, ch], bins=16, range=(0, 256))
        hist.append(h.astype(np.float32))
    vec = np.concatenate(hist)
    norm = np.linalg.norm(vec)
    return vec / (norm + 1e-6)


# ═══════════════════════════════════════════════════════════════
#  3D position: pixel + depth → world XYZ
# ═══════════════════════════════════════════════════════════════

def person_to_3d(person, depth_map, frame_w, frame_h,
                 global_depth_scale, transform_4x4):
    """
    Back-projects the person's bounding-box centre into 3D world space
    using the MiDaS depth at that pixel and the frame's pose transform.

    Returns (x, y, z) in world coordinates, or None if depth is degenerate.
    """
    cx_px, cy_px = person["cx"], person["cy"]
    focal = max(frame_w, frame_h) * 0.9
    px_cx, px_cy = frame_w / 2.0, frame_h / 2.0

    # Clamp to image bounds
    cx_px = np.clip(cx_px, 0, frame_w - 1)
    cy_px = np.clip(cy_px, 0, frame_h - 1)

    # Sample depth in a small patch around the centre for robustness
    r = 8
    patch = depth_map[
        max(0, cy_px-r):min(frame_h, cy_px+r),
        max(0, cx_px-r):min(frame_w, cx_px+r)
    ]
    d_val = float(np.median(patch)) if patch.size > 0 else float(depth_map[cy_px, cx_px])

    # Rescale depth the same way frame_to_pcd does
    mean_d = depth_map.mean()
    if global_depth_scale is not None and mean_d > 1e-4:
        d_val = d_val * (global_depth_scale / mean_d)

    eps = 0.05
    Z = DEPTH_SCALE / (d_val + eps)
    X = (cx_px - px_cx) * Z / focal
    Y = (cy_px - px_cy) * Z / focal

    # Place at foot of person (lower half of bbox) rather than centre
    y2_px = person["y2"]
    y2_px = np.clip(y2_px, 0, frame_h - 1)
    Y_foot = (y2_px - px_cy) * Z / focal

    local_pt = np.array([X, Y_foot, Z, 1.0])
    world_pt = transform_4x4 @ local_pt
    return tuple(world_pt[:3])


# ═══════════════════════════════════════════════════════════════
#  Unique-person tracker across frames
# ═══════════════════════════════════════════════════════════════

class PersonTracker:
    """
    Maintains a list of unique people seen across all frames.
    A new detection is merged with an existing person if:
      1. Appearance similarity (cosine) >= REID_APPEARANCE_THRESH
      2. 3D world-space distance <= SAME_PERSON_DIST

    The stored 3D position is the running average across all sightings.
    """
    def __init__(self):
        self.people = []   # list of dicts: {pos3d, embedding, count, label}

    def update(self, pos3d, embedding):
        """
        Try to match pos3d+embedding to an existing person.
        Returns the matched (or newly created) person dict.
        """
        best_idx  = -1
        best_sim  = -1.0

        for idx, p in enumerate(self.people):
            # Appearance similarity
            sim = 1.0 - cosine(embedding, p["embedding"])
            if sim < REID_APPEARANCE_THRESH:
                continue
            # 3D proximity
            dist = np.linalg.norm(np.array(pos3d) - np.array(p["pos3d"]))
            if dist > SAME_PERSON_DIST:
                continue
            # Pick the closest-appearing match
            if sim > best_sim:
                best_sim = sim
                best_idx = idx

        if best_idx >= 0:
            # Merge: update running average position
            p = self.people[best_idx]
            n = p["count"]
            p["pos3d"] = tuple(
                (np.array(p["pos3d"]) * n + np.array(pos3d)) / (n + 1))
            # Update embedding as running average too
            p["embedding"] = (p["embedding"] * n + embedding) / (n + 1)
            p["embedding"] /= np.linalg.norm(p["embedding"]) + 1e-6
            p["count"] += 1
            return p
        else:
            # New unique person
            label = f"Person {len(self.people) + 1}"
            new_p = {
                "pos3d":     pos3d,
                "embedding": embedding.copy(),
                "count":     1,
                "label":     label,
            }
            self.people.append(new_p)
            print(f"    [Re-ID] New person detected: {label}  pos={pos3d}")
            return new_p

    def all_people(self):
        return self.people


# ═══════════════════════════════════════════════════════════════
#  3D label geometry: sphere + axis-aligned "pole"
# ═══════════════════════════════════════════════════════════════

def make_person_label(person_dict):
    """
    Creates an Open3D sphere at the person's 3D position.
    Returns a list of meshes (sphere + thin pole connecting to floor).
    """
    pos = np.array(person_dict["pos3d"])
    geometries = []

    # Sphere marker at foot position
    sphere = o3d.geometry.TriangleMesh.create_sphere(
        radius=LABEL_SPHERE_RADIUS, resolution=LABEL_SPHERE_STEPS)
    sphere.translate(pos)
    sphere.paint_uniform_color(LABEL_SPHERE_COLOR)
    sphere.compute_vertex_normals()
    geometries.append(sphere)

    # Thin vertical pole from sphere down to floor
    pole_bot = np.array([pos[0], FLOOR_Y - 0.03, pos[2]])
    pole_top = pos.copy()
    if pole_bot[1] < pole_top[1]:   # only draw if floor is below
        pole = o3d.geometry.TriangleMesh.create_cylinder(
            radius=0.015, height=float(abs(pole_bot[1] - pole_top[1])))
        mid = (pole_top + pole_bot) / 2.0
        pole.translate(mid)
        pole.paint_uniform_color([0.8, 0.8, 0.8])
        pole.compute_vertex_normals()
        geometries.append(pole)

    return geometries


# ═══════════════════════════════════════════════════════════════
#  Frame to point cloud (unchanged from midas_room_360.py)
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
#  Scene builder (extended with human detection)
# ═══════════════════════════════════════════════════════════════

def build_scene(frame_data, yolo_model, imu_data=None):
    n = len(frame_data)
    global_scale = float(np.mean([d.mean() for _, d in frame_data]))
    print(f"[Build] {n} frames | scale: {global_scale:.4f}")

    merged  = o3d.geometry.PointCloud()
    tracker = PersonTracker()

    for i, (frame, depth) in enumerate(frame_data):
        T           = get_frame_transform(i, n, imu_data)
        angle_label = f"{(i/n)*TOTAL_YAW_DEGREES:.0f} deg"
        print(f"  Frame {i+1}/{n} ({angle_label})", end="  ", flush=True)
        t0 = time.time()

        h_f, w_f = frame.shape[:2]

        # ── Point cloud ───────────────────────────────────────────────
        pcd, _ = frame_to_pcd(frame, depth, global_scale)
        pcd.transform(T)

        # ── ICP ───────────────────────────────────────────────────────
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
                T = result.transformation @ T   # update T so person pos is right
                tag = f"ICP={result.fitness:.2f}"
            else:
                tag = f"no ICP ({result.fitness:.2f})"
        else:
            tag = "pose only"

        merged += pcd

        # ── Human detection & Re-ID ───────────────────────────────────
        people = detect_people(frame, yolo_model)
        for p in people:
            pos3d = person_to_3d(p, depth, w_f, h_f, global_scale, T)
            emb   = appearance_embedding(frame, p)
            tracker.update(pos3d, emb)

        det_str = f"  {len(people)} person(s)" if people else ""
        print(f"{tag}{det_str}  {time.time()-t0:.1f}s")

    # ── Denoise ───────────────────────────────────────────────────────
    print("[Build] Downsampling + denoising ...")
    merged = merged.voxel_down_sample(VOXEL_FINAL)
    merged, _ = merged.remove_statistical_outlier(
        nb_neighbors=STAT_NB_NEIGHBORS, std_ratio=STAT_STD_RATIO)
    merged, _ = merged.remove_radius_outlier(
        nb_points=RADIUS_MIN_PTS, radius=RADIUS_RADIUS)

    print(f"[Build] Clean cloud: {len(merged.points):,} points")

    # ── Normals ───────────────────────────────────────────────────────
    print("[Build] Estimating normals ...")
    merged.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=30))
    centre = np.asarray(merged.points).mean(axis=0)
    merged.orient_normals_towards_camera_location(centre)

    # ── Poisson ───────────────────────────────────────────────────────
    print(f"[Build] Poisson mesh (depth={POISSON_DEPTH}) ~30s ...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        merged, depth=POISSON_DEPTH)
    densities = np.asarray(densities)
    mesh.remove_vertices_by_mask(
        densities < np.percentile(densities, POISSON_DENSITY_PCT))
    mesh.compute_vertex_normals()

    print(f"[Build] Mesh: {len(mesh.vertices):,} verts  {len(mesh.triangles):,} tris")

    # ── Summary ───────────────────────────────────────────────────────
    people_found = tracker.all_people()
    print(f"\n[People] {len(people_found)} unique person(s) detected:")
    for p in people_found:
        pos = p["pos3d"]
        print(f"  {p['label']:12s}  pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
              f"  seen {p['count']} time(s)")

    return merged, mesh, people_found


# ═══════════════════════════════════════════════════════════════
#  Floor (unchanged)
# ═══════════════════════════════════════════════════════════════

def build_floor():
    slab = o3d.geometry.TriangleMesh.create_box(
        width=FLOOR_SIZE*2, height=0.03, depth=FLOOR_SIZE*2)
    slab.translate([-FLOOR_SIZE, FLOOR_Y, -FLOOR_SIZE*0.15])
    slab.paint_uniform_color([0.14, 0.15, 0.19])
    slab.compute_vertex_normals()
    return slab


# ═══════════════════════════════════════════════════════════════
#  Webcam capture (unchanged)
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
    print(f"  360 Room Scan  ({MAX_FRAMES} frames x {step:.0f} deg each)")
    print("  SPACE=capture   ENTER=build   Q=quit")
    print("="*62 + "\n")

    win = "Room Scan  |  SPACE=capture  ENTER=build  Q=quit"
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
            print(f"[{len(frames)+1}/{MAX_FRAMES}] {next_angle:.0f} deg — MiDaS ...",
                  end=" ", flush=True)
            t0 = time.time()
            depth = estimate_depth(frame, model, transform, device)
            print(f"{time.time()-t0:.1f}s")
            frames.append((frame.copy(), depth))
            last_depth_vis = cv2.applyColorMap(
                (depth*255).astype(np.uint8), cv2.COLORMAP_MAGMA)
            if len(frames) >= MAX_FRAMES:
                print("[Capture] Auto-stopped.")
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
#  Viewer with person labels
# ═══════════════════════════════════════════════════════════════

def view_scene(mesh, pcd, people):
    """
    Open3D viewer showing the room mesh + point cloud + person spheres.
    Open3D doesn't support floating text natively, so each person gets:
      • A coloured sphere at their 3D foot position
      • A thin pole connecting to the floor
    The terminal prints a legend: Person 1 = red sphere, etc.
    """
    print("\n[Viewer] Left-drag=rotate  Right-drag=pan  Scroll=zoom  Q=close")
    if people:
        print("\n  Person legend (red spheres in scene):")
        for p in people:
            pos = p["pos3d"]
            print(f"    {p['label']:12s}  at ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})"
                  f"  seen in {p['count']} frame(s)")
    print()

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Room Scan + People", width=1440, height=900)

    vis.add_geometry(mesh)
    if pcd is not None:
        vis.add_geometry(pcd)

    # Add one sphere+pole per unique person
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


# ═══════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════

def main():
    print("\n" + "="*62)
    print("  MiDaS Room Scanner + Human Detection")
    print("="*62)

    model, transform, device = load_midas()
    yolo_model = load_yolo()
    frames = capture_frames(model, transform, device)
    if not frames:
        return

    pcd, mesh, people = build_scene(frames, yolo_model, imu_data=None)

    mesh = mesh + build_floor()
    mesh.compute_vertex_normals()

    o3d.io.write_triangle_mesh("room_humans.ply", mesh)
    o3d.io.write_point_cloud("room_humans_cloud.ply", pcd)
    print(f"\n[Saved] room_humans.ply -> {os.path.abspath('.')}")

    view_scene(mesh, pcd, people)


if __name__ == "__main__":
    main()


# ═══════════════════════════════════════════════════════════════
#  TUNING
#
#  Too many duplicate person labels?
#    Lower  REID_APPEARANCE_THRESH  (try 0.70)
#    Lower  SAME_PERSON_DIST        (try 0.8)
#
#  Same person split into two labels?
#    Raise  REID_APPEARANCE_THRESH  (try 0.90)
#    Raise  SAME_PERSON_DIST        (try 2.0)
#
#  Missing detections?
#    Lower  YOLO_CONF               (try 0.25)
#    Swap   YOLO_MODEL to "yolov8m.pt" for better accuracy
#
#  Spheres in wrong position?
#    The depth estimate for fast-moving people will be off.
#    For stationary people it should land close to their feet.
#
#  RELOAD:
#    import open3d as o3d
#    m = o3d.io.read_triangle_mesh("room_humans.ply")
#    m.compute_vertex_normals()
#    o3d.visualization.draw_geometries([m])
# ═══════════════════════════════════════════════════════════════
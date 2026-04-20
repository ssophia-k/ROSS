"""3-D reconstruction: poses, point clouds, ICP registration, Poisson mesh."""

import time

import cv2
import numpy as np
import open3d as o3d
from scipy.ndimage import median_filter

from ross.slam.vision import (
    PersonTracker,
    appearance_embedding,
    detect_people,
)

TOTAL_YAW_DEGREES = 360.0

MAX_PTS_PER_FRAME = 100_000
DEPTH_SCALE = 4.0
DEPTH_SMOOTH_K = 5

ICP_MAX_DIST = 0.35
ICP_MAX_ITER = 80
USE_ICP_REFINEMENT = True

STAT_NB_NEIGHBORS = 25
STAT_STD_RATIO = 1.8
RADIUS_RADIUS = 0.06
RADIUS_MIN_PTS = 6

POISSON_DEPTH = 9
POISSON_DENSITY_PCT = 5

VOXEL_ICP = 0.025
VOXEL_FINAL = 0.018

FLOOR_Y = 2.5
FLOOR_SIZE = 12.0

LABEL_SPHERE_RADIUS = 0.12
LABEL_SPHERE_COLOR = [0.95, 0.35, 0.20]
LABEL_SPHERE_STEPS = 12


def yaw_matrix(degrees: float) -> np.ndarray:
    theta = np.radians(degrees)
    return np.array(
        [
            [np.cos(theta), 0, np.sin(theta), 0],
            [0, 1, 0, 0],
            [-np.sin(theta), 0, np.cos(theta), 0],
            [0, 0, 0, 1],
        ],
        dtype=np.float64,
    )


def accel_to_gravity_rotation(accel) -> np.ndarray:
    """4x4 transform that aligns measured gravity with world -Y.

    Assumes the robot is not accelerating significantly, so accel ≈ gravity.
    """
    g = np.array(accel, dtype=np.float64)
    norm = np.linalg.norm(g)
    if norm < 1e-6:
        return np.eye(4)
    g = g / norm

    world_down = np.array([0.0, 1.0, 0.0])
    axis = np.cross(g, world_down)
    sin_angle = np.linalg.norm(axis)
    cos_angle = np.dot(g, world_down)

    if sin_angle < 1e-6:
        return np.eye(4)

    axis = axis / sin_angle
    K = np.array(
        [
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0],
        ]
    )
    R = np.eye(3) + sin_angle * K + (1 - cos_angle) * (K @ K)

    T = np.eye(4)
    T[:3, :3] = R
    return T


def get_frame_transform(i: int, n: int, imu_readings=None) -> np.ndarray:
    """Pose for frame i. Combines yaw sweep with IMU gravity tilt if available."""
    yaw = yaw_matrix((i / n) * TOTAL_YAW_DEGREES)
    if imu_readings is not None and i < len(imu_readings):
        reading = imu_readings[i]
        if reading is not None:
            tilt = accel_to_gravity_rotation(reading["accel"])
            return tilt @ yaw
    return yaw


def person_to_3d(
    person: dict,
    depth_map: np.ndarray,
    frame_w: int,
    frame_h: int,
    global_depth_scale: float | None,
    transform_4x4: np.ndarray,
) -> tuple[float, float, float]:
    cx_px = np.clip(person["cx"], 0, frame_w - 1)
    cy_px = np.clip(person["cy"], 0, frame_h - 1)
    focal = max(frame_w, frame_h) * 0.9
    px_cx, px_cy = frame_w / 2.0, frame_h / 2.0

    r = 8
    patch = depth_map[
        max(0, cy_px - r) : min(frame_h, cy_px + r),
        max(0, cx_px - r) : min(frame_w, cx_px + r),
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


def frame_to_pcd(
    frame_bgr: np.ndarray,
    depth_map: np.ndarray,
    global_depth_scale: float | None = None,
) -> tuple[o3d.geometry.PointCloud, float]:
    """Back-project a depth map to a coloured point cloud in camera space."""
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

    pts = np.stack([X, Y, Z], axis=-1).reshape(-1, 3)
    colors = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB).reshape(-1, 3) / 255.0

    if len(pts) > MAX_PTS_PER_FRAME:
        idx = np.random.choice(len(pts), MAX_PTS_PER_FRAME, replace=False)
        pts, colors = pts[idx], colors[idx]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
    return pcd, float(mean_d)


def build_scene(frame_data, yolo_model, imu_readings=None):
    """Merge per-frame point clouds via ICP and return (cloud, mesh, people)."""
    n = len(frame_data)
    global_scale = float(np.mean([d.mean() for _, d in frame_data]))
    print(f"[Build] {n} frames | scale: {global_scale:.4f}")

    merged = o3d.geometry.PointCloud()
    tracker = PersonTracker()

    for i, (frame, depth) in enumerate(frame_data):
        T = get_frame_transform(i, n, imu_readings)
        angle = (i / n) * TOTAL_YAW_DEGREES
        print(f"  Frame {i + 1}/{n} ({angle:.0f} deg)", end="  ", flush=True)
        t0 = time.time()

        h_f, w_f = frame.shape[:2]
        pcd, _ = frame_to_pcd(frame, depth, global_scale)
        pcd.transform(T)

        if USE_ICP_REFINEMENT and i > 0 and len(merged.points) > 100:
            pcd_ds = pcd.voxel_down_sample(VOXEL_ICP)
            merged_ds = merged.voxel_down_sample(VOXEL_ICP)
            pcd_ds.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=30)
            )
            merged_ds.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=30)
            )
            result = o3d.pipelines.registration.registration_icp(
                pcd_ds,
                merged_ds,
                ICP_MAX_DIST,
                np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=ICP_MAX_ITER
                ),
            )
            if result.fitness > 0.25:
                pcd.transform(result.transformation)
                T = result.transformation @ T
                tag = f"ICP={result.fitness:.2f}"
            else:
                tag = f"no ICP ({result.fitness:.2f})"
        else:
            tag = "pose only"

        merged += pcd

        people = detect_people(frame, yolo_model)
        for p in people:
            pos3d = person_to_3d(p, depth, w_f, h_f, global_scale, T)
            emb = appearance_embedding(frame, p)
            tracker.update(pos3d, emb)

        det_str = f"  {len(people)} person(s)" if people else ""
        print(f"{tag}{det_str}  {time.time() - t0:.1f}s")

    print("[Build] Downsampling + denoising ...")
    merged = merged.voxel_down_sample(VOXEL_FINAL)
    merged, _ = merged.remove_statistical_outlier(
        nb_neighbors=STAT_NB_NEIGHBORS, std_ratio=STAT_STD_RATIO
    )
    merged, _ = merged.remove_radius_outlier(
        nb_points=RADIUS_MIN_PTS, radius=RADIUS_RADIUS
    )
    print(f"[Build] Clean cloud: {len(merged.points):,} points")

    print("[Build] Estimating normals ...")
    merged.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=30)
    )
    centre = np.asarray(merged.points).mean(axis=0)
    merged.orient_normals_towards_camera_location(centre)

    print(f"[Build] Poisson mesh (depth={POISSON_DEPTH}) ...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        merged, depth=POISSON_DEPTH
    )
    densities = np.asarray(densities)
    mesh.remove_vertices_by_mask(
        densities < np.percentile(densities, POISSON_DENSITY_PCT)
    )
    mesh.compute_vertex_normals()
    print(f"[Build] Mesh: {len(mesh.vertices):,} verts  {len(mesh.triangles):,} tris")

    people_found = tracker.all_people()
    if people_found:
        print(f"\n[People] {len(people_found)} unique person(s):")
        for p in people_found:
            pos = p["pos3d"]
            print(
                f"  {p['label']:12s}  ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
                f"  seen {p['count']}x"
            )

    return merged, mesh, people_found


def build_floor() -> o3d.geometry.TriangleMesh:
    slab = o3d.geometry.TriangleMesh.create_box(
        width=FLOOR_SIZE * 2, height=0.03, depth=FLOOR_SIZE * 2
    )
    slab.translate([-FLOOR_SIZE, FLOOR_Y, -FLOOR_SIZE * 0.15])
    slab.paint_uniform_color([0.14, 0.15, 0.19])
    slab.compute_vertex_normals()
    return slab


def make_person_label(person_dict: dict) -> list[o3d.geometry.TriangleMesh]:
    pos = np.array(person_dict["pos3d"])
    geometries = []

    sphere = o3d.geometry.TriangleMesh.create_sphere(
        radius=LABEL_SPHERE_RADIUS, resolution=LABEL_SPHERE_STEPS
    )
    sphere.translate(pos)
    sphere.paint_uniform_color(LABEL_SPHERE_COLOR)
    sphere.compute_vertex_normals()
    geometries.append(sphere)

    pole_bot = np.array([pos[0], FLOOR_Y - 0.03, pos[2]])
    if pole_bot[1] < pos[1]:
        pole = o3d.geometry.TriangleMesh.create_cylinder(
            radius=0.015, height=float(abs(pole_bot[1] - pos[1]))
        )
        pole.translate((pos + pole_bot) / 2.0)
        pole.paint_uniform_color([0.8, 0.8, 0.8])
        pole.compute_vertex_normals()
        geometries.append(pole)

    return geometries


def view_scene(mesh, pcd, people, mesh_path=None) -> None:
    """Open an Open3D viewer. Falls back to a hint if GL init fails."""
    print("\n[Viewer] Left-drag=rotate  Right-drag=pan  Scroll=zoom  Q=close")
    if people:
        print("  Person legend (red spheres):")
        for p in people:
            pos = p["pos3d"]
            print(
                f"    {p['label']:12s}  ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})"
                f"  seen {p['count']}x"
            )
    print()

    vis = o3d.visualization.Visualizer()
    ok = vis.create_window(window_name="ROSS Room Scan", width=1440, height=900)
    opt = vis.get_render_option()
    if not ok or opt is None:
        print(
            "[Viewer] Open3D could not create a GL window "
            "(Wayland/GLEW init failed)."
        )
        print("         Scan files are saved — open them with:")
        hint_path = mesh_path or "<scan>.ply"
        print(
            "           python -c \"import open3d as o3d; "
            f"o3d.visualization.draw_geometries(["
            f"o3d.io.read_triangle_mesh('{hint_path}')])\""
        )
        print("         or any other PLY viewer (MeshLab, CloudCompare).")
        try:
            vis.destroy_window()
        except Exception:
            pass
        return

    vis.add_geometry(mesh)
    if pcd is not None:
        vis.add_geometry(pcd)
    for p in people:
        for geom in make_person_label(p):
            vis.add_geometry(geom)

    opt.background_color = np.array([0.04, 0.05, 0.08])
    opt.mesh_show_back_face = True
    opt.light_on = True
    opt.point_size = 1.5

    ctr = vis.get_view_control()
    ctr.set_zoom(0.5)
    ctr.set_front([0.0, -0.3, -1.0])
    ctr.set_lookat([0.0, 0.5, 3.0])
    ctr.set_up([0.0, -1.0, 0.0])

    vis.run()
    vis.destroy_window()

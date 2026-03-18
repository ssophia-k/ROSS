"""
Step 3 — TSDF reconstruction using Open3D.
Uses webcam intrinsics approximated from image size.
Swap in calibrated K matrix later for better accuracy.
"""

import open3d as o3d
import numpy as np
import glob
import os

os.makedirs("output", exist_ok=True)

# ── Intrinsics: approximate for Surface webcam at 640×480 ──────────────────
# Once calibrated, replace with:
#   K = np.load("surface_K.npy")
#   intrinsic = o3d.camera.PinholeCameraIntrinsic(640, 480, K[0,0], K[1,1], K[0,2], K[1,2])

W, H = 640, 480
F = 525.0
intrinsic = o3d.camera.PinholeCameraIntrinsic(W, H, F, F, W / 2, H / 2)

# ── Depth encoding from 16-bit PNG ─────────────────────────────────────────
DEPTH_SCALE = 65535.0 / 4.0
DEPTH_TRUNC = 4.0
VOXEL_SIZE  = 0.05

color_files = sorted(glob.glob("data/color/*.jpg"))
depth_files = sorted(glob.glob("data/depth/*.png"))

assert len(color_files) > 0, "No color frames found in data/color/"
assert len(depth_files) > 0, "No depth maps found in data/depth/"
assert len(color_files) == len(depth_files), \
    f"Mismatch: {len(color_files)} color vs {len(depth_files)} depth frames"

print(f"Reconstructing from {len(color_files)} frames...\n")

# ── Helper: load RGB-D for odometry (grayscale intensity) ──────────────────
def load_rgbd_for_odometry(color_path, depth_path):
    color = o3d.io.read_image(color_path)
    depth = o3d.io.read_image(depth_path)
    return o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth,
        depth_scale=DEPTH_SCALE,
        depth_trunc=DEPTH_TRUNC,
        convert_rgb_to_intensity=True    # grayscale for photometric term
    )

# ── Helper: load RGB-D for integration (full color) ───────────────────────
def load_rgbd_for_integration(color_path, depth_path):
    color = o3d.io.read_image(color_path)
    depth = o3d.io.read_image(depth_path)
    return o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth,
        depth_scale=DEPTH_SCALE,
        depth_trunc=DEPTH_TRUNC,
        convert_rgb_to_intensity=False   # full color for final output
    )

# ── Build pose graph via frame-to-frame RGB-D odometry ─────────────────────
pose_graph = o3d.pipelines.registration.PoseGraph()
odometry   = np.identity(4)
pose_graph.nodes.append(
    o3d.pipelines.registration.PoseGraphNode(odometry)
)

failed = 0
for i in range(len(color_files) - 1):
    rgbd_src = load_rgbd_for_odometry(color_files[i],     depth_files[i])
    rgbd_dst = load_rgbd_for_odometry(color_files[i + 1], depth_files[i + 1])

    '''
        HOW THIS WORKS:
            IN:
                - rgbd_src: RGB image
                - rgbd_dst: RGB image
                - intrinsic: intrinsic camera matrix
                - np.identity(4): intial guess
                - JacobianFromHybrid: error correction function
                - OdometryOption: when to stop
            OUT:
                - success: is the approx transformation trustworthy?
                - trans: 4x4 approx rotation + transformation matrix
                - info: 6x6 confidence matrix used by optimizer
            OPERATION:
                - approx transformation matrix using source and destination frames
                1. 2D point in source frame =depth+intrinsic (unproject)=> 3D point in WCF
                2. 3D point WCF =guess transform=> 3D point in dst camera's perspective
                3. 3D point in dst space =intrinsic (project)=> 2D point in dst frame
                4. computes the error in brightness and depth
                5. computes jacobian (#pixels x 6) matrix
                6. 
    '''

    success, trans, info = o3d.pipelines.odometry.compute_rgbd_odometry(
        rgbd_src, rgbd_dst, intrinsic,
        np.identity(4),
        o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(),
        o3d.pipelines.odometry.OdometryOption()
    )

    if success:
        odometry = trans @ odometry
        uncertain = False
    else:
        failed += 1
        uncertain = True
        trans = np.identity(4)

    # odometry takes a point in world and gives you point in image
    # NEED point from image as a point in world --> invert odometry
    pose_graph.nodes.append(
        o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry))
    )

    # the transform between two frames
    pose_graph.edges.append(
        o3d.pipelines.registration.PoseGraphEdge(
            i, i + 1, trans, info, uncertain=uncertain
        )
    )

    print(f"  Frame {i+1:03d}/{len(color_files)-1}  "
          f"{'OK' if success else 'FAILED (using identity)'}")

print(f"\nOdometry: {len(color_files)-1-failed} OK, {failed} failed")


# ── Global pose graph optimization ─────────────────────────────────────────
# Fixes drift by distributing correction across the nodes

print("\nOptimizing pose graph...")
o3d.pipelines.registration.global_optimization(
    pose_graph,
    o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
    o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
    o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=0.03,
        edge_prune_threshold=0.25,
        preference_loop_closure=0.1,
        reference_node=0
    )
)

# ── TSDF integration ────────────────────────────────────────────────────────
print("\nIntegrating frames into TSDF volume...")
volume = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=VOXEL_SIZE,
    sdf_trunc=0.04,
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
)

for i in range(len(color_files)):
    rgbd = load_rgbd_for_integration(color_files[i], depth_files[i])
    pose = pose_graph.nodes[i].pose
    volume.integrate(rgbd, intrinsic, np.linalg.inv(pose))
    print(f"  Integrated {i+1}/{len(color_files)}", end="\r")

print("\n")

# ── Extract and clean point cloud ───────────────────────────────────────────
print("Extracting point cloud...")
pcd = volume.extract_point_cloud()
if len(pcd.points) == 0:
    print("  WARNING: Point cloud is empty — check depth maps and odometry results")
else:
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    o3d.io.write_point_cloud("output/scene_360.ply", pcd)
    print(f"  Saved → output/scene_360.ply  ({len(pcd.points)} points)")

# ── Extract and clean mesh ──────────────────────────────────────────────────
print("Extracting mesh...")
mesh = volume.extract_triangle_mesh()
if len(mesh.vertices) == 0:
    print("  WARNING: Mesh is empty — check depth maps and odometry results")
else:
    mesh.compute_vertex_normals()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    o3d.io.write_triangle_mesh("output/scene_mesh.ply", mesh)
    print(f"  Saved → output/scene_mesh.ply  ({len(mesh.vertices)} vertices)")

print("\nDone! Run  python visualize.py  to view the result.")
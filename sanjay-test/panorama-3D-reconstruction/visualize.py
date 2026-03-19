"""
Step 4 — View the reconstructed scene.
Usage:
  python visualize.py                        # loads mesh by default
  python visualize.py output/scene_360.ply   # loads point cloud
"""

# DOESNT WORK ON SURFACE (SNAPDRAGON) BC OF SUPPORT ISSUES
# OPEN IN 3D VIEWER APP INSTEAD

import open3d as o3d
import sys
import os

target = sys.argv[1] if len(sys.argv) > 1 else "output/scene_mesh.ply"

if not os.path.exists(target):
    print(f"File not found: {target}")
    print("Available outputs:")
    for f in os.listdir("output"):
        print(f"  output/{f}")
    sys.exit(1)

print(f"Loading {target}...")
geo = o3d.io.read_triangle_mesh(target)

if geo.is_empty():
    print("  Not a mesh or mesh is empty, trying as point cloud...")
    geo = o3d.io.read_point_cloud(target)

if geo.is_empty():
    print("  ERROR: Could not load geometry from file.")
    sys.exit(1)

if hasattr(geo, "compute_vertex_normals"):
    geo.compute_vertex_normals()

print(f"Loaded successfully.")
if hasattr(geo, "vertices"):
    print(f"  Vertices : {len(geo.vertices)}")
    print(f"  Triangles: {len(geo.triangles)}")
else:
    print(f"  Points: {len(geo.points)}")

print("\nControls:")
print("  Left drag   = rotate")
print("  Right drag  = pan")
print("  Scroll      = zoom")
print("  R           = reset view")
print("  Q / Esc     = quit")

o3d.visualization.draw_geometries(
    [geo],
    window_name="360° Scene Reconstruction",
    width=1280,
    height=720,
    mesh_show_back_face=True
)
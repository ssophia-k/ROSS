import open3d as o3d

# Load the PLY file
mesh = o3d.io.read_triangle_mesh("room_360.ply")

# If it's a point cloud instead of a mesh
pcd = o3d.io.read_point_cloud("room_360_cloud.ply")

# Visualize
o3d.visualization.draw_geometries([mesh])   # for mesh
# or
o3d.visualization.draw_geometries([pcd])    # for point cloud
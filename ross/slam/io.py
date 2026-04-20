"""PLY save/load helpers."""

from pathlib import Path

import open3d as o3d


def save_scan(
    output_prefix: Path,
    mesh: o3d.geometry.TriangleMesh,
    pcd: o3d.geometry.PointCloud,
) -> tuple[Path, Path]:
    """Write `<prefix>.ply` (mesh) and `<prefix>_cloud.ply` (point cloud).

    Returns the two paths. Parent directory is created if needed.
    """
    prefix = Path(output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)

    mesh_path = prefix.with_suffix(".ply")
    cloud_path = prefix.with_name(prefix.stem + "_cloud.ply")

    o3d.io.write_triangle_mesh(str(mesh_path), mesh)
    o3d.io.write_point_cloud(str(cloud_path), pcd)
    return mesh_path, cloud_path

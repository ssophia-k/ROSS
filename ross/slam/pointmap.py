"""
3-D map management and visualisation.

Replaces the original Pangolin-based viewer with an **Open3D** non-blocking
visualiser running in a background process (multiprocessing).  If Open3D is
not installed, a headless stub is provided so the SLAM pipeline can still run
without 3-D rendering.

Classes
-------
Map   – Global SLAM map (frames + 3-D points) with a 3-D viewer process.
Point – A single 3-D map point observed across multiple frames.
"""

from __future__ import annotations

import multiprocessing as mp
from multiprocessing import Process, Queue
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from ross.slam.extractor import Frame


# ---------------------------------------------------------------------------
# Point class
# ---------------------------------------------------------------------------

class Point:
    """A 3-D point in the world, observed in multiple frames.

    Parameters
    ----------
    mapp : Map
        The global map this point belongs to.
    loc : np.ndarray
        Homogeneous 3-D coordinates [X, Y, Z, W].
    """

    def __init__(self, mapp: "Map", loc: np.ndarray):
        self.frames: list[Frame] = []
        self.pt = loc  # homogeneous 4-vec or Euclidean 3-vec
        self.idxs: list[int] = []

        self.id = len(mapp.points)
        mapp.points.append(self)

    def add_observation(self, frame: "Frame", idx: int) -> None:
        """Record that this point was seen in *frame* at feature index *idx*."""
        self.frames.append(frame)
        self.idxs.append(idx)


# ---------------------------------------------------------------------------
# Open3D viewer process
# ---------------------------------------------------------------------------

def _viewer_process(q: Queue, w: int = 1280, h: int = 720) -> None:  # noqa: C901
    """Run in a separate process – reads (poses, points) from *q* and draws them.

    Parameters
    ----------
    q : Queue
        Receives tuples ``(poses_array, pts_array)`` from the main process.
    w, h : int
        Initial viewer window size.
    """
    try:
        import open3d as o3d  # noqa: F811
    except ImportError:
        # If Open3D is missing, just drain the queue silently.
        while True:
            q.get()

    vis = o3d.visualization.Visualizer()  # type: ignore[attr-defined]
    vis.create_window(window_name="ROSS – 3-D Map", width=w, height=h)

    # Render options
    opt = vis.get_render_option()
    opt.background_color = np.array([1, 1, 1])  # white background
    opt.point_size = 2.0

    # Persistent geometries we update each tick
    pcd = o3d.geometry.PointCloud()
    cam_lines = o3d.geometry.LineSet()  # thin lines connecting camera centres
    vis.add_geometry(pcd)
    vis.add_geometry(cam_lines)

    # Keep a list of camera-frustum LineSet objects
    cam_frustums: list[o3d.geometry.LineSet] = []

    state = None
    view_initialised = False  # becomes True after first non-empty update

    while True:
        # Drain queue, keep only the latest state
        while not q.empty():
            state = q.get()

        if state is not None:
            poses, pts = state

            # --- Update point cloud ----------------------------------------
            if pts.shape[0] > 0:
                pcd.points = o3d.utility.Vector3dVector(pts[:, :3])
                pcd.paint_uniform_color([1, 0, 0])  # red points
            else:
                pcd.points = o3d.utility.Vector3dVector(np.empty((0, 3)))

            vis.update_geometry(pcd)

            # Refit the camera the first time we have real geometry so the
            # viewer doesn't stay locked to the initial empty bounding box.
            if not view_initialised and pts.shape[0] > 0:
                vis.reset_view_point(True)
                view_initialised = True

            # --- Camera trajectory line ------------------------------------
            if poses.shape[0] >= 2:
                centres = []
                for pose in poses:
                    # Camera centre = -R^T · t  (or just the translation col)
                    centres.append(pose[:3, 3])
                centres = np.array(centres)
                lines = [[i, i + 1] for i in range(len(centres) - 1)]
                cam_lines.points = o3d.utility.Vector3dVector(centres)
                cam_lines.lines = o3d.utility.Vector2iVector(lines)
                cam_lines.paint_uniform_color([0, 0.8, 0])  # green trajectory
            vis.update_geometry(cam_lines)

            # --- Camera frustums -------------------------------------------
            # Remove old frustums
            for ls in cam_frustums:
                vis.remove_geometry(ls, reset_bounding_box=False)
            cam_frustums.clear()

            # Draw a small frustum for the last few cameras
            n_draw = min(len(poses), 20)
            for pose in poses[-n_draw:]:
                ls = _make_frustum(pose, size=0.3)
                cam_frustums.append(ls)
                vis.add_geometry(ls, reset_bounding_box=False)

            state = None  # consumed

        # Keep the window alive
        if not vis.poll_events():
            break
        vis.update_renderer()


def _make_frustum(pose: np.ndarray, size: float = 0.5):
    """Build a small camera-frustum LineSet for a given 4×4 pose matrix."""
    import open3d as o3d

    # Camera centre
    c = pose[:3, 3]
    R = pose[:3, :3]

    # Four corners of the image plane (in camera coords) 
    # mapped to world coords via the pose
    s = size
    corners_cam = np.array([
        [-s, -s, s * 1.5],
        [s, -s, s * 1.5],
        [s, s, s * 1.5],
        [-s, s, s * 1.5],
    ])

    corners_world = (R @ corners_cam.T).T + c

    points = np.vstack([c, corners_world])  # 0=centre, 1-4=corners
    lines = [
        [0, 1], [0, 2], [0, 3], [0, 4],  # centre → corners
        [1, 2], [2, 3], [3, 4], [4, 1],  # rectangle
    ]

    ls = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    ls.paint_uniform_color([0, 0, 1])  # blue frustums
    return ls


# ---------------------------------------------------------------------------
# Map class
# ---------------------------------------------------------------------------

class Map:
    """Global SLAM map holding camera frames and 3-D points.

    Call :meth:`create_viewer` once to spawn the Open3D viewer process, then
    call :meth:`display` after each frame to push the latest state.
    """

    def __init__(self):
        self.frames: list[Frame] = []
        self.points: list[Point] = []
        self._q: Queue | None = None

    # -- viewer control -----------------------------------------------------

    def create_viewer(self) -> None:
        """Spawn the background Open3D visualisation process."""
        # Use 'spawn' context so forked processes don't inherit OpenGL state
        ctx = mp.get_context("spawn")
        self._q = ctx.Queue()
        p = ctx.Process(target=_viewer_process, args=(self._q,), daemon=True)
        p.start()

    def display(self) -> None:
        """Push the current poses and map points to the viewer."""
        if self._q is None:
            return

        poses = np.array([f.pose for f in self.frames])
        if len(self.points) > 0:
            pts = np.array([p.pt[:3] for p in self.points])
        else:
            pts = np.empty((0, 3))

        self._q.put((poses, pts))

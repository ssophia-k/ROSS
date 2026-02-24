"""
Feature extraction, matching, pose estimation, and the Frame class.

This module implements the front-end algorithms of monocular Visual SLAM:
  1. ORB feature extraction via goodFeaturesToTrack + ORB descriptors
  2. Feature matching via BFMatcher with Lowe's ratio test
  3. RANSAC-based fundamental matrix estimation (scikit-image)
  4. Pose extraction (R, t) from the fundamental matrix via SVD
  5. Triangulation of 3-D points from two views via linear SVD
"""

import cv2
import numpy as np
from skimage.measure import ransac
from skimage.transform import FundamentalMatrixTransform

# Identity 4×4 pose – used as the initial camera pose
IRt = np.eye(4)


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

def add_ones(x: np.ndarray) -> np.ndarray:
    """Convert points to homogeneous coordinates by appending a column of 1s.

    Parameters
    ----------
    x : np.ndarray, shape (N, D)
        Input points.

    Returns
    -------
    np.ndarray, shape (N, D+1)
        Homogeneous points.
    """
    return np.concatenate([x, np.ones((x.shape[0], 1))], axis=1)


def normalize(Kinv: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Transform pixel coordinates to normalised image coordinates.

    The inverse intrinsic matrix K⁻¹ maps 2-D homogeneous pixel points to
    normalised coordinates centred on the principal point and scaled by the
    focal lengths.

    Parameters
    ----------
    Kinv : np.ndarray, shape (3, 3)
        Inverse of the camera intrinsic matrix.
    pts : np.ndarray, shape (N, 2)
        Pixel coordinates.

    Returns
    -------
    np.ndarray, shape (N, 2)
        Normalised image coordinates.
    """
    return np.dot(Kinv, add_ones(pts).T).T[:, 0:2]


def denormalize(K: np.ndarray, pt: np.ndarray) -> tuple[int, int]:
    """Convert a normalised point back to integer pixel coordinates.

    Parameters
    ----------
    K : np.ndarray, shape (3, 3)
        Camera intrinsic matrix.
    pt : np.ndarray
        Normalised 2-D point (x, y).

    Returns
    -------
    tuple[int, int]
        Pixel coordinates (u, v).
    """
    ret = np.dot(K, [pt[0], pt[1], 1.0])
    ret /= ret[2]
    return int(round(ret[0])), int(round(ret[1]))


# ---------------------------------------------------------------------------
# Feature extraction
# ---------------------------------------------------------------------------

def extract(img: np.ndarray, max_corners: int = 3000) -> tuple[np.ndarray, np.ndarray | None]:
    """Detect keypoints with goodFeaturesToTrack and compute ORB descriptors.

    Parameters
    ----------
    img : np.ndarray
        BGR or grayscale image.
    max_corners : int
        Maximum number of corners to detect.

    Returns
    -------
    tuple[np.ndarray, np.ndarray]
        (keypoint_coords, descriptors)
        keypoint_coords has shape (N, 2), descriptors has shape (N, 32).
    """
    orb = cv2.ORB_create()

    # Convert to grayscale for corner detection
    gray = np.mean(img, axis=-1).astype(np.uint8) if img.ndim == 3 else img

    # Detect corners using Shi-Tomasi (goodFeaturesToTrack)
    pts = cv2.goodFeaturesToTrack(
        gray,
        max_corners,
        qualityLevel=0.01,
        minDistance=7,
    )

    if pts is None:
        return np.empty((0, 2)), None

    # Convert to ORB KeyPoints and compute descriptors
    kps = [cv2.KeyPoint(x=f[0][0], y=f[0][1], size=20) for f in pts]
    kps, des = orb.compute(img, kps)

    if des is None:
        return np.empty((0, 2)), None

    return np.array([(kp.pt[0], kp.pt[1]) for kp in kps]), des


# ---------------------------------------------------------------------------
# Pose extraction from the Fundamental matrix
# ---------------------------------------------------------------------------

def extract_pose(F: np.ndarray) -> np.ndarray:
    """Decompose a Fundamental matrix into a 4×4 [R|t] transformation via SVD.

    Parameters
    ----------
    F : np.ndarray, shape (3, 3)
        Fundamental (or Essential) matrix.

    Returns
    -------
    np.ndarray, shape (4, 4)
        Homogeneous transformation matrix [R | t].
    """
    W = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)

    # SVD of F
    U, d, Vt = np.linalg.svd(F)

    # Ensure proper rotation (det > 0)
    if np.linalg.det(U) < 0:
        U *= -1
    if np.linalg.det(Vt) < 0:
        Vt *= -1

    # Compute rotation R = U · W · Vt
    R = U @ W @ Vt

    # If the trace is negative, use W^T instead
    if np.sum(R.diagonal()) < 0:
        R = U @ W.T @ Vt

    # Translation is the last column of U
    t = U[:, 2]

    # Build the 4×4 homogeneous pose
    ret = np.eye(4)
    ret[:3, :3] = R
    ret[:3, 3] = t
    return ret


# ---------------------------------------------------------------------------
# Feature matching between two frames
# ---------------------------------------------------------------------------

def match_frames(f1, f2) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Match features between two Frame objects and estimate relative pose.

    Uses BFMatcher with Lowe's ratio test, an additional distance check in
    normalised coordinates, and RANSAC to robustly fit a Fundamental matrix.

    Parameters
    ----------
    f1, f2 : Frame
        Two consecutive frames.

    Returns
    -------
    tuple[np.ndarray, np.ndarray, np.ndarray]
        (idx1, idx2, Rt) – inlier index arrays and the 4×4 relative pose.
    """
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    matches = bf.knnMatch(f1.des, f2.des, k=2)

    ret = []
    idx1, idx2 = [], []

    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            p1 = f1.pts[m.queryIdx]
            p2 = f2.pts[m.trainIdx]

            # Additional distance test in normalised coordinates
            if np.linalg.norm(p1 - p2) < 0.1:
                idx1.append(m.queryIdx)
                idx2.append(m.trainIdx)
                ret.append((p1, p2))

    assert len(ret) >= 8, f"Not enough matches ({len(ret)}), need ≥ 8"

    ret = np.array(ret)
    idx1 = np.array(idx1)
    idx2 = np.array(idx2)

    # Robustly fit a Fundamental matrix with RANSAC
    model, inliers = ransac(
        (ret[:, 0], ret[:, 1]),
        FundamentalMatrixTransform,
        min_samples=8,
        residual_threshold=0.001,
        max_trials=200,
    )

    # Keep only inliers
    Rt = extract_pose(model.params)
    return idx1[inliers], idx2[inliers], Rt


# ---------------------------------------------------------------------------
# Triangulation
# ---------------------------------------------------------------------------

def triangulate(
    pose1: np.ndarray,
    pose2: np.ndarray,
    pts1: np.ndarray,
    pts2: np.ndarray,
) -> np.ndarray:
    """Triangulate 3-D points from two camera poses and corresponding 2-D points.

    Uses the linear SVD method (DLT) to solve for each 3-D point in
    homogeneous coordinates.

    Parameters
    ----------
    pose1, pose2 : np.ndarray, shape (4, 4)
        Camera-to-world transformation matrices.
    pts1, pts2 : np.ndarray, shape (N, 2)
        Normalised 2-D point correspondences.

    Returns
    -------
    np.ndarray, shape (N, 4)
        Triangulated 3-D points in homogeneous coordinates [X, Y, Z, W].
    """
    ret = np.zeros((pts1.shape[0], 4))

    # Invert poses: camera-to-world → world-to-camera (projection matrices)
    proj1 = np.linalg.inv(pose1)
    proj2 = np.linalg.inv(pose2)

    for i, (p1, p2) in enumerate(zip(add_ones(pts1), add_ones(pts2))):
        A = np.zeros((4, 4))
        A[0] = p1[0] * proj1[2] - proj1[0]
        A[1] = p1[1] * proj1[2] - proj1[1]
        A[2] = p2[0] * proj2[2] - proj2[0]
        A[3] = p2[1] * proj2[2] - proj2[1]

        # The solution is the right singular vector for the smallest σ
        _, _, vt = np.linalg.svd(A)
        ret[i] = vt[3]

    return ret


# ---------------------------------------------------------------------------
# Frame class
# ---------------------------------------------------------------------------

class Frame:
    """Represents a single camera frame in the SLAM system.

    On construction the frame:
      - extracts ORB features from the image
      - normalises pixel coordinates using K⁻¹
      - registers itself with the Map

    Attributes
    ----------
    K : np.ndarray
        Intrinsic camera matrix (3×3).
    Kinv : np.ndarray
        Inverse intrinsic matrix.
    pose : np.ndarray
        4×4 camera pose in world frame.
    id : int
        Sequential frame identifier.
    pts : np.ndarray
        Normalised keypoint coordinates (N, 2).
    des : np.ndarray
        ORB descriptors (N, 32).
    """

    def __init__(self, mapp, img: np.ndarray, K: np.ndarray):
        self.K = K
        self.Kinv = np.linalg.inv(K)
        self.pose = np.copy(IRt)

        self.id = len(mapp.frames)
        mapp.frames.append(self)

        # Extract features and normalise to camera coordinates
        pts, self.des = extract(img)
        self.pts = normalize(self.Kinv, pts)

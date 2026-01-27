"""
Camera calibration using a chessboard pattern.

Requires:
    pip install opencv-python numpy

Example:
    python calibrate.py --images "calib/*.jpg" \
                        --pattern_width 9 \
                        --pattern_height 6 \
                        --square_size 0.022
"""

import argparse
import glob

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Chessboard camera calibration (OpenCV-style sample)."
    )
    parser.add_argument(
        "--images",
        default="develop/calib/*.JPG",
        help="Glob pattern for calibration images (default: calib/*.jpg)",
    )
    parser.add_argument(
        "--pattern_width",
        type=int,
        default=9,
        help="Number of inner corners along the checkerboard width (default: 9)",
    )
    parser.add_argument(
        "--pattern_height",
        type=int,
        default=6,
        help="Number of inner corners along the checkerboard height (default: 6)",
    )
    parser.add_argument(
        "--square_size",
        type=float,
        default=0.022,
        help="Size of one checkerboard square in meters (default: 0.022 for 22mm)",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # OpenCV-style variable names
    w = args.pattern_width     # number of inner corners along width
    h = args.pattern_height    # number of inner corners along height
    square_size = args.square_size

    # Termination criteria (same style as the sample)
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ... in real units
    objp = np.zeros((h * w, 3), np.float32)
    objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2)
    objp *= square_size

    # Arrays to store object points and image points from all images
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane

    images = glob.glob(args.images)
    if not images:
        raise SystemExit(f"No images found for pattern: {args.images}")

    print(f"Found {len(images)} images")
    pattern_size = (w, h)

    img_shape = None

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"[WARN] Could not read image: {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_shape is None:
            img_shape = gray.shape[::-1]  # (width, height)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                criteria,
            )
            imgpoints.append(corners2)

            # Draw and display the corners (as in the sample)
            cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
            cv2.imshow("img", img)
            cv2.waitKey(100)

            print(f"[OK] {fname}")
        else:
            print(f"[FAIL] Corners not found in: {fname}")

    cv2.destroyAllWindows()

    if len(objpoints) < 1:
        raise SystemExit("No valid calibration views found (no corners detected).")

    # Calibrate camera (same core call as the sample)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        img_shape,
        None,
        None,
    )

    print("\n=== Calibration Results ===")
    print(f"RMS reprojection error: {ret}")
    print("\nCamera matrix (mtx) [mm]:")
    print(mtx)
    print("\nDistortion coefficients (dist):")
    print(dist.ravel())

    # Compute mean reprojection error (same style as OpenCV docs)
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(
            objpoints[i], rvecs[i], tvecs[i], mtx, dist
        )
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    mean_error /= len(objpoints)
    print(f"\nTotal mean reprojection error: {mean_error}")


if __name__ == "__main__":
    main()

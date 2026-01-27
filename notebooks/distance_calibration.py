# calibration.py
"""
Focal-length calibration for red/green ball using a webcam.

Procedure:
1. Set CALIB_OBJECT_RADIUS_CM and TARGET_COLOR below.
2. Run:  python calibration.py
3. Place the ball at a known distance (cm) from the camera.
4. Press 'c' to capture a sample (you'll be prompted for the distance).
5. Repeat for several distances (5–10 samples is reasonable).
6. Press 'q' to quit; the script prints the estimated focal length
   and a ready-to-paste radius_to_distance_cm() for your main script.
"""

import cv2
import numpy as np

# Real radius of your calibration ball in centimeters
CALIB_OBJECT_RADIUS_CM = 5 

# Which color ball to calibrate on: "red" or "green"
TARGET_COLOR = "red"

# Minimum area (pixels) for a detection to be considered
# Ensures we pick the correct object to calibrate
# (gets rid of small flashes of red on screen)
MIN_AREA = 150

# HSV color ranges (match to main script)
RANGES = {
    "red": [
        ((0, 120, 70), (10, 255, 255)),
        ((170, 120, 70), (179, 255, 255)),
    ],
    "green": [
        ((35, 80, 60), (85, 255, 255)),
    ],
}


def build_mask(hsv, color):
    """Binary mask for the given color in HSV."""
    masks = [cv2.inRange(hsv, np.array(lo), np.array(hi))
             for lo, hi in RANGES[color]]
    mask = masks[0]
    for m in masks[1:]:
        mask = cv2.bitwise_or(mask, m)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def analyze(mask, color, min_area=MIN_AREA):
    """Return detections in color and shape (with min area)."""
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    out = []
    for c in contours:
        hull = cv2.convexHull(c)
        (cx_f, cy_f), radius = cv2.minEnclosingCircle(hull)
        circle_area = float(np.pi * radius * radius)
        if circle_area < min_area:
            continue
        center_int = (int(round(cx_f)), int(round(cy_f)))
        out.append({
            "color": color,
            "radius_px": float(radius),
            "center": center_int,
            "contour": hull,
        })
    return out


def annotate(frame, detections):
    """Draw detected target and some info."""
    vis = frame.copy()
    for d in detections:
        color_bgr = (0, 0, 255) if d["color"] == "red" else (0, 255, 0)
        cv2.drawContours(vis, [d["contour"]], -1, color_bgr, 2)
        cv2.circle(vis, d["center"], int(round(d["radius_px"])), color_bgr, 2)
        txt = f"{d['color']}: R={d['radius_px']:.1f}px"
        cv2.putText(
            vis,
            txt,
            (d["center"][0] + 6, d["center"][1] - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return vis


def main():
    calib_samples = []  # list of (Z_cm, r_px)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise SystemExit("Cannot open camera")

    print("Calibration mode")
    print("----------------")
    print(f"Using color: {TARGET_COLOR}")
    print(f"Calibration ball radius: {CALIB_OBJECT_RADIUS_CM} cm")
    print("Controls:")
    print("  q : quit and compute focal length")
    print("  c : capture sample (will prompt for distance in cm)")
    print()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Optional small blur to reduce noise (matches main pipeline if you add it)
        frame_blur = cv2.GaussianBlur(frame, (5, 5), 0)

        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)
        mask = build_mask(hsv, TARGET_COLOR)
        detections = analyze(mask, TARGET_COLOR)

        # For visualization, highlight all detections
        annotated = annotate(frame, detections)
        cv2.imshow("Calibration", annotated)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("c"):
            if not detections:
                print("No target detected; sample ignored.")
                continue

            # Use the largest detected blob as the calibration target
            largest = max(detections, key=lambda d: d["radius_px"])
            r_px = largest["radius_px"]
            print(f"Detected radius: {r_px:.2f} px")

            try:
                Z_cm_str = input("Enter current distance (cm) from camera to ball center: ")
                Z_cm = float(Z_cm_str)
            except ValueError:
                print("Invalid distance; sample ignored.")
                continue

            calib_samples.append((Z_cm, r_px))
            print(f"Sample added: Z={Z_cm:.2f} cm, r={r_px:.2f} px")
            print(f"Total samples: {len(calib_samples)}")
            print()

    cap.release()
    cv2.destroyAllWindows()

    if not calib_samples:
        print("No calibration samples collected.")
        return

    # Compute focal length estimates for each sample: f_i = Z_i * r_i / R
    # Uses the pinhole camera projection relationship
    f_values = [Z * r / CALIB_OBJECT_RADIUS_CM for (Z, r) in calib_samples]
    f_mean = float(sum(f_values) / len(f_values))

    print("\nCalibration results")
    print("-------------------")
    for i, (Z, r) in enumerate(calib_samples, 1):
        f_i = Z * r / CALIB_OBJECT_RADIUS_CM
        print(f"Sample {i}: Z={Z:.2f} cm, r={r:.2f} px -> f_i={f_i:.2f} px")

    print(f"\nEstimated focal length (pixels): FOCAL_PX = {f_mean:.3f}\n")

    print("Paste this into your main script:")
    print("---------------------------------")
    print("FOCAL_PX = {:.3f}".format(f_mean))
    print("OBJECT_RADIUS_CM = {:.3f}  # same as your calibration ball".format(CALIB_OBJECT_RADIUS_CM))
    print()
    print("def radius_to_distance_cm(r_px: float) -> float:")
    print("    r_px = max(r_px, 1e-6)  # avoid division by zero")
    print("    return (FOCAL_PX * OBJECT_RADIUS_CM) / r_px")
    print()


if __name__ == "__main__":
    main()

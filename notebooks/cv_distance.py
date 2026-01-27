import cv2
import numpy as np
from datetime import datetime
from functools import reduce

# HSV color ranges
RANGES = {
    "red": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    "green": [((35, 80, 60), (85, 255, 255))],
}

FOCAL_PX = 977.434
OBJECT_RADIUS_CM = 5.000  # same as calibration ball

# def radius_to_distance_cm(r_px: float) -> float:
#     return 25.0 - 2.0 * (r_px + 25.0)
def radius_to_distance_cm(r_px: float) -> float:
    r_px = max(r_px, 1e-6)  # avoid division by zero
    return (FOCAL_PX * OBJECT_RADIUS_CM) / r_px

def build_mask(hsv, color):
    masks = [cv2.inRange(hsv, np.array(lo), np.array(hi)) for lo, hi in RANGES[color]]
    mask = reduce(cv2.bitwise_or, masks)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask

def analyze(mask, color, min_area=150):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        hull = cv2.convexHull(c)
        (cx_f, cy_f), radius = cv2.minEnclosingCircle(hull)
        circle_area = float(np.pi * radius * radius)
        if circle_area < min_area:
            continue
        center_int = (int(round(cx_f)), int(round(cy_f)))
        dist_cm = radius_to_distance_cm(float(radius))
        out.append({
            "color": color,
            "radius_px": float(radius),
            "distance_cm": float(dist_cm),
            "center": center_int,
            "contour": hull
        })
    return out

def annotate(frame, detections):
    vis = frame.copy()
    for d in detections:
        color_bgr = (0, 0, 255) if d["color"] == "red" else (0, 255, 0)
        cv2.drawContours(vis, [d["contour"]], -1, color_bgr, 2)
        cv2.circle(vis, d["center"], int(round(d["radius_px"])), color_bgr, 2)
        txt = f"{d['color']}: R={d['radius_px']:.1f}px  Z={d['distance_cm']:.1f}cm"
        cv2.putText(vis, txt, (d["center"][0]+6, d["center"][1]-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2, cv2.LINE_AA)
    return vis

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise SystemExit("Cannot open camera")

print("Press 'q' to quit, 's' to save frame.")
while True:
    ret, frame = cap.read()
    if not ret:
        break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detections = []
    for color in ("red", "green"):
        mask = build_mask(hsv, color)
        detections += analyze(mask, color)
    annotated = annotate(frame, detections)
    cv2.imshow("Red/Green Object Detection", annotated)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        filename = f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        cv2.imwrite(filename, annotated)
        print("Saved:", filename)
cap.release()
cv2.destroyAllWindows()

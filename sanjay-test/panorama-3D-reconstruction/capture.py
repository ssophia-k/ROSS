"""
Step 1 — Capture frames from Surface webcam.
Slowly rotate the camera 360° around the scene.
Press Q to stop, SPACE to force-save a frame.
"""
print("hello world")
import cv2
import numpy as np
import os
import time

OUTPUT_DIR = "data/color"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Surface webcam — 0 is usually the built-in camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# Motion threshold — only save when camera has moved enough
MIN_MOTION = 12.0       # lower = more frames saved
MAX_FRAMES = 150        # cap to avoid huge datasets
SAVE_INTERVAL = 0.3     # seconds minimum between saves

frame_idx = 0
prev_gray = None
last_saved = 0

print("=" * 50)
print("Surface Webcam Capture")
print("  Move camera SLOWLY in a full 360° arc")
print("  SPACE = force save frame")
print("  Q     = quit and proceed to depth estimation")
print("=" * 50)

while frame_idx < MAX_FRAMES:
    ret, frame = cap.read()
    if not ret:
        print("Camera error — check device index")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    now = time.time()
    force_save = False

    # Compute inter-frame motion
    motion = 0.0
    if prev_gray is not None:
        diff = cv2.absdiff(gray, prev_gray)
        motion = diff.mean()

    # Decide whether to save
    should_save = (
        (motion >= MIN_MOTION and now - last_saved >= SAVE_INTERVAL)
        or prev_gray is None  # always save first frame
        or force_save
    )

    key = cv2.waitKey(1) & 0xFF
    if key == ord(' '):
        should_save = True
    if key == ord('q'):
        break

    if should_save:
        path = os.path.join(OUTPUT_DIR, f"frame_{frame_idx:04d}.jpg")
        cv2.imwrite(path, frame)
        last_saved = now
        prev_gray = gray
        frame_idx += 1
        print(f"  Saved frame {frame_idx:03d}  (motion={motion:.1f})")

    # Overlay HUD
    status = f"Saved: {frame_idx}/{MAX_FRAMES}  Motion: {motion:.1f}"
    color = (0, 255, 0) if motion >= MIN_MOTION else (0, 100, 255)
    cv2.putText(frame, status, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.putText(frame, "Q=quit  SPACE=force save", (10, 460),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.imshow("360 Capture — Surface Webcam", frame)

cap.release()
cv2.destroyAllWindows()
print(f"\nDone. {frame_idx} frames saved to {OUTPUT_DIR}/")
print("Next: run  python depth_estimate.py")
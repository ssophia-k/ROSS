import cv2
import numpy as np
from ultralytics import YOLO

# 1. CONFIGURATION
# Define known width of objects (in centimeters) for distance estimation
# The code uses this to calculate Z-depth
KNOWN_WIDTHS = {
    'person': 50,  # Average shoulder width
    'cup': 8,
    'bottle': 6,
    'cell phone': 7,
    'laptop': 35,
    'chair': 45,
    'potted plant': 20
}
FOCAL_LENGTH = 600 # Approximate focal length for webcams (can be tuned)

# 2. LOAD MODEL
print("Loading YOLO AI Model (this might take a moment)...")
model = YOLO('yolov8n.pt') 

def estimate_distance(label, pixel_width):
    # Formula: Distance = (Real Width * Focal Length) / Pixel Width
    if label in KNOWN_WIDTHS:
        real_width = KNOWN_WIDTHS[label]
        dist = (real_width * FOCAL_LENGTH) / pixel_width
        return dist
    return 0

cap = cv2.VideoCapture(0)

# Map Settings
map_size = 600
# Initialize the map as a black image using NumPy (The Fix)
map_img = np.zeros((map_size, map_size, 3), dtype=np.uint8)

print("System Started. Point camera at objects.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 3. AI OBJECT DETECTION
    # stream=True is faster for video
    results = model(frame, stream=True, verbose=False)

    # Reset the map to black every frame so points don't smear
    map_img[:] = (0, 0, 0)
    
    # Draw 'YOU' (The Camera) at the bottom center
    center_x = map_size // 2
    bottom_y = map_size - 50
    cv2.circle(map_img, (center_x, bottom_y), 10, (0, 0, 255), -1)
    cv2.putText(map_img, "YOU", (center_x - 15, bottom_y + 25), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)

    # 4. PROCESS EACH DETECTED OBJECT
    for r in results:
        boxes = r.boxes
        for box in boxes:
            # Get box coordinates
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # Get Class Label
            cls = int(box.cls[0])
            label = model.names[cls]
            
            # Calculate Distance
            pixel_width = x2 - x1
            distance_cm = estimate_distance(label, pixel_width)

            # Draw "3D-Style" Box on Camera
            # We draw the main box in Green
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Display Label and Distance
            if distance_cm > 0:
                text = f"{label}: {distance_cm:.0f}cm"
            else:
                text = label
                
            cv2.putText(frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 5. PLOT ON TOP-DOWN MAP
            # Only map if we know the distance (width is in our list)
            if distance_cm > 0 and distance_cm < 500: # Ignore things > 5 meters away
                # Calculate X offset (Left/Right)
                # If object is in center of screen -> Center of map
                screen_width = frame.shape[1]
                obj_center_x = (x1 + x2) // 2
                
                # Map the screen X (0 to 1280) to Map X (-300 to +300)
                deviation_from_center = obj_center_x - (screen_width // 2)
                scale_factor = 0.5 # Scale down the left/right movement
                
                map_x = center_x + int(deviation_from_center * scale_factor)
                map_y = bottom_y - int(distance_cm) # Move 'up' the map as distance increases

                # Ensure points stay inside the map window
                map_x = max(0, min(map_x, map_size))
                map_y = max(0, min(map_y, map_size))

                # Draw the object as a Green Dot
                cv2.circle(map_img, (map_x, map_y), 6, (0, 255, 0), -1)
                cv2.putText(map_img, label, (map_x + 10, map_y), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)

    # Show the windows
    cv2.imshow('YOLO Object Tracker', frame)
    cv2.imshow('Top-Down Map', map_img)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
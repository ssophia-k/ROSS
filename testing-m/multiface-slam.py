import cv2
import numpy as np
import math
from ultralytics import YOLO

# --- CONFIGURATION ---
MAP_SIZE = 1000
INITIAL_DIST = 80.0     # Starting distance (cm)
SMOOTHING = 0.15        # Movement smoothing (0.1 = heavy smoothing)
DEAD_ZONE = 2.0         # Ignore tiny movements (cm)
ROTATION_SENSITIVITY = 0.7 # Overlap threshold (Lower = easier to detect rotation)

# ONLY lock onto these
TARGET_CLASSES = ['cup', 'bottle', 'wine glass', 'cell phone', 'mouse', 'remote', 'keyboard', 'book']

class MultiFaceSLAM:
    def __init__(self):
        print("Initializing Multi-Face SLAM...")
        self.model = YOLO('yolov8n.pt')
        
        # State
        self.cam_x = 0
        self.cam_z = INITIAL_DIST
        self.current_face_angle = 0 # 0=Front, 90=Right, 180=Back, 270=Left
        
        # Anchor Memory
        self.anchor_id = None       
        self.anchor_class = ""
        self.ref_box = None     # The box of the current "Face" we are tracking
        self.ref_width = 0      # Width of the current face
        
        # Map
        self.map_img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        self.path = [] 

    def calculate_iou(self, boxA, boxB):
        # Calculate overlap between two boxes to detect if view changed significantly
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        
        interArea = max(0, xB - xA) * max(0, yB - yA)
        boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
        boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
        
        iou = interArea / float(boxAArea + boxBArea - interArea)
        return iou

    def get_position(self, box_w, box_center_x, screen_w):
        # 1. Distance (Z)
        ratio = self.ref_width / box_w
        dist = INITIAL_DIST * ratio
        
        # 2. Offset (X)
        screen_center = screen_w / 2
        deviation = box_center_x - screen_center
        scale_factor = dist / 800.0 
        offset = deviation * scale_factor * -2.0
        
        return offset, dist

    def update_map(self):
        self.map_img[:] = (30, 30, 30)
        cx, cy = MAP_SIZE // 2, MAP_SIZE // 2
        
        # 1. DRAW ANCHOR (Rotates based on Face Angle)
        # We draw a square representing the object
        box_size = 30
        
        # Rotate the square on map to match current face
        # If we are looking at Face 0 (Front), Cube is normal
        # If we are looking at Face 90 (Side), Cube is rotated 90 deg
        
        rect = ((cx, cy), (box_size*2, box_size*2), self.current_face_angle)
        box_pts = cv2.boxPoints(rect)
        box_pts = np.int64(box_pts)
        cv2.drawContours(self.map_img, [box_pts], 0, (0, 255, 0), 2)
        
        # Draw "Front" Indicator (Red Line) on the cube
        # This helps visualize which side is which
        front_pt1 = (cx - box_size, cy + box_size)
        front_pt2 = (cx + box_size, cy + box_size)
        cv2.line(self.map_img, front_pt1, front_pt2, (0, 0, 255), 3)
        
        cv2.putText(self.map_img, f"ANCHOR ({int(self.current_face_angle)} deg)", (cx-60, cy-40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        # 2. DRAW CAMERA (You)
        # We calculate global position by rotating the local offset around the anchor
        
        # Convert local X/Z to Polar coordinates
        rad_angle = math.radians(self.current_face_angle)
        
        # Rotate the relative position based on which "Face" we are tracking
        # New X = X * cos(a) - Z * sin(a)
        # New Z = X * sin(a) + Z * cos(a)
        
        global_x = self.cam_x * math.cos(rad_angle) - self.cam_z * math.sin(rad_angle)
        global_z = self.cam_x * math.sin(rad_angle) + self.cam_z * math.cos(rad_angle)
        
        draw_x = cx + int(global_x * 3.0)
        draw_z = cy + int(global_z * 3.0)
        
        self.path.append((draw_x, draw_z))
        if len(self.path) > 300: self.path.pop(0)
        
        if len(self.path) > 2:
            cv2.polylines(self.map_img, [np.array(self.path)], False, (0, 255, 255), 2)
            
        # Draw Camera Arrow
        angle = math.atan2(cy - draw_z, cx - draw_x)
        p_tip = (int(draw_x + 20 * np.cos(angle)), int(draw_z + 20 * np.sin(angle)))
        cv2.arrowedLine(self.map_img, (draw_x, draw_z), p_tip, (0, 165, 255), 3)
        cv2.circle(self.map_img, (draw_x, draw_z), 8, (0, 165, 255), -1)
        
        cv2.putText(self.map_img, "YOU", (draw_x-15, draw_z+30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    def run(self):
        cap = cv2.VideoCapture(0)
        print("SYSTEM READY. Point at object.")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret: break
            
            results = self.model.track(frame, persist=True, verbose=False, stream=True)
            
            frame_h, frame_w = frame.shape[:2]
            found_anchor = False
            
            for r in results:
                if r.boxes.id is not None:
                    boxes = r.boxes.xyxy.cpu().numpy()
                    track_ids = r.boxes.id.int().cpu().numpy()
                    classes = r.boxes.cls.int().cpu().numpy()
                    
                    # A. LOCKING LOGIC
                    if self.anchor_id is None:
                        largest_area = 0
                        for box, tid, cls in zip(boxes, track_ids, classes):
                            name = self.model.names[cls]
                            if name in TARGET_CLASSES:
                                area = (box[2]-box[0]) * (box[3]-box[1])
                                if area > largest_area:
                                    largest_area = area
                                    self.anchor_id = tid
                                    self.anchor_class = name
                                    self.ref_box = box
                                    self.ref_width = box[2] - box[0]
                                    print(f"-> LOCKED FRONT FACE: {name}")

                    # B. TRACKING LOGIC
                    else:
                        for box, tid in zip(boxes, track_ids):
                            if tid == self.anchor_id:
                                found_anchor = True
                                x1, y1, x2, y2 = map(int, box)
                                curr_w = x2 - x1
                                center_x = (x1 + x2) / 2
                                
                                # --- ROTATION DETECTION (Overlap Check) ---
                                # If the box moves drastically or changes shape, we assume rotation
                                iou = self.calculate_iou(self.ref_box, box)
                                
                                # If overlap is low, user likely moved to a new face
                                if iou < ROTATION_SENSITIVITY:
                                    # Detect Direction of Rotation based on movement
                                    prev_center = (self.ref_box[0] + self.ref_box[2]) / 2
                                    
                                    if center_x > prev_center: 
                                        # Box moved Right -> Camera moved Left -> -90 deg
                                        self.current_face_angle -= 15 # Smooth rotation
                                    else:
                                        # Box moved Left -> Camera moved Right -> +90 deg
                                        self.current_face_angle += 15
                                    
                                    # Reset Reference to this new "Face"
                                    self.ref_box = box
                                    self.ref_width = curr_w # Re-calibrate depth for new face
                                
                                # --- POSITION CALCULATION ---
                                raw_x, raw_z = self.get_position(curr_w, center_x, frame_w)
                                
                                # Dead Zone & Smoothing
                                if abs(raw_x - self.cam_x) > DEAD_ZONE or abs(raw_z - self.cam_z) > DEAD_ZONE:
                                    self.cam_x = (self.cam_x * (1-SMOOTHING)) + (raw_x * SMOOTHING)
                                    self.cam_z = (self.cam_z * (1-SMOOTHING)) + (raw_z * SMOOTHING)
                                
                                # Draw
                                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                info = f"FACE: {int(self.current_face_angle)} deg | IOU: {iou:.2f}"
                                cv2.putText(frame, info, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                self.ref_box = box # Update reference slightly to track drift
            
            if not found_anchor and self.anchor_id is not None:
                 cv2.putText(frame, "LOST VISUAL - TURN BACK", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            self.update_map()
            
            # Combine Windows
            map_resized = cv2.resize(self.map_img, (frame_h, frame_h))
            combined = np.hstack((frame, map_resized))
            cv2.imshow("Multi-Face SLAM", combined)
            
            if cv2.waitKey(1) == ord('q'): break
            if cv2.waitKey(1) == ord('r'): 
                self.anchor_id = None
                self.current_face_angle = 0
                self.path = []

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    slam = MultiFaceSLAM()
    slam.run()
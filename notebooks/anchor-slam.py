import cv2
import numpy as np
import math
from ultralytics import YOLO

# --- CONFIGURATION ---
MAP_SIZE = 1000         # Increased Map Size for more space
COLLISION_DIST = 30.0   # Centimeters (Threshold for Red Box)
INITIAL_DIST = 80.0     # Starting distance assumption
SMOOTHING = 0.1         # Lower = Smoother (Less Jitter)
DEAD_ZONE = 2.0         # Ignore movements smaller than 2cm (Stops drift)

# ONLY lock onto these objects (prevents locking onto walls/people)
TARGET_CLASSES = ['cup', 'bottle', 'wine glass', 'cell phone', 'mouse', 'remote', 'keyboard', 'book']

class AnchorSLAM:
    def __init__(self):
        # Initialize YOLO
        print("Initializing AI Brain...")
        self.model = YOLO('yolov8n.pt')
        
        # State Variables
        self.anchor_id = None       
        self.anchor_ref_w = 0       
        self.anchor_class = ""      
        
        # Camera Position (Relative to Anchor at 0,0)
        self.cam_x = 0
        self.cam_z = INITIAL_DIST
        
        # Map Image
        self.map_img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        self.path = [] 

    def get_distance_and_offset(self, box_w, box_center_x, screen_w):
        # 1. DEPTH (Z) Calculation
        ratio = self.anchor_ref_w / box_w
        raw_dist = INITIAL_DIST * ratio
        
        # 2. LATERAL (X) Calculation
        screen_center = screen_w / 2
        deviation = box_center_x - screen_center
        
        # Scale deviation based on distance (Parallax)
        # We assume 1 pixel deviation = more movement when object is close
        scale_factor = raw_dist / 800.0 
        raw_offset = deviation * scale_factor * -2.0 # -2.0 amplifies side movement
        
        return raw_offset, raw_dist

    def draw_3d_prism(self, img, x1, y1, x2, y2, depth_color):
        w, h = x2 - x1, y2 - y1
        
        # Front Face
        cv2.rectangle(img, (x1, y1), (x2, y2), depth_color, 2)
        
        # Back Face (Perspective Shift towards center)
        h_img, w_img = img.shape[:2]
        cx, cy = w_img // 2, h_img // 2
        
        # Vector from object center to screen center
        obj_cx, obj_cy = x1 + w//2, y1 + h//2
        vec_x, vec_y = obj_cx - cx, obj_cy - cy
        
        # Shift back face
        shift_x = int(vec_x * 0.1) # 10% shift
        shift_y = int(vec_y * 0.1)
        
        bx1, by1 = x1 + int(w*0.1) - shift_x, y1 - int(h*0.1) - shift_y
        bx2, by2 = bx1 + w, by1 + h
        
        # Draw Back Face & Connectors
        cv2.rectangle(img, (bx1, by1), (bx2, by2), depth_color, 1)
        corners = [(x1, y1, bx1, by1), (x2, y1, bx2, by1), 
                   (x1, y2, bx1, by2), (x2, y2, bx2, by2)]
        for (fx, fy, bx, by) in corners:
            cv2.line(img, (fx, fy), (bx, by), depth_color, 1)

    def update_map(self):
        # Clear Map
        self.map_img[:] = (30, 30, 30) # Dark Gray Background
        
        cx, cy = MAP_SIZE // 2, MAP_SIZE // 2
        
        # 1. DRAW GRID
        grid_color = (60, 60, 60)
        for i in range(0, MAP_SIZE, 100):
            cv2.line(self.map_img, (i, 0), (i, MAP_SIZE), grid_color, 1)
            cv2.line(self.map_img, (0, i), (MAP_SIZE, i), grid_color, 1)
            
        # 2. DRAW ANCHOR (Fixed at Center)
        cv2.rectangle(self.map_img, (cx-25, cy-25), (cx+25, cy+25), (0, 255, 0), -1) # Filled Green Square
        cv2.putText(self.map_img, "ANCHOR", (cx-35, cy-35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if self.anchor_class:
            cv2.putText(self.map_img, f"({self.anchor_class})", (cx-40, cy+45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 255, 200), 1)

        # 3. DRAW CAMERA (Relative Position)
        # Scale: 3 pixels = 1 cm (Zoomed in map)
        map_scale = 3.0
        
        draw_x = cx + int(self.cam_x * map_scale)
        draw_z = cy + int(self.cam_z * map_scale) 
        
        # Keep dots inside map bounds
        draw_x = max(10, min(MAP_SIZE-10, draw_x))
        draw_z = max(10, min(MAP_SIZE-10, draw_z))

        # Store path
        self.path.append((draw_x, draw_z))
        if len(self.path) > 300: self.path.pop(0)
        
        # Draw Path
        if len(self.path) > 2:
            cv2.polylines(self.map_img, [np.array(self.path)], False, (0, 255, 255), 2)
        
        # Draw Camera Arrow (facing Anchor)
        angle = math.atan2(cy - draw_z, cx - draw_x)
        size = 20
        p_tip = (int(draw_x + size * np.cos(angle)), int(draw_z + size * np.sin(angle)))
        
        cv2.arrowedLine(self.map_img, (draw_x, draw_z), p_tip, (0, 165, 255), 3)
        cv2.circle(self.map_img, (draw_x, draw_z), 8, (0, 165, 255), -1) # Orange Dot
        
        cv2.putText(self.map_img, "YOU", (draw_x-15, draw_z+30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    def run(self):
        cap = cv2.VideoCapture(0)
        
        print("---------------------------------------")
        print(" STABLE ANCHOR SLAM READY.")
        print(" 1. Point at a Cup, Bottle, or Phone.")
        print(" 2. System will ignore walls/people.")
        print(" 3. 'r' to Reset, 'q' to Quit.")
        print("---------------------------------------")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret: break
            
            # Use 'stream=True' for speed
            results = self.model.track(frame, persist=True, verbose=False, stream=True)
            
            # We need to process the generator from stream=True
            for r in results:
                frame_h, frame_w = frame.shape[:2]
                
                if r.boxes.id is not None:
                    boxes = r.boxes.xyxy.cpu().numpy()
                    track_ids = r.boxes.id.int().cpu().numpy()
                    classes = r.boxes.cls.int().cpu().numpy()
                    
                    target_box = None
                    
                    # --- A. LOCK LOGIC ---
                    if self.anchor_id is None:
                        largest_area = 0
                        
                        for box, track_id, cls in zip(boxes, track_ids, classes):
                            class_name = self.model.names[cls]
                            
                            # STRICT FILTER: Only lock if it's in our allowed list
                            if class_name in TARGET_CLASSES:
                                area = (box[2]-box[0]) * (box[3]-box[1])
                                # Find largest valid object
                                if area > largest_area:
                                    largest_area = area
                                    self.anchor_id = track_id
                                    self.anchor_class = class_name
                                    self.anchor_ref_w = box[2] - box[0]
                                    target_box = box
                                    print(f"-> LOCKED: {class_name} (ID: {track_id})")
                            
                            # Visual Debug: Draw yellow boxes around candidates
                            if class_name in TARGET_CLASSES:
                                cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 255), 1)

                    # --- B. TRACKING LOGIC ---
                    else:
                        for box, track_id in zip(boxes, track_ids):
                            if track_id == self.anchor_id:
                                target_box = box
                                break
                    
                    # --- C. MATH & VISUALS ---
                    if target_box is not None:
                        x1, y1, x2, y2 = map(int, target_box)
                        curr_w = x2 - x1
                        center_x = (x1 + x2) / 2
                        
                        # Calculate raw position
                        raw_x, raw_z = self.get_distance_and_offset(curr_w, center_x, frame_w)
                        
                        # --- THE FIX: DEAD ZONE & SMOOTHING ---
                        # Calculate how much we supposedly moved
                        delta_x = abs(raw_x - self.cam_x)
                        delta_z = abs(raw_z - self.cam_z)
                        
                        # Only update if movement > DEAD_ZONE (e.g. 2cm)
                        if delta_x > DEAD_ZONE or delta_z > DEAD_ZONE:
                            # Apply smoothing (Low Pass Filter)
                            self.cam_x = (self.cam_x * (1-SMOOTHING)) + (raw_x * SMOOTHING)
                            self.cam_z = (self.cam_z * (1-SMOOTHING)) + (raw_z * SMOOTHING)
                        
                        # Collision Warning
                        color = (0, 255, 0)
                        if self.cam_z < COLLISION_DIST:
                            color = (0, 0, 255)
                            cv2.putText(frame, "COLLISION WARNING!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                        
                        self.draw_3d_prism(frame, x1, y1, x2, y2, color)
                        
                        # Info Text
                        info = f"DIST: {int(self.cam_z)}cm | OFFSET: {int(self.cam_x)}"
                        cv2.putText(frame, info, (x1, y1-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    
                    else:
                        # Only show "Searching" if we have an ID but lost sight of it
                        if self.anchor_id is not None:
                             cv2.putText(frame, "SEARCHING...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)

            self.update_map()
            
            # Combine Windows
            map_resized = cv2.resize(self.map_img, (frame.shape[0], frame.shape[0]))
            combined = np.hstack((frame, map_resized))
            
            cv2.imshow("Stable Anchor SLAM", combined)
            
            key = cv2.waitKey(1)
            if key == ord('q'): break
            if key == ord('r'): 
                self.anchor_id = None
                self.path = []
                print("RESETTING...")

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    slam = AnchorSLAM()
    slam.run()
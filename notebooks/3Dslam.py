import cv2
import numpy as np
from ultralytics import YOLO

# --- CONFIGURATION ---
MAP_SIZE = 1000            
GRID_SIZE = 100            
MOVEMENT_SCALE = 10.0      # Slightly reduced for stability
DEAD_ZONE = 0.05           # The Fix: Ignore moves smaller than this value
FOCAL_LENGTH = 700         
KNOWN_WIDTHS = {           
    'person': 50, 'cup': 8, 'bottle': 6, 'cell phone': 7, 
    'laptop': 35, 'chair': 45, 'tv': 80, 'book': 15, 'keyboard': 45
}

class SemanticSLAM_Stable:
    def __init__(self):
        self.map_img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        self.path = []
        
        # Robot State
        self.curr_R = np.eye(3)
        self.curr_t = np.zeros((3, 1))
        
        # VO Variables
        self.px_ref = None
        self.frame_ref = None
        self.focal = 718.8560
        self.pp = (307.5, 307.5)
        
        print("Loading AI Model...")
        self.model = YOLO('yolov8n.pt')
        
        self.lk_params = dict(winSize=(21, 21), criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        self.feature_params = dict(maxCorners=3000, qualityLevel=0.01, minDistance=5, blockSize=7)

    def draw_grid(self, img):
        color = (50, 50, 50)
        for i in range(0, MAP_SIZE, GRID_SIZE):
            cv2.line(img, (i, 0), (i, MAP_SIZE), color, 1)
            cv2.line(img, (0, i), (MAP_SIZE, i), color, 1)

    def draw_robot_pointer(self, img, x, y, size=40):
        yaw = np.arctan2(self.curr_R[0, 2], self.curr_R[2, 2])
        pt_tip = (int(x + size * np.sin(yaw)), int(y + size * np.cos(yaw)))
        pt_left = (int(x + size/2 * np.sin(yaw + 2.5)), int(y + size/2 * np.cos(yaw + 2.5)))
        pt_right = (int(x + size/2 * np.sin(yaw - 2.5)), int(y + size/2 * np.cos(yaw - 2.5)))
        
        triangle = np.array([pt_tip, pt_left, pt_right])
        cv2.drawContours(img, [triangle], 0, (255, 255, 0), -1) 
        cv2.drawContours(img, [triangle], 0, (255, 255, 255), 2)

    def draw_3d_box(self, img, box):
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        depth = (x2 - x1) * 0.4 
        shift = int(depth)
        bx1, by1 = x1 + shift, y1 - shift
        bx2, by2 = x2 + shift, y2 - shift
        
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.line(img, (x1, y1), (bx1, by1), (0, 255, 0), 1)
        cv2.line(img, (x2, y1), (bx2, by1), (0, 255, 0), 1)
        cv2.line(img, (x1, y2), (bx1, by2), (0, 255, 0), 1)
        cv2.line(img, (x2, y2), (bx2, by2), (0, 255, 0), 1)
        cv2.rectangle(img, (bx1, by1), (bx2, by2), (0, 255, 0), 1)

    def process(self):
        cap = cv2.VideoCapture(0)
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret: break
            
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # --- 1. MOVEMENT TRACKING ---
            if self.px_ref is not None:
                p1, st, _ = cv2.calcOpticalFlowPyrLK(self.frame_ref, frame_gray, self.px_ref, None, **self.lk_params)
                if p1 is not None:
                    good_new = p1[st.flatten() == 1]
                    good_old = self.px_ref[st.flatten() == 1]
                    
                    if len(good_new) > 20:
                        E, _ = cv2.findEssentialMat(good_new, good_old, self.focal, self.pp, cv2.RANSAC, 0.999, 1.0)
                        if E is not None:
                            try:
                                _, R, t, _ = cv2.recoverPose(E, good_new, good_old, focal=self.focal, pp=self.pp)
                                
                                # --- THE FIX: DEAD ZONE FILTER ---
                                # Calculate magnitude of movement (how much did we move?)
                                move_magnitude = np.sqrt(t[0]**2 + t[1]**2 + t[2]**2)
                                
                                # Only update if movement is larger than threshold
                                if move_magnitude > DEAD_ZONE:
                                    self.curr_t += MOVEMENT_SCALE * self.curr_R.dot(t)
                                    self.curr_R = R.dot(self.curr_R)
                                    
                            except: pass
                    
                    if len(good_new) < 1500:
                         self.px_ref = cv2.goodFeaturesToTrack(frame_gray, mask=None, **self.feature_params)
                    else:
                        self.px_ref = good_new
                self.frame_ref = frame_gray
            else:
                self.px_ref = cv2.goodFeaturesToTrack(frame_gray, mask=None, **self.feature_params)
                self.frame_ref = frame_gray

            # --- 2. MAP DRAWING ---
            self.map_img[:] = (0, 0, 0)
            self.draw_grid(self.map_img)
            
            rx, _, rz = self.curr_t.flatten()
            map_rx = int(rx) + MAP_SIZE // 2
            map_rz = int(rz) + MAP_SIZE // 2
            
            self.path.append((map_rx, map_rz))
            if len(self.path) > 2:
                cv2.polylines(self.map_img, [np.array(self.path)], False, (0, 255, 0), 2)

            self.draw_robot_pointer(self.map_img, map_rx, map_rz)
            
            cv2.putText(self.map_img, f"X: {rx:.1f} Z: {rz:.1f}", (20, MAP_SIZE - 20), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1)

            # --- 3. OBJECT MAPPING ---
            results = self.model(frame, stream=True, verbose=False)
            for r in results:
                for box in r.boxes:
                    if float(box.conf[0]) > 0.5:
                        label = self.model.names[int(box.cls[0])]
                        self.draw_3d_box(frame, box)
                        
                        if label in KNOWN_WIDTHS:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            pixel_width = x2 - x1
                            dist_cm = (KNOWN_WIDTHS[label] * FOCAL_LENGTH) / pixel_width
                            
                            rel_z = dist_cm / 5.0
                            obj_vec = np.array([[0], [0], [rel_z]])
                            global_obj = self.curr_t + self.curr_R.dot(obj_vec)
                            
                            ox, _, oz = global_obj.flatten()
                            map_ox = int(ox) + MAP_SIZE // 2
                            map_oz = int(oz) + MAP_SIZE // 2
                            
                            cv2.circle(self.map_img, (map_ox, map_oz), 10, (0, 0, 255), -1)
                            cv2.putText(self.map_img, label, (map_ox+15, map_oz), cv2.FONT_HERSHEY_DUPLEX, 1.2, (255, 255, 255), 1)

            # --- DISPLAY ---
            h, w, _ = frame.shape
            map_resized = cv2.resize(self.map_img, (h, h))
            combined = np.hstack((frame, map_resized))
            cv2.imshow("Stable Semantic SLAM", combined)
            
            if cv2.waitKey(1) == ord('q'): break
            
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    slam = SemanticSLAM_Stable()
    slam.process()
import cv2
import numpy as np

# --- CONFIGURATION ---
MAP_SIZE = 800
TRAJECTORY_SCALE = 1.0

class VisualOdometry:
    def __init__(self):
        self.traj_img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        self.curr_R = np.eye(3)
        self.curr_t = np.zeros((3, 1)) # Position is a 3x1 Matrix (x, y, z)
        
        self.px_ref = None
        self.frame_ref = None
        self.focal = 718.8560
        self.pp = (307.5, 307.5)
        
        self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
        self.lk_params = dict(winSize=(21, 21), 
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

    def process_frame(self, frame):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1. INITIALIZATION
        if self.px_ref is None or len(self.px_ref) < 1:
            self.detect_new_features(frame_gray)
            return frame, self.traj_img

        # 2. OPTICAL FLOW
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.frame_ref, frame_gray, self.px_ref, None, **self.lk_params)
        
        # 3. SAFETY CHECKS
        if p1 is not None and st is not None:
            st = st.flatten()
            good_new = p1[st == 1]
            good_old = self.px_ref[st == 1]
        else:
            good_new = np.empty((0, 2))

        # 4. CALCULATE MOVEMENT
        if len(good_new) > 20:
            E, mask = cv2.findEssentialMat(good_new, good_old, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            
            if E is not None:
                try:
                    _, R, t, mask = cv2.recoverPose(E, good_new, good_old, focal=self.focal, pp=self.pp)
                    
                    # Update Trajectory
                    absolute_scale = 1.0
                    self.curr_t = self.curr_t + absolute_scale * self.curr_R.dot(t)
                    self.curr_R = R.dot(self.curr_R)
                except cv2.error:
                    pass

        # 5. DRAW MAP (THE FIX IS HERE)
        # We use .flatten() to turn the matrix [[x]] into a simple list [x] 
        # and take the first item [0]
        x, y, z = self.curr_t.flatten() 
        
        draw_x = int(x) + MAP_SIZE // 2
        draw_y = int(z) + MAP_SIZE // 2 # We map 'z' (depth) to the Y-axis of the screen
        
        # Keep dots within bounds
        draw_x = min(max(draw_x, 0), MAP_SIZE-1)
        draw_y = min(max(draw_y, 0), MAP_SIZE-1)

        cv2.circle(self.traj_img, (draw_x, draw_y), 1, (0, 255, 0), 1)
        
        display_map = self.traj_img.copy()
        cv2.circle(display_map, (draw_x, draw_y), 5, (0, 0, 255), -1)
        
        # Display coordinates text
        cv2.putText(display_map, f"Pos: [{x:.2f}, {z:.2f}]", (20, 40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)

        # 6. UPDATE NEXT LOOP
        if len(good_new) < 1000:
            self.detect_new_features(frame_gray)
        else:
            self.px_ref = good_new
            self.frame_ref = frame_gray

        if self.px_ref is not None:
             for pt in self.px_ref:
                cv2.circle(frame, (int(pt[0]), int(pt[1])), 2, (0, 255, 0), -1)

        return frame, display_map

    def detect_new_features(self, gray_frame):
        new_features = self.detector.detect(gray_frame, None)
        if new_features:
            self.px_ref = np.array([x.pt for x in new_features], dtype=np.float32)
            self.frame_ref = gray_frame
        else:
            self.px_ref = None

# --- MAIN LOOP ---
cap = cv2.VideoCapture(0)
vo = VisualOdometry()

print("Robust Visual Odometry started.")
print("Move camera slowly.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cam_view, map_view = vo.process_frame(frame)

    h, w, _ = cam_view.shape
    map_view_resized = cv2.resize(map_view, (w, h))
    combined = np.hstack((cam_view, map_view_resized))

    cv2.imshow("Visual Odometry", combined)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
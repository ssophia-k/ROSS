import cv2
import numpy as np

# --- ADVANCED CONFIGURATION ---
MAP_SIZE = 1000
MOVEMENT_AMPLIFIER = 5.0
MIN_FEATURES = 1500

class AdvancedSLAM:
    def __init__(self):
        self.traj_img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        self.curr_R = np.eye(3)
        self.curr_t = np.zeros((3, 1))
        self.path = [] 
        self.focal = 718.8560
        self.pp = (307.5, 307.5)
        
        self.px_ref = None
        self.frame_ref = None
        
        self.feature_params = dict(maxCorners=2000, qualityLevel=0.01, minDistance=7, blockSize=7)
        self.lk_params = dict(winSize=(21, 21), criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

    def process_frame(self, frame):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1. INITIALIZATION
        if self.px_ref is None or len(self.px_ref) < 1:
            self.detect_new_features(frame_gray)
            return frame, self.traj_img

        # 2. OPTICAL FLOW
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.frame_ref, frame_gray, self.px_ref, None, **self.lk_params)
        
        # 3. FILTER GOOD POINTS
        if p1 is not None and st is not None:
            st = st.flatten()
            good_new = p1[st == 1]
            good_old = self.px_ref[st == 1]
        else:
            good_new = np.empty((0, 2))

        # 4. MOTION ESTIMATION
        if len(good_new) > 20:
            E, mask = cv2.findEssentialMat(good_new, good_old, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            
            if E is not None:
                try:
                    _, R, t, mask = cv2.recoverPose(E, good_new, good_old, focal=self.focal, pp=self.pp)
                    
                    scale = MOVEMENT_AMPLIFIER
                    self.curr_t = self.curr_t + scale * self.curr_R.dot(t)
                    self.curr_R = R.dot(self.curr_R)
                except:
                    pass

        # 5. DRAW MAP
        x, y, z = self.curr_t.flatten()
        
        draw_x = int(x) + MAP_SIZE // 2
        draw_y = int(z) + MAP_SIZE // 2
        
        self.path.append((draw_x, draw_y))

        self.traj_img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)

        # Draw Path
        if len(self.path) > 2:
            cv2.polylines(self.traj_img, [np.array(self.path)], isClosed=False, color=(0, 255, 0), thickness=2)

        # Draw Triangle
        self.draw_orientation_triangle(self.traj_img, draw_x, draw_y)
        
        cv2.putText(self.traj_img, f"X: {x:.2f}", (20, 50), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2)
        cv2.putText(self.traj_img, f"Z: {z:.2f}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2)

        # 6. UPDATE
        if len(good_new) < MIN_FEATURES:
            self.detect_new_features(frame_gray)
        else:
            self.px_ref = good_new
            self.frame_ref = frame_gray

        # --- THE FIX IS HERE ---
        # We flatten 'pt' using ravel() so we get simple x, y numbers
        for pt in good_new:
            a, b = pt.ravel()
            cv2.circle(frame, (int(a), int(b)), 2, (0, 0, 255), -1)

        return frame, self.traj_img

    def detect_new_features(self, gray_frame):
        self.px_ref = cv2.goodFeaturesToTrack(gray_frame, mask=None, **self.feature_params)
        self.frame_ref = gray_frame
        
    def draw_orientation_triangle(self, img, x, y):
        yaw = np.arctan2(self.curr_R[0, 2], self.curr_R[2, 2])
        size = 20
        pt1 = (int(x + size * np.sin(yaw)), int(y + size * np.cos(yaw)))
        pt2 = (int(x + size * np.sin(yaw + 2.5)), int(y + size * np.cos(yaw + 2.5)))
        pt3 = (int(x + size * np.sin(yaw - 2.5)), int(y + size * np.cos(yaw - 2.5)))
        triangle_cnt = np.array([pt1, pt2, pt3])
        cv2.drawContours(img, [triangle_cnt], 0, (0, 0, 255), -1)

# --- RUNNER ---
cap = cv2.VideoCapture(0)
slam = AdvancedSLAM()

print("ADVANCED SLAM INITIALIZED")
while True:
    ret, frame = cap.read()
    if not ret:
        break

    cam_view, map_view = slam.process_frame(frame)

    h, w, _ = cam_view.shape
    map_view_resized = cv2.resize(map_view, (h, h))

    combined = np.hstack((cam_view, map_view_resized))

    cv2.imshow("Advanced SLAM", combined)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
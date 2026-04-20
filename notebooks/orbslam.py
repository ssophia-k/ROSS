import cv2
import numpy as np

# --- CONFIGURATION ---
FOCAL_LENGTH = 718.8560
PP = (607.1928, 185.2157)
W, H = 1280, 720
IDLE_THRESHOLD = 10.0  # Pixels. If points move less than this, we are idle.

class FeatureExtractor:
    def __init__(self):
        self.orb = cv2.ORB_create(nfeatures=2000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)

    def detectAndCompute(self, frame):
        # Using a mask to ignore edges (reduces noise)
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.rectangle(mask, (100, 100), (W-100, H-100), 255, -1)
        
        pts = cv2.goodFeaturesToTrack(np.mean(frame, axis=2).astype(np.uint8), 
                                     maxCorners=3000, qualityLevel=0.01, minDistance=7, mask=mask)
        if pts is None: return [], None
        kps = [cv2.KeyPoint(x=f[0][0], y=f[0][1], size=20) for f in pts]
        kps, des = self.orb.compute(frame, kps)
        return kps, des

class PureORBSLAM:
    def __init__(self):
        self.extractor = FeatureExtractor()
        self.cap = cv2.VideoCapture(0)
        
        # State
        self.last_kps = []
        self.last_des = None
        
        # Trajectory
        self.cur_R = np.eye(3)
        self.cur_t = np.array([0, 0, 0], dtype=np.float32)
        self.trajectory = []

    def process_frame(self, frame):
        kps, des = self.extractor.detectAndCompute(frame)
        vis = frame.copy()
        
        # Default status
        status_text = "IDLE"
        color = (0, 0, 255) # Red for idle
        
        if self.last_kps and self.last_des is not None and des is not None:
            matches = self.extractor.bf.knnMatch(self.last_des, des, k=2)
            
            good = []
            for m, n in matches:
                if m.distance < 0.75 * n.distance:
                    good.append(m)
            
            if len(good) > 20:
                q1 = np.float32([self.last_kps[m.queryIdx].pt for m in good])
                q2 = np.float32([kps[m.trainIdx].pt for m in good])
                
                # --- THE FIX: CALCULATE PIXEL MOVEMENT ---
                # Calculate average distance points moved on screen
                pixel_diff = np.mean(np.linalg.norm(q1 - q2, axis=1))
                
                # Only run math if pixels moved enough
                if pixel_diff > IDLE_THRESHOLD:
                    E, mask = cv2.findEssentialMat(q1, q2, focal=FOCAL_LENGTH, pp=PP, method=cv2.RANSAC, prob=0.999, threshold=1.0)
                    
                    if E is not None:
                        _, R, t, _ = cv2.recoverPose(E, q1, q2, focal=FOCAL_LENGTH, pp=PP)
                        
                        scale = 1.0 # We are moving!
                        status_text = "MOVING"
                        color = (0, 255, 0) # Green
                        
                        # Update Position
                        self.cur_t = self.cur_t + scale * self.cur_R.dot(t.flatten())
                        self.cur_R = R.dot(self.cur_R)
                        self.trajectory.append((self.cur_t[0], self.cur_t[2]))
                else:
                    # We are idle (scale = 0)
                    pass

                # Draw Matches
                for pt1, pt2 in zip(q1, q2):
                    u1, v1 = map(int, pt1)
                    u2, v2 = map(int, pt2)
                    cv2.line(vis, (u1, v1), (u2, v2), color, 1)

        cv2.putText(vis, f"STATUS: {status_text}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        self.last_kps = kps
        self.last_des = des
        return vis

    def draw_map(self):
        map_img = np.zeros((600, 600, 3), dtype=np.uint8)
        
        if len(self.trajectory) > 2:
            traj_arr = np.array(self.trajectory)
            mean_x = np.mean(traj_arr[:, 0])
            mean_z = np.mean(traj_arr[:, 1])
            
            center_x, center_z = 300, 300
            scale = 5.0
            
            scaled_path = []
            for x, z in self.trajectory:
                dx = int((x - mean_x) * scale + center_x)
                dy = int((z - mean_z) * scale + center_z)
                scaled_path.append((dx, dy))
            
            cv2.polylines(map_img, [np.array(scaled_path)], False, (0, 255, 255), 2)
            
            cur_x, cur_z = scaled_path[-1]
            cv2.circle(map_img, (cur_x, cur_z), 5, (0, 0, 255), -1)
            cv2.putText(map_img, "YOU", (cur_x+10, cur_z), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)

        return map_img

    def run(self):
        print("Starting Stable ORB-SLAM...")
        while True:
            ret, frame = self.cap.read()
            if not ret: break
            
            frame = cv2.resize(frame, (W, H))
            
            tracking_view = self.process_frame(frame)
            map_view = self.draw_map()
            
            cv2.imshow("Tracking", tracking_view)
            cv2.imshow("Map", map_view)
            
            if cv2.waitKey(1) == ord('q'): break
            
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    slam = PureORBSLAM()
    slam.run()
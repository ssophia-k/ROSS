import cv2
import numpy as np
import math

# --- CONFIGURATION ---
FOCAL_LENGTH = 718.8560
PP = (607.1928, 185.2157)
W, H = 1280, 720

class FeatureExtractor:
    def __init__(self):
        self.orb = cv2.ORB_create(nfeatures=3000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)

    def detectAndCompute(self, frame):
        # Ignore edges to reduce noise
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.rectangle(mask, (100, 100), (W-100, H-100), 255, -1)
        
        pts = cv2.goodFeaturesToTrack(np.mean(frame, axis=2).astype(np.uint8), 
                                     maxCorners=3000, qualityLevel=0.01, minDistance=7, mask=mask)
        if pts is None: return [], None
        kps = [cv2.KeyPoint(x=f[0][0], y=f[0][1], size=20) for f in pts]
        kps, des = self.orb.compute(frame, kps)
        return kps, des

class AdvancedORBSLAM:
    def __init__(self):
        self.extractor = FeatureExtractor()
        self.cap = cv2.VideoCapture(0)
        
        # SLAM State
        self.last_kps = []
        self.last_des = None
        
        # Position (X, Z) and Rotation (Yaw)
        self.cur_R = np.eye(3)
        self.cur_t = np.array([0, 0, 0], dtype=np.float32)
        self.trajectory = []
        self.global_yaw = 0.0 # Degrees
        
        # Tuning
        self.rotation_threshold = 2.0 # Pixels of horizontal flow to consider "Rotation"

    def analyze_flow(self, q1, q2):
        # q1 = Old Points, q2 = New Points
        # Calculate Flow Vectors: How much did points move (dx, dy)?
        flow = q2 - q1
        dx = flow[:, 0]
        dy = flow[:, 1]
        
        avg_dx = np.mean(dx)
        avg_dy = np.mean(dy)
        
        # Calculate divergence (Expansion/Contraction) for Forward/Back
        # We compare distance from center
        center = np.array([W/2, H/2])
        dist_old = np.linalg.norm(q1 - center, axis=1)
        dist_new = np.linalg.norm(q2 - center, axis=1)
        expansion = np.mean(dist_new - dist_old)
        
        return avg_dx, avg_dy, expansion

    def process_frame(self, frame):
        kps, des = self.extractor.detectAndCompute(frame)
        vis = frame.copy()
        status_text = "IDLE"
        
        if self.last_kps and self.last_des is not None and des is not None:
            matches = self.extractor.bf.knnMatch(self.last_des, des, k=2)
            
            good = []
            for m, n in matches:
                if m.distance < 0.70 * n.distance:
                    good.append(m)
            
            if len(good) > 20:
                q1 = np.float32([self.last_kps[m.queryIdx].pt for m in good])
                q2 = np.float32([kps[m.trainIdx].pt for m in good])
                
                # --- 1. OPTICAL FLOW ANALYSIS ---
                avg_dx, avg_dy, expansion = self.analyze_flow(q1, q2)
                
                # Logic: Distinguish Rotation vs Translation
                is_rotating = False
                turn_direction = ""
                
                # If everything moves LEFT (avg_dx < -Threshold) -> Rotating RIGHT
                if avg_dx < -self.rotation_threshold:
                    is_rotating = True
                    turn_direction = "RIGHT (CW)"
                    self.global_yaw += 2.0 # Hardcoded turn speed for stability
                    
                # If everything moves RIGHT (avg_dx > Threshold) -> Rotating LEFT
                elif avg_dx > self.rotation_threshold:
                    is_rotating = True
                    turn_direction = "LEFT (CCW)"
                    self.global_yaw -= 2.0
                
                # Update Rotation Matrix from our manually calculated Yaw
                # This keeps the math clean (Pure Rotation around Y-axis)
                rad_yaw = math.radians(self.global_yaw)
                c, s = math.cos(rad_yaw), math.sin(rad_yaw)
                # Rotation matrix for Y-axis rotation
                self.cur_R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
                
                # --- 2. MOVEMENT LOGIC ---
                # Only move if we are NOT rotating significantly
                # AND there is expansion (forward/back)
                if not is_rotating and abs(expansion) > 0.5:
                    status_text = "MOVING"
                    
                    # Calculate Forward Vector based on Yaw
                    # If expansion > 0, we move forward. If < 0, backward.
                    step = 2.0 if expansion > 0 else -2.0
                    
                    self.cur_t[0] += step * math.sin(rad_yaw)
                    self.cur_t[2] += step * math.cos(rad_yaw)
                    
                    self.trajectory.append((self.cur_t[0], self.cur_t[2]))
                elif is_rotating:
                    status_text = f"ROTATING {turn_direction}"

                # Draw Flow Lines
                for pt1, pt2 in zip(q1, q2):
                    u1, v1 = map(int, pt1)
                    u2, v2 = map(int, pt2)
                    color = (0, 0, 255) if is_rotating else (0, 255, 0)
                    cv2.line(vis, (u1, v1), (u2, v2), color, 1)

        # Status Display
        cv2.putText(vis, f"STATUS: {status_text}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.putText(vis, f"YAW: {int(self.global_yaw)} deg", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        self.last_kps = kps
        self.last_des = des
        return vis

    def draw_map(self):
        map_img = np.zeros((600, 600, 3), dtype=np.uint8)
        
        # Center Map
        cx, cy = 300, 300
        
        # Draw Trajectory
        if len(self.trajectory) > 2:
            # We need to shift trajectory so the start is at center
            start_x, start_z = self.trajectory[0]
            
            scaled_path = []
            for x, z in self.trajectory:
                # Scale 3.0
                dx = cx + int((x - start_x) * 3.0)
                dy = cy - int((z - start_z) * 3.0) # Invert Z for map display
                scaled_path.append((dx, dy))
            
            cv2.polylines(map_img, [np.array(scaled_path)], False, (0, 255, 255), 2)
            
            # Current Pos
            cur_x, cur_z = scaled_path[-1]
        else:
            cur_x, cur_z = cx, cy

        # Draw Rotating Arrow (You)
        # Use our clean global_yaw
        rad = math.radians(self.global_yaw)
        
        # Triangle Geometry
        size = 20
        # Tip points in direction of yaw
        tip_x = int(cur_x + size * math.sin(rad))
        tip_y = int(cur_z - size * math.cos(rad))
        
        left_x = int(cur_x + size/2 * math.sin(rad + 2.5))
        left_y = int(cur_z - size/2 * math.cos(rad + 2.5))
        
        right_x = int(cur_x + size/2 * math.sin(rad - 2.5))
        right_y = int(cur_z - size/2 * math.cos(rad - 2.5))
        
        triangle = np.array([(tip_x, tip_y), (left_x, left_y), (right_x, right_y)])
        cv2.drawContours(map_img, [triangle], 0, (0, 0, 255), -1)
        
        return map_img

    def run(self):
        print("Starting Flow-Based SLAM...")
        while True:
            ret, frame = self.cap.read()
            if not ret: break
            frame = cv2.resize(frame, (W, H))
            
            tracking_view = self.process_frame(frame)
            map_view = self.draw_map()
            
            h, w = tracking_view.shape[:2]
            map_resized = cv2.resize(map_view, (h, h))
            combined = np.hstack((tracking_view, map_resized))
            
            cv2.imshow("Advanced Flow SLAM", combined)
            if cv2.waitKey(1) == ord('q'): break
            
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    slam = AdvancedORBSLAM()
    slam.run()
import numpy as np
import cv2
import os

# read the video file frame by frame
video_path = os.path.join('sanjay-test', "videos", 'test-vid.mp4')
cap = cv2.VideoCapture(video_path)

# set Lucas-Kanade Parameters
lk_params = dict(
    winSize=(15, 15),
    maxLevel=2,
    criteria=(cv2.TERM_CRITERIA_EPS |   cv2.TERM_CRITERIA_COUNT, 10, 0.03)
    )

# random color to draw with
color = np.random.randint(0, 255, (100, 3))

# outputs a token ret that returns true if frame was successfully obtained
# and outputs the previous frame itself
ret, prev_frame = cap.read()
if ret is False:
    print("Failed to read video")
    exit()
# make previous frame gray
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

# get corners using ORB
prev_kpts = cv2.goodFeaturesToTrack(prev_gray, mask=None, maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
# need to make mask for drawing stuff
mask = np.zeros_like(prev_frame)

# Loop through the rest of the video
while True:
    ret, frame = cap.read()
    if ret is False:
        break
    
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Calculate optical flow
    next_pts, status, err = cv2.calcOpticalFlowPyrLK(prev_gray, frame_gray, prev_kpts, None, **lk_params)
     
    if next_pts is not None:
        good_new = next_pts[status==1]
        good_old = prev_kpts[status==1]
    # draw the tracks
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
        frame = cv2.circle(frame, (int(a), int(b)), 5, color[i].tolist(), -1)
    img = cv2.add(frame, mask)
    cv2.imshow('frame', img)
    cv2.waitKey(1)  # 1ms delay between frames
    
    # Update for next iteration
    prev_gray = frame_gray
    prev_kpts = good_new.reshape(-1, 1, 2) if next_pts is not None else prev_kpts

cap.release()
cv2.destroyAllWindows()

import cv2
import numpy as np
import pandas as pd
import os

BASE = os.path.join(os.path.dirname(__file__), "..")
CAM_CSV    = os.path.join(BASE, "data/mav0/cam0/data.csv")
CAM_DIR    = os.path.join(BASE, "data/mav0/cam0/data")
OUT_CSV    = os.path.join(BASE, "results/poses.csv")

# camera intrinsics from EuRoC calibration
fx, fy = 458.654, 457.296
cx, cy = 367.215, 248.375
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])

# FAST + KLT parameters
FAST_THRESHOLD  = 20
MAX_CORNERS     = 200
KLT_WIN_SIZE    = (21, 21)
KLT_MAX_LEVEL   = 3

def load_frames(csv_path):
    df = pd.read_csv(csv_path, comment="#", header=None)
    return [(row[0], os.path.join(CAM_DIR, row[1])) for _, row in df.iterrows()]

def detect_features(img):
    keypoints = cv2.FastFeatureDetector_create(FAST_THRESHOLD).detect(img)

    # Keeping corners below 200
    if len(keypoints) > MAX_CORNERS:
        keypoints = keypoints[:MAX_CORNERS]

    features_xy = np.array([kp.pt for kp in keypoints], dtype=np.float32)

    return features_xy

def track_features(img_prev, img_curr, pts_prev):

    # Figure out which points were tracked succesfully and what the current point positions are.
    pts_curr, status, _ = cv2.calcOpticalFlowPyrLK(img_prev, img_curr, pts_prev, None,
                                                 winSize=KLT_WIN_SIZE,
                                                 maxLevel=KLT_MAX_LEVEL)
    # Filtering for successfully tracked points
    good = status[:, 0] == 1

    return pts_curr, good

def estimate_pose(pts_prev, pts_curr):

    # estimate essential matrix from point movement. Use RANSAC to filter out outliers. 
    E, mask = cv2.findEssentialMat(pts_prev, pts_curr, K, method=cv2.RANSAC)

    _, R, t, mask = cv2.recoverPose(E, pts_prev, pts_curr, K)
    return R, t, mask


if __name__ == "__main__":
    frames = load_frames(CAM_CSV)
    out = open(OUT_CSV, "w")
    out.write("t_ns,r00,r01,r02,r10,r11,r12,r20,r21,r22,tx,ty,tz\n")

    pts_prev = None
    img_prev = None

    for i, (t_ns, fpath) in enumerate(frames):
        img = cv2.imread(fpath, cv2.IMREAD_GRAYSCALE)

        if img_prev is None:
            img_prev = img
            pts_prev = detect_features(img)
            continue

        pts_curr, good = track_features(img_prev, img_curr=img, pts_prev=pts_prev)

        if good.sum() > 8:
            R, t, _ = estimate_pose(pts_prev[good], pts_curr[good])

            # reject bad poses — drone can't rotate more than 5° in 50ms
            angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
            if angle < 5.0:
                r = R.flatten()
                out.write(f"{t_ns},{r[0]},{r[1]},{r[2]},{r[3]},{r[4]},{r[5]},{r[6]},{r[7]},{r[8]},{t[0][0]},{t[1][0]},{t[2][0]}\n")

        # re-detect features periodically to replace lost tracks
        if good.sum() < MAX_CORNERS // 2:
            pts_prev = detect_features(img)
        else:
            pts_prev = pts_curr[good]

        img_prev = img

    out.close()
    print(f"Done. Poses saved to {OUT_CSV}")

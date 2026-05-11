import cv2
import numpy as np
import pandas as pd
import os

BASE = os.path.join(os.path.dirname(__file__), "../..")
CAM_CSV    = os.path.join(BASE, "data/mav0/cam0/data.csv")
CAM_DIR    = os.path.join(BASE, "data/mav0/cam0/data")
OUT_CSV    = os.path.join(BASE, "results/poses_mono_final.csv")

# camera intrinsics from EuRoC calibration
fx, fy = 458.654, 457.296
cx, cy = 367.215, 248.375
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])
dist_coeffs = np.array([-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05])

# T_BS (body-from-cam0) from data/mav0/cam0/sensor.yaml — needed to express
# VO translation in the body frame, where the ESKF state lives.
R_cam0_to_body = np.array([
    [ 0.0148655429818, -0.999880929698,    0.00414029679422],
    [ 0.999557249008,   0.0149672133247,   0.025715529948  ],
    [-0.0257744366974,  0.00375618835797,  0.999660727178  ],
])

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
    keypoints = sorted(keypoints, key = lambda kp: kp.response, reverse = True)

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

  

    # Use distortion coefficients to correct pixels due to wide angle lens
    pts_prev_u = cv2.undistortPoints(pts_prev, K, dist_coeffs, P=K).squeeze()
    pts_curr_u = cv2.undistortPoints(pts_curr, K, dist_coeffs, P=K).squeeze()

     # estimate essential matrix from point movement. Use RANSAC to filter out outliers. 
    E, mask = cv2.findEssentialMat(pts_prev_u, pts_curr_u, K, method=cv2.RANSAC)
    _, R, t, mask = cv2.recoverPose(E, pts_prev_u, pts_curr_u, K)
     
    return R, t, mask


def run(args):

    output_path = args.output
    frames = load_frames(CAM_CSV)
    out = open(output_path, "w")
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
                # t from recoverPose is unit translation in cam_curr frame.
                # Convert to direction in body frame so ESKF (body-frame state)
                # consumes a consistent measurement. Stereo tracker already
                # does this; mono was missing it.
                t_cam = t.flatten()
                t_body = R_cam0_to_body @ t_cam
                r = R.flatten()
                out.write(f"{t_ns},{r[0]},{r[1]},{r[2]},{r[3]},{r[4]},{r[5]},{r[6]},{r[7]},{r[8]},{t_body[0]},{t_body[1]},{t_body[2]}\n")

        # re-detect features periodically to replace lost tracks
        if good.sum() < MAX_CORNERS // 2:
            pts_prev = detect_features(img)
        else:
            pts_prev = pts_curr[good]

        img_prev = img

    out.close()
    print(f"Done. Poses saved to {output_path}")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--output", default=OUT_CSV)
    run(p.parse_args())
    

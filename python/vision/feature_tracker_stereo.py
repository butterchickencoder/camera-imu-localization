"""
Stereo VO pipeline for EuRoC.

Pipeline:
  Frame k:   detect features in left image → stereo-match to right → triangulate → pts_3d_k
  Frame k+1: temporal-track features in left image → solvePnP(pts_3d_k, pts_2d_k+1) → R, t
  Write metric velocity (body frame) to poses.csv
"""
import cv2
import numpy as np
import pandas as pd
import os
from .stereo_calibration import load_sensor_yaml, compute_stereo_extrinsics

BASE = os.path.join(os.path.dirname(__file__), "../..")
CAM0_CSV = os.path.join(BASE, "data/mav0/cam0/data.csv")
CAM1_CSV = os.path.join(BASE, "data/mav0/cam1/data.csv")
CAM0_DIR = os.path.join(BASE, "data/mav0/cam0/data")
CAM1_DIR = os.path.join(BASE, "data/mav0/cam1/data")
OUT_CSV  = os.path.join(BASE, "results/poses.csv")

FAST_THRESHOLD = 20
MAX_CORNERS    = 300
MIN_FEATURES   = 50
KLT_WIN_SIZE   = (21, 21)
KLT_MAX_LEVEL  = 3
STEREO_MAX_LEVEL = 5

# Load calibration
K0, dist0, T_BS_0 = load_sensor_yaml(os.path.join(BASE, "data/mav0/cam0/sensor.yaml"))
K1, dist1, T_BS_1 = load_sensor_yaml(os.path.join(BASE, "data/mav0/cam1/sensor.yaml"))
R_c1_c0, t_c1_c0, BASELINE = compute_stereo_extrinsics(T_BS_0, T_BS_1)
IMG_SIZE = (752, 480)
print(f"Baseline: {BASELINE*100:.2f} cm")

# Stereo rectification — aligns epipolar lines horizontally
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    K0, dist0, K1, dist1, IMG_SIZE, R_c1_c0, t_c1_c0,
    flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
)
MAP0_X, MAP0_Y = cv2.initUndistortRectifyMap(K0, dist0, R1, P1, IMG_SIZE, cv2.CV_32FC1)
MAP1_X, MAP1_Y = cv2.initUndistortRectifyMap(K1, dist1, R2, P2, IMG_SIZE, cv2.CV_32FC1)

# After rectification, the NEW intrinsics are P1[:3,:3] (same for both rectified cams)
fx, fy = P1[0, 0], P1[1, 1]
cx, cy = P1[0, 2], P1[1, 2]
BASELINE_RECT = -P2[0, 3] / fx
print(f"Rectified baseline: {BASELINE_RECT*100:.2f} cm, fx_rect={fx:.2f}")

# Frame transformations:
# - R1 transforms from rectified cam0 frame to original cam0 frame
# - T_BS_0 transforms from original cam0 frame to body frame
# Combined: rectified_cam0 -> body
R_rect_to_cam0 = R1.T   # rectified points to original cam0
R_cam0_to_body = T_BS_0[:3, :3]  # from sensor.yaml
R_RECT_TO_BODY = R_cam0_to_body @ R_rect_to_cam0


def load_frames(csv_path, img_dir):
    df = pd.read_csv(csv_path, comment="#", header=None)
    return {int(row[0]): os.path.join(img_dir, row[1]) for _, row in df.iterrows()}


def detect_features(img):
    kps = cv2.FastFeatureDetector_create(FAST_THRESHOLD).detect(img)
    kps = sorted(kps, key=lambda kp: kp.response, reverse=True)
    if len(kps) > MAX_CORNERS:
        kps = kps[:MAX_CORNERS]
    return np.array([kp.pt for kp in kps], dtype=np.float32).reshape(-1, 1, 2)


_ORB = cv2.ORB_create(nfeatures=1500, scaleFactor=1.2, nlevels=6)
_BF = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)


def stereo_match(img_l, img_r):
    """ORB + BFMatcher. Returns matched left/right pixel coordinates (Nx2)."""
    kp_l, des_l = _ORB.detectAndCompute(img_l, None)
    kp_r, des_r = _ORB.detectAndCompute(img_r, None)
    if des_l is None or des_r is None:
        return np.empty((0, 2), np.float32), np.empty((0, 2), np.float32)

    matches = _BF.match(des_l, des_r)
    if len(matches) == 0:
        return np.empty((0, 2), np.float32), np.empty((0, 2), np.float32)

    pts_l = np.array([kp_l[m.queryIdx].pt for m in matches], dtype=np.float32)
    pts_r = np.array([kp_r[m.trainIdx].pt for m in matches], dtype=np.float32)

    y_diff = np.abs(pts_l[:, 1] - pts_r[:, 1])
    disparity = pts_l[:, 0] - pts_r[:, 0]
    valid = (y_diff < 2.5) & (disparity > 1.0) & (disparity < 150.0)
    return pts_l[valid], pts_r[valid]


def triangulate(pts_l, pts_r):
    """Triangulation from rectified stereo: Z = fx * B / d."""
    disparity = pts_l[:, 0] - pts_r[:, 0]
    Z = fx * BASELINE_RECT / disparity
    X = (pts_l[:, 0] - cx) * Z / fx
    Y = (pts_l[:, 1] - cy) * Z / fy
    return np.stack([X, Y, Z], axis=1).astype(np.float32)


def run(args):
    output_path = args.output
    frames_l = load_frames(CAM0_CSV, CAM0_DIR)
    frames_r = load_frames(CAM1_CSV, CAM1_DIR)
    # Only use timestamps that exist in both
    timestamps = sorted(set(frames_l.keys()) & set(frames_r.keys()))
    print(f"Matched stereo frames: {len(timestamps)}")

    out = open(output_path, "w")
    out.write("t_ns,r00,r01,r02,r10,r11,r12,r20,r21,r22,tx,ty,tz\n")

    img_l_prev = None
    feat_l_prev = None     # 2D features in previous left frame (Nx2)
    pts_3d_prev = None     # 3D points matching feat_l_prev (Nx3)
    t_prev = None

    written = 0
    rejected = 0

    for i, t_ns in enumerate(timestamps):
        img_l = cv2.imread(frames_l[t_ns], cv2.IMREAD_GRAYSCALE)
        img_r = cv2.imread(frames_r[t_ns], cv2.IMREAD_GRAYSCALE)

        # Rectify: undistorts AND aligns epipolar lines horizontally
        img_l = cv2.remap(img_l, MAP0_X, MAP0_Y, cv2.INTER_LINEAR)
        img_r = cv2.remap(img_r, MAP1_X, MAP1_Y, cv2.INTER_LINEAR)

        # --- Step A: temporal tracking from previous left frame (if we have one) ---
        if img_l_prev is not None and feat_l_prev is not None and len(feat_l_prev) >= 8:
            feat_l_tracked, status, _ = cv2.calcOpticalFlowPyrLK(
                img_l_prev, img_l,
                feat_l_prev.reshape(-1, 1, 2), None,
                winSize=KLT_WIN_SIZE,
                maxLevel=KLT_MAX_LEVEL,
            )
            status = status.flatten().astype(bool)
            feat_l_tracked = feat_l_tracked.reshape(-1, 2)

            good = status & np.all(np.isfinite(feat_l_tracked), axis=1)
            if good.sum() >= 8:
                pts_3d_good = pts_3d_prev[good]
                pts_2d_good = feat_l_tracked[good]

                # solvePnP: finds R,t such that X_in_cam_curr = R * X_in_cam_prev + t
                try:
                    K_rect = P1[:3, :3]
                    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                        pts_3d_good.reshape(-1, 1, 3),
                        pts_2d_good.reshape(-1, 1, 2),
                        K_rect, None,
                        reprojectionError=3.0,
                        iterationsCount=100,
                        flags=cv2.SOLVEPNP_ITERATIVE,
                    )
                except cv2.error:
                    ok = False

                if ok and inliers is not None and len(inliers) >= 6:
                    R_vo, _ = cv2.Rodrigues(rvec)
                    t_vo = tvec.flatten()  # position of prev cam origin in curr cam frame

                    # Rejection: large rotation implies bad pose
                    rot_angle = np.degrees(np.arccos(np.clip((np.trace(R_vo) - 1) / 2, -1, 1)))
                    if rot_angle < 5.0:
                        dt = (t_ns - t_prev) * 1e-9
                        # Velocity in rectified cam0 frame: -R^T * t / dt
                        v_rect = (-R_vo.T @ t_vo) / dt
                        # Transform to body frame (ESKF expects body-frame velocity)
                        v_body = R_RECT_TO_BODY @ v_rect
                        r = R_vo.flatten()
                        out.write(
                            f"{t_ns},{r[0]},{r[1]},{r[2]},{r[3]},{r[4]},{r[5]},{r[6]},{r[7]},{r[8]},"
                            f"{v_body[0]},{v_body[1]},{v_body[2]}\n"
                        )
                        written += 1
                    else:
                        rejected += 1

        # --- Step B: stereo-match current frame → 3D points for NEXT iteration ---
        fl_matched, fr_matched = stereo_match(img_l, img_r)

        if len(fl_matched) >= MIN_FEATURES:
            pts_3d_curr = triangulate(fl_matched, fr_matched)
            # Reject points with unrealistic depth
            good_depth = (pts_3d_curr[:, 2] > 0.5) & (pts_3d_curr[:, 2] < 50.0)
            fl_matched = fl_matched[good_depth]
            pts_3d_curr = pts_3d_curr[good_depth]

            feat_l_prev = fl_matched
            pts_3d_prev = pts_3d_curr
        else:
            feat_l_prev = None
            pts_3d_prev = None

        img_l_prev = img_l
        t_prev = t_ns

        if i % 500 == 0:
            print(f"  {i}/{len(timestamps)} frames processed, {written} poses written")

    out.close()
    print(f"Done. Wrote {written} poses to {output_path}, rejected {rejected} by rotation gate.")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--output", default=OUT_CSV)
    run(p.parse_args())

"""
EuRoC Dataset Explorer
----------------------
Run this after extracting MH_01_easy.zip to data/MH_01_easy/

Usage:
    python python/explore_data.py

Prints a summary of all sensor data: IMU, camera, ground truth.
Then plots the first 5 seconds of IMU data and shows a few camera frames.
"""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2

# ── Paths ──────────────────────────────────────────────────────────────────────
BASE = os.path.join(os.path.dirname(__file__), "..", "data", "MH_01_easy", "mav0")
IMU_CSV        = os.path.join(BASE, "imu0", "data.csv")
CAM_CSV        = os.path.join(BASE, "cam0", "data.csv")
CAM_IMAGES_DIR = os.path.join(BASE, "cam0", "data")
GT_CSV         = os.path.join(BASE, "state_groundtruth_estimate0", "data.csv")


def load_imu(path):
    """
    Columns: timestamp [ns], omega_x, omega_y, omega_z [rad/s],
                              alpha_x, alpha_y, alpha_z [m/s^2]
    """
    df = pd.read_csv(path, comment="#")
    df.columns = ["t_ns", "gx", "gy", "gz", "ax", "ay", "az"]
    df["t_s"] = (df["t_ns"] - df["t_ns"].iloc[0]) * 1e-9
    return df


def load_camera(path):
    """
    Columns: timestamp [ns], filename
    """
    df = pd.read_csv(path, comment="#")
    df.columns = ["t_ns", "filename"]
    df["t_s"] = (df["t_ns"] - df["t_ns"].iloc[0]) * 1e-9
    return df


def load_ground_truth(path):
    """
    Columns: timestamp, p_x, p_y, p_z [m],
             q_w, q_x, q_y, q_z,
             v_x, v_y, v_z [m/s],
             b_omega_x/y/z, b_alpha_x/y/z
    """
    df = pd.read_csv(path, comment="#")
    df.columns = [
        "t_ns",
        "px", "py", "pz",
        "qw", "qx", "qy", "qz",
        "vx", "vy", "vz",
        "bgx", "bgy", "bgz",
        "bax", "bay", "baz",
    ]
    df["t_s"] = (df["t_ns"] - df["t_ns"].iloc[0]) * 1e-9
    return df


def print_summary(imu, cam, gt):
    print("=" * 60)
    print("EuRoC MH_01_easy — Dataset Summary")
    print("=" * 60)

    dur = imu["t_s"].iloc[-1]
    rate_imu = len(imu) / dur
    rate_cam = len(cam) / dur

    print(f"\nDuration       : {dur:.1f} s")
    print(f"\nIMU")
    print(f"  Samples      : {len(imu)}")
    print(f"  Rate         : {rate_imu:.0f} Hz")
    print(f"  Gyro range   : {imu[['gx','gy','gz']].abs().max().max():.3f} rad/s")
    print(f"  Accel range  : {imu[['ax','ay','az']].abs().max().max():.3f} m/s²")

    print(f"\nCamera")
    print(f"  Frames       : {len(cam)}")
    print(f"  Rate         : {rate_cam:.1f} Hz")

    print(f"\nGround Truth")
    print(f"  Samples      : {len(gt)}")
    pos_range = gt[["px", "py", "pz"]].max() - gt[["px", "py", "pz"]].min()
    print(f"  Position range (x,y,z): "
          f"{pos_range['px']:.2f} m, {pos_range['py']:.2f} m, {pos_range['pz']:.2f} m")
    print("=" * 60)


def plot_imu(imu, duration_s=5.0):
    mask = imu["t_s"] <= duration_s
    t = imu.loc[mask, "t_s"]

    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    fig.suptitle(f"IMU data — first {duration_s} s", fontsize=13)

    axes[0].plot(t, imu.loc[mask, "gx"], label="gx")
    axes[0].plot(t, imu.loc[mask, "gy"], label="gy")
    axes[0].plot(t, imu.loc[mask, "gz"], label="gz")
    axes[0].set_ylabel("Angular rate [rad/s]")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, imu.loc[mask, "ax"], label="ax")
    axes[1].plot(t, imu.loc[mask, "ay"], label="ay")
    axes[1].plot(t, imu.loc[mask, "az"], label="az")
    axes[1].set_ylabel("Acceleration [m/s²]")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("results/imu_preview.png", dpi=150)
    print("\nSaved: results/imu_preview.png")
    plt.show()


def plot_ground_truth(gt):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Ground Truth Trajectory", fontsize=13)

    axes[0].plot(gt["px"], gt["py"])
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].set_title("Top view (x-y)")
    axes[0].axis("equal")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(gt["t_s"], gt["pz"])
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("z [m]")
    axes[1].set_title("Altitude over time")
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("results/ground_truth.png", dpi=150)
    print("Saved: results/ground_truth.png")
    plt.show()


def show_camera_frames(cam, n=6):
    indices = np.linspace(0, len(cam) - 1, n, dtype=int)
    fig, axes = plt.subplots(2, 3, figsize=(14, 6))
    fig.suptitle("Camera frames (evenly spaced)", fontsize=13)

    for ax, idx in zip(axes.flat, indices):
        img_path = os.path.join(CAM_IMAGES_DIR, cam.iloc[idx]["filename"])
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            ax.imshow(img, cmap="gray")
            ax.set_title(f"t = {cam.iloc[idx]['t_s']:.1f} s")
        ax.axis("off")

    plt.tight_layout()
    plt.savefig("results/camera_frames.png", dpi=150)
    print("Saved: results/camera_frames.png")
    plt.show()


if __name__ == "__main__":
    os.makedirs("results", exist_ok=True)

    print("Loading data...")
    imu = load_imu(IMU_CSV)
    cam = load_camera(CAM_CSV)
    gt  = load_ground_truth(GT_CSV)

    print_summary(imu, cam, gt)
    plot_imu(imu)
    plot_ground_truth(gt)
    show_camera_frames(cam)

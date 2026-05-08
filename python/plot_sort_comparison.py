"""
Compare trajectories: feature tracker with sorted keypoints vs unsorted.
Shows both configs: IMU + VO, and IMU + Baro + VO.
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

BASE = os.path.join(os.path.dirname(__file__), "..")


def load_traj(path):
    df = pd.read_csv(path)
    return df["t_ns"].values, df[["px", "py", "pz"]].values


def rmse(est_xyz, est_t, gt_t, gt_xyz):
    gx = np.interp(est_t, gt_t, gt_xyz[:, 0])
    gy = np.interp(est_t, gt_t, gt_xyz[:, 1])
    gz = np.interp(est_t, gt_t, gt_xyz[:, 2])
    err = np.sqrt((est_xyz[:, 0] - gx) ** 2
                  + (est_xyz[:, 1] - gy) ** 2
                  + (est_xyz[:, 2] - gz) ** 2)
    return err.mean()


# Ground truth
gt = pd.read_csv(os.path.join(BASE, "data/mav0/state_groundtruth_estimate0/data.csv"))
gt_t = gt.iloc[:, 0].values
gt_xyz = gt.iloc[:, 1:4].values

# Load all four variants
configs = [
    ("traj_imu_vo",       "IMU + VO"),
    ("traj_imu_baro_vo",  "IMU + Baro + VO"),
]
variants = [
    ("unsorted", "r"),
    ("sorted",   "b"),
]

fig, axes = plt.subplots(2, 2, figsize=(14, 12))

for col, (fname, label) in enumerate(configs):
    ax_xy = axes[0, col]
    ax_z  = axes[1, col]

    # Ground truth
    ax_xy.plot(gt_xyz[:, 0], gt_xyz[:, 1], "k-", label="Ground Truth", linewidth=2)
    ax_z.plot(gt_t * 1e-9, gt_xyz[:, 2], "k-", label="Ground Truth", linewidth=2)

    for variant, color in variants:
        path = os.path.join(BASE, f"results/{fname}_{variant}.csv")
        t, xyz = load_traj(path)
        r = rmse(xyz, t, gt_t, gt_xyz)
        ax_xy.plot(xyz[:, 0], xyz[:, 1], color + "-",
                   label=f"{variant} ({r:.2f} m RMSE)", alpha=0.7)
        ax_z.plot(t * 1e-9, xyz[:, 2], color + "-",
                  label=f"{variant}", alpha=0.7)

    ax_xy.set_xlabel("X [m]")
    ax_xy.set_ylabel("Y [m]")
    ax_xy.set_title(f"{label} — XY Top-Down")
    ax_xy.legend()
    ax_xy.grid(True, alpha=0.3)
    ax_xy.axis("equal")

    ax_z.set_xlabel("Zeit [s]")
    ax_z.set_ylabel("Z [m]")
    ax_z.set_title(f"{label} — Höhe über Zeit")
    ax_z.legend()
    ax_z.grid(True, alpha=0.3)

plt.suptitle("Keypoint Sortierung — Vergleich beider Varianten", fontsize=14)
plt.tight_layout()

out_path = os.path.join(BASE, "results/sort_comparison.png")
plt.savefig(out_path, dpi=120, bbox_inches="tight")
print(f"Plot saved to {out_path}\n")

# Print RMSE summary
print(f"{'Config':<20} {'Variant':<12} {'RMSE':>8}")
print("-" * 42)
for fname, label in configs:
    for variant, _ in variants:
        path = os.path.join(BASE, f"results/{fname}_{variant}.csv")
        t, xyz = load_traj(path)
        r = rmse(xyz, t, gt_t, gt_xyz)
        print(f"{label:<20} {variant:<12} {r:>6.2f} m")

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

BASE = os.path.join(os.path.dirname(__file__), "..")

poses = pd.read_csv(os.path.join(BASE, "results/poses.csv"))
gt    = pd.read_csv(os.path.join(BASE, "data/mav0/state_groundtruth_estimate0/data.csv"), comment="#", header=None)
gt.columns = ["t_ns","px","py","pz","qw","qx","qy","qz","vx","vy","vz","bgx","bgy","bgz","bax","bay","baz"]

# integrate relative poses to get a trajectory
positions = [np.zeros(3)]
R_total = np.eye(3)

for _, row in poses.iterrows():
    R = np.array([[row.r00, row.r01, row.r02],
                  [row.r10, row.r11, row.r12],
                  [row.r20, row.r21, row.r22]])
    t = np.array([row.tx, row.ty, row.tz])

    R_total = R_total @ R
    positions.append(positions[-1] + R_total @ t)

positions = np.array(positions)

# scale to match ground truth (mono VO has no absolute scale)
gt_length = np.linalg.norm(gt[["px","py","pz"]].diff().dropna().values, axis=1).sum()
vo_length = np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()
scale = gt_length / vo_length
positions *= scale

# align start
positions += gt[["px","py","pz"]].iloc[0].values

fig, ax = plt.subplots(figsize=(8, 8))
ax.plot(gt["px"], gt["py"], label="Ground truth", linewidth=2)
ax.plot(positions[:, 0], positions[:, 1], label="VO only", linewidth=1, linestyle="--")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("Visual odometry trajectory vs ground truth")
ax.legend()
ax.axis("equal")
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(BASE, "results/vo_trajectory.png"), dpi=150)
print("Saved: results/vo_trajectory.png")
plt.show()

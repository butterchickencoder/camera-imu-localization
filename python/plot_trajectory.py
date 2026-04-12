import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

BASE = os.path.join(os.path.dirname(__file__), "..")

traj = pd.read_csv(os.path.join(BASE, "results/imu_trajectory.csv"))
gt   = pd.read_csv(os.path.join(BASE, "data/mav0/state_groundtruth_estimate0/data.csv"), comment="#")
gt.columns = ["t_ns","px","py","pz","qw","qx","qy","qz","vx","vy","vz","bgx","bgy","bgz","bax","bay","baz"]

# align start position
traj["px"] += gt["px"].iloc[0]
traj["py"] += gt["py"].iloc[0]
traj["pz"] += gt["pz"].iloc[0]

fig, axes = plt.subplots(1, 2, figsize=(14, 5))
fig.suptitle("IMU-only dead reckoning vs ground truth", fontsize=13)

axes[0].plot(gt["px"],   gt["py"],   label="Ground truth", linewidth=2)
axes[0].plot(traj["px"], traj["py"], label="IMU only",     linewidth=1, linestyle="--")
axes[0].set_xlabel("x [m]")
axes[0].set_ylabel("y [m]")
axes[0].set_title("Top view")
axes[0].legend()
axes[0].axis("equal")
axes[0].grid(True, alpha=0.3)

t = (traj["t_ns"] - traj["t_ns"].iloc[0]) * 1e-9
axes[1].plot(t, traj["pz"] - traj["pz"].iloc[0], label="IMU only", linestyle="--")
axes[1].set_xlabel("Time [s]")
axes[1].set_ylabel("z [m]")
axes[1].set_title("Altitude drift")
axes[1].legend()
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(BASE, "results/imu_vs_gt.png"), dpi=150)
print("Saved: results/imu_vs_gt.png")
plt.show()

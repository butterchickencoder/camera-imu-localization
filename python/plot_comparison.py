import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os

BASE = os.path.join(os.path.dirname(__file__), "..")

gt = pd.read_csv(os.path.join(BASE, "data/mav0/state_groundtruth_estimate0/data.csv"), comment="#", header=None)
gt.columns = ["t_ns","px","py","pz","qw","qx","qy","qz","vx","vy","vz","bgx","bgy","bgz","bax","bay","baz"]

imu_only = pd.read_csv(os.path.join(BASE, "results/traj_imu_only.csv"))
imu_vo   = pd.read_csv(os.path.join(BASE, "results/traj_imu_vo.csv"))
imu_baro = pd.read_csv(os.path.join(BASE, "results/traj_imu_baro.csv"))
baro_vo  = pd.read_csv(os.path.join(BASE, "results/traj_imu_baro_vo.csv"))

def calc_rmse(traj, gt):
    gt_interp = {}
    for col in ["px", "py", "pz"]:
        f = interp1d(gt["t_ns"].values, gt[col].values, fill_value="extrapolate")
        gt_interp[col] = f(traj["t_ns"].values)
    err = np.sqrt((traj["px"].values - gt_interp["px"])**2 +
                  (traj["py"].values - gt_interp["py"])**2 +
                  (traj["pz"].values - gt_interp["pz"])**2)
    return np.sqrt(np.mean(err**2))

trajs = {
    "IMU only":       imu_only,
    "IMU+VO":         imu_vo,
    "IMU+Baro":       imu_baro,
    "IMU+Baro+VO":    baro_vo,
}

colors = {
    "IMU only":    "tab:red",
    "IMU+VO":      "tab:blue",
    "IMU+Baro":    "tab:orange",
    "IMU+Baro+VO": "green",
}

styles = {
    "IMU only":    "--",
    "IMU+VO":      ":",
    "IMU+Baro":    "-.",
    "IMU+Baro+VO": "-",
}

rmse = {name: calc_rmse(t, gt) for name, t in trajs.items()}
for name, val in rmse.items():
    print(f"RMSE  {name:20s}: {val:.2f} m")

fig, axes = plt.subplots(1, 2, figsize=(15, 6))
fig.suptitle("GPS-Denied Sensor Fusion Comparison", fontsize=13)

ax = axes[0]
ax.plot(gt["px"], gt["py"], label="Ground truth", linewidth=2, color="black")
for name, t in trajs.items():
    ax.plot(t["px"], t["py"], label=f"{name} ({rmse[name]:.1f}m)",
            linewidth=1.2, linestyle=styles[name], color=colors[name])
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_title("Top view")
ax.legend(fontsize=9); ax.axis("equal"); ax.grid(True, alpha=0.3)

ax = axes[1]
t_gt = (gt["t_ns"] - gt["t_ns"].iloc[0]) * 1e-9
ax.plot(t_gt, gt["pz"], label="Ground truth", linewidth=2, color="black")
for name, traj in trajs.items():
    t_s = (traj["t_ns"] - traj["t_ns"].iloc[0]) * 1e-9
    ax.plot(t_s, traj["pz"], label=name, linewidth=1.2, linestyle=styles[name], color=colors[name])
ax.set_xlabel("Time [s]"); ax.set_ylabel("z [m]"); ax.set_title("Altitude")
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(BASE, "results/comparison.png"), dpi=150)
print("Saved: results/comparison.png")
plt.show()

"""
Plots für Folie "Ergebnisse ohne Fusion"
  1. IMU only  – XY-Trajektorie
  2. VO only   – XY-Trajektorie (Stereo, integriert)
  3. Geschwindigkeitsbetrag: IMU only vs Ground truth
  4. RMSE-Tabelle
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
RES  = os.path.join(BASE, "results")

plt.rcParams.update({
    "font.size": 11,
    "axes.titlesize": 12,
    "axes.labelsize": 11,
    "legend.fontsize": 9.5,
    "figure.dpi": 120,
})

# ── Load data ───────────────────────────────────────────────────────────────
gt_raw  = pd.read_csv(os.path.join(BASE, "data/mav0/state_groundtruth_estimate0/data.csv"),
                       comment="#", header=None).values
GT_TS   = gt_raw[:, 0]
GT_POS  = gt_raw[:, 1:4]
GT_VEL  = gt_raw[:, 8:11]         # vx, vy, vz from GT
GT_T0   = GT_TS[0]

imu_only = pd.read_csv(os.path.join(RES, "traj_imu_only.csv")).values  # t_ns, px, py, pz

# Stereo VO poses: R (3x3) + t (3) per frame, 20 Hz
poses_df = pd.read_csv(os.path.join(RES, "poses.csv"))   # stereo

# ── Build VO-only trajectory ────────────────────────────────────────────────
# VO gives t_body (velocity-like, metric for stereo) at 20 Hz.
# Integrate: p += R_world * t_body * dt  (same convention as ESKF updateVelocity, scale=0.5)
dt_vo    = 0.05   # 20 Hz
SCALE    = 1.0    # stereo: t is already metric velocity (no scale needed)

# Start at first GT position and orientation
def nearest_gt(t_ns):
    j = np.argmin(np.abs(GT_TS - t_ns))
    return GT_POS[j]

def gt_vel_at(t_ns):
    j = np.argmin(np.abs(GT_TS - t_ns))
    return GT_VEL[j]

t_vo    = poses_df["t_ns"].values
R_arr   = poses_df[["r00","r01","r02","r10","r11","r12","r20","r21","r22"]].values
t_arr   = poses_df[["tx","ty","tz"]].values

p_vo  = nearest_gt(t_vo[0]).copy()
vo_traj = [np.array([t_vo[0], *p_vo])]
R_world = np.eye(3)   # accumulated world rotation, start as identity offset

for i in range(1, len(t_vo)):
    R_inc = R_arr[i].reshape(3, 3)
    t_body = t_arr[i]
    # Update world rotation
    R_world = R_world @ R_inc
    # Integrate displacement
    p_vo = p_vo + R_world @ t_body * SCALE * dt_vo
    vo_traj.append(np.array([t_vo[i], *p_vo]))

vo_traj = np.array(vo_traj)

# ── RMSE helper ─────────────────────────────────────────────────────────────
def rmse_max(traj):
    e = []
    for r in traj:
        j = np.argmin(np.abs(GT_TS - r[0]))
        if abs(GT_TS[j] - r[0]) < 2e7:
            e.append(np.linalg.norm(r[1:4] - GT_POS[j]))
    e = np.array(e)
    return np.sqrt(np.mean(e**2)), e.max()

# ── Velocity from IMU-only (finite differences) ──────────────────────────────
dt_imu = np.diff(imu_only[:, 0]) * 1e-9
dp     = np.diff(imu_only[:, 1:4], axis=0)
v_imu  = np.linalg.norm(dp / dt_imu[:, None], axis=1)
t_imu_s = (imu_only[1:, 0] - imu_only[0, 0]) * 1e-9

# GT velocity interpolated to IMU timestamps
gt_vmag_at_imu = []
for t in imu_only[1:, 0]:
    j = np.argmin(np.abs(GT_TS - t))
    gt_vmag_at_imu.append(np.linalg.norm(GT_VEL[j]))
gt_vmag_at_imu = np.array(gt_vmag_at_imu)

# Cap outliers for display
v_imu_clipped = np.clip(v_imu, 0, 30)

# ── Plot ─────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 5))
gs  = gridspec.GridSpec(1, 3, figure=fig, wspace=0.35)

# --- 1. IMU only XY --------------------------------------------------------
ax1 = fig.add_subplot(gs[0])
ax1.plot(GT_POS[:, 0], GT_POS[:, 1], "k-", lw=1.5, label="Ground truth", zorder=3)
ax1.plot(imu_only[:, 1], imu_only[:, 2], color="#e74c3c", lw=1.0, label="IMU only")
ax1.plot(*imu_only[0, 1:3], "go", ms=7, zorder=4, label="Start")
rmse_imu, max_imu = rmse_max(imu_only)
ax1.set_title(f"IMU only\nRMSE {rmse_imu:.1f} m")
ax1.set_xlabel("x [m]"); ax1.set_ylabel("y [m]")
ax1.legend(); ax1.grid(alpha=0.3); ax1.axis("equal")

# --- 2. VO only XY ---------------------------------------------------------
ax2 = fig.add_subplot(gs[1])
ax2.plot(GT_POS[:, 0], GT_POS[:, 1], "k-", lw=1.5, label="Ground truth", zorder=3)
ax2.plot(vo_traj[:, 1], vo_traj[:, 2], color="#3498db", lw=1.0, label="Stereo VO only")
ax2.plot(*vo_traj[0, 1:3], "go", ms=7, zorder=4, label="Start")
rmse_vo, max_vo = rmse_max(vo_traj)
ax2.set_title(f"Stereo VO only\nRMSE {rmse_vo:.1f} m")
ax2.set_xlabel("x [m]"); ax2.set_ylabel("y [m]")
ax2.legend(); ax2.grid(alpha=0.3); ax2.axis("equal")

# --- 3. Velocity: IMU only vs GT -------------------------------------------
ax3 = fig.add_subplot(gs[2])
ax3.plot(t_imu_s, gt_vmag_at_imu, "k-", lw=1.5, label="Ground truth", zorder=3)
ax3.plot(t_imu_s, v_imu_clipped, color="#e74c3c", lw=0.8, alpha=0.85, label="IMU only (est.)")
ax3.set_xlabel("t [s]"); ax3.set_ylabel("|v| [m/s]")
ax3.set_title("Velocity – IMU only vs GT")
ax3.legend(); ax3.grid(alpha=0.3)
ax3.set_ylim(0, 25)

plt.suptitle("Ergebnisse ohne Sensorfusion", fontsize=13, fontweight="bold", y=1.01)

out = os.path.join(BASE, "plot_no_fusion_stereo_check.png")
plt.savefig(out, dpi=150, bbox_inches="tight")
print(f"Saved: {out}")

# ── RMSE Tabelle ─────────────────────────────────────────────────────────────
print("\n── RMSE für Folientabelle ───────────────────────")
print(f"{'Konfiguration':<30}  {'RMSE':>8}  {'Max':>8}")
print("-" * 52)
print(f"{'IMU only':<30}  {rmse_imu:>7.2f}m  {max_imu:>7.2f}m")
print(f"{'Stereo VO only':<30}  {rmse_vo:>7.2f}m  {max_vo:>7.2f}m")
plt.show()

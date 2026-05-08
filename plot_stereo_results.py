"""
Präsentationsplots für Stereo VO Ergebnisse
  → plot_stereo_fusion.png    (IMU + Stereo VO)
  → plot_stereo_only.png      (Stereo VO only)

Beide im selben Stil wie die Mono-Folie.
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os

BASE = os.path.dirname(os.path.abspath(__file__))
RES  = os.path.join(BASE, "results")

plt.rcParams.update({
    "font.size": 11,
    "axes.titlesize": 12,
    "axes.labelsize": 11,
    "legend.fontsize": 9.5,
})

# ── Ground truth ──────────────────────────────────────────────────────────────
gt_raw = pd.read_csv(
    os.path.join(BASE, "data/mav0/state_groundtruth_estimate0/data.csv"),
    comment="#", header=None
).values
GT_TS  = gt_raw[:, 0]
GT_POS = gt_raw[:, 1:4]
GT_VEL = gt_raw[:, 8:11]   # vx, vy, vz

# ── Helper: RMSE position ─────────────────────────────────────────────────────
def pos_rmse(traj, max_dt_ns=2e7):
    """traj: array [t_ns, px, py, pz]"""
    errs = []
    for row in traj:
        j = np.argmin(np.abs(GT_TS - row[0]))
        if abs(GT_TS[j] - row[0]) < max_dt_ns:
            errs.append(np.linalg.norm(row[1:4] - GT_POS[j]))
    errs = np.array(errs)
    return np.sqrt(np.mean(errs**2)), errs.max()

# ── Helper: GT speed interpolated to trajectory timestamps ────────────────────
def gt_speed_at(timestamps):
    out = []
    for t in timestamps:
        j = np.argmin(np.abs(GT_TS - t))
        out.append(np.linalg.norm(GT_VEL[j]))
    return np.array(out)

# ── Helper: speed from finite differences ────────────────────────────────────
def traj_speed(traj, smooth_window=1):
    dt  = np.diff(traj[:, 0]) * 1e-9           # ns → s
    dp  = np.diff(traj[:, 1:4], axis=0)
    spd = np.linalg.norm(dp / dt[:, None], axis=1)
    if smooth_window > 1:
        spd = np.convolve(spd, np.ones(smooth_window) / smooth_window, mode='same')
    return spd, (traj[1:, 0] - traj[0, 0]) * 1e-9

# ─────────────────────────────────────────────────────────────────────────────
# PLOT 1 — IMU + Stereo VO Fusion
# ─────────────────────────────────────────────────────────────────────────────
COLOR_STEREO_FUSED = "#8e44ad"   # purple (same as before)

fusion = pd.read_csv(os.path.join(RES, "traj_aligned_imu_vo_stereo.csv")).values   # 200 Hz, mit Pre-Launch Alignment

rmse_f, max_f = pos_rmse(fusion)

# Velocity: raw (for RMSE) + smoothed (for display)
spd_raw, t_f = traj_speed(fusion)
window = 200   # ~1s at 200Hz
spd_smooth = np.convolve(spd_raw, np.ones(window) / window, mode='same')
gt_spd_f   = gt_speed_at(fusion[1:, 0])
rmse_v_f   = np.sqrt(np.mean((spd_raw - gt_spd_f) ** 2))

fig1, axes = plt.subplots(1, 2, figsize=(14, 5.5))
fig1.subplots_adjust(wspace=0.32)

# XY
ax = axes[0]
ax.plot(GT_POS[:, 0], GT_POS[:, 1], "k-", lw=1.8, label="Ground truth", zorder=3)
ax.plot(fusion[:, 1], fusion[:, 2], color=COLOR_STEREO_FUSED, lw=1.1,
        label=f"IMU + Stereo VO  (RMSE {rmse_f:.2f} m)")
ax.plot(*fusion[0, 1:3],  "go", ms=8, zorder=5, label="Start")
ax.plot(*fusion[-1, 1:3], "rs", ms=8, zorder=5, label="End")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("XY Top-Down")
ax.axis("equal"); ax.grid(alpha=0.3); ax.legend()

# Velocity
ax = axes[1]
ax.plot(t_f, gt_spd_f, "k-", lw=1.8, label="Ground truth", zorder=3)
ax.plot(t_f, spd_smooth, color=COLOR_STEREO_FUSED, lw=1.2, alpha=0.9,
        label=f"IMU + Stereo VO  (RMSE {rmse_v_f:.2f} m/s)")
ax.set_xlabel("t [s]"); ax.set_ylabel("|v| [m/s]")
ax.set_title("Geschwindigkeitsbetrag")
ax.set_ylim(0, 3.5); ax.grid(alpha=0.3); ax.legend()

fig1.suptitle("IMU + Stereo VO Fusion", fontsize=13, fontweight="bold")

out1 = os.path.join(BASE, "plot_stereo_fusion.png")
fig1.savefig(out1, dpi=150, bbox_inches="tight")
print(f"Gespeichert: {out1}")

# ─────────────────────────────────────────────────────────────────────────────
# PLOT 2 — Stereo VO only
# ─────────────────────────────────────────────────────────────────────────────
COLOR_STEREO_ONLY = "#3498db"   # blue

stereo = pd.read_csv(os.path.join(RES, "traj_stereo_only.csv")).values   # 20 Hz

rmse_s, max_s = pos_rmse(stereo)

# Velocity: 20Hz → no heavy smoothing needed (5-sample window = 0.25s)
spd_s_raw, t_s = traj_speed(stereo)
spd_s_smooth = np.convolve(spd_s_raw, np.ones(5) / 5, mode='same')
gt_spd_s     = gt_speed_at(stereo[1:, 0])
rmse_v_s     = np.sqrt(np.mean((spd_s_raw - gt_spd_s) ** 2))

fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5.5))
fig2.subplots_adjust(wspace=0.32)

# XY
ax = axes2[0]
ax.plot(GT_POS[:, 0], GT_POS[:, 1], "k-", lw=1.8, label="Ground truth", zorder=3)
ax.plot(stereo[:, 1], stereo[:, 2], color=COLOR_STEREO_ONLY, lw=1.1,
        label=f"Stereo VO only  (RMSE {rmse_s:.2f} m)")
ax.plot(*stereo[0, 1:3],  "go", ms=8, zorder=5, label="Start")
ax.plot(*stereo[-1, 1:3], "rs", ms=8, zorder=5, label="End")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("XY Top-Down")
ax.axis("equal"); ax.grid(alpha=0.3); ax.legend()

# Velocity
ax = axes2[1]
ax.plot(t_s, gt_spd_s, "k-", lw=1.8, label="Ground truth", zorder=3)
ax.plot(t_s, spd_s_smooth, color=COLOR_STEREO_ONLY, lw=1.2, alpha=0.9,
        label=f"Stereo VO only  (RMSE {rmse_v_s:.2f} m/s)")
ax.set_xlabel("t [s]"); ax.set_ylabel("|v| [m/s]")
ax.set_title("Geschwindigkeitsbetrag")
ax.set_ylim(0, 3.5); ax.grid(alpha=0.3); ax.legend()

fig2.suptitle("Stereo VO only (ohne IMU)", fontsize=13, fontweight="bold")

out2 = os.path.join(BASE, "plot_stereo_only.png")
fig2.savefig(out2, dpi=150, bbox_inches="tight")
print(f"Gespeichert: {out2}")

# ─────────────────────────────────────────────────────────────────────────────
# RMSE Tabelle
# ─────────────────────────────────────────────────────────────────────────────
print("\n── RMSE-Tabelle ────────────────────────────────────")
print(f"{'Konfiguration':<30}  {'Pos RMSE':>10}  {'Pos Max':>9}  {'Vel RMSE':>10}")
print("-" * 65)
print(f"{'Stereo VO only':<30}  {rmse_s:>9.2f}m  {max_s:>8.2f}m  {rmse_v_s:>8.2f}m/s")
print(f"{'IMU + Stereo VO':<30}  {rmse_f:>9.2f}m  {max_f:>8.2f}m  {rmse_v_f:>8.2f}m/s")

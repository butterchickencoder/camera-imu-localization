"""
Präsentationsplot: Mono VO + IMU Fusion
  - XY top-down
  - Geschwindigkeitsplot (|v| vs Zeit)
  - RMSE-Tabelle (Position + Geschwindigkeit)
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

# ── Daten laden ──────────────────────────────────────────────────────────────
gt_raw = pd.read_csv(os.path.join(BASE, "data/mav0/state_groundtruth_estimate0/data.csv"),
                     comment="#", header=None).values
GT_TS  = gt_raw[:, 0]
GT_POS = gt_raw[:, 1:4]
GT_VEL = gt_raw[:, 8:11]   # vx, vy, vz

mono = pd.read_csv(os.path.join(RES, "traj_aligned_imu_vo_mono.csv")).values  # t_ns,px,py,pz

# ── Geschwindigkeit aus Trajektorie ableiten (finite Differenzen) ────────────
dt_arr   = np.diff(mono[:, 0]) * 1e-9
dp_arr   = np.diff(mono[:, 1:4], axis=0)
v_raw    = np.linalg.norm(dp_arr / dt_arr[:, None], axis=1)
# Smooth: rolling average over 1s window (~200 samples at 200Hz)
window = 200
v_est  = np.convolve(v_raw, np.ones(window)/window, mode='same')
t_s    = (mono[1:, 0] - mono[0, 0]) * 1e-9

# GT-Geschwindigkeit an IMU-Timestamps interpoliert
gt_vmag = []
for t in mono[1:, 0]:
    j = np.argmin(np.abs(GT_TS - t))
    gt_vmag.append(np.linalg.norm(GT_VEL[j]))
gt_vmag = np.array(gt_vmag)

# ── RMSE Position ────────────────────────────────────────────────────────────
def pos_errors(traj):
    e = []
    for r in traj:
        j = np.argmin(np.abs(GT_TS - r[0]))
        if abs(GT_TS[j] - r[0]) < 2e7:
            e.append(np.linalg.norm(r[1:4] - GT_POS[j]))
    return np.array(e)

e_pos  = pos_errors(mono)
rmse_p = np.sqrt(np.mean(e_pos**2))
max_p  = e_pos.max()

# ── RMSE Geschwindigkeit ─────────────────────────────────────────────────────
e_vel  = np.abs(v_raw - gt_vmag)   # RMSE on raw (not smoothed) for honest metric
rmse_v = np.sqrt(np.mean(e_vel**2))
max_v  = e_vel.max()

# ── Figure ───────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 5.5))
gs  = gridspec.GridSpec(1, 2, figure=fig, wspace=0.32)

# --- Plot 1: XY top-down ----------------------------------------------------
ax1 = fig.add_subplot(gs[0])
ax1.plot(GT_POS[:, 0], GT_POS[:, 1], "k-", lw=1.8, label="Ground truth", zorder=3)
ax1.plot(mono[:, 1], mono[:, 2], color="#8e44ad", lw=1.1,
         label=f"IMU + Mono VO  (RMSE {rmse_p:.2f} m)")
ax1.plot(*mono[0, 1:3], "go", ms=8, zorder=5, label="Start")
ax1.plot(*mono[-1, 1:3], "rs", ms=8, zorder=5, label="End")
ax1.set_xlabel("x [m]"); ax1.set_ylabel("y [m]")
ax1.set_title("XY Top-Down")
ax1.axis("equal"); ax1.grid(alpha=0.3); ax1.legend()

# --- Plot 2: Geschwindigkeit -------------------------------------------------
ax2 = fig.add_subplot(gs[1])
ax2.plot(t_s, gt_vmag,           "k-",  lw=1.8, label="Ground truth",    zorder=3)
ax2.plot(t_s, v_est, color="#8e44ad", lw=1.2, alpha=0.9,
         label=f"IMU + Mono VO  (RMSE {rmse_v:.2f} m/s)")
ax2.set_xlabel("t [s]"); ax2.set_ylabel("|v| [m/s]")
ax2.set_title("Geschwindigkeitsbetrag")
ax2.set_ylim(0, 3.5); ax2.grid(alpha=0.3); ax2.legend()

plt.suptitle("IMU + Mono VO Fusion (mit Pre-Launch Alignment)", fontsize=13, fontweight="bold")

out = os.path.join(BASE, "plot_mono_fusion.png")
plt.savefig(out, dpi=150, bbox_inches="tight")
print(f"Gespeichert: {out}")

# ── Tabelle ------------------------------------------------------------------
print("\n── RMSE-Tabelle für Folie ──────────────────────────")
print(f"{'Metrik':<25}  {'RMSE':>10}  {'Max':>10}")
print("-" * 50)
print(f"{'Position':<25}  {rmse_p:>9.2f}m  {max_p:>9.2f}m")
print(f"{'Geschwindigkeit':<25}  {rmse_v:>8.2f}m/s  {max_v:>8.2f}m/s")

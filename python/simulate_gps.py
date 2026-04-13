import numpy as np
import pandas as pd
import os

BASE = os.path.join(os.path.dirname(__file__), "..")
gt = pd.read_csv(os.path.join(BASE, "data/mav0/state_groundtruth_estimate0/data.csv"), comment="#", header=None)
gt.columns = ["t_ns","px","py","pz","qw","qx","qy","qz","vx","vy","vz","bgx","bgy","bgz","bax","bay","baz"]

# sample every 10 seconds, add 1m gaussian noise
t0 = gt["t_ns"].iloc[0]
gps_rows = []
last_t = -np.inf

for _, row in gt.iterrows():
    t_s = (row["t_ns"] - t0) * 1e-9
    if t_s - last_t >= 10.0:
        noise = np.random.normal(0, 1.0, 3)
        gps_rows.append([row["t_ns"],
                         row["px"] + noise[0],
                         row["py"] + noise[1],
                         row["pz"] + noise[2]])
        last_t = t_s

gps = pd.DataFrame(gps_rows, columns=["t_ns", "px", "py", "pz"])
gps["t_ns"] = gps["t_ns"].astype("int64")
gps.to_csv(os.path.join(BASE, "results/gps_simulated.csv"), index=False)
print(f"Generated {len(gps)} GPS fixes")

# Barometer: every 10th ground truth sample (~50 Hz), z + 0.5m noise
baro_rows = []
for i, row in gt.iterrows():
    if i % 10 == 0:
        noise_z = np.random.normal(0, 0.5)
        baro_rows.append([row["t_ns"], row["pz"] + noise_z])

baro = pd.DataFrame(baro_rows, columns=["t_ns", "pz"])
baro["t_ns"] = baro["t_ns"].astype("int64")
baro.to_csv(os.path.join(BASE, "results/baro_simulated.csv"), index=False)
print(f"Generated {len(baro)} barometer samples")

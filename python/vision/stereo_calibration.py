import numpy as np
import yaml


def load_sensor_yaml(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    fu, fv, cu, cv = data["intrinsics"]
    K = np.array([[fu, 0, cu], [0, fv, cv], [0, 0, 1]], dtype=np.float64)
    dist = np.array(data["distortion_coefficients"], dtype=np.float64)
    T_BS = np.array(data["T_BS"]["data"], dtype=np.float64).reshape(4, 4)
    return K, dist, T_BS


def compute_stereo_extrinsics(T_BS_0, T_BS_1):
    # T_c1_c0 = inv(T_BS_1) @ T_BS_0
    # gives rotation and translation from cam0 to cam1
    T_c1_c0 = np.linalg.inv(T_BS_1) @ T_BS_0
    R = T_c1_c0[:3, :3]
    t = T_c1_c0[:3, 3]
    baseline = np.linalg.norm(t)
    return R, t, baseline

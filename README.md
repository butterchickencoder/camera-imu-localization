# Camera-IMU Localization

Loosely-coupled monocular visual-inertial odometry using an Error-State Kalman Filter.

- **Vision (Python):** feature tracking → relative pose estimation → `poses.csv`
- **Filter (C++):** IMU propagation + VO update → `trajectory.csv`
- **Visualization (Python):** trajectory vs. ground truth

Tested on the [EuRoC MAV dataset](https://projects.asl.ethz.ch/datasets/euroc-mav/) (MH_01_easy).

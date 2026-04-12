# Camera-IMU Localization



## Motivation: 
IMU drift for a drone in a 3D environment will add up very quickly. Within minutes, the error is in the order of multiple kilometers. 

![alt text](images/IMU_without_filter.png)


With camera frames, we don't really have absolute scale to work with. Also, in case there is only pure rotation or very small translation, the computation of the essential matrix doesn't work very well (degeneration)

![alt text](results/vo_trajectory.png)

## Project goal

Use state estimation methods for fusing visual odometry with IMU


## Method

Loosely-coupled monocular visual-inertial odometry using an Error-State Kalman Filter.

- **Vision (Python):** feature tracking → relative pose estimation → `poses.csv`
- **Filter (C++):** IMU propagation + VO update → `trajectory.csv`
- **Visualization (Python):** trajectory vs. ground truth



Tested on the [EuRoC MAV dataset](https://projects.asl.ethz.ch/datasets/euroc-mav/) (MH_01_easy).

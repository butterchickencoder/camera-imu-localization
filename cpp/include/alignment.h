#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>
#include "csv_utils.h"

// Estimated IMU biases from a stationary pre-launch phase
struct BiasEstimate {
    Eigen::Vector3d ba;  // accelerometer bias (m/s^2)
    Eigen::Vector3d bg;  // gyroscope bias (rad/s)
};

// Estimate IMU biases by averaging measurements over a stationary window.
//
// The idea: when the platform is perfectly still, the only thing the
// accelerometer measures is gravity (rotated into the body frame) plus bias.
// The gyroscope should read zero plus bias. So we average over the window
// to get clean bias estimates before the filter starts running.
//
// imu_data    : full vector of IMU samples (we only look inside the window)
// t_start_ns  : start of stationary window (nanoseconds)
// t_end_ns    : end of stationary window — filter starts from here
// R_wb        : rotation matrix (body -> world) at t_end_ns, taken from GT
BiasEstimate estimateBiases(const std::vector<IMUSample>& imu_data,
                            int64_t t_start_ns,
                            int64_t t_end_ns,
                            const Eigen::Matrix3d& R_wb);

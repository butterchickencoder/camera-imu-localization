#pragma once
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class ESKF {
public:
    // State
    Eigen::Vector3d p;       // position (x, y, z) in world frame
    Eigen::Vector3d v;       // velocity in world frame
    Eigen::Quaterniond q;    // orientation: body -> world
    Eigen::Vector3d ba;      // accelerometer bias
    Eigen::Vector3d bg;      // gyroscope bias

    // Covariance matrix — 15x15 (we use 3+3+3+3+3, quaternion error is 3 not 4)
    Eigen::Matrix<double, 15, 15> P;

    // Noise parameters
    double sigma_acc;        // accelerometer noise
    double sigma_gyro;       // gyroscope noise
    double sigma_ba;         // accel bias random walk
    double sigma_bg;         // gyro bias random walk

    ESKF();
    void propagate(const Eigen::Vector3d& acc_m,
                   const Eigen::Vector3d& gyro_m,
                   double dt);

private:
    static Eigen::Matrix3d skew(const Eigen::Vector3d& v);
};

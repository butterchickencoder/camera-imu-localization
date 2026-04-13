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

    // Covariance matrix — 15x15 (quaternion error is 3)
    Eigen::Matrix<double, 15, 15> P;

    // Noise parameters
    double sigma_acc;        // accelerometer noise
    double sigma_gyro;       // gyroscope noise
    double sigma_ba;         // accel bias random walk
    double sigma_bg;         // gyro bias random walk

    ESKF();
    void initState(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0,
                   const Eigen::Quaterniond& q0,
                   const Eigen::Vector3d& ba0, const Eigen::Vector3d& bg0);
    void propagate(const Eigen::Vector3d& acc_m,
                   const Eigen::Vector3d& gyro_m,
                   double dt);
    void updatePosition(const Eigen::Vector3d& p_meas, double sigma);
    void updateAltitude(double z_meas, double sigma);
    void updateVelocity(const Eigen::Matrix3d& R_vo, const Eigen::Vector3d& t_vo);

private:
    void applyCorrection(const Eigen::Matrix<double, 15, 1>& dx);
    static Eigen::Matrix3d skew(const Eigen::Vector3d& v);
};

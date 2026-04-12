#include "eskf.h"

const Eigen::Vector3d GRAVITY(0.0, 0.0, -9.81);

ESKF::ESKF() {
    p.setZero(); // Set (x, y, z) position to zero (m)
    v.setZero(); // Set (vx, vy, vz) velocity to zero (m/s)
    q = Eigen::Quaterniond::Identity(); // w = 1, x, y, z = 0. No rotation
    ba.setZero(); // set accelerometer bias to zero (m/s^2) 
    bg.setZero(); // Set gyroscope bias to zero 


    P = Eigen::Matrix<double, 15, 15>::Zero();
    P.diagonal() << 
        1.0, 1.0, 1.0,       // δp  (1m)
        0.25, 0.25, 0.25,    // δv  (0.5 m/s → variance = 0.5² = 0.25)
        0.01, 0.01, 0.01,    // δθ  (~5°)
        0.04, 0.04, 0.04,    // δba
        0.001, 0.001, 0.001; // δbg
    sigma_acc  = 0.0028;    // m/s²  - accelerometer noise density
    sigma_gyro = 0.00016;   // rad/s - gyroscope noise density
    sigma_ba   = 0.00043;   // m/s²  - accel bias random walk
    sigma_bg   = 0.0000022; // rad/s - gyro bias random walk
}

void ESKF::propagate(const Eigen::Vector3d& acc_m,
                     const Eigen::Vector3d& gyro_m,
                     double dt)
{
    Eigen::Vector3d acc_c, gyro_c; // bias corrected acceration and angular velocity
    acc_c = acc_m - ba;
    gyro_c = gyro_m - bg;

    // get rotation matrix from quaternion

    Eigen::Matrix3d R = q.toRotationMatrix();


    // update position

    p += v * dt;

    // update velocity
    v += (R * acc_c + GRAVITY) * dt;

    // update quaternion from gyro (watch for near-zero case)

    // calculate angle moved during time step
    double angle = gyro_c.norm() * dt;
    Eigen::Quaterniond dq;

    // avoid division by zero
    if (angle < 1e-10) {
        dq = Eigen::Quaterniond::Identity();
    } else {

        Eigen::Vector3d axis = gyro_c / gyro_c.norm();
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
    }

    // Update rotation matrix based on change in axis
    q = (q * dq).normalized();


    // TODO: build F matrix (linearised state transition)

    // TODO: build Q matrix (process noise)

    // TODO: propagate covariance P = F*P*F' + Q
}

Eigen::Matrix3d ESKF::skew(const Eigen::Vector3d& v) {
    // TODO: fill in skew-symmetric matrix
    return Eigen::Matrix3d::Zero();
}

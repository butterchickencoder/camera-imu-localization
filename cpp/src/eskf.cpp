#include "eskf.h"

const Eigen::Vector3d GRAVITY(0.0, 0.0, -9.81);
double EPSILON = 1e-9;

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
    sigma_acc  = 0.25;       // m/s²  - tuned (datasheet: 0.0028, inflated for unmodeled errors)
    sigma_gyro = 0.25;      // rad/s - tuned (datasheet: 0.00016)
    sigma_ba   = 0.01;      // m/s²  - tuned (datasheet: 0.00043)
    sigma_bg   = 0.001;     // rad/s - tuned (datasheet: 0.0000022)
}

void ESKF::initState(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0,
                     const Eigen::Quaterniond& q0,
                     const Eigen::Vector3d& ba0, const Eigen::Vector3d& bg0)
{
    p  = p0;
    v  = v0;
    q  = q0.normalized();
    ba = ba0;
    bg = bg0;
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


    // sanity check: clamp velocity to physically possible range
    const double max_speed = 10.0; // m/s — max drone speed
    if (v.norm() > max_speed) {
        v = v.normalized() * max_speed;
    }

    // state transition matrix F (linearised error dynamics)
    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
    F.block<3,3>(0, 3)  =  Eigen::Matrix3d::Identity() * dt;   // dp/dv
    F.block<3,3>(3, 6)  = -R * skew(acc_c) * dt;               // dv/dtheta
    F.block<3,3>(3, 9)  = -R * dt;                              // dv/dba
    F.block<3,3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;   // dtheta/dbg

    // process noise Q
    Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
    Q.block<3,3>(3, 3)  = Eigen::Matrix3d::Identity() * sigma_acc  * sigma_acc  * dt;
    Q.block<3,3>(6, 6)  = Eigen::Matrix3d::Identity() * sigma_gyro * sigma_gyro * dt;
    Q.block<3,3>(9, 9)  = Eigen::Matrix3d::Identity() * sigma_ba   * sigma_ba   * dt;
    Q.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * sigma_bg   * sigma_bg   * dt;

    P = F * P * F.transpose() + Q;
}


void ESKF::updatePosition(const Eigen::Vector3d& p_meas, double sigma)
{
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R_meas = Eigen::Matrix3d::Identity() * sigma * sigma;
    Eigen::Vector3d innov  = p_meas - p;

    Eigen::Matrix3d S = H * P * H.transpose() + R_meas;

    // Mahalanobis distance (for diagnostics, no rejection — GPS is trusted)
    // double mahal = innov.transpose() * S.inverse() * innov;

    Eigen::Matrix<double, 15, 3> K = P * H.transpose() * S.inverse();
    Eigen::Matrix<double, 15, 1> dx = K * innov;

    applyCorrection(dx);

    P = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P;
}

void ESKF::updateAltitude(double z_meas, double sigma)
{
    // 1x15 H matrix — selects only z from position
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H(0, 2) = 1.0;

    double R_meas = sigma * sigma;
    double innov  = z_meas - p.z();

    double S = (H * P * H.transpose())(0,0) + R_meas;
    Eigen::Matrix<double, 15, 1> K = P * H.transpose() / S;
    Eigen::Matrix<double, 15, 1> dx = K * innov;

    applyCorrection(dx);

    P = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P;
}

void ESKF::updateVelocity(const Eigen::Matrix3d& R_vo, const Eigen::Vector3d& t_vo)
{
    double angle = std::acos(std::min(1.0, (R_vo.trace() - 1.0) / 2.0));
    if (angle > 0.087) return; // reject bad poses > 5 degrees

    Eigen::Matrix3d R = q.toRotationMatrix();
    const double fixed_scale = 0.5;
    Eigen::Vector3d v_measured = R * t_vo * fixed_scale;

    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R_meas = Eigen::Matrix3d::Identity() * 0.5;
    Eigen::Vector3d innov  = v_measured - v;

    Eigen::Matrix3d S = H * P * H.transpose() + R_meas;
    Eigen::Matrix<double, 15, 3> K = P * H.transpose() * S.inverse();
    Eigen::Matrix<double, 15, 1> dx = K * innov;

    applyCorrection(dx);

    P = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P;
}

void ESKF::applyCorrection(const Eigen::Matrix<double, 15, 1>& dx) {
    p  += dx.segment<3>(0);
    v  += dx.segment<3>(3);

    // orientation correction via small-angle rotation
    Eigen::Vector3d dtheta = dx.segment<3>(6);
    double angle = dtheta.norm();
    if (angle > EPSILON) {
        q = (q * Eigen::Quaterniond(Eigen::AngleAxisd(angle, dtheta / angle))).normalized();
    }

    ba += dx.segment<3>(9);
    bg += dx.segment<3>(12);
}

Eigen::Matrix3d ESKF::skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<     0, -v.z(),  v.y(),
         v.z(),      0, -v.x(),
        -v.y(),  v.x(),      0;
    return S;
}

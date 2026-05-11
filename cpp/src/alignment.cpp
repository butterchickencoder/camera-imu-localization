#include "alignment.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>

// Gravity vector in world frame (NED-style: z points down, so -9.81)
const Eigen::Vector3d GRAVITY(0.0, 0.0, -9.81);

BiasEstimate estimateBiases(const std::vector<IMUSample>& imu_data,
                            int64_t t_start_ns,
                            int64_t t_end_ns,
                            const Eigen::Matrix3d& R_wb)
{
    // Sum gyro and accel readings inside the stationary window
    Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d sum_acc  = Eigen::Vector3d::Zero();
    int n = 0;
    for (const auto& s : imu_data) {
        if (s.t_ns >= t_start_ns && s.t_ns < t_end_ns) {
            sum_gyro += s.gyro;
            sum_acc  += s.acc;
            n++;
        }
    }

    // Gyro bias: when stationary, gyro should read zero, so the mean is the bias
    Eigen::Vector3d bg = sum_gyro / n;

    // Accel bias: when stationary the accelerometer reads the reaction force against
    // gravity, not gravity itself — i.e. it measures specific force, which points
    // upward at rest. In world frame, gravity g_world = (0, 0, -9.81), so the
    // measured specific force in the body frame is R_wb^T * (-g_world). Adding bias:
    //   acc_measured = R_wb^T * (-g_world) + ba
    //   ba = acc_mean - R_wb^T * (-g_world)
    Eigen::Vector3d acc_mean = sum_acc / n;
    Eigen::Vector3d ba = acc_mean - R_wb.transpose() * (-GRAVITY);

    std::cout << "Pre-launch alignment (" << n << " IMU samples in window):\n";
    std::cout << "  bg estimate: " << bg.transpose() << "\n";
    std::cout << "  ba estimate: " << ba.transpose() << "\n";

    // Debug override: set USE_GT_BIAS=1 to use the ground truth bias instead.
    // Useful for checking how much the alignment estimation is off.
    const char* use_gt_env = std::getenv("USE_GT_BIAS");
    if (use_gt_env && std::string(use_gt_env) == "1") {
        std::ifstream f("../data/mav0/state_groundtruth_estimate0/data.csv");
        std::string line;
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string tok;
            double v[17];
            for (int i = 0; i < 17; i++) { std::getline(ss, tok, ','); v[i] = std::stod(tok); }
            if ((int64_t)v[0] >= t_end_ns) {
                bg << v[11], v[12], v[13];
                ba << v[14], v[15], v[16];
                std::cout << "  [DEBUG: using ground truth bias]\n";
                break;
            }
        }
    }

    return {ba, bg};
}

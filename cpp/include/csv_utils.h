#pragma once

#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// VO pose: rotation matrix (row-major, 9 values) + translation (3 values)
struct Pose { double r[9]; double t[3]; };

// One IMU measurement: timestamp + gyroscope + accelerometer
struct IMUSample {
    int64_t t_ns;
    Eigen::Vector3d gyro;  // angular velocity (rad/s)
    Eigen::Vector3d acc;   // linear acceleration (m/s^2)
};

// One ground truth sample from the EuRoC state estimator CSV
struct GTSample {
    int64_t t_ns;
    Eigen::Vector3d p;      // position
    Eigen::Vector3d v;      // velocity
    Eigen::Quaterniond q;   // orientation (w, x, y, z)
    Eigen::Vector3d ba;     // accelerometer bias
    Eigen::Vector3d bg;     // gyroscope bias
};

namespace utils {

    inline std::map<int64_t, double> loadBaro(const std::string& path) {
        std::map<int64_t, double> baro;
        std::ifstream f(path);
        std::string line;
        std::getline(f, line); // skip header
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string tok;
            int64_t t; double z;
            std::getline(ss, tok, ','); t = std::stoll(tok);
            std::getline(ss, tok, ','); z = std::stod(tok);
            baro[t] = z;
        }
        return baro;
    }

    inline std::map<int64_t, Pose> loadPoses(const std::string& path) {
        std::map<int64_t, Pose> poses;
        std::ifstream f(path);
        std::string line;
        std::getline(f, line); // skip header
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string tok;
            int64_t t; Pose p;
            std::getline(ss, tok, ','); t = std::stoll(tok);
            for (int i = 0; i < 9; i++) { std::getline(ss, tok, ','); p.r[i] = std::stod(tok); }
            for (int i = 0; i < 3; i++) { std::getline(ss, tok, ','); p.t[i] = std::stod(tok); }
            poses[t] = p;
        }
        return poses;
    }

    // Load all IMU samples from an EuRoC imu0/data.csv file.
    // Format: t_ns, w_x, w_y, w_z, a_x, a_y, a_z
    inline std::vector<IMUSample> loadIMU(const std::string& path) {
        std::vector<IMUSample> out;
        std::ifstream f(path);
        std::string line;
        std::getline(f, line); // skip header
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string tok;
            double v[7];
            for (int i = 0; i < 7; i++) { std::getline(ss, tok, ','); v[i] = std::stod(tok); }
            IMUSample s;
            s.t_ns = (int64_t)v[0];
            s.gyro << v[1], v[2], v[3];
            s.acc  << v[4], v[5], v[6];
            out.push_back(s);
        }
        return out;
    }

    // Load ground truth samples from an EuRoC state_groundtruth_estimate0/data.csv file.
    // Format: t_ns, p_x, p_y, p_z, q_w, q_x, q_y, q_z, v_x, v_y, v_z, bg_x, bg_y, bg_z, ba_x, ba_y, ba_z
    inline std::vector<GTSample> loadGT(const std::string& path) {
        std::vector<GTSample> out;
        std::ifstream f(path);
        std::string line;
        std::getline(f, line); // skip header
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string tok;
            double v[17];
            for (int i = 0; i < 17; i++) { std::getline(ss, tok, ','); v[i] = std::stod(tok); }
            GTSample s;
            s.t_ns = (int64_t)v[0];
            s.p  << v[1],  v[2],  v[3];
            s.q   = Eigen::Quaterniond(v[4], v[5], v[6], v[7]);
            s.v  << v[8],  v[9],  v[10];
            s.bg << v[11], v[12], v[13];
            s.ba << v[14], v[15], v[16];
            out.push_back(s);
        }
        return out;
    }

    // Find the GT sample closest to t_ns (simple nearest-neighbour, no interpolation).
    inline GTSample nearestGT(const std::vector<GTSample>& gt, int64_t t_ns) {
        auto it = std::lower_bound(gt.begin(), gt.end(), t_ns,
                                   [](const GTSample& s, int64_t t) { return s.t_ns < t; });
        if (it == gt.end())   return gt.back();
        if (it == gt.begin()) return gt.front();
        return *it;
    }

} // namespace utils

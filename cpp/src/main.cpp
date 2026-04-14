#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include "eskf.h"
#include "csv_utils.h"

//struct Pose { double r[9]; double t[3]; };

/* We want to be able to pass a config to to runFilter method so that it knows which
    measurements to use and which to ignore
*/
struct FilterExperiment {
    std::string name;           
    std::string output_path;    
    bool use_gps = false;
    bool use_baro = false;
    bool use_vo = false;
    
    // Paths to the actual data
    std::string gps_path;
    std::string baro_path;
    std::string vo_path;
};
void initFromGT(ESKF& eskf) {
    std::ifstream gt_file("../data/mav0/state_groundtruth_estimate0/data.csv");
    std::string gt_line;
    while (std::getline(gt_file, gt_line)) {
        if (gt_line.empty() || gt_line[0] == '#') continue;
        std::stringstream gss(gt_line);
        std::string tok;
        double vals[17];
        for (int i = 0; i < 17; i++) { std::getline(gss, tok, ','); vals[i] = std::stod(tok); }
        eskf.initState(
            Eigen::Vector3d(vals[1], vals[2], vals[3]),
            Eigen::Vector3d(vals[8], vals[9], vals[10]),
            Eigen::Quaterniond(vals[4], vals[5], vals[6], vals[7]),
            Eigen::Vector3d(vals[14], vals[15], vals[16]),
            Eigen::Vector3d(vals[11], vals[12], vals[13])
        );
        break;
    }
}

void runFilter(const std::string& imu_path,
               std::map<int64_t, double> baro,
               std::map<int64_t, Pose> vo_poses,
               bool use_gps, bool use_baro, bool use_vo,
               const std::string& out_path)
{
    std::ifstream imu_file(imu_path);
    std::ofstream out(out_path);
    out << "t_ns,px,py,pz\n";

    ESKF eskf;
    initFromGT(eskf);

    std::string line;
    int64_t t_prev = -1;
    int gps_count = 0, baro_count = 0, vo_count = 0;

    while (std::getline(imu_file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::stringstream ss(line);
        std::string token;
        int64_t t_ns;
        double gx, gy, gz, ax, ay, az;
        std::getline(ss, token, ','); t_ns = std::stoll(token);
        std::getline(ss, token, ','); gx = std::stod(token);
        std::getline(ss, token, ','); gy = std::stod(token);
        std::getline(ss, token, ','); gz = std::stod(token);
        std::getline(ss, token, ','); ax = std::stod(token);
        std::getline(ss, token, ','); ay = std::stod(token);
        std::getline(ss, token, ','); az = std::stod(token);

        if (t_prev < 0) { t_prev = t_ns; continue; }

        double dt = (t_ns - t_prev) * 1e-9;
        eskf.propagate(Eigen::Vector3d(ax, ay, az),
                       Eigen::Vector3d(gx, gy, gz), dt);

        // Barometer update after timestamp matching
        if (use_baro && !baro.empty()) {
            auto bit = baro.lower_bound(t_ns);
            bool matched = false;
            if (bit != baro.end() && std::abs(bit->first - t_ns) < 50000000LL)
                matched = true;
            else if (bit != baro.begin()) {
                --bit;
                if (std::abs(bit->first - t_ns) < 50000000LL) matched = true;
            }
            if (matched) {
                eskf.updateAltitude(bit->second, 0.5);
                baro.erase(bit);
                baro_count++;
            }
        }

        // VO update after tiemstamp matches
        if (use_vo && !vo_poses.empty()) {
            auto vit = vo_poses.lower_bound(t_ns);
            if (vit != vo_poses.end() && std::abs(vit->first - t_ns) < 6000000LL) {
                Pose& p = vit->second;
                Eigen::Matrix3d R; R << p.r[0],p.r[1],p.r[2],p.r[3],p.r[4],p.r[5],p.r[6],p.r[7],p.r[8];
                Eigen::Vector3d t(p.t[0], p.t[1], p.t[2]);
                eskf.updateVelocity(R, t);
                vo_poses.erase(vit);
                vo_count++;
            }
        }

        out << t_ns << "," << eskf.p.x() << "," << eskf.p.y() << "," << eskf.p.z() << "\n";
        t_prev = t_ns;
    }
    std::cout << "Done: " << out_path
              << " | final p: " << eskf.p.transpose()
              << " | GPS: " << gps_count
              << " | Baro: " << baro_count
              << " | VO: " << vo_count << std::endl;
}

int main() {
    std::string imu_path = "../data/mav0/imu0/data.csv";
    // --- Loading Barometer (double) ---
    auto baro = utils::loadBaro("../results/baro_simulated.csv");

    // --- Loading Poses (Pose) ---
    auto poses = utils::loadPoses("../results/poses.csv");

    std::map<int64_t, Eigen::Vector3d> no_gps;
    std::map<int64_t, double> no_baro;
    std::map<int64_t, Pose> no_vo;

    // Run 1: IMU only
    runFilter(imu_path, no_baro, no_vo, false, false, false, "../results/traj_imu_only.csv");

    // Run 2: IMU + VO
    runFilter(imu_path, no_baro, poses, false, false, true, "../results/traj_imu_vo.csv");

    // Run 3: IMU + Baro + VO
    runFilter(imu_path, baro, poses, false, true, true, "../results/traj_imu_baro_vo.csv");

    return 0;
}

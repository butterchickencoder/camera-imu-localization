#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include "eskf.h"
#include "csv_utils.h"
#include "config.h"


// Get initial state from Ground truth data
void initFromGT(ESKF &eskf)
{
    std::ifstream gt_file("../data/mav0/state_groundtruth_estimate0/data.csv");
    std::string gt_line;
    while (std::getline(gt_file, gt_line))
    {
        if (gt_line.empty() || gt_line[0] == '#')
            continue;
        std::stringstream gss(gt_line);
        std::string tok;
        double vals[17];
        for (int i = 0; i < 17; i++)
        {
            std::getline(gss, tok, ',');
            vals[i] = std::stod(tok);
        }
        eskf.initState(
            Eigen::Vector3d(vals[1], vals[2], vals[3]),
            Eigen::Vector3d(vals[8], vals[9], vals[10]),
            Eigen::Quaterniond(vals[4], vals[5], vals[6], vals[7]),
            Eigen::Vector3d(vals[14], vals[15], vals[16]),
            Eigen::Vector3d(vals[11], vals[12], vals[13]));
        break;
    }
}

void runFilter(const FilterConfig &filtercfg)
{
    std::ifstream imu_file(filtercfg.IMU_path);
    std::ofstream out(filtercfg.filter_output_path);
    out << "t_ns,px,py,pz\n";

    ESKF eskf;
    initFromGT(eskf);

    std::string line;
    int64_t t_prev = -1;
    int vo_count = 0;

    // Load visual odometry poses
    std::map<int64_t, Pose> vo_poses;
    if (filtercfg.VO_mode != VOMode::None)
    {
        vo_poses = utils::loadPoses(filtercfg.VO_poses_path);
    }

    // Extract IMU data
    while (std::getline(imu_file, line))
    {
        if (line.empty() || line[0] == '#')
            continue;

        std::stringstream ss(line);
        std::string token;
        int64_t t_ns;
        double gx, gy, gz, ax, ay, az;
        std::getline(ss, token, ',');
        t_ns = std::stoll(token);
        std::getline(ss, token, ',');
        gx = std::stod(token);
        std::getline(ss, token, ',');
        gy = std::stod(token);
        std::getline(ss, token, ',');
        gz = std::stod(token);
        std::getline(ss, token, ',');
        ax = std::stod(token);
        std::getline(ss, token, ',');
        ay = std::stod(token);
        std::getline(ss, token, ',');
        az = std::stod(token);

        if (t_prev < 0)
        {
            t_prev = t_ns;
            continue;
        }

        double dt = (t_ns - t_prev) * 1e-9;
        eskf.propagate(Eigen::Vector3d(ax, ay, az),
                       Eigen::Vector3d(gx, gy, gz), dt);


        // VO update after tiemstamp matches

        if (filtercfg.VO_mode != VOMode::None)
        {
            auto vit = vo_poses.lower_bound(t_ns);
            if (vit != vo_poses.end() && std::abs(vit->first - t_ns) < 6000000LL)
            {
                Pose &p = vit->second;
                Eigen::Matrix3d R;
                R << p.r[0], p.r[1], p.r[2], p.r[3], p.r[4], p.r[5], p.r[6], p.r[7], p.r[8];
                Eigen::Vector3d t(p.t[0], p.t[1], p.t[2]);
                eskf.updateVelocity(R, t);
                vo_poses.erase(vit);
                vo_count++;
            }
        }

        out << t_ns << "," << eskf.p.x() << "," << eskf.p.y() << "," << eskf.p.z() << "\n";
        t_prev = t_ns;
    }
    std::cout << "Done: " << filtercfg.filter_output_path
              << " | final p: " << eskf.p.transpose()
              << " | VO: " << vo_count << std::endl;
}

int main()
{

    // Create filter configurations for the three runs, with imu paths, visual odometry output paths and output path for ESKF results after fusion
    FilterConfig imu_only_cfg{VOMode::None, "../data/mav0/imu0/data.csv", "", "../results/traj_imu_only.csv"};
    FilterConfig imu_mono_cfg{VOMode::Mono, "../data/mav0/imu0/data.csv", "../results/poses_mono_final.csv", "../results/traj_aligned_imu_vo_mono.csv" };
    FilterConfig imu_stereo_cfg{VOMode::Stereo, "../data/mav0/imu0/data.csv", "../results/poses.csv", "../results/traj_aligned_imu_vo_stereo.csv"};

    // Run 1: IMU only
    runFilter(imu_only_cfg);

    // Run 2: IMU + Mono VO
    runFilter(imu_mono_cfg);

    // Run 3: IMU + Stereo VO
    runFilter(imu_stereo_cfg);

    return 0;
}

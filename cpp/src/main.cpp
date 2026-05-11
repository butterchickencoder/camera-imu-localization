#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include "eskf.h"
#include "csv_utils.h"
#include "config.h"
#include "alignment.h"


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
    // Load all IMU samples upfront — needed for both bias estimation and the filter loop
    auto imu_data = utils::loadIMU(filtercfg.IMU_path);

    std::ofstream out(filtercfg.filter_output_path);
    out << "t_ns,px,py,pz\n";

    ESKF eskf;
    int64_t t_filter_start = 0; // samples before this timestamp are skipped in the filter loop

    if (filtercfg.init_mode == InitMode::PreLaunchAlignment)
    {
        // Load ground truth to get position and orientation at the end of the stationary phase
        auto gt = utils::loadGT("../data/mav0/state_groundtruth_estimate0/data.csv");

        int64_t t0_gt         = gt.front().t_ns;
        int64_t t_align_start = t0_gt + (int64_t)(filtercfg.t_start_alignment * 1e9);
        int64_t t_align_end   = t0_gt + (int64_t)(filtercfg.t_end_alignment   * 1e9);

        // Get the orientation at alignment end to separate gravity from accel bias
        GTSample gt_end  = utils::nearestGT(gt, t_align_end);
        Eigen::Matrix3d R_wb = gt_end.q.toRotationMatrix();

        // Estimate gyro and accel biases from the stationary window
        BiasEstimate biases = estimateBiases(imu_data, t_align_start, t_align_end, R_wb);

        // Init ESKF: known position from GT, zero velocity (drone is still), estimated biases
        eskf.initState(gt_end.p, Eigen::Vector3d::Zero(), gt_end.q, biases.ba, biases.bg);

        // Tight initial covariance — we know the state well after alignment
        eskf.P.setZero();
        eskf.P.diagonal() <<
            1e-4, 1e-4, 1e-4,    // position  (~1 cm known from launch site)
            1e-4, 1e-4, 1e-4,    // velocity  (exactly zero, drone is still)
            1e-3, 1e-3, 1e-3,    // orientation (~2 degrees from GT quaternion)
            1e-4, 1e-4, 1e-4,    // accel bias
            1e-6, 1e-6, 1e-6;    // gyro bias

        t_filter_start = t_align_end;

        // Write the known launch position for all samples during the stationary phase
        for (const auto& s : imu_data) {
            if (s.t_ns >= t_align_end) break;
            out << s.t_ns << "," << gt_end.p.x() << "," << gt_end.p.y() << "," << gt_end.p.z() << "\n";
        }
    }
    else
    {
        // Ground truth init: read position, velocity, orientation and biases from first GT sample
        initFromGT(eskf);
    }

    // Load VO poses if needed
    std::map<int64_t, Pose> vo_poses;
    if (filtercfg.VO_mode != VOMode::None)
    {
        vo_poses = utils::loadPoses(filtercfg.VO_poses_path);
    }

    // Main filter loop — propagate IMU and apply VO updates
    int64_t t_prev = -1;
    int vo_count = 0;

    for (const auto& s : imu_data)
    {
        // Skip the stationary phase — filter starts at t_filter_start
        if (s.t_ns < t_filter_start) continue;

        if (t_prev < 0) { t_prev = s.t_ns; continue; }

        double dt = (s.t_ns - t_prev) * 1e-9;
        eskf.propagate(s.acc, s.gyro, dt);

        // VO update: find the closest VO pose within 6ms of current IMU timestamp
        if (filtercfg.VO_mode != VOMode::None)
        {
            auto vit = vo_poses.lower_bound(s.t_ns);
            if (vit != vo_poses.end() && std::abs(vit->first - s.t_ns) < 6000000LL)
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

        out << s.t_ns << "," << eskf.p.x() << "," << eskf.p.y() << "," << eskf.p.z() << "\n";
        t_prev = s.t_ns;
    }

    std::cout << "Done: " << filtercfg.filter_output_path
              << " | final p: " << eskf.p.transpose()
              << " | VO updates: " << vo_count << std::endl;
}

int main()
{

    // Create filter configurations for the three runs, with imu paths, visual odometry output paths and output path for ESKF results after fusion
    FilterConfig imu_only_cfg{VOMode::None, "../data/mav0/imu0/data.csv", "", "../results/traj_imu_only.csv", InitMode::PreLaunchAlignment};
    FilterConfig imu_mono_cfg{VOMode::Mono, "../data/mav0/imu0/data.csv", "../results/poses_mono_final.csv", "../results/traj_aligned_imu_vo_mono.csv", InitMode::PreLaunchAlignment };
    FilterConfig imu_stereo_cfg{VOMode::Stereo, "../data/mav0/imu0/data.csv", "../results/poses.csv", "../results/traj_aligned_imu_vo_stereo.csv", InitMode::PreLaunchAlignment};

    // Run 1: IMU only
    runFilter(imu_only_cfg);

    // Run 2: IMU + Mono VO
    runFilter(imu_mono_cfg);

    // Run 3: IMU + Stereo VO
    runFilter(imu_stereo_cfg);

    return 0;
}

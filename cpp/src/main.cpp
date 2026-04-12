#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "eskf.h"

int main() {
    // paths
    std::string imu_path = "../data/mav0/imu0/data.csv";
    std::string out_path = "../results/imu_trajectory.csv";

    // open IMU file
    std::ifstream imu_file(imu_path);
    if (!imu_file.is_open()) {
        std::cerr << "Could not open: " << imu_path << std::endl;
        return 1;
    }

    // open output file
    std::ofstream out_file(out_path);
    out_file << "t_ns,px,py,pz\n";

    ESKF eskf;

    std::string line;
    int64_t t_prev = -1;
    bool first_line = true;

    while (std::getline(imu_file, line)) {
        if (line.empty() || line[0] == '#') continue;

        // parse line into: t_ns, gx, gy, gz, ax, ay, az
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

        Eigen::Vector3d acc_m(ax, ay, az);
        Eigen::Vector3d gyro_m(gx, gy, gz);
        eskf.propagate(acc_m, gyro_m, dt);

        out_file << t_ns << ","
                 << eskf.p.x() << ","
                 << eskf.p.y() << ","
                 << eskf.p.z() << "\n";

        t_prev = t_ns;
    }

    std::cout << "Done. Trajectory saved to " << out_path << std::endl;
    return 0;
}

#pragma once
#include <string>

//Define if we are using visual odometry and if we are using Mono or Stereo visual odometry
enum class VOMode {
    None, 
    Mono, 
    Stereo
};

enum class InitMode {
    GroundTruth,
    PreLaunchAlignment
};

struct FilterConfig {


    // Which VO Mode is going to be used for the filter
    VOMode VO_mode;

    // Path to IMU Data
    std::string IMU_path;

    // Path to poses that are computed by Visual Odometry
    std::string VO_poses_path;



    // Path to file where we will save the filtered poses
    std::string filter_output_path;

    // Which InitMode the filter would use

    InitMode init_mode;
    // Start and end times for pre launch alignement (using default values)
    double t_start_alignment = 22.0;
    double t_end_alignment = 43.0;
};
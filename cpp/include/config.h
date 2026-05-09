#pragma once
#include <string>

//Define if we are using visual odometry and if we are using Mono or Stereo visual odometry
enum class VOMode {
    None, 
    Mono, 
    Stereo
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
};
cmake_minimum_required(VERSION 3.15)
project(camera_imu_fusion)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_executable(fusion
    src/main.cpp
    src/eskf.cpp
)

target_include_directories(fusion PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
)

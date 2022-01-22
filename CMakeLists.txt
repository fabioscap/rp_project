cmake_minimum_required(VERSION 3.16)
project(rp_2d_simulator)

find_package(roscpp REQUIRED)

link_directories("/opt/ros/noetic/lib")
include_directories("/opt/ros/noetic/include")
include_directories(${PROJECT_SOURCE_DIR}/sim2d)

add_subdirectory(${PROJECT_SOURCE_DIR}/sim2d)
add_subdirectory(${PROJECT_SOURCE_DIR}/exec)
cmake_minimum_required(VERSION 3.0.2)
project(rp_project)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_ros)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/sim2d)
include_directories(${PROJECT_SOURCE_DIR}/sim2d_ros)

add_subdirectory(${PROJECT_SOURCE_DIR}/sim2d)
add_subdirectory(${PROJECT_SOURCE_DIR}/sim2d_ros)

add_executable(${PROJECT_NAME}_sim2d exec/sim2d.cpp)
target_link_libraries(${PROJECT_NAME}_sim2d ${catkin_LIBRARIES} sim2d sim2d_ros SDL2)
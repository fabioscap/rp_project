add_executable(Prova prova.cpp)
target_link_libraries(Prova libsim2d)
target_link_libraries(Prova roscpp)

add_executable(ROS_P my_ros_publisher.cpp)
target_link_libraries(ROS libsim2d)
target_link_libraries(ROS roscpp)

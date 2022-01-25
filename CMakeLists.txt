cmake_minimum_required(VERSION 3.0.2)
project(rp_project)

find_package(SDL2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/sim2d)
include_directories(${PROJECT_SOURCE_DIR}/sim2d_ros)

add_subdirectory(${PROJECT_SOURCE_DIR}/sim2d)
add_subdirectory(${PROJECT_SOURCE_DIR}/sim2d_ros)

add_executable(Prova_ros exec/prova_ros.cpp)
target_link_libraries(Prova_ros ${catkin_LIBRARIES})
target_link_libraries(Prova_ros sim2d)
target_link_libraries(Prova_ros sim2d_ros)

add_executable(Prova_sdl exec/prova_sdl.cpp)
target_link_libraries(Prova_sdl ${catkin_LIBRARIES})
target_link_libraries(Prova_sdl sim2d)
target_link_libraries(Prova_sdl sim2d_ros)
target_link_libraries(Prova_sdl SDL2)
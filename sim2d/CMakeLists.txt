find_package(SDL2 REQUIRED)

add_library(sim2d SHARED
    src/map.cpp
    src/robot.cpp
    src/world.cpp
    src/laser.cpp
    src/gui.cpp
)
target_link_libraries(sim2d SDL2)
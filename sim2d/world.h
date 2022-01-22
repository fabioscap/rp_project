/*

*/
#pragma once
#include "map.h"
#include "robot.h"
#include "vector"
#include "static_vec.h"

#define UPDATE_RATE 100 // ms

typedef rp::Vec_<cell_index,2> grid2;

namespace sim2d {
class World {
    public:
    Map map;
    Robot robot;
    const int radius; // robot size, but in pixels
    const std::vector<grid2> robot_footprint; // a circle that represents the robot in cells

    World(Map& m, Robot& r): map(m), 
                             robot(r),
                             radius((int)(robot.size/map.resolution)),
                             robot_footprint(get_footprint())
                             {}

    void update();

    // for a single robot, with non changing map 
    // I can compute all cells where robot is /not colliding.
    // but I don't know whether it makes sense or not.
    
    bool check_collision();

    std::vector<grid2> get_footprint();
};
}
/*

*/
#pragma once
#include "map.h"
#include "robot.h"

#define UPDATE_RATE 100 // ms

namespace sim2d {
class World {
    public:
    Map map;
    Robot robot;
    const int radius; // robot size, but in pixels

    World(Map& m, Robot& r): map(m), 
                                         robot(r),
                                         radius(robot.size/map.resolution) {}

    void update();
    bool check_collision();
};
}
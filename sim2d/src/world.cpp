#include "world.h"
#include <math.h>

using namespace sim2d;

void World::update(){
    pos2 new_pos = robot.pxy() + robot.get_xy_speed()*(UPDATE_RATE/1000);
    if (!check_collision(new_pos))
        robot.move((double)(UPDATE_RATE)/1000);
}
bool World::check_collision(pos2 pxy) const {
    // get the robot position in the map
    grid2 gpos = real_units_to_cell(pxy);
    auto it = robot_footprint.begin();
    while (it != robot_footprint.end()) { //openmp?
        auto point = *it + gpos;
        // out of bounds             
        if(point[0] < 0 || point[0] >= map.width) return true;
        if(point[1] < 0 || point[1] >= map.height) return true;
        // hit wall
        if (map.at(point) == WALL) return true;

        it++;
    }
    return false;
}

std::vector<grid2> World::get_footprint() {
    // could be made smaller by memorizing only a quadrant. (0->radius)
    std::vector<grid2> vec;
    int r2 = radius*radius;
    for(cell_index x =-radius; x<=radius; ++x) {
        for(cell_index y =-radius; y<=radius; ++y) {
            // x^2 + y^2 < r
            if ((x)*(x)+(y)*(y)<=radius*radius+1) {
                // point belongs to robot
                vec.push_back({x,y});
            }
        }
    }
    return vec;
}

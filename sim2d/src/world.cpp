#include "world.h"
#include <math.h>
#include <chrono>

using namespace sim2d;

uint64_t timeSinceEpochMillisec() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(high_resolution_clock::now().time_since_epoch()).count();
}

void World::update(){
    vec2 new_pos = robot.pxy() + robot.get_xy_speed()*(UPDATE_RATE/1000);
    if (!check_collision(new_pos))
        robot.move(UPDATE_RATE/1000);
}

// with this method robot may pass through wall if it is very fast
bool World::check_collision(vec2 pxy) const { 
    // get the robot position in the map
    grid2 gpos = real_units_to_cell(pxy);
    auto it = robot_footprint.begin();
    while (it != robot_footprint.end()) { //openmp?
        auto point = *it + gpos;
        // out of bounds             
        if(point[0] < 0 || point[0] >= map.width) {std::cout << "bounds" <<std::endl;return true;}
        if(point[1] < 0 || point[1] >= map.height) {std::cout << "bounds" <<std::endl;return true;}
        // hit wall
        if (map.at(point) == SIM2d_WALL) {std::cout << "wall at: "<< point <<std::endl;return true;}

        it++;
    }
    return false;
}

std::vector<grid2> World::get_footprint() {
    // could be made smaller by memorizing only a quadrant. (0->radius_cells)
    std::vector<grid2> vec;
    int r2 = radius_cells*radius_cells;
    for(cell_index x =-radius_cells; x<=radius_cells; ++x) {
        for(cell_index y =-radius_cells; y<=radius_cells; ++y) {
            // x^2 + y^2 < r
            if ((x)*(x)+(y)*(y)<=radius_cells*radius_cells+1) {
                // point belongs to robot
                vec.push_back({x,y});
            }
        }
    }
    return vec;
}

std::thread World::run() {
    running = true;

    // my world object may be in the stack of the main thread but i think threads share 
    // addresses so if i pass "this" then the simulation thread should be able to update the
    // position of the robot.
    return std::thread(&World::_run,this,UPDATE_RATE); 
}

void World::_run(int MS) {
    std::cout << "start simulation"<< std::endl;
    //https://stackoverflow.com/questions/46609863/execute-function-every-10-ms-in-c
    auto now = std::chrono::high_resolution_clock::now(); 
    auto next = now + std::chrono::milliseconds(MS);
    while(running) {

        m.lock();
        update();
        m.unlock();

        std::this_thread::sleep_until(next);
        next+=std::chrono::milliseconds(MS);
    }
    std::cout << "stop simulation"<< std::endl;
}

void World::compute_laser_scans() {
    // https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    // https://github.com/rtv/Stage/blob/master/libstage/world.cc
    
}
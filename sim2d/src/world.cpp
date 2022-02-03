#include "world.h"
#include <math.h>
#include <chrono>

using namespace sim2d;


void World::update(){
    vec3 new_pose = odom_to_map(robot.pose) + robot.get_xya_speed(initial_pose[2])*(UPDATE_RATE/1000);
    if (!check_collision({new_pose[0],new_pose[1]}))
        robot.move(UPDATE_RATE/1000);

    compute_laser_scans();
}

// with this method robot may pass through wall if it is very fast
bool World::check_collision(const vec2& pxy) const { 
    // get the robot position in the map
    grid2 gpos = real_units_to_cell(pxy);
    auto it = robot_footprint.begin();
    while (it != robot_footprint.end()) { //openmp?
        auto point = *it + gpos;
        // out of bounds             
        if(point[0] < 0 || point[0] >= map.width) {
            //std::cout << "bounds" <<std::endl;
            return true;}
        if(point[1] < 0 || point[1] >= map.height) {
            //std::cout << "bounds" <<std::endl;
            return true;}
        // hit wall
        if (map.at(point) == SIM2d_WALL) {
            //std::cout << "wall at: "<< point <<std::endl;
            return true;}

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
    auto now = std::chrono::high_resolution_clock::now(); // or from ros clock?
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
    // https://en.wikipedia.org/wiki/Digital_differential_analyzer_(graphics_algorithm)
    
    vec3 pose = odom_to_map(robot.pose);

    grid2 start = real_units_to_cell({pose[0],pose[1]});
    
    //grid2 start = grid2({0,0});

    
    for (int i=0; i<laser.n_samples; ++i) {
        grid2 end = real_units_to_cell(vec2({
                        laser.range*cos(laser.sample_orientations[i]+pose[2]),
                        laser.range*sin(laser.sample_orientations[i]+pose[2]),
                        })) + start; 
        grid2 hit =laser_hit(start,end);
        laser_scans_grid[i] = hit;
        laser_scans_ranges[i] = sqrt(hit[0]*hit[0]+hit[1]*hit[1])*map.resolution;
        
    }
    
    
}

grid2 World::laser_hit(const grid2& start,const grid2& end) {

    grid2 dxdy = (end-start);

    int index = 0;
    if (abs(dxdy[1]) > abs(dxdy[0])) index = 1;

    int index_direction = +1;
    if (dxdy[index] <0) {
        dxdy[index] *= -1;
        index_direction = -1;
    }
    
    int other_direction = +1;
    if (dxdy[!index] < 0) {
        dxdy[!index] *= -1;
        other_direction = -1;
    }

    cell_index discriminant = 2*(dxdy[!index]) - (dxdy[index]);
    grid2 point;
    for (point=start; (point[index]-end[index])*index_direction<=0; point[index]+=index_direction) {
        // https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm 

        // check bounds
        if (point[0]<=0 || point[0] >=map.width) return end-start;
        if (point[1]<=0 || point[1] >=map.height) return end-start;
        // check wall
        if (map.at(point) == SIM2d_WALL) {
            return point-start;
        }

        // determine if the next pixel is on the next row/column or the previous.
        if (discriminant > 0) {
            point[!index] += other_direction;
            discriminant += 2*(dxdy[!index]-dxdy[index]);
        } else {
            discriminant += 2*dxdy[!index];
        }
    }
    return point-start;
    
}

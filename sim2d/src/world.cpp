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
    compute_laser_scans();
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
    // https://en.wikipedia.org/wiki/Digital_differential_analyzer_(graphics_algorithm)

    grid2 start = real_units_to_cell(robot.pxy());
    
    //grid2 start = grid2({0,0});

    
    for (int i=0; i<laser.n_samples; ++i) {
        grid2 end = real_units_to_cell(vec2({
                        laser.range*cos(laser.sample_orientations[i]+robot.pa()),
                        laser.range*sin(laser.sample_orientations[i]+robot.pa()),
                        })) + start; 
        laser_scans[i] = laser_hit(start,end);
    }
    
    
}

grid2 World::laser_hit(grid2 start, grid2 end) {
    /*  i need to check in which region of the plane the line is:
        1) 0< <pi/4      -> x++, y++
        2) pi/4< <pi/2   -> y++, x++
        3) pi/2< <3pi/4  -> y++, x--
        4) 3pi/4< <pi    -> x--. y++
        5) pi< <5pi/4    -> x--, y--
        6) 5pi/4< <3pi/2 -> y--, x--
        7) 3pi/2< <7pi/4 -> y--, x++
        8) 7pi/4< <2pi   -> x++, y--

        8 regions so i need 3 checks
    */
    cell_index dx = end[0]-start[0];
    cell_index dy = end[1]-start[1];
    
    if (dy > 0) { // upper half circle
        if (dx > 0) { // first quadrant
            if (dx > dy) { 
                // 1)
                //std::cout<<"1"<<std::endl;
                return laser_hit_1o(start,end,0,+1); // no transformation needed
            } else {
                // 2)
                //std::cout<<"2"<<std::endl;
                return laser_hit_1o(start,end,1,+1);
            }
        } else { // second quadrant
            if (dy > -dx) {
                // 3)
                //std::cout<<"3"<<std::endl;
                return laser_hit_1o(start,end,1,+1);
            } else {
                // 4)
                //std::cout<<"4"<<std::endl;
                return laser_hit_1o(start,end,0,-1);
            }
        }
    } else { // lower half circle
        if (dx < 0) { // third quadrant
            if (dx < dy) { 
                // 5)
                //std::cout<<"5"<<std::endl;
                return laser_hit_1o(start,end,0,-1);
            } else {
                // 6)
                //std::cout<<"6"<<std::endl;
                return laser_hit_1o(start,end,1,-1);
            }
        } else { // fourth quadrant
            if (dx < -dy) {
                // 7)
                //std::cout<<"7"<<std::endl;
                return laser_hit_1o(start,end,1,-1);
            } else {
                // 8)
                //std::cout<<"8"<<std::endl;
                return laser_hit_1o(start,end,0,1);
            }
        }
    }

    std::cout<<"other"<<std::endl;  
    return grid2({0,0});
    
}

grid2 World::laser_hit_1o(grid2 start, grid2 end,int index,int index_direction) {
    // assume 1st quadrant and slope < 1 
    grid2 dxdy = (end-start);

    grid2 point;

    dxdy[index] *= index_direction;
    
    int increment_other = +1;
    if (dxdy[!index] < 0) {
        dxdy[!index] *= -1;
        increment_other = -1;
    }

    cell_index discriminant = 2*(dxdy[!index]) - (dxdy[index]);

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
            point[!index] += increment_other;
            discriminant += 2*(dxdy[!index]-dxdy[index]);
        } else {
            discriminant += 2*dxdy[!index];
        }
    }
    return point-start;
    
}

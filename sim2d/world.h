/*
    TODO there is a bug where once the robot collides then it is stuck forever
    probably better to use only integers instead of doubles (convert at the end of each step)
    modify run method to handle quit without crashing
    program often does x/map.resolution maybe store map.resolution^-1
*/
#pragma once
#include "map.h"
#include "robot.h"
#include "vector"
#include "static_vec.h"
#include <mutex>

#define UPDATE_RATE 100 // ms


namespace sim2d {
class World {
    public:
    Map map;
    Robot robot;
    const int radius_cells; // robot size, but in pixels
    const std::vector<grid2> robot_footprint; // a circle that represents the robot in cells

    World(Map&m,double robot_size): map(m), 
                                      robot(robot_size),                             
                                      radius_cells((int)(robot.radius/map.resolution)),
                                      robot_footprint(get_footprint()) {}

    World(Map& m, Robot& r): map(m), 
                             robot(r),
                             radius_cells((int)(robot.radius/map.resolution)),
                             robot_footprint(get_footprint())
                             {}

    void update();
    
    bool check_collision(vec2 pxy) const;

    // assuming the robot is a plate like a roomba the footprint is all points 
    // such that x^2 + y^2 <= radius^2
    std::vector<grid2> get_footprint();

    inline grid2 real_units_to_cell(vec2 pxy) const {
        return {(cell_index)(pxy[0]/map.resolution),
                (cell_index)(pxy[1]/map.resolution)};
    }

    void run(); 

    inline const vec3 get_xyp() { // blocking
        m.lock();
        const vec3 pose(robot.pose);
        m.unlock();
        return pose;
    }

    inline void set_xyp(vec3 pose) { // blocking
        m.lock();
        robot.pose = pose;
        m.unlock();
    }

    inline const vec2 get_vel() { // blocking
        m.lock();
        const vec2 vel(robot.vel);
        m.unlock();
        return vel;
    }

    inline void set_vel(vec2 vel) { // blocking
        m.lock();
        robot.vel = vel;
        m.unlock();
    }

    protected:

    std::mutex m;
    void _run(int MS);
};
}
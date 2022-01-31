/*
    modify run method to handle quit without crashing
    program often does x/map.resolution maybe store map.resolution^-1
*/
#pragma once
#include "map.h"
#include "robot.h"
#include "laser.h"
#include "vector"
#include "static_vec.h"
#include <mutex>
#include <thread>

#define LASER_INCREMENT 0.1 // m

namespace sim2d {
class World {
    public:

    const double UPDATE_RATE = 1; // ms;

    Map map;
    Robot robot;
    Laser laser;
    const int radius_cells; // robot size, but in pixels
    const int range_cells; // laser range, but in pixels
    const std::vector<grid2> robot_footprint; // a circle that represents the robot in cells
    std::vector<grid2> laser_scans;


    World(Map&m,double robot_size): map(m), 
                                      robot(robot_size),                             
                                      radius_cells((int)(robot.radius/map.resolution)),
                                      range_cells((int)(laser.range/map.resolution)),
                                      laser_scans(laser.n_samples),
                                      robot_footprint(get_footprint()) {}

    World(Map& m, Robot& r): map(m), 
                             robot(r),
                             radius_cells((int)(robot.radius/map.resolution)),
                             range_cells((int)(laser.range/map.resolution)),
                             laser_scans(laser.n_samples),
                             robot_footprint(get_footprint()) {}

    inline grid2 real_units_to_cell(vec2 pxy) const {
        return {(cell_index)(pxy[0]/map.resolution),
                (cell_index)(pxy[1]/map.resolution)};
    }

    std::thread run(); 

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

    inline void stop() {
        m.lock();
        running = false;
        m.unlock();
    }

    protected:
    bool running;
    std::mutex m;

    void _run(int MS);
    void update();
    bool check_collision(vec2 pxy) const;
    // assuming the robot is a plate like a roomba the footprint is all points 
    // such that x^2 + y^2 <= radius^2
    std::vector<grid2> get_footprint();
    void compute_laser_scans();
};
}
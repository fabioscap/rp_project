/*
    program often does x/map.resolution maybe store map.resolution^-1 to avoid float division
*/
#pragma once
#include "map.h"
#include "robot.h"
#include "laser.h"
#include "vector"
#include "static_vec.h"
#include <mutex>
#include <thread>

namespace sim2d {
class World {
    public:

    const double UPDATE_RATE = 1; // s;

    Map map;
    Robot robot;
    Laser laser;
    std::vector<grid2> laser_scans_grid; // contains the points in the map where the beams bounce (or max range if don't bounce)
    std::vector<float> laser_scans_ranges;
    const int radius_cells; // robot size, but in pixels
    const std::vector<grid2> robot_footprint; // all the cells occupied by the robot

    vec3 initial_pose; // useful for odom

    World(Map&m,double robot_size): map(m), 
                                      robot(robot_size),                             
                                      radius_cells(round((robot.radius/map.resolution))),
                                      range_cells(round((laser.range/map.resolution))),
                                      laser_scans_grid(laser.n_samples),
                                      laser_scans_ranges(laser.n_samples),
                                      robot_footprint(get_footprint()) {}

    World(Map& m, Robot& r): map(m), 
                             robot(r),
                             radius_cells(round((robot.radius/map.resolution))),
                             range_cells(round((laser.range/map.resolution))),
                             laser_scans_grid(laser.n_samples),
                             laser_scans_ranges(laser.n_samples),
                             robot_footprint(get_footprint()) {}

    std::thread run(); 

    inline const vec3 get_xyp() { // blocking until it can lock
        m.lock();
        const vec3 pose(odom_to_map(robot.pose));
        m.unlock();
        return pose;
    }

    inline const vec3 get_xyp_odom() { // blocking
        m.lock();
        const vec3 pose(robot.pose);
        m.unlock();
        return pose;
    }

    inline const void get_laser_scans_grid(std::vector<grid2>& scans) {
        m.lock();
        scans = laser_scans_grid; // copy
        m.unlock();
    }

    inline const void get_laser_scans_ranges(std::vector<float>& scans) {
        m.lock();
        scans = laser_scans_ranges; // copy
        m.unlock();
    }

    inline void set_xyp(const vec3& pose) { // blocking
        m.lock();
        initial_pose = pose;
        robot.pose = {0,0,0};
        m.unlock();
    }

    inline const vec2 get_vel() { // blocking
        m.lock();
        const vec2 vel(robot.vel);
        m.unlock();
        return vel;
    }

    inline const vec3 get_vel_cartesian() { // blocking
        m.lock();
        const vec3 vel(robot.get_xya_speed(initial_pose[2]));
        m.unlock();
        return vel;
    }

    inline void set_vel(const vec2& vel) { // blocking
        m.lock();
        robot.vel_reference = vel;
        m.unlock();
    }

    inline void stop() {
        m.lock();
        running = false;
        m.unlock();
    }

    protected:

    const int range_cells; // laser range, but in pixels

    bool running;
    std::mutex m;

    void _run(int MS);
    void update();

    // checks if the robot in pxy collides with a wall or is out of the map
    bool check_collision(const vec2& pxy) const;

    // returns a vector of all the cells occupied by the robot
    // assuming the robot is a plate like a roomba the footprint is all points 
    // such that x^2 + y^2 <= radius^2
    std::vector<grid2> get_footprint();

    void compute_laser_scans();
    // traces a beam on the map
    grid2 laser_hit(const grid2& start,const grid2& end);

    inline vec3 odom_to_map(const vec3& pose) {
        return initial_pose + vec3({robot.pose[0]*cos(initial_pose[2])-robot.pose[1]*sin(initial_pose[2]),
                                    robot.pose[0]*sin(initial_pose[2])+robot.pose[1]*cos(initial_pose[2]),
                                    robot.pose[2]});
    }
    /*
    inline vec2 odom_to_map(const vec2& pose) {
        return {initial_pose[0]+robot.pose[0]*cos(initial_pose[2])-robot.pose[1]*sin(initial_pose[2]),
                initial_pose[1]+robot.pose[0]*sin(initial_pose[2])-robot.pose[1]*cos(initial_pose[2])};
    }
    */

    inline grid2 real_units_to_cell(const vec2& pxy) const {
        return {(cell_index)(round(pxy[0]/map.resolution)),
                (cell_index)(round(pxy[1]/map.resolution))};
    }


};
}
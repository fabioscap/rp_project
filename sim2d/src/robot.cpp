
#include "robot.h"
#include <cmath>


using namespace sim2d;
// kinematics
rp::Vec_<double,2> Robot::get_xy_speed() {
    rp::Vec_<double,2> cartesian_speed;
    cartesian_speed[0] = vl()*cos(pa());
    cartesian_speed[1] = vl()*sin(pa());
    
    return cartesian_speed;
}

void Robot::move(double dt) {
    pose[0] += dt*vel[0]*cos(pose[2]);
    pose[1] += dt*vel[0]*sin(pose[2]);
    pose[2] += dt*vel[1]; // TODO wrap around [-pi;pi)
}
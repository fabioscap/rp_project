
#include "robot.h"
#include <cmath>


using namespace sim2d;
// kinematics
vec3 Robot::get_xya_speed(double offset) {
    return {
        vl()*cos(pa()+offset),
        vl()*sin(pa()+offset),
        va()};
}

void Robot::move(double dt) {
    pose[0] += dt*vel[0]*cos(pose[2]);
    pose[1] += dt*vel[0]*sin(pose[2]);
    pose[2] += dt*vel[1]; // TODO wrap around [-pi;pi)
}
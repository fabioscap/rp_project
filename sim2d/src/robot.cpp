
#include "robot.h"
#include <cmath>


template <typename T>
T clamp(T v, T value){
  if (v>value)
    return value;
  if (v<-value)
    return -value;
  return v;
}

using namespace sim2d;

void Robot::update(double dt) {
    // P 
    vel[0] = clamp(10*(vel_reference[0]-vel[0]),al_max) + vel[0];
    vel[1] = clamp(1*(vel_reference[1]-vel[1]),aa_max) + vel[1];
    //
}

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
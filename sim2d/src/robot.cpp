#include "robot.h"
#include <cmath>
#include <iostream>

using namespace sim2d;


// kinematics
void Robot::move(double dt) {
    
    (*this).px += vl*cos(pa)*dt;
    (*this).py += vl*sin(pa)*dt;
    (*this).pa += va*dt;
    
}
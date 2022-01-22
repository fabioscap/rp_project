
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

}
/*
virtual 2D robot (suppose a unicycle model)
it is characterized by:
-> its size (radius)
-> linear speed 
-> rotational speed

[ maybe not related in the robot but to the world]
-> position
-> orientation
*/
#pragma once

#include <iostream>
#include <array>
#include <static_vec.h>

typedef rp::Vec_<double,2> pos2;

namespace sim2d {
class Robot {
    public:

    const double size; // m
    rp::Vec_<double,2> vel; // linear vel and angular vel (m/s rad/s)
    rp::Vec_<double,3> pose; // m m rad

    Robot(double s): size(s) {}

    inline double& vl() {return vel.at(0);}
    inline double& va() {return vel[1];}
    inline double& px() {return pose.at(0);}
    inline double& py() {return pose[1];}
    inline double& pa() {return pose[3];}
    inline const double& vl() const {return vel.at(0);}
    inline const double& va() const {return vel[1];}
    inline const double& px() const {return pose[0];}
    inline const double& py() const {return pose[1];}
    inline const double& pa() const {return pose[3];}
    inline const pos2 pxy() const {return {px(),py()};}


    pos2 get_xy_speed();
    void move(double dt);


    

};
}
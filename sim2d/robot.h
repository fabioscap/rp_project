/*
virtual 2D robot (suppose a unicycle model)
it is characterized by:
-> its size (radius)
-> linear speed 
-> rotational speed

[ maybe not related to the robot but to the world]
-> position
-> orientation
*/
#pragma once

#include <iostream>
#include <array>
#include <static_vec.h>

typedef rp::Vec_<double,2> vec2;
typedef rp::Vec_<double,3> vec3;

namespace sim2d {
class Robot {
    public:

    const double radius; // m

    vec2 vel; // linear vel and angular vel (m/s rad/s)
    vec2 vel_reference; // linear vel and angular target vel (m/s rad/s) 
    vec3 pose; // m m rad

    // currently both for accelerating and for braking
    const double al_max = 0.05; // max linear acc m/s^2
    double aa_max = 0.05; // max angular acc rad/s^2

    Robot(double s): radius(s),pose({0,0,0}),vel({0,0}),vel_reference({0,0}) {}

    inline double& vl() {return vel[0];}
    inline double& va() {return vel[1];}
    inline double& px() {return pose[0];}
    inline double& py() {return pose[1];}
    inline double& pa() {return pose[2];}
    inline const double& vl() const {return vel[0];}
    inline const double& va() const {return vel[1];}
    inline const double& px() const {return pose[0];}
    inline const double& py() const {return pose[1];}
    inline const double& pa() const {return pose[2];}
    inline const vec2 pxy() const {return {px(),py()};}


    vec3 get_xya_speed(double offset);
    void move(double dt);
    void update(double dt);

    

};
}
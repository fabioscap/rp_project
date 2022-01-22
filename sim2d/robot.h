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
namespace sim2d {
class Robot {
    public:

    const double size; // m
    double vl, va; // m/s rad/s
    double px, py, pa; // m m rad

    void move(double dt);

    Robot(double s): size(s) {}
};
}
#pragma once

#include "world.h"
#include "geometry_msgs/Twist.h"
#include <memory>

namespace sim2d_ros {

class World_ros {
    public:
    sim2d::World* w; // I use C pointers because the world is in the stack. 

    World_ros(sim2d::World& world) { w = &world;}

    void cmd_vel_callback(const geometry_msgs::TwistConstPtr& msg);
};
}
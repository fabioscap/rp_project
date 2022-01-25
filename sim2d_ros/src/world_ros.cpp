#include "world_ros.h"



using namespace sim2d_ros;

void World_ros::cmd_vel_callback(const geometry_msgs::TwistConstPtr& msg) {
    w->set_vel({ 
                msg->linear.x,
                msg->angular.z
                });
}

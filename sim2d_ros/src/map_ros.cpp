#include "map_ros.h"

using namespace sim2d_ros;

void Map_ros::map_from_ros_msg(const nav_msgs::OccupancyGridConstPtr& msg) {
    got_map = 1;

    map_ptr.reset(new sim2d::Map(msg->info.resolution,
                                 msg->info.width,
                                 msg->info.height,
                                 msg->data));
}
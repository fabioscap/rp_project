#pragma once

#include "map.h"
#include "nav_msgs/OccupancyGrid.h"


namespace sim2d_ros {
// this class is used to create a map within a topic callback so I can avoid using 
// global variables
class Map_ros {
    public:
    std::unique_ptr<sim2d::Map> map_ptr;
    bool got_map = 0;
    
    //https://answers.ros.org/question/11810/how-to-pass-arguments-tofrom-subscriber-callback-functions/
    //https://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
    void map_from_ros_msg(const nav_msgs::OccupancyGridConstPtr& msg);

};
}
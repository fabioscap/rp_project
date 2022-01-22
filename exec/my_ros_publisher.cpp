#include "world.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace sim2d;

//http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
int main(int argc, char* argv[]) {
    ros::init(argc,argv,"my_publisher");
}
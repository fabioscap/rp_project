#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "world.h"
#include "map_ros.h"


uint64_t timeSinceEpochMillisec() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "listener");


    ros::NodeHandle n;
    sim2d_ros::Map_ros m_ros;

    std::string map_topic;
    if (!n.getParam("/map_topic",map_topic)) map_topic = "/map";

    ros::Subscriber sub = n.subscribe(map_topic, 1, &sim2d_ros::Map_ros::map_from_ros_msg, &m_ros);

    ROS_INFO_STREAM("waiting for map publisher on topic: "<< map_topic);
    while (ros::ok()) { // TODO add max delay and polling every n sec
        if (m_ros.got_map) {
            break;
        }
        ros::spinOnce();
    }
    sub.shutdown();
    sim2d::Map m(*(m_ros.map_ptr));
    ROS_INFO_STREAM("got map: "<< m.width<<"x"<<m.height<<" @"<<m.resolution<<" m/pixel");
    /*
        using namespace std::chrono;
    auto now = system_clock::now(); 
    auto next = now + milliseconds(UPDATE_RATE);
    using namespace sim2d;
    while(1) {
        std::cerr << w.robot.pxy() << std::endl;
        w.update();

        std::this_thread::sleep_until(next);
        next+=milliseconds(UPDATE_RATE);
    }
    */
    return 0;
}
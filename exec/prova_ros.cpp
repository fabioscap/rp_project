#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "world.h"
#include "map_ros.h"
#include "world_ros.h"

int main(int argc, char* argv[]) {
    
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;
    sim2d_ros::Map_ros m_ros;
    double robot_radius;

    std::string map_topic, cmd_vel_topic, odom_topic, scan_topic;

    if (!n.getParam("/map_topic",map_topic)) map_topic = "/map";
    if (!n.getParam("/cmd_vel_topic",cmd_vel_topic)) cmd_vel_topic = "/cmd_vel";
    if (!n.getParam("/scan_topic",scan_topic)) scan_topic = "/base_scan";
    if (!n.getParam("/robot_radius",robot_radius)) robot_radius = 0.15;

    //n.setParam("/use_sim_time",true); but then i have to sync the simulator with /clock?

    ros::Subscriber sub = n.subscribe(map_topic, 1, &sim2d_ros::Map_ros::map_from_ros_msg, &m_ros);

    ROS_INFO_STREAM("waiting for map publisher on topic: "<< map_topic);
    while (ros::ok()) { // TODO add timeout and maybe polling every n seconds or swith to service request
        if (m_ros.got_map) {
            // should get the map as soon as it subscribes
            break;
        }
        ros::spinOnce();
    }
    sub.shutdown();
    sim2d::World w(*(m_ros.map_ptr),robot_radius);
    m_ros.map_ptr.reset(); // free the copied map
    ROS_INFO_STREAM("got map: "<< w.map.width<<"x"<<w.map.height<<" @"<<w.map.resolution<<" m/cell");
    
    sim2d_ros::World_ros w_ros(w);
    ros::Subscriber vel_commands = n.subscribe(cmd_vel_topic, 10, &sim2d_ros::World_ros::cmd_vel_callback, &w_ros);

    w.set_xyp( {10,10,0} ); // test position that does not collide in aula41
    w.set_vel( {0,0});
    w.run();

    ros::spin();
    return 0;
}
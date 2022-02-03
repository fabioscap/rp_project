#include "ros/ros.h"
#include "ros/console.h"

#include "map_ros.h"
#include "world_ros.h"
#include "gui.h"

#include <thread>

#include "sensor_msgs/LaserScan.h"

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace sim2d;

int main(int argc, char* argv[]) {
    
    ros::init(argc, argv, "sim2d");

    ros::NodeHandle n;
    sim2d_ros::Map_ros m_ros;
    double robot_radius;
    std::string map_topic, cmd_vel_topic, odom_topic, scan_topic;

    if (!n.getParam("/map_topic",map_topic)) map_topic = "/map";
    if (!n.getParam("/cmd_vel_topic",cmd_vel_topic)) cmd_vel_topic = "/cmd_vel";
    if (!n.getParam("/scan_topic",scan_topic)) scan_topic = "/scan";
    if (!n.getParam("/robot_radius",robot_radius)) robot_radius = 0.15;

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

    w.set_xyp( {3.5,7.7, M_PI/4});
    w.set_vel( {0, 0} );

    sim2d_g::Gui gui(w,10);
    std::thread simul_thread = w.run();
    std::thread gui_thread = gui.run();

    ros::Publisher scan_publisher = n.advertise<sensor_msgs::LaserScan>(scan_topic,10);
    std::vector<float> scans;
    sensor_msgs::LaserScan scan_msg;

    // publish base_link->laser (static)
    tf2_ros::StaticTransformBroadcaster base_link_to_laser;
    geometry_msgs::TransformStamped tf_msg;

    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "base_link";
    tf_msg.child_frame_id = "laser";
    // currently they are in the same place
    tf_msg.transform.translation.x = 0;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    base_link_to_laser.sendTransform(tf_msg);

    ros::Rate r(10); // 10 hz
    while (ros::ok()) {
        ros::Time now = ros::Time::now();

        ros::spinOnce(); // fetch incoming messages

        // publish odom->base_link

        // publish odom

        // publish base scan
        w.get_laser_scans_ranges(scans);
        scan_msg.angle_min = w.laser.from;
        scan_msg.angle_max = w.laser.to;
        scan_msg.angle_increment = w.laser.increment;
        scan_msg.range_min = 0; // i have not implemented range_min yet
        scan_msg.range_max = w.laser.range;
        scan_msg.ranges = scans;
        //scan_msg.scan_time
        //scan_msg.time_increment
        //scan_msg.intensities
        scan_msg.header.frame_id = "laser";
        scan_msg.header.stamp = now;
        scan_publisher.publish(scan_msg);

        r.sleep();
    }

    gui.stop();
    gui_thread.join();
    w.stop();
    simul_thread.join();

    return 0;
}
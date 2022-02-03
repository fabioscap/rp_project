# rp_project 2d robot simulator
#### fabio scaparro

A ROS node that implements a 1 robot simulator. Reads the map from the map_server. Reads /cmd_vel, produces /odom /tf and /base_scan. To run this program, you need a ROS installation with the basic packages:

- tf2
- geometry_msgs
- nav_msgs
- roscpp
- sensor_msgs

To use the graphical interface you also need SDL2 (https://www.libsdl.org/index.php). On Ubuntu you can do:
> sudo apt install libsdl2-dev

To build the package clone it in a catkin workspace and compile it with catkin_make.

To run the package you need to:
- run roscore

either
- source the workspace where the package is and run "rosrun rp_project rp_project_sim2d"

or 
- run "./{the catkin ws}/devel/lib/rp_project/rp_project_sim2d"

The program will start and will be waiting for a message on the topic /map. To provide a map run the map_server in another terminal. Currently this package supports only maps in "trinary" mode. (see http://wiki.ros.org/map_server#Value_Interpretation). A map that works is in the folder maps/ (aula41). After receiving the map the simulator will start and you will be able to control the robot through twist messages in the /cmd_vel topic. 

To modify the size of the robot you can set the "/robot_radius" parameter before executing the node. You can also modify the parameters of the laser scan by modifying the constructor in sim2d/laser.h (other methods not yet implemented).

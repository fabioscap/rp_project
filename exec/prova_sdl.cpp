#include "ros/ros.h"
#include "ros/console.h"

#include "map_ros.h"
#include "world.h"
#include "SDL2/SDL.h"
#include "world_ros.h"

#define MAP_WALL    0x00000000
#define MAP_EMPTY   0xffffffff
#define MAP_UNKNOWN 0xff4b5e5b

using namespace sim2d;

void draw_circle(SDL_Renderer *renderer, int x, int y, int radius, SDL_Color color,const sim2d::World &w)
{
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    auto it = w.robot_footprint.begin();
    while (it != w.robot_footprint.end()) { //openmp?
        auto point = *it;
        SDL_RenderDrawPoint(renderer, point[0]+x,point[1]+y);
        it++;
    }

    
}


int main(int argc, char* argv[]) {
    
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;
    sim2d_ros::Map_ros m_ros;
    double robot_radius;
    std::string map_topic, cmd_vel_topic, odom_topic, scan_topic;

    if (!n.getParam("/map_topic",map_topic)) map_topic = "/map";
    if (!n.getParam("/cmd_vel_topic",cmd_vel_topic)) cmd_vel_topic = "/cmd_vel";
    if (!n.getParam("/scan_topic",scan_topic)) scan_topic = "/base_scan";
    if (!n.getParam("/robot_radius",robot_radius)) robot_radius = 0.5;

    ros::Subscriber sub = n.subscribe(map_topic, 1, &sim2d_ros::Map_ros::map_from_ros_msg, &m_ros);

    //ROS_INFO_STREAM("waiting for map publisher on topic: "<< map_topic);
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
    ROS_INFO_STREAM("robot radius: "<< w.radius_cells);
    sim2d_ros::World_ros w_ros(w);
    ros::Subscriber vel_commands = n.subscribe(cmd_vel_topic, 10, &sim2d_ros::World_ros::cmd_vel_callback, &w_ros);
    

    SDL_Init(SDL_INIT_VIDEO);
    auto window = SDL_CreateWindow( "Prova SDL", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, w.map.width, w.map.height, SDL_WINDOW_SHOWN );
    auto renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED );

    // this stores the map
    auto map_texture = SDL_CreateTexture( renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, w.map.width, w.map.height );
    
    // convert occupancy grid into argb
    uint32_t *texture_buffer = new uint32_t[ w.map.dimension ];

    for (int i=0; i<w.map.width; ++i) {
        for (int j=0 ; j<w.map.height; ++j) {

            if (w.map.at(i,j) == SIM2d_WALL) texture_buffer[(w.map.height-1-j)*w.map.width+i] = MAP_WALL;
            else if (w.map.at(i,j) == SIM2d_EMPTY) texture_buffer[(w.map.height-1-j)*w.map.width+i] = MAP_EMPTY;
            else if (w.map.at(i,j) == SIM2d_UNK ) texture_buffer[(w.map.height-1-j)*w.map.width+i]  = MAP_UNKNOWN;
            else {
                std::cout << "value "<< (int)(w.map.at(i,j)) << " not supported" << std::endl;
            }
        }
    }

    SDL_SetRenderDrawColor(renderer, 0,255,255,255);
    SDL_RenderClear(renderer);

    SDL_UpdateTexture( map_texture , NULL, texture_buffer, w.map.width * sizeof (uint32_t));
    delete[] texture_buffer;

    SDL_RenderCopy(renderer,map_texture,NULL,NULL);

    SDL_RenderPresent(renderer);

    w.set_xyp( {10,10, M_PI/4});
    w.set_vel( {0, 0} );
    
    w.run();
    SDL_Event event;
    bool running = true;
    
    while(1) {
            while (SDL_PollEvent(&event)){
            if (event.type == SDL_QUIT){
                std::cout << "stop " << std::endl;
                running = false;
                break;
            }
        }
        if (!running) break;
        vec3 pose = w.get_xyp();
        ros::spinOnce();
        //SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer,map_texture,NULL,NULL);

        draw_circle(renderer, pose[0]/w.map.resolution, w.map.height-1-pose[1]/w.map.resolution,w.radius_cells,SDL_Color{0xff,0x00,0xff,0xff},w);

        SDL_RenderPresent(renderer);

        SDL_Delay(100);
    }
    

    //ros::spin();
    SDL_Quit();
    return 0;
}
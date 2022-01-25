#include "ros/ros.h"
#include "ros/console.h"

#include "map_ros.h"
#include "SDL2/SDL.h"

#define MAP_BLACK 0x00000000
#define MAP_WHITE 0xffffffff
#define MAP_UNKNOWN 0xff4b5e5b

void draw_circle(SDL_Renderer *renderer, int x, int y, int radius, SDL_Color color)
{
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    for (int w = 0; w < radius * 2; w++)
    {
        for (int h = 0; h < radius * 2; h++)
        {
            int dx = radius - w; // horizontal offset
            int dy = radius - h; // vertical offset
            if ((dx*dx + dy*dy) <= (radius * radius))
            {
                SDL_RenderDrawPoint(renderer, x + dx, y + dy);
            }
        }
    }
}


int main(int argc, char* argv[]) {
    
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;
    sim2d_ros::Map_ros m_ros;

    std::string map_topic;

    if (!n.getParam("/map_topic",map_topic)) map_topic = "/map";

    ros::Subscriber sub = n.subscribe(map_topic, 1, &sim2d_ros::Map_ros::map_from_ros_msg, &m_ros);

    ROS_INFO_STREAM("waiting for map publisher on topic: "<< map_topic);
    while (ros::ok()) { // TODO add timeout and maybe polling every n seconds or swith to service request
        if (m_ros.got_map) {
            // should get the map as soon as it subscribes
            break;
        }
        ros::spinOnce();
    }
    //sub.shutdown();
    sim2d::Map m(*(m_ros.map_ptr));
    m_ros.map_ptr.reset(); // free the copied map

    SDL_Init(SDL_INIT_VIDEO);
    auto window = SDL_CreateWindow( "SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, m.width, m.height, SDL_WINDOW_SHOWN );
    auto renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED );

    auto map_texture = SDL_CreateTexture( renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, m.width, m.height );
    
    // convert occupancy grid into argb
    uint32_t *texture_buffer = new uint32_t[ m.dimension ];
    for (int i=0; i<m.dimension; ++i) {
        if (m.data[i] > 0) texture_buffer[i] = MAP_BLACK;
        else if (m.data[i] == SIM2d_EMPTY) texture_buffer[i] = MAP_WHITE;
        else if (m.data[i] == SIM2d_UNK ) texture_buffer[i]  = MAP_UNKNOWN;
        else {
            
        }
    }


    SDL_SetRenderDrawColor(renderer, 0,255,255,255);
    SDL_RenderClear(renderer);

    SDL_UpdateTexture( map_texture , NULL, texture_buffer, m.width * sizeof (uint32_t));
    delete[] texture_buffer;

    SDL_RenderCopy(renderer,map_texture,NULL,NULL);

    draw_circle(renderer, 30,30,10,SDL_Color{0xff,0xff,0x00,0xff});

    SDL_RenderPresent(renderer);


    ros::spin();
    SDL_Quit();
    return 0;
}
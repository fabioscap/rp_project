#include "gui.h"
#include <thread>
#include <chrono>


#define MAP_WALL    0x00000000
#define MAP_EMPTY   0xffffffff
#define MAP_UNKNOWN 0xff4b5e5b

void sim2d_g::Gui::draw_robot(const vec3&pose) // draws the robot on the screen
{
    int x = pose[0]/world.map.resolution;
    int y = world.map.height-1-pose[1]/world.map.resolution;
    double a = pose[2];
    SDL_SetRenderDrawColor(renderer, 0xff, 0x00, 0xff, 0xff);
    
    auto it = world.robot_footprint.begin();
    while (it != world.robot_footprint.end()) { //openmp?
        auto point = *it;
        SDL_RenderDrawPoint(renderer, point[0]+x,point[1]+y);
        it++;
    }
    SDL_SetRenderDrawColor(renderer, 0x00, 0x55, 0xff, 0xff);
    SDL_RenderDrawLine(renderer,x,y,x+world.radius_cells*cos(a),y-world.radius_cells*sin(a));
}

std::thread sim2d_g::Gui::run() {

    SDL_Init(SDL_INIT_VIDEO);
    window = SDL_CreateWindow( "Prova SDL", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, world.map.width, world.map.height, SDL_WINDOW_SHOWN );
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED );

    // this stores the map
    map_texture = SDL_CreateTexture( renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, world.map.width, world.map.height );
    
    // convert occupancy grid into argb
    uint32_t *texture_buffer = new uint32_t[ world.map.dimension ];

    for (int i=0; i<world.map.width; ++i) {
        for (int j=0 ; j<world.map.height; ++j) {

            if (world.map.at(i,j) == SIM2d_WALL) texture_buffer[(world.map.height-1-j)*world.map.width+i] = MAP_WALL;
            else if (world.map.at(i,j) == SIM2d_EMPTY) texture_buffer[(world.map.height-1-j)*world.map.width+i] = MAP_EMPTY;
            else if (world.map.at(i,j) == SIM2d_UNK ) texture_buffer[(world.map.height-1-j)*world.map.width+i]  = MAP_UNKNOWN;
        }
    }
 
    SDL_UpdateTexture( map_texture , NULL, texture_buffer, world.map.width * sizeof (uint32_t));

    delete[] texture_buffer;

    running = true;
    return std::thread(&sim2d_g::Gui::_run,this);

}

void sim2d_g::Gui::_run() {
    SDL_Event event;

    auto now = std::chrono::system_clock::now(); 
    auto next = now + std::chrono::milliseconds(update_rate);
    while(running) {
        if (SDL_PollEvent(&event)){
            if (event.type == SDL_QUIT){
                running = false;
                break;
            }
        }

        vec3 pose = world.get_xyp();

        SDL_RenderCopy(renderer,map_texture,NULL,NULL);

        draw_robot(pose);
    
        SDL_RenderPresent(renderer);

        std::this_thread::sleep_until(next);
        next+=std::chrono::milliseconds(update_rate);
    }

    SDL_Quit();
}

#include "gui.h"
#include <thread>
#include <chrono>


#define MAP_WALL    0x00000000
#define MAP_EMPTY   0xffffffff
#define MAP_UNKNOWN 0xff4b5e5b

void sim2d_g::Gui::draw_robot(const vec3&pose) // draws the robot on the screen
{
    int x = pose[0]/w_ptr->map.resolution;
    int y = w_ptr->map.height-1-pose[1]/w_ptr->map.resolution;
    double a = pose[2];
    SDL_SetRenderDrawColor(renderer, 0xff, 0x00, 0xff, 0xff);
    
    auto it = w_ptr->robot_footprint.begin();
    while (it != w_ptr->robot_footprint.end()) { //openmp?
        auto point = *it;
        SDL_RenderDrawPoint(renderer, point[0]+x,point[1]+y);
        it++;
    }
    SDL_SetRenderDrawColor(renderer, 0x00, 0x55, 0xff, 0xff);
    SDL_RenderDrawLine(renderer,x,y,x+w_ptr->radius_cells*cos(a),y-w_ptr->radius_cells*sin(a));
}

std::thread sim2d_g::Gui::run() {

    SDL_Init(SDL_INIT_VIDEO);
    window = SDL_CreateWindow( "Prova SDL", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, w_ptr->map.width, w_ptr->map.height, SDL_WINDOW_SHOWN );
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED );

    // this stores the map
    map_texture = SDL_CreateTexture( renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, w_ptr->map.width, w_ptr->map.height );
    
    // convert occupancy grid into argb
    uint32_t *texture_buffer = new uint32_t[ w_ptr->map.dimension ];

    for (int i=0; i<w_ptr->map.width; ++i) {
        for (int j=0 ; j<w_ptr->map.height; ++j) {

            if (w_ptr->map.at(i,j) == SIM2d_WALL) texture_buffer[(w_ptr->map.height-1-j)*w_ptr->map.width+i] = MAP_WALL;
            else if (w_ptr->map.at(i,j) == SIM2d_EMPTY) texture_buffer[(w_ptr->map.height-1-j)*w_ptr->map.width+i] = MAP_EMPTY;
            else if (w_ptr->map.at(i,j) == SIM2d_UNK ) texture_buffer[(w_ptr->map.height-1-j)*w_ptr->map.width+i]  = MAP_UNKNOWN;
        }
    }
 
    SDL_UpdateTexture( map_texture , NULL, texture_buffer, w_ptr->map.width * sizeof (uint32_t));

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

        vec3 pose = w_ptr->get_xyp();

        SDL_RenderCopy(renderer,map_texture,NULL,NULL);

        draw_robot(pose);
    
        SDL_RenderPresent(renderer);

        std::this_thread::sleep_until(next);
        next+=std::chrono::milliseconds(update_rate);
    }

    SDL_Quit();
}

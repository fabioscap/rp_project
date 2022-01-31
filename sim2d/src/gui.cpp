#include "gui.h"
#include <thread>
#include <chrono>


#define MAP_WALL    0x00000000
#define MAP_EMPTY   0xffffffff
#define MAP_UNKNOWN 0xff4b5e5b

void sim2d_g::Gui::draw_robot(cell_index x, cell_index y, double a) { // draws the robot on the screen 

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

    std::vector<grid2> scans;

    while(running) {
        if (SDL_PollEvent(&event)){
            if (event.type == SDL_QUIT){
                running = false;
                break;
            }
        }

        vec3 pose = world.get_xyp();
        cell_index x = round(pose[0]/world.map.resolution);
        cell_index y = round(world.map.height-1-pose[1]/world.map.resolution); // flip y to draw correctly
        double a = pose[2];

        world.get_laser_scans(scans);
        SDL_SetRenderDrawColor(renderer, 0x00, 0xff, 0xcc, 0xff);

        SDL_RenderCopy(renderer,map_texture,NULL,NULL); // draw map


    
        auto it = scans.begin();
        while (it != scans.end()) {

            SDL_RenderDrawLine(renderer,x,y,x+(*it)[0],y-(*it)[1]); // y is always flipped
            it++;
        }          

        draw_robot(x,y,a); // draw robot
        
        SDL_RenderPresent(renderer);

        std::this_thread::sleep_until(next);
        next+=std::chrono::milliseconds(update_rate);
    }

    SDL_Quit();
}

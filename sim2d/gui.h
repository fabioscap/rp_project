#pragma once

#include "world.h"
#include <memory>
#include <thread>

#include "SDL2/SDL.h"

namespace sim2d_g {

class Gui {
    public:

    SDL_Renderer* renderer;
    SDL_Window* window;
    SDL_Texture* map_texture;

    int update_rate = 100; //ms

    sim2d::World* w_ptr;

    Gui(sim2d::World& w) {
        w_ptr = &w;
    }
    Gui(sim2d::World& w, int update_rate): update_rate(update_rate) {
        w_ptr = &w;

    }

    std::thread run();

    inline void stop() {
        m.lock();
        running = false;
        m.unlock();
    }
    
    protected:
    std::mutex m;
    bool running = false;
    void draw_robot(const vec3&pose);
    void _run();

};

}
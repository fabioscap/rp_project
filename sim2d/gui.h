#pragma once

#include "world.h"
#include <memory>
#include <thread>

#include "SDL2/SDL.h"

namespace sim2d_g {

// TODO implement zoom, pan, resize...
// https://gigi.nullneuron.net/gigilabs/handling-keyboard-and-mouse-events-in-sdl2/
class Gui {
    public:

    SDL_Renderer* renderer;
    SDL_Window* window;
    SDL_Texture* map_texture;

    int update_rate = 100; //ms

    sim2d::World& world;

    Gui(sim2d::World& w): world(w) {}
    Gui(sim2d::World& w, int update_rate): update_rate(update_rate), world(w) {}

    std::thread run();

    inline void stop() {
        m.lock();
        running = false;
        m.unlock();
    }
    
    protected:
    std::mutex m;
    bool running = false;
    void draw_robot(cell_index x, cell_index y, double a);
    void _run();

};

}
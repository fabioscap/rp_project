#include "world.h"
#include <math.h>

using namespace sim2d;

void World::update(){

    robot.move((double)(UPDATE_RATE)/1000);

}
bool World::check_collision() {
    

}

std::vector<pos2> World::get_footprint() {
    std::vector<pos2> vec;
    int r2 = radius*radius;
    for(int x =-radius; x<=radius; ++x) {
        for(int y =-radius; y<=radius; ++y) {
            int d = (x)*(x)+(y)*(y);
            std::cout << d<< std::endl;
            if (d<=radius*radius+1) {
                // point belongs to robot
                vec.push_back(pos2{{x,y}});
            }
        }
    }
    return vec;
}

#include "world.h"

using namespace sim2d;

void World::update(){

    robot.move((double)(UPDATE_RATE)/1000);

}
bool World::check_collision() {

    return false;

}
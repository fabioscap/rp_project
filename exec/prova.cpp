#include "world.h"
#include "ros/ros.h"

#include <iostream>
#include <thread>
#include <chrono>


using namespace sim2d;

uint64_t timeSinceEpochMillisec() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

int main() {
    std::cout << "ciao" << std::endl;

    Map m = Map(0.02,30,30,nullptr);

    int8_t data[30*30];
    m.data = data;
    for (int8_t i=0; i<30; ++i) {
        for (int8_t j=0; j<30; ++j) {
            if (i==0 || i == 30-1 || j == 0 || j == 30-1) 
                m.at(i,j) = WALL;
            else m.at(i,j) = 0;
        }
    }

    Robot r = Robot(0.5);
    r.vl = 1;
    r.pa = 0;
    r.px = 1; r.py = 0;
    World w = World(m,r);

    //https://stackoverflow.com/questions/46609863/execute-function-every-10-ms-in-c
    // how to do a timed loop
    /*
    using namespace std::chrono;
    auto now = system_clock::now(); 
    auto next = now + milliseconds(UPDATE_RATE);
    using namespace sim2d;
    while(1) {
        std::cerr << w.robot.px << " " << w.robot.py << std::endl;
        w.update();
        std::this_thread::sleep_until(next);
        next+=milliseconds(UPDATE_RATE);
    }
    */

   return 0;
}
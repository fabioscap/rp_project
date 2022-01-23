#include "world.h"
#include "ros/ros.h"
#include "static_vec.h"
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
    int8_t data[30*30];
    Map m = Map(0.2,30,30,data);

    for (int8_t i=0; i<30; ++i) {
        for (int8_t j=0; j<30; ++j) {
            if (i==0 || i == 30-1 || j == 0 || j == 30-1 || i == 25 || j == 25) 
                m.at(i,j) = WALL;
            else m.at(i,j) = 0;
        }
    }

    std::cout<< m << std::endl;

    Robot r = Robot(0.1);
    World w = World(m,r);


    w.robot.px() = 0.2*15;
    w.robot.py() = 0.2*15;
    w.robot.pa() = M_PI/4;
    w.robot.vl() = +0.1;
    //https://stackoverflow.com/questions/46609863/execute-function-every-10-ms-in-c
    // how to do a timed loop
    
    using namespace std::chrono;
    auto now = system_clock::now(); 
    auto next = now + milliseconds(UPDATE_RATE);
    using namespace sim2d;
    while(1) {
        std::cerr << w.robot.pxy() << std::endl;
        w.update();

        std::this_thread::sleep_until(next);
        next+=milliseconds(UPDATE_RATE);
    }
    

   
   
   return 0;
}
#pragma once

#include <iostream>

#define WALL 100



namespace sim2d {
// maybe template the dimensions
class Map {
    public:
    const float resolution; // m/pixel
    const uint32_t width;  // pixels
    const uint32_t height; // pixels
    const uint32_t dimension; // pixels
    int8_t* data; // occupancy grid

    inline int8_t& at(int i, int j) {return data[i*height+j];}
    inline const int8_t& at(int i, int j) const {return data[i*height+j];}

    Map(float res, uint32_t w, uint32_t h, int8_t* d): resolution(res), 
                                            width(w),
                                            height(h),
                                            data(d),
                                            dimension(w*h) {};
};


std::ostream&  operator << (std::ostream& os, const Map& m) {
    for (int i=0; i<m.width; ++i) { 
        for (int j=0; j<m.height; ++j) {
            if (m.at(i,j) == WALL) os << "* ";
            else os << "  ";
        }
        os << std::endl;
    }
    return os;
}


}
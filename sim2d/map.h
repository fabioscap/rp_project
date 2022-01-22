#pragma once

#include <iostream>
#define WALL 100
#define OTHER 50

typedef int32_t cell_index;

namespace sim2d {
// maybe template the dimensions

class Map {
    public:
    const double resolution; // m/pixel
    const cell_index width;  // pixels
    const cell_index height; // pixels
    const cell_index dimension; // pixels
    int8_t* data; // occupancy grid

    inline int8_t& at(cell_index i, cell_index j) {return data[i*height+j];}
    inline const int8_t& at(cell_index i, cell_index j) const {return data[i*height+j];}

    Map(double res, cell_index w, cell_index h, int8_t* d): resolution(res), 
                                            width(w),
                                            height(h),
                                            data(d),
                                            dimension(w*h) {};
};


std::ostream&  operator << (std::ostream& os, const Map& m) {
    for (int i=0; i<m.width; ++i) { 
        for (int j=0; j<m.height; ++j) {
            if (m.at(i,j) == WALL) os << "* ";
            else if (m.at(i,j) == OTHER) os << "o ";
            else os << "  ";
        }
        os << std::endl;
    }
    return os;
}

}
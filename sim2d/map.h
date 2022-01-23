#pragma once

#include <iostream>
#include "static_vec.h"
#include <assert.h>

#define WALL 100
#define OTHER 50

typedef int32_t cell_index;
typedef rp::Vec_<cell_index,2> grid2;

namespace sim2d {
// maybe template the dimensions

class Map {
    public:
    const double resolution; // m/pixel
    const cell_index width;  // pixels
    const cell_index height; // pixels
    const cell_index dimension; // pixels


    inline int8_t& at(cell_index i, cell_index j) {return _at(i,j);}
    inline const int8_t& at(cell_index i, cell_index j) const {return _at(i,j);}
    inline int8_t& at(const grid2& ij) {return _at(ij[0],ij[1]);}
    inline const int8_t& at(const grid2& ij) const {return _at(ij[0],ij[1]);}

    Map(double res, cell_index w, cell_index h, int8_t* d): resolution(res), 
                                            width(w),
                                            height(h),
                                            data(d),
                                            dimension(w*h) {};
    protected:
        int8_t* data; // occupancy grid

        inline int8_t& _at(cell_index i, cell_index j) {
            assert(i>=0&&j>=0&&i<width&&"please access map with valid index");
            return data[i*height+j];
        }
        inline const int8_t& _at(cell_index i, cell_index j) const {            
            assert(i>=0&&j>=0&&j<height&&"please access map with valid index");
            return data[i*height+j];
        }
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
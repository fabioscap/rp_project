#pragma once

#include <iostream>
#include "static_vec.h"
#include <assert.h>
#include <vector>

#define WALL 100
#define OTHER 50

typedef int32_t cell_index;
typedef int8_t cell_value;
typedef rp::Vec_<cell_index,2> grid2;

namespace sim2d {
// maybe template the dimensions

class Map {
    public:
    const double resolution; // m/pixel
    const cell_index width;  // pixels
    const cell_index height; // pixels
    const cell_index dimension; // pixels


    inline cell_value& at(cell_index i, cell_index j) {return _at(i,j);}
    inline const cell_value& at(cell_index i, cell_index j) const {return _at(i,j);}
    inline cell_value& at(const grid2& ij) {return _at(ij[0],ij[1]);}
    inline const cell_value& at(const grid2& ij) const {return _at(ij[0],ij[1]);}
    
    Map(const Map& other):  width(other.width),
                            height(other.height),
                            dimension(other.dimension),
                            data(other.data),
                            resolution(other.resolution) {};

    Map(double res, cell_index w, cell_index h): resolution(res), 
                                            width(w),
                                            height(h),
                                            dimension(w*h),
                                            data(dimension) {};

    Map(double res, cell_index w, cell_index h, std::vector<cell_value> d): resolution(res), 
                                            width(w),
                                            height(h),
                                            data(d),
                                            dimension(w*h) {};
    //protected:
        std::vector<cell_value> data; // occupancy grid

        inline cell_value& _at(cell_index i, cell_index j) {
            assert(i>=0&&j>=0&&i<width&&"please access map with valid index");
            return data[i*height+j];
        }
        inline const cell_value& _at(cell_index i, cell_index j) const {            
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
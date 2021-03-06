#pragma once

#include <iostream>
#include "static_vec.h"
#include <assert.h>
#include <vector>

#define SIM2d_WALL 100
#define SIM2d_EMPTY 0
#define SIM2d_OTHER 50
#define SIM2d_UNK -1

typedef int32_t cell_index;
typedef int8_t cell_value;
typedef rp::Vec_<cell_index,2> grid2;

namespace sim2d {

// TODO add map origin and rotation (as of now map origin is 0,0 and rotation is 0)
class Map {
    public:
    const double resolution; // m/pixel
    const cell_index width;  // pixels
    const cell_index height; // pixels
    const cell_index dimension; // pixels


    //inline cell_value& at(cell_index i, cell_index j) {return _at(i,j);}
    inline const cell_value& at(cell_index i, cell_index j) const {return _at(i,j);}
    //inline cell_value& at(const grid2& ij) {return _at(ij[0],ij[1]);}
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
    protected:
    std::vector<cell_value> data; // occupancy grid
    /*
    inline cell_value& _at(cell_index i, cell_index j) {
        assert(i>=0&&j>=0&&i<width&&j<height&&"please access map with valid index");
        return data[j*width+i];
    }
    */
    inline const cell_value& _at(cell_index i, cell_index j) const {            
        assert(i>=0&&j>=0&&j<height&&i<width&&"please access map with valid index");
        return data[j*width+i];
    }
};


std::ostream&  operator << (std::ostream& os, const Map& m);

}
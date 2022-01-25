#include "map.h"

using namespace sim2d;

std::ostream& operator << (std::ostream& os, const Map& m) {
    for (int i=0; i<m.width; ++i) { 
        for (int j=0; j<m.height; ++j) {
            if (m.at(i,j) == SIM2d_WALL) os << "* ";
            else if (m.at(i,j) == SIM2d_OTHER) os << "o ";
            else os << "  ";
        }
        os << std::endl;
    }
    return os;
}
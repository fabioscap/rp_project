#include "map.h"

std::ostream& sim2d::operator << (std::ostream& os, const sim2d::Map& m) {
    for (int i=0; i<m.width; ++i) { 
        for (int j=0; j<m.height; ++j) {
            if (m.at(i,j) == SIM2d_WALL ) os << "*";
            else if (m.at(i,j) == SIM2d_EMPTY ) os << " ";
            else if (m.at(i,j) == SIM2d_OTHER ) os << "-";
        }
        os << std::endl;
    }
    return os;
}

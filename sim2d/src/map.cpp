#include "map.h"

std::ostream& sim2d::operator << (std::ostream& os, const sim2d::Map& m) {
    for (int i=0; i<m.width; ++i) { 
        for (int j=0; j<m.height; ++j) {
            if (m.at(i,j) >0 ) os << "*";
            else if (m.at(i,j) == 0 ) os << " ";
            else if (m.at(i,j) == -1 ) os << " ";
        }
        os << std::endl;
    }
    return os;
}

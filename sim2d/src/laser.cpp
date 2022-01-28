#include "laser.h"

namespace sim2d {

void sim2d::Laser::init_samples() {
    //double increment = (to-from)/(n_samples-1); // [from->to]
    double increment = (to-from)/(n_samples); // [from->to)
    double value = 0;
    for (int i=0; i<n_samples; ++i) {
        sample_orientations[i] = from + value;
        value += increment;
    }
}
}
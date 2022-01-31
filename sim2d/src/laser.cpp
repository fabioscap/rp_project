#include "laser.h"

namespace sim2d {

void sim2d::Laser::init_samples() {
    double value = from;
    for (int i=0; i<n_samples; ++i) {
        sample_orientations[i] = value;
        value += increment;
    }
}
}

#include <vector>
#include <math.h>

namespace sim2d {
class Laser {
    public:
    const int n_samples;
    const double range; // m
    const double from; // rad
    const double to;  // rad 
    std::vector<double> sample_orientations;

    Laser(int n_samples, double range, double from, double to):
        sample_orientations(n_samples), // init a vector of n_samples
                          range(range),
                  n_samples(n_samples),
                            from(from),
                                to(to) {init_samples();}
    
    Laser():
            n_samples(100),
            sample_orientations(100),
            range(5),
            from(0),
            to(M_PI) {init_samples();}

    void init_samples();

};
}
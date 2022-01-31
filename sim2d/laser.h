
#include <vector>
#include <math.h>

namespace sim2d {
class Laser {
    public:
    const int n_samples;
    const double range; // m
    const double from; // rad
    const double to;  // rad 
    const double increment;
    std::vector<double> sample_orientations;

    Laser(int n_samples, double range, double from, double to):
        sample_orientations(n_samples), // init a vector of n_samples
                          range(range),
                  n_samples(n_samples),
                            from(from),
                                to(to),
                                //increment((to-from)/(n_samples-1)) // [from->to]
                                increment((to-from)/(n_samples)) // [from->to)
                                {init_samples();}
    Laser() :Laser(50,5,0,2*M_PI) {}


    void init_samples();

};
}
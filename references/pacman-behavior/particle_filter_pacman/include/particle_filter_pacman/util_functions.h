#ifndef UTIL_FUNCTIONS_PARTICLE_FILTER_H
#define UTIL_FUNCTIONS_PARTICLE_FILTER_H

namespace util
{
    float getProbOfMeasurementGivenPosition(int pos_x, int pos_y, int measurement_x, int measurement_y, double standard_deviation);
}

#endif // UTIL_FUNCTIONS_PARTICLE_FILTER_H
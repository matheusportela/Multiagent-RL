#include "particle_filter_pacman/util_functions.h"

#include <math.h>

float util::getProbOfMeasurementGivenPosition(int pos_x, int pos_y, int measurement_x, int measurement_y, double standard_deviation)
{
    // e^( (-1/2) * ( ( x - mean ) / std_deviation ) ^ 2 )
    // e^( - ( ( ( x - mean_x ) ^ 2 + ( y - mean_y ) ^ 2 ) / ( 2 * std_deviation ) ) ^ 2 )

    float diff_x = measurement_x - pos_x;
    float diff_y = measurement_y - pos_y;

    float exponencial_value = - (diff_x*diff_x + diff_y*diff_y) / (2*standard_deviation*standard_deviation);

    return exp(exponencial_value);
}
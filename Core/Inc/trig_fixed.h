#ifndef  TRIG_FIXED_H
#define TRIG_FIXED_H
#include <stdint.h>

// _12B indicates that the constant is mutiplied by 2^12 and rounded to the nearest integer
#define HALF_PI_12B             6434	
#define THREE_BY_TWO_PI_12B     19302
#define PI_12B                  12868
#define TWO_PI_12B              25736	

int32_t atan2_fixed(int32_t y, int32_t x);
int32_t sin_12b(int32_t theta);
int32_t cos_12b(int32_t theta);
int32_t wrap_2pi_12b(int32_t in);

#endif // ! TRIG_FIXED_H


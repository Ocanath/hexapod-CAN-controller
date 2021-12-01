#ifndef  TRIG_FIXED_H
#define TRIG_FIXED_H
#include <stdint.h>

// _12B indicates that the constant is mutiplied by 2^12 and rounded to the nearest integer
#define HALF_PI_12B             6434
#define THREE_BY_TWO_PI_12B     19302
#define PI_12B                  12868
#define TWO_PI_12B              25736

/*
 * A 32 bit signed integer ranges from -2^31 to 2^31-1.
 * That means the maximum value we can convert from fixed point-scaled
 * to the original value is 2^30, because 2^30 >> 30 is 1 but
 * 2^31-1 >> 31 = 0.
 *
 *
 *
 * */
#define ORDER_LOOKUP_TABLE 30	//this is the maximum order any 32bit lookup table can be, while using shifting to change the binary 'decimal' point

extern int32_t lookup_sin_30bit[HALF_PI_12B+1];	//external definition of the lookup table, generated from any software with double precision sin

int32_t atan2_fixed(int32_t y, int32_t x);
int32_t sin_12b(int32_t theta);
int32_t cos_12b(int32_t theta);
int32_t sin_lookup(int32_t theta, int result_order);
int32_t cos_lookup(int32_t theta, int result_order);
int32_t wrap_2pi_12b(int32_t in);
int64_t cos64b(int32_t theta);
int64_t sin62b(int32_t theta);

#endif // ! TRIG_FIXED_H


#include "trig_fixed.h"

/*
 * NOTE TO FUTURE JESSE:
 *
 * If you're thinking about switching all these functions to inline... STOP
 * You already benchmarked them both ways. Switching to inline REDUCES performance
 * when compiler optimizations are on. Basically, stop trying to outsmart the compiler.
 * */

/*Poly Coefficients used for sin calculation, scaled up by 2^12*/
static const int32_t sc1 = 117;
static const int32_t sc2 = -834;
static const int32_t sc3 = 85;
static const int32_t sc4 = 4078;
static const int32_t sc5 = 1;

/*Poly coefficients, scaled up by 2^12*/
static const int32_t tc1 = 580;    //2^12 scale
static const int32_t tc2 = -1406;    //2^12
static const int32_t tc3 = -66;    //2^12
static const int32_t tc4 = 4112;    //2^12
static const int32_t tc5 = -1*4096;    //2^12, with extra scaling factor applied for speed

/*
*   Input:
        theta. MUST BE EXTERNALLY CONSTRAINED TO BE BETWEEN -PI_12B and PI_2B
    OUTPUT:
        ~sin(theta). range -4096 to 4096
*/
int32_t sin_12b(int32_t theta)
{
    //Preprocess theta to force it in the range 0-pi/2 for poly calculation
    uint8_t is_neg = 0;
    if (theta > HALF_PI_12B && theta <= PI_12B) // if positiveand in quadrant II, put in quadrant I(same)
    {
        theta = PI_12B - theta;
    }
    else if (theta >= PI_12B && theta < THREE_BY_TWO_PI_12B)
    {
        is_neg = 1;
        theta = theta - PI_12B;
    }
    else if (theta > THREE_BY_TWO_PI_12B && theta < TWO_PI_12B)
    {
        theta = theta - TWO_PI_12B;
    }
    else if (theta < -HALF_PI_12B && theta >= -PI_12B) // if negativeand in quadrant III,
    {
        is_neg = 1;
        theta = theta + PI_12B;
    }
    else if (theta < 0 && theta >= -HALF_PI_12B) // necessary addition for 4th order asymmetry
    {
        is_neg = 1;
        theta = -theta;
    }

    // 7 fixed point multiplies. compute polynomial output 0-1 for input 0-pi/2
    int32_t theta2 = (theta * theta) >> 12;
    int32_t theta3 = (theta2 * theta) >> 12;
    int32_t theta4 = (theta3 * theta) >> 12;
    int32_t res = sc1 * theta4 + sc2 * theta3 + sc3 * theta2 + sc4 * theta + (sc5 << 12);
    res = res >> 12;

    int32_t y;
    if (is_neg == 1)
        y = -res;
    else
        y = res;

    return y;
}

int32_t cos_12b(int32_t theta)
{
    return sin_12b(theta + HALF_PI_12B);
}

/**/
int32_t sin_lookup(int32_t theta, int result_order)
{
	/*Guarantee theta is in lookup table range.*/
	theta = wrap_2pi_12b(theta);	//initial bounding performed with modulo

	//The wrap call above means we have to check fewer 'quadrants'. Only 3 quadrants required to check
    uint8_t is_neg = 0;
    if (theta > HALF_PI_12B && theta <= PI_12B) // if positiveand in quadrant II, put in quadrant I(same)
    {
        theta = PI_12B - theta;
    }
    else if (theta < -HALF_PI_12B && theta >= -PI_12B) // if negativeand in quadrant III,
    {
        is_neg = 1;
        theta = theta + PI_12B;
    }
    else if (theta < 0 && theta >= -HALF_PI_12B) // necessary addition for 4th order asymmetry
    {
        is_neg = 1;
        theta = -theta;
    }	//further bounding performed with on a per-quadrant basis, recording the sign before transforming it

    //WE MUST DO THIS HERE (because theta could be nonzero at the top)
    if(theta == 0)	//we lookup into our table with theta-1, so we must catch the 0 case. sin(0) = 0
		return 0;	//this is the value, not a code.


    int shift = ORDER_LOOKUP_TABLE - result_order;	//determine the shift required to produce the desired radix
    if(is_neg)
    	return (-lookup_sin_30bit[theta-1]) >> shift;
    else
    	return (lookup_sin_30bit[theta-1]) >> shift;
}

int32_t cos_lookup(int32_t theta, int result_order)
{
	return sin_lookup(theta+HALF_PI_12B,result_order);
}


int32_t atan2_fixed(int32_t y, int32_t x)
{
    //assert(isa(y, 'int32') & isa(x, 'int32'), 'Error: inputs must be of type int32');

    //capture edge cases, prevent div by 0
    if (x == 0)
    {
        if (y == 0)
            return 0;
        else if (y > 0)
            return HALF_PI_12B;// pi / 
        else
            return -HALF_PI_12B; // -pi / 2
    }
    if (y == 0)
    {
        if (x > 0)
            return 0;
        else
            return PI_12B;// pi
    }

    // get absolute value of both
    int32_t abs_s = y;
    if (abs_s < 0)
        abs_s = -abs_s;
    int32_t abs_c = x;
    if (abs_c < 0)
        abs_c = -abs_c;

    // get min value of both
    int32_t minv = abs_c;
    int32_t maxv = abs_s;
    if (maxv < minv)
    {
        minv = abs_s;
        maxv = abs_c;
    }

    //// do a fixed point division...
    // pre-apply a fixed point scaling factor of 2 ^ 12 with a shift
    int32_t a = (minv << 12) / maxv;// this is guaranteed to range from 0 - 4096, as minv-maxv is constrained from 0-1 by above logic

    //compute some exponents for the polyomial approximation
    int32_t a2 = (a * a) >> 12;// remove double application of scaling factor 2 ^ 12 with a rightshift
    int32_t a3 = (a2 * a) >> 12;
    int32_t a4 = (a3 * a) >> 12;

    // best, most multiplies, 4th order
    int32_t r = a4 * tc1 + a3 * tc2 + a2 * tc3 + a * tc4 + tc5; //compute polynomial result. Shift the last coefficient, to apply the factor of 2^12 present in all the exponents
    r = r >> 12;    //remove the factor of 2^15 added by the coefficints, leaving only the factor of 2^12 that was preserved in the exponents

    // fp_coef from fixed_point_foc
    if (abs_s > abs_c)
        r = HALF_PI_12B - r;
    if (x < 0)
        r = PI_12B - r;
    if (y < 0)
        r = -r;

    return r;   //output range -PI_12B to PI_12B
}

/*This function performs 'wrapping' operations on input angles. Can have arbitrary scale.
The conditionals are there to capture the fact that mod() in matlab has different behavior than % operator
on negative numbers. We are capturing the cases where (in + PI_12B) is less than 0
*/
int32_t wrap_2pi_12b(int32_t in)
{
    int32_t result = ((in + PI_12B) % TWO_PI_12B) - PI_12B;
    if (in < -PI_12B) //if( (in + PI_12B) < 0 )
        return TWO_PI_12B + result;
    else 
        return result;
}

/*64 bit analogue which we need for unwrap_64*/
int64_t wrap_2pi12b_64(int64_t in)
{
	int64_t result = ((in + PI_12B) % TWO_PI_12B) - PI_12B;
    if (in < -PI_12B) //if( (in + PI_12B) < 0 )
        return TWO_PI_12B + result;
    else
        return result;
}

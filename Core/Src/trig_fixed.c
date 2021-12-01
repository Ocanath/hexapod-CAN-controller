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
 * Taylor series coefficients for high precision low memory use sin-cos
 * These are 64bit to save compute time (no casting on each)*/
static const int64_t c_n3 = 11097578693;
static const int64_t c_n5 = 5476435575;
static const int64_t c_n7 = 1286910778;
static const int64_t c_n9 = 176406948;
static const int64_t c_n11 = 15827880;
static const int64_t one_by_pi_31b = 683565276;
static const int64_t two_to_the_50th = 1125899906842624;	//(1 << 50)

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

/*
 * High accuracy, fast as possible with 64 bit, nearly as accurate as lookup but
 * with substantially reduced MEMORY overhead.
 *
 * If you need a very accurate sine-cosine:
 * 		1. If you can spare 6435 int32's (25740 bytes of memory), use the lookup table. It is extremely fast and produces exact results
 * 		2. If you can't, use this. It (much) slower, but consumes far less memory and has similar accuracy and precision
 * */
int64_t sin62b(int32_t theta)	//returns a -2^30 to 2^30range sin of the input angle, which is -4096*pi t0 4096*pi
{
	theta = wrap_2pi_12b(theta);	//initial bounding performed with modulo

    uint8_t is_neg = 0;
    if(theta > HALF_PI_12B && theta <= PI_12B)	// if positive and in quadrant II, put in quadrant I (same)
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
    else if(theta < -HALF_PI_12B && theta >= -PI_12B ) // if negative and in quadrant III,
    {
		is_neg = 1;
		theta = theta+PI_12B;
	}
    int64_t theta64 = (int64_t)theta;
    int64_t theta_31b_norm = (theta64*one_by_pi_31b) >> 12;
    int64_t theta2 = (theta_31b_norm*theta_31b_norm) >> 31;
    int64_t theta3 = (theta2*theta_31b_norm) >> 31;
    int64_t theta5 = (theta2*theta3) >> 31;
    int64_t theta7 = (theta2*theta5) >> 31;
    int64_t theta9 = (theta2*theta7) >> 31;
    int64_t theta11 = (theta2*theta9) >> 31;

    //uint8_t lshift = 32; //30-(31+31)	//produce 30bit resolution output value
    int64_t res = theta64*two_to_the_50th - theta3*c_n3;	//could do a right shift, because theta64 is constrained to be only positive. this removes a warning though
    res += theta5*c_n5;
    res -= theta7*c_n7;
    res += theta9*c_n9;
    res -= theta11*c_n11;

    if(is_neg)
    	return -res;
    else
    	return res;
}

/*
 *	wrapper for sin taylor
 * */
int64_t cos64b(int32_t theta)
{
    return sin_12b(theta + HALF_PI_12B);
}

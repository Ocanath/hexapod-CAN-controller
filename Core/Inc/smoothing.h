/*
 * smoothing.h
 *
 *  Created on: Mar 20, 2021
 *      Author: Ocanath Robotman
 */

#ifndef INC_SMOOTHING_H_
#define INC_SMOOTHING_H_

#include "sin-math.h"
#include "init.h"
#include "CAN.h"

/* two-vector floating point structure*/
typedef struct vect2
{
	float v[2];
}vect2;

/*Wrapper for all variables used for motor smoothing*/
typedef struct smooth_mem_t
{
    float qd_in; //output
    float qd_out;
    float period; //input
    float qd_change_thresh;	//input
    float vjump_thresh;

	uint32_t start_ts;
	float qd_start;
	vect2 v_start;
	vect2 v_end;
	float qd_in_prev;
	float freq;
	float offset;
	float prev_fp; //previous period or frequency term.
}smooth_mem_t;

/*returns absolute value of floating point input*/
static inline float abs_f(float in)
{
	if(in < 0)
		return -in;
	else
		return in;
}

#endif /* INC_SMOOTHING_H_ */

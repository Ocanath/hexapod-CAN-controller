/*
 * smoothing.c
 *
 *  Created on: Mar 20, 2021
 *      Author: Ocanath Robotman
 */
#include "smoothing.h"

/*
 * INPUTS:
 *
 * This function computes a smooth path between a start angle and end angle by
 * transforming it into vector space. The
 *
 * USER INPUTS:
 * qd_end: 		new setpoint, to approach in a smooth fashion
 * period: 		the time in seconds it takes for the finger to track its new position
 *
 * INPUT PARAMETERS
 * q:	 		finger position
 *
 *PASS BY POINTER/REFERENCE/HELPER VARIABLES
 * start_ts, qd_start, qd_end_prev, freq
 *
 *OUTPUTS
 *qd, true setpoint for position control, smoothed in absolute angle space (rotations are taken out, and the
 * output can wrap)
 *
 */
void smooth_qd_vect(smooth_mem_t * sm, joint * j)
{
	float abs_diff = abs_f(sm->qd_in - (sm->qd_in_prev));

	float qr = j->q;
	sm->v_start.v[0] = cos_fast(qr);
	sm->v_start.v[1] = sin_fast(qr);

	float qd_end_r = sm->qd_in;
	sm->v_end.v[0] = cos_fast(qd_end_r);
	sm->v_end.v[1] = sin_fast(qd_end_r);

	vect2 dif =
	{
		{
				sm->v_end.v[0] - sm->v_start.v[0],
				sm->v_end.v[1] - sm->v_start.v[1]
		}
	};

	float vjump = abs_f(dif.v[0]) + abs_f(dif.v[1]);

	if(abs_diff > sm->qd_change_thresh || (sm->prev_fp > 500.f && sm->period < 500.f) || vjump > sm->vjump_thresh)	//if the jump in setpoint value is large, or the previous period is > 500 and the current period is < 500
	{
		sm->start_ts = HAL_GetTick();	//reset the tracking time
		sm->offset = 0.0f;	//offset is a time offset, used to calculate a dynamically changing period
		sm->freq = PI/sm->period;	//precalculate so we don't have to constantly divide
	}

	if(abs_f(sm->period - sm->prev_fp) > 0.001f)	//if the previous period is not equal to the current period
	{
		sm->offset = ((float)(HAL_GetTick() - sm->start_ts)*.001f)*(PI/(sm->prev_fp)) + sm->offset;
		sm->start_ts = HAL_GetTick();
		sm->freq = PI/sm->period;	//if we get a new period, calculate the frequency for that period
	}

	float ft_st = ((float)(HAL_GetTick() - sm->start_ts)*.001f)*sm->freq + sm->offset;
	float t = ft_st*sm->period*ONE_BY_PI;

	if(t <= sm->period)
	{
		float sw = (.5f*(sin_fast(ft_st-HALF_PI)+1.0f));	//sweep scalar
		float x = (dif.v[0])*sw + sm->v_start.v[0];
		float y = (dif.v[1])*sw + sm->v_start.v[1];
		sm->qd_out = atan2_approx(y,x);
	}
	else
		sm->qd_out = sm->qd_in;

	sm->qd_in_prev = sm->qd_in;		//block entry into this routine again/update the previous value
	sm->prev_fp = sm->period;
}

#include "trig_fixed.h"
#include "sin-math.h"
#include "hexapod_footpath.h"

/*Computes the piecewise parametric function to create a footpath.
*
* The resultant vector must be rotated and translated before it is a legitimate result for walking motion generation

INPUTS:
	time: current time, in same time units as period
	period: total time the motion is scheduled to take
	h: the height of the path (y)
	w: the width of the path. will go x = +w/2 to x=-w/2, starting at (0,0)
	p1: the percentage of the total period after which the parabolic arc will begin.
		It will return to the second half of the linear motion at p2, which is equal
		to 1-p1
OUTPUTS:
	v: output vector. Z always = 0. Pass by reference.
*/
void foot_path(float time, float h, float w, float period, vect3_t* v)
{
	float t = fmod_2pi(time*TWO_PI/period)/TWO_PI;	//easy way of getting time normalized to 0-1 using prior work, without needing standard lib fmod

	float p1 = 0.25f;	//half down, half up.
	float p2 = 1.f - p1;

	if (t >= 0 && t < p1)
	{
		t = (t - 0) / (p1 - 0);	//parametric function expects 0-1.

		v->v[0] = t * (w / 2);
		v->v[1] = 0;
		v->v[2] = 0;
	}
	else if(t >= p1 && t < p2)
	{
		t = 2.f * (t - p1) / (p2 - p1) - 1.f; // this function expects t = -1 to 1

		/*
			-1 to 1, except that the first couple of derivatives are lower at the ends.
			Consequence for smoother motion is that the footspeed is much higher in the middle of the motion
		*/
		for(int i = 0; i < 1; i++)	//the more iterations of this you run, the more like a step function this becomes.
			t = sin_fast(HALF_PI * t);

		v->v[0] = -t * w / 2.f;
		//v->v[0] = -sin_fast(t)*sin_fast(t + HALF_PI) * 2.199f * (w / 2.f);
		v->v[1] = (-t*t + 1) * h;
		v->v[2] = 0;
	}
	else if(t >= p2 && t < 1)
	{
		t = (t - p2) / (1 - p2);

		v->v[0] = t * (w / 2) - (w / 2);
		v->v[1] = 0;
		v->v[2] = 0;
	}
	else
	{
		for (int i = 0; i < 3; i++)
			v->v[i] = 0;
	}
}


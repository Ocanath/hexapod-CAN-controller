#ifndef DYNAHEX_H
#define DYNAHEX_H
#include "kinematics.h"

/*
* Definition of the hexleg in C style
*/

#define NUM_FRAMES_HEXLEG 4	//0,1,2,3
#define NUM_JOINTS_HEXLEG 3
#define NUM_LEGS	6
typedef struct dynahexleg_t
{
	joint chain[NUM_FRAMES_HEXLEG];
	vect3_t ef_0;
	vect3_t ef_b;
	vect3_t ef_knee;	//some example anchor points associated with each leg
}dynahexleg_t;

typedef struct dynahex_t
{
	//mat4_t hw_b;
	dynahexleg_t leg[NUM_LEGS];
}dynahex_t;

void init_dynahex_kinematics(dynahex_t* h);
void forward_kinematics_dynahexleg(dynahex_t* h);
void ik_closedform_hexapod(mat4_t * hb_0, joint * start, vect3_t * targ_b);

#endif // !DYNAHEX_H

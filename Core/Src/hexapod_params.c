/*
 * hexapod_params.c
 *
 *  Created on: Nov 21, 2021
 *      Author: Ocanath Robotman
 */


#include "hexapod_params.h"


dynamic_hex_t gl_hex = {0};	//init to null

const vect3 ef_foot_f3 = { -15.31409f, -9.55025f, 0.f };	//location of the foot in frame 3


static const dh_entry hexleg_dh[] = {
	{65.66f,	-53.2f,				PI/2},
	{29.00f,	-100.46602344f,		PI},
	{21.50f,	-198.31677025f,		0.f}
};


/*Un-rotated location of the leg base frame
 * wrt. the centered 0 frame*/
static const mat4_t hb_0_leg0 =
{
	{
		{1.f,	0,		0,		109.7858f},
		{0,		1.f,	0,		0},
		{0,		0,		1.f,	0},
		{0,		0,		0,		1.f}
	}
};

void setup_dynamic_hex(dynamic_hex_t * robot)
{


	/*Set up the chain singly linked list. We will assume the chain
	 * is formatted in ascending order, with each triple corresponding
	 * to a leg. If not, this for loop is invalid*/
	int leg = 0;
	for(int i = 0; i < NUM_JOINTS-1; i++)
	{
		if(i % 3 == 0)
			robot->p_joint[leg++] = &chain[i];
		chain[i].child = &chain[i+1];
	}

	/*Initialize the hb_0 frame definitions*/
	const float angle_f = (2 * PI) / 6.f;
	for (int leg = 0; leg < NUM_LEGS; leg++)
	{
		mat4_t tmp = mat4_t_mult(hb_0_leg0, Hz(PI));
		tmp = mat4_t_mult(tmp, Hx(PI));
		tmp = mat4_t_mult( Hz(((float)leg)*angle_f), tmp );
		copy_mat4_t(&(robot->hb_0[leg]),&tmp);
	}

	/**/
	for(int leg = 0; leg < NUM_LEGS; leg++)
	{
		joint * j = robot->p_joint[leg];
		while(j != NULL)
		{
			if(j->frame == 1)
			{
				dh_to_mat4(&j->h_link, (dh_entry*)&hexleg_dh[0]);
			}
			else if (j->frame == 2)
			{
				dh_to_mat4(&j->h_link, (dh_entry*)&hexleg_dh[1]);
			}
			else if (j->frame == 3)
			{
				dh_to_mat4(&j->h_link, (dh_entry*)&hexleg_dh[2]);
			}
			j = j->child;
		}
	}
}

/*
 * hexapod_params.c
 *
 *  Created on: Nov 21, 2021
 *      Author: Ocanath Robotman
 */


#include "hexapod_params.h"
#include "vect.h"
#include "trig_fixed.h"


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
		{-1.f,	0,		0,		109.7858f},
		{0,		1.f,	0,		0},
		{0,		0,		-1.f,	0},
		{0,		0,		0,		1.f}
	}
};


/*Externally generated frame list*/
static const mat4_32b_t h32_b_0_leg0 = {
		{
			{-ONE_ROT, 0, 0, 7194922},
			{0, ONE_ROT, 0, 0},
			{0, 0, -ONE_ROT, 0},
			{0, 0, 0, ONE_ROT}
		}
};
static const mat4_32b_t h32_link1 = {
    {
        {ONE_ROT, 0, 0, -3486515},
        {0, 0, -ONE_ROT, 0},
        {0, ONE_ROT, 0, 4303094},
        {0, 0, 0, ONE_ROT}
    }
};
static const mat4_32b_t h32_link2 = {
    {
        {ONE_ROT, 0, 0, -6584141},
        {0, -ONE_ROT, 0, 0},
        {0, 0, -ONE_ROT, 1900544},
        {0, 0, 0, ONE_ROT}
    }
};
static const mat4_32b_t h32_link3 = {
    {
        {ONE_ROT, 0, 0, -12996888},
        {0, ONE_ROT, 0, 0},
        {0, 0, ONE_ROT, 1409024},
        {0, 0, 0, ONE_ROT}
    }
};

void setup_dynamic_hex(dynamic_hex_t * robot)
{
	int n = KINEMATICS_SIN_ORDER;

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
	//next, scan through the list and set every third joint child to ZERO
	//so that our chains are null terminated!
	for(int i = 0; i < NUM_JOINTS; i++)
	{
		if(i % 3 == 2)	//
		{
			chain[i].child = NULL;
		}
	}

	/*Initialize the hb_0 frame definitions*/
	const float angle_f = (2 * PI) / 6.f;
	const int32_t angle_12 = TWO_PI_12B/6;

	for (int leg = 0; leg < NUM_LEGS; leg++)
	{
		mat4_t* rh_b_0 = &(robot->hb_0[leg]);
		mat4_32b_t* rh32_b_0 = &(robot->h32_b_0[leg]);

		mat4_t zrot = Hz(leg*angle_f);
		mat4_32b_t z32_rot = Hz_nb(leg*angle_12, n);

		mat4_t_mult_pbr(&zrot, (mat4_t*)(&hb_0_leg0),rh_b_0);
		ht32_mult64_pbr(&z32_rot, (mat4_32b_t*)(&h32_b_0_leg0), rh32_b_0, n);
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
				m_mcpy(&j->h32_link,(void*)(&h32_link1),sizeof(mat4_32b_t));
			}
			else if (j->frame == 2)
			{
				dh_to_mat4(&j->h_link, (dh_entry*)&hexleg_dh[1]);
				m_mcpy(&j->h32_link,(void*)(&h32_link2),sizeof(mat4_32b_t));
			}
			else if (j->frame == 3)
			{
				dh_to_mat4(&j->h_link, (dh_entry*)&hexleg_dh[2]);
				m_mcpy(&j->h32_link,(void*)(&h32_link3),sizeof(mat4_32b_t));
			}
			j = j->child;
		}
	}
}

#include "dynahex.h"



const dh_entry hexleg_dh[NUM_FRAMES_HEXLEG] = {
	{0,			0,					0},		//d,a,alpha
	{65.66f,	-53.2f,				PI/2},
	{29.00f,	-100.46602344f,		PI},
	{21.50f,	-198.31677025f,		0.f}
};


void init_dynahex_kinematics(dynahex_t * h)
{
	mat4_t hb_0_leg0 =
	{
		{
			{-1.f,	0,		0,		109.7858f},
			{0,		1.f,	0,		0},
			{0,		0,		-1.f,	0},
			{0,		0,		0,		1.f}
		}
	};
	const float angle_f = (2 * PI) / 6.f;
	for (int leg = 0; leg < NUM_LEGS; leg++)
	{
		joint * j = h->leg[leg].chain;
		j[0].him1_i = mat4_t_mult(Hz((float)leg * angle_f), hb_0_leg0);
		copy_mat4_t(&j[0].hb_i, &j[0].him1_i);

		init_forward_kinematics_dh(j, hexleg_dh, NUM_JOINTS_HEXLEG);
	}
}

//static vect3_t o_foottip_3 = {{ -15.31409f, -9.55025f, 0.f }};

void forward_kinematics_dynahexleg(dynahex_t* h)
{
	for (int leg = 0; leg < NUM_LEGS; leg++)
	{
		joint* j = &(h->leg[leg].chain[0]);
		load_q(j->child);
		forward_kinematics(&j->hb_i, j->child);

		//htmatrix_vect3_mult(&j[3].hb_i, &o_foottip_3, &h->leg[leg].ef_b);

		//calc_J_point(j, NUM_JOINTS_HEXLEG, h->leg[leg].ef_b);
	}
}


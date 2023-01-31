#include "dynahex.h"
#include "vect.h"



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





/*Get the distance between two 3 vectors*/
float dist_vect3(vect3_t * p0, vect3_t * p1)
{
	double sum_sq = 0.f;
	for (int i = 0; i < 3; i++)
	{
		float tmp = (p0->v[i] - p1->v[i]);
		tmp = tmp * tmp;
		sum_sq += (double)tmp;
	}
	//return (float)(sqrt(sum_sq));
	return asm_sqrt(sum_sq);
}

/*
    Get the intersection of two circles in the x-y plane. Two solutions for valid inputs.
    Invalid inputs are those which result in the following cases:
        1. No intersection
        2. infinite intersections
        3. one solution
    It is simple to check for and reject these cases, but to save a little time and complexity we will
    assume correct inputs (all inputs for the 4bar linkage case are correct inputs)
 */
uint8_t get_intersection_circles(vect3_t * o0, float r0, vect3_t * o1, float r1, vect3_t solutions[2])
{
	float d = dist_vect3(o0, o1);
	//can do validity checks here (intersecting, contained in each other, equivalent) but they're unnecessary for our application

	//get some reused squares out of the way
	float r0_sq = r0 * r0;
	float r1_sq = r1 * r1;
	float d_sq = d * d;

	// solve for a
	float a = (r0_sq - r1_sq + d_sq) / (2 * d);

	// solve for h
	float h_sq = r0_sq - a * a;
	float h = asm_sqrt(h_sq);

	float one_by_d = 1.f / d;
	// find p2
	vect3_t p2;
	p2.v[2] = 0.f;
	for (int i = 0; i < 2; i++)
		p2.v[i] = o0->v[i] + a * (o1->v[i] - o0->v[i]) * one_by_d;

	float t1 = h * (o1->v[1] - o0->v[1]) * one_by_d;
	float t2 = h * (o1->v[0] - o0->v[0]) * one_by_d;

	solutions[0].v[0] = p2.v[0] + t1;
	solutions[0].v[1] = p2.v[1] - t2;
	solutions[0].v[2] = 0.f;

	solutions[1].v[0] = p2.v[0] - t1;
	solutions[1].v[1] = p2.v[1] + t2;
	solutions[1].v[2] = 0.f;
	return 1;
}

/*Note: this is further complicated by the fact that the q1 method of taking atan2
 * is only valid for anchor points which are
 *
 */
void ik_closedform_hexapod(mat4_t * hb_0, joint * start, vect3_t * targ_b)
{
	mat4_t h0_b;
	ht_inverse_ptr(hb_0, &h0_b);

	vect3_t targ_0;
	htmatrix_vect3_mult(&h0_b, targ_b, &targ_0);

	/*A bit of geometric fuckery to account for the fact that the actual origin of the foot has a y offset from
	 * the angle formed at q={0,0,0}. A hack, basically. I think it works for
	 * any y offset you would encounter though, making it kinda useful.
	 *
	 * The q=0,0,0 y offset is currently -7.5mm
	 *
	 */
	vect3_t targ_0_offset;
	vect3_t z = { {0, 0, 1} };
	cross_pbr(&targ_0, &z, &targ_0_offset);	vect_normalize(targ_0_offset.v, 3);
	for (int i = 0; i < 3; i++)
		targ_0_offset.v[i] *= -7.5f;	//the y offset when the arm is in q={0,0,0}

	//final q1 result, load it into kinematic structure
	start->q = (atan2_approx(targ_0.v[1] - targ_0_offset.v[1], targ_0.v[0] - targ_0_offset.v[0]) - PI);
	start->sin_q = sin_fast(start->q);
	start->cos_q = cos_fast(start->q);

	//do FK so we can express the target in frame 1 and do a 2 link planar arm solution
	forward_kinematics(hb_0, start);





	mat4_t h1_0;
	ht_inverse_ptr(&start->him1_i, &h1_0);

	vect3_t targ_1;
	htmatrix_vect3_mult(&h1_0, &targ_0, &targ_1);
	targ_1.v[2] = 0.f;	//set z value to 0, since it doesn't matter how we slide around z in our plane

	vect3_t zero = { {0,0,0} };
	vect3_t sols[2] = { 0 };
	get_intersection_circles(&zero, -hexleg_dh[2].a, &targ_1, -hexleg_dh[3].a, sols);

	int solidx = 1;
	joint* j = start->child;
	float theta1 = atan2_approx(sols[solidx].v[1], sols[solidx].v[0]);
	j->q = theta1 - PI;
	j->sin_q = sin_fast(j->q);
	j->cos_q = cos_fast(j->q);



	j = j->child;
	vect3_t vdif;
	for (int i = 0; i < 3; i++)
		vdif.v[i] = targ_1.v[i] - sols[solidx].v[i];
	float theta2 = atan2_approx(vdif.v[1], vdif.v[0]);
	j->q = PI-((PI-theta1)+theta2);
	j->sin_q = sin_fast(j->q);
	j->cos_q = cos_fast(j->q);

}

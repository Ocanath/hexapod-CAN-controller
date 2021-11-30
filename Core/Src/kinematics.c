/*
 * kinematics.c
 *
 *  Created on: Nov 21, 2021
 *      Author: Ocanath Robotman
 */
#include "kinematics.h"
#include "sin-math.h"

/*
	Copies the contents of one mat4_t to the other. could use memcpy interchangably
*/
void copy_mat4_t(mat4_t * dest, mat4_t * src)
{
	for (int r = 0; r < 4; r++)
	{
		for (int c = 0; c < 4; c++)
		{
			dest->m[r][c] = src->m[r][c];
		}
	}
}

void rpy_to_mat4(mat4_t * m, vect3 rpy, vect3 xyz)
{
	mat4_t tmp = Hz(rpy[2]);
	tmp = mat4_t_mult(tmp, Hy(rpy[1]));
	tmp = mat4_t_mult(tmp, Hz(rpy[0]));
	for(int r = 0; r < 3; r++)
		tmp.m[r][3] = xyz[r];
	copy_mat4_t(m, &tmp);
}

void dh_to_mat4(mat4_t * m, dh_entry * dh)
{
	float sin_alpha = sin_fast(dh->alpha);
	float cos_alpha = cos_fast(dh->alpha);
	mat4_t tmp = (mat4_t){
		{
			{1,		0,				0,				dh->a},
			{0,		cos_alpha,		-sin_alpha,		0},
			{0,		sin_alpha,		cos_alpha,		dh->d},
			{0,		0,				0,				1}
		}
	};
	copy_mat4_t(m, &tmp);
}

/*
*	Initialize links with urdf-style inputs (xyz and roll pitch yaw)
*/
void init_forward_kinematics_urdf(joint* j, vect3_t * xyz, vect3_t * rpy, int num_joints)
{
	for(int i = 1; i <= num_joints; i++)
	{
		mat4_t Hlink = Hz(rpy[i].v[2]);
		Hlink = mat4_t_mult(Hlink, Hy(rpy[i].v[1]));
		Hlink = mat4_t_mult(Hlink, Hx(rpy[i].v[0]));	//obtained Hrpy
		for(int r = 0; r < 3; r++)
			Hlink.m[r][3] = xyz[i].v[r];
		copy_mat4_t(&j[i].h_link, &Hlink);	//load result into joint matrix
		copy_mat4_t(&j[i].him1_i, &j[i].h_link);	//initial q=0 loading for him1_i
		j[i - 1].child = &j[i];	//load child reference in case we want to traverse this as a singly linked list
	}
	copy_mat4_t(&j[0].h_link, &j[0].hb_i);
	j[0].q = 0;
	j[num_joints].child = NULL;
	for (int i = 1; i <= num_joints; i++)
		mat4_t_mult_pbr(&j[i - 1].hb_i, &j[i].him1_i, &j[i].hb_i);
}

/*
load the kinematic chain Si, h0_i, him1_i variables based on the dh_table.
NOTE: only done ONCE, as the forward kinematics function only loads the elements which change with q

*/
/*
	load the kinematic chain Si, variables based on the dh_table.
	TODO: make this a linked list
*/
void init_forward_kinematics_dh(joint* j, const dh_entry* dh, int num_joints)
{
	for (int i = 1; i <= num_joints; i++)
	{
		float sin_alpha = sin_fast(dh[i].alpha);
		float cos_alpha = cos_fast(dh[i].alpha);
		/*Precomputed Hd*Ha*Halpha*/
		j[i].h_link = (mat4_t){
			{
				{1,		0,				0,				dh[i].a},
				{0,		cos_alpha,		-sin_alpha,		0},
				{0,		sin_alpha,		cos_alpha,		dh[i].d},
				{0,		0,				0,				1}
			}
		};
		copy_mat4_t(&j[i].him1_i, &j[i].h_link);	//htheta of 0 is the identity
		j[i - 1].child = &j[i];
	}
	copy_mat4_t(&j[0].h_link, &j[0].hb_i);
	j[0].q = 0;
	j[num_joints].child = NULL;	//initialize the last node in the linked list to NULL
	for (int i = 1; i <= num_joints; i++)
		mat4_t_mult_pbr(&j[i - 1].hb_i, &j[i].him1_i, &j[i].hb_i);
}
/*
Do forward kinematics on a singly linked list of joints!
Inputs:
		j: the root of the chain
Outputs:
		updates him1_i and hb_i of every element in the list

obviously singly linked list has same limitation as array. On
an embedded system, dynamic memory and recursion are not allowed
(both of which would be quite nice here; i.e. we can implement this as a dfs
type recursive run through the tree) so we'll have to operate on the tree
in a rigid structure (i.e. hard-coding what would normally be recursive)

//this could be optimized even further by using fixed point
//floating point version could be optimized by using multiply-accumulate
//fpu instructions directly

*/
void forward_kinematics(mat4_t * hb_0, joint* f1_joint)
{
	if(f1_joint == NULL)		//catch null pointer case for now, just to be sure
		return;

	joint* j = f1_joint;
	while(j != NULL)
	{
		float cth = j->cos_q_float;
		float sth = j->sin_q_float;

		mat4_t* r = &j->h_link;
		mat4_t * him1_i = &j->him1_i;	//specify lookup ptr first for faster loading

		him1_i->m[0][0] = cth * r->m[0][0] - r->m[1][0] * sth;
		him1_i->m[0][1] = cth * r->m[0][1] - r->m[1][1] * sth;
		him1_i->m[0][2] = cth * r->m[0][2] - r->m[1][2] * sth;
		him1_i->m[0][3] = cth * r->m[0][3] - r->m[1][3] * sth;
		him1_i->m[1][0] = cth * r->m[1][0] + r->m[0][0] * sth;
		him1_i->m[1][1] = cth * r->m[1][1] + r->m[0][1] * sth;
		him1_i->m[1][2] = cth * r->m[1][2] + r->m[0][2] * sth;
		him1_i->m[1][3] = cth * r->m[1][3] + r->m[0][3] * sth;
		him1_i->m[2][0] = r->m[2][0];
		him1_i->m[2][1] = r->m[2][1];
		him1_i->m[2][2] = r->m[2][2];
		him1_i->m[2][3] = r->m[2][3];

		j = j->child;
	}

	joint * parent = f1_joint;
	j = f1_joint;
	ht_mat4_mult_pbr(hb_0, &j->him1_i, &j->hb_i);	//load hb_1.		hb_0 * h0_1 = hb_1
	while (j->child != NULL)
	{
		j = j->child;
		ht_mat4_mult_pbr(&parent->hb_i, &j->him1_i, &j->hb_i);
		parent = j;
	}
}


/*
 * N is the binary radix of the sin and cosine of the joint angle.
 * max value is 29 (should be 30, but stability not guaranteed)
 */
void forward_kinematics_64(mat4_32b_t * hb_0, joint*f1_joint, int n)
{
	if(f1_joint == NULL)		//catch null pointer case for now, just to be sure
		return;

	joint* j = f1_joint;
	while(j != NULL)
	{
		int64_t cth = (int64_t)j->cos_q;
		int64_t sth = (int64_t)j->sin_q;

		mat4_32b_t* r = &j->h32_link;
		mat4_32b_t * him1_i = &j->h32_im1_i;	//specify lookup ptr first for faster loading

		int64_t r00 = (int64_t)r->m[0][0];
		int64_t r01 = (int64_t)r->m[0][1];
		int64_t r02 = (int64_t)r->m[0][2];
		int64_t r03 = (int64_t)r->m[0][3];
		int64_t r10 = (int64_t)r->m[1][0];
		int64_t r11 = (int64_t)r->m[1][1];
		int64_t r12 = (int64_t)r->m[1][2];
		int64_t r13 = (int64_t)r->m[1][3];

		him1_i->m[0][0] = (int32_t)((cth * r00 - r10 * sth) >> n);
		him1_i->m[0][1] = (int32_t)((cth * r01 - r11 * sth) >> n);
		him1_i->m[0][2] = (int32_t)((cth * r02 - r12 * sth) >> n);
		him1_i->m[0][3] = (int32_t)((cth * r03 - r13 * sth) >> n);
		him1_i->m[1][0] = (int32_t)((cth * r10 + r00 * sth) >> n);
		him1_i->m[1][1] = (int32_t)((cth * r11 + r01 * sth) >> n);
		him1_i->m[1][2] = (int32_t)((cth * r12 + r02 * sth) >> n);
		him1_i->m[1][3] = (int32_t)((cth * r13 + r03 * sth) >> n);
		him1_i->m[2][0] = r->m[2][0];
		him1_i->m[2][1] = r->m[2][1];
		him1_i->m[2][2] = r->m[2][2];
		him1_i->m[2][3] = r->m[2][3];

		j = j->child;
	}

	joint * parent = f1_joint;
	j = f1_joint;
	ht32_mult64_pbr(hb_0, &j->h32_im1_i, &j->h32_b_i, n);	//load hb_1.		hb_0 * h0_1 = hb_1
	while (j->child != NULL)
	{
		j = j->child;
		ht32_mult64_pbr(&parent->h32_b_i, &j->h32_im1_i, &j->h32_b_i, n);
		parent = j;
	}
}
/*
	Calculates the robot jacobian matrix satisfying the relationship v = J*qdot, where qdot is a vector of generalized joint velocities,
	and v is the linear velocity of the argument 'point' (expressed in chain frame 0, and rigidly attached to the end effector frame).

INPUTS:
	j : pointer to a list of joints, sized 'num_joints'. This function loads the Si vector in this list, which is eqal to the column vector of the jacobian matrix for that joint
	num_joints : number of joints in the joint list
	point: point, rigidly attached to the end effector (highest numerical index of the joint matrix) frame, expressed in frame 0

OUTPUTS:
	Si vectors in the joint list j.
NOTE:
	This jacobian matrix is following the jacobian matrix convention defined in Featherstone's texts, which
	loads the 'linear' (cross product) component in the latter 3 elements of the 6 vector. Therefore the resulting
	velocity vector of the jacobian multiply will be:
	{
		w0,
		w1,
		w2,
		v0,
		v1,
		v2
	};
	where v is the linear and w is the rotational component.
*******************************************************************************************************************
	Note also that the torque component for a given end effector force, for a given joint is given as follows:

		1. tau_i = dot(Si  f)

	Where f is the (6 vector) force and torque expressed in the 0 frame, Si is the column vector of
	the jacobian, and tau_i is the torque of that joint. This is equivalent to:

		2. [tau] = J^T * f

	Where [tau] is a vector of torques (size n/number of joints), J^T is the transpose of the jacobian matrix defined above, and
	f is the same generalized force/torque 6 vector expressed in the 0 frame.
*/
void calc_J_point(joint * j, int num_joints, vect3_t point)
{
	int i, v_idx;
	vect3_t z;
	vect3_t d;
	for (i = 1; i <= num_joints; i++)
	{
		for (v_idx = 0; v_idx < 3; v_idx++)
			z.v[v_idx] = j[i - 1].hb_i.m[v_idx][2];					//extract the unit vector corresponding to the axis of rotation of frame i-1 (i.e., the axis of rotation of q)
		for (v_idx = 0; v_idx < 3; v_idx++)
			d.v[v_idx] = point.v[v_idx] - j[i - 1].hb_i.m[v_idx][3];	//extract the difference between the target point in the base frame and the origin of frame i-1
		vect3_t res;
		cross_pbr(&z, &d, &res);

		j[i].Si.v[0] = z.v[0];		//Si = [w, v]^T;
		j[i].Si.v[1] = z.v[1];		//z_im1*q = w
		j[i].Si.v[2] = z.v[2];

		j[i].Si.v[3] = res.v[0];	//(z_im1 x (p - o_im1))*q = v
		j[i].Si.v[4] = res.v[1];
		j[i].Si.v[5] = res.v[2];
	}
}


vect6_t calc_w_v(joint * chain, vect3_t * w, vect3_t * v)
{
	vect6_t ret;
	int i;
	for (i = 0; i < 6; i++)
		ret.v[i] = 0;			//initialize the sum

	joint * j = chain;
	while(j->child != NULL)	//traverse singly linked list. discard the starting element
	{
		j = j->child;
		for(int r = 0; r < 6; r++)
			ret.v[r] += j->Si.v[r] * j->dq_rotor;
	}

	w->v[0] = ret.v[0];
	w->v[1] = ret.v[1];
	w->v[2] = ret.v[2];
	v->v[0] = ret.v[3];
	v->v[1] = ret.v[4];
	v->v[2] = ret.v[5];
	return ret;
}

/**/
void calc_tau(joint * j, int num_joints, vect6_t f, float * tau)
{

	int i;
	for (i = 1; i <= num_joints; i++)
	{
		tau[i] = 0.f;
		for(int r = 0; r < 6; r++)
			tau[i] += j[i].Si.v[r] * f.v[r];

	}
}
/*Faster, only valid for rotational. eliminate the translational
 * component of the Si vector, which featherstone puts in the bottom
 * */
void calc_tau3(joint * j, int num_joints, vect3_t * f, float* tau)
{
	for (int i = 1; i <= num_joints; i++)
	{
		tau[i] = 0.f;
		for (int r = 0; r < 3; r++)
			tau[i] += j[i].Si.v[r + 3] * f->v[r];
	}
}

/*pass by reference htmatrix (special subset of mat4_t) and 3 vector)*/
void htmatrix_vect3_mult(mat4_t* m, vect3_t* v, vect3_t* ret)
{
	int rsize = 3; int vsize = 4;
	int r, i;
	for (r = 0; r < rsize; r++)
	{
		float tmp = 0;
		for (i = 0; i < vsize; i++)
		{
			if (i < 3)
				tmp = tmp + m->m[r][i] * v->v[i];
			if (i == 3)
				tmp = tmp + m->m[r][i];
		}
		ret->v[r] = tmp;
	}
}

/*
helper function for extracting the origin of a ht matrix
*/
vect3_t h_origin(mat4_t h)
{
	vect3_t ret;
	int i;
	for (i = 0; i < 3; i++)
		ret.v[i] = h.m[i][3];
	return ret;
}
/*
helper function for extracting the origin of a ht matrix
in the form of a 4 vector
*/
vect4_t h_origin_vect4(mat4_t h)
{
	vect4_t ret;
	int i;
	for (i = 0; i < 3; i++)
		ret.v[i] = h.m[i][3];
	ret.v[3] = 1;
	return ret;
}

/*
* TODO TEST THIS FUNCTION
* Transforms a NORMALIZED quaternion to a rotation matrix
*/
mat4_t quat_to_mat4_t(vect4_t quat, vect3_t origin)
{
	mat4_t m;
	float q1 = quat.v[0];
	float q2 = quat.v[1];
	float q3 = quat.v[2];
	float q4 = quat.v[3];

	float qq1 = q1*q1;
	float qq2 = q2*q2;
	float qq3 = q3*q3;
	float qq4 = q4*q4;

	float q2q3 = q2*q3;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q1q4 = q1*q4;
	float q2q4 = q2*q4;
	float q3q4 = q3*q4;

	m.m[0][0] = qq1 + qq2 - qq3 - qq4;
	m.m[0][1] = 2.0f*(q2q3 - q1q4);
	m.m[0][2] = 2.0f*(q2q4 + q1q3);
	m.m[1][0] = 2.0f*(q2q3 + q1q4);
	m.m[1][1] = qq1 - qq2 + qq3 - qq4;
	m.m[1][2] = 2.0f*(q3q4 - q1q2);
	m.m[2][0] = 2.0f*(q2q4 - q1q3);
	m.m[2][1] = 2.0f*(q3q4 + q1q2);
	m.m[2][2] = qq1 - qq2 - qq3 + qq4;

	for (int r = 0; r < 3; r++)
		m.m[r][3] = origin.v[r];
	for (int c = 0; c < 3; c++)
		m.m[3][c] = 0;
	m.m[3][3] = 1.0f;
	return m;
}

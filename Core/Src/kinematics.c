/*
 * kinematics.c
 *
 *  Created on: Jan 28, 2023
 *      Author: Ocanath Robotman
 */



#include "kinematics.h"
#include "m_mcpy.h"

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

/*
* Tell me the xyz and rpy from an input homogeneous transoformation matrix.
* INPUT: M. Pass by reference (pbr)
* OUTPUTS:
*	xyz (pbr)
*	rpy (pbr)
*
* Uses math.h atan2. Is slow and accurate
*
*/
//void get_xyz_rpy(mat4_t* M, vect3_t* xyz, vect3_t * rpy)
//{
//	if (M == NULL || xyz == NULL || rpy == NULL)
//		return;
//	double yaw = atan2_approx(M->m[1][0], M->m[0][0]);
//	double pitch = atan2_approx(-M->m[2][0], sqrt((double)(M->m[2][1] * M->m[2][1] + M->m[2][2] * M->m[2][2])));
//	double roll = atan2_approx(M->m[2][1], M->m[2][2]);
//	h_origin_pbr(M, xyz);
//}

mat4_t get_rpy_xyz_htmatrix(vect3_t* xyz, vect3_t* rpy)
{
	mat4_t Hlink = Hz(rpy->v[2]);
	Hlink = mat4_t_mult(Hlink, Hy(rpy->v[1]));
	Hlink = mat4_t_mult(Hlink, Hx(rpy->v[0]));	//obtained Hrpy
	for (int r = 0; r < 3; r++)
		Hlink.m[r][3] = xyz->v[r];
	return Hlink;
}

/*
*	Initialize links with urdf-style inputs (xyz and roll pitch yaw)
*/
void init_forward_kinematics_urdf(joint* j, vect3_t * xyz, vect3_t * rpy, int num_joints)
{
	for(int i = 1; i <= num_joints; i++)
	{
		mat4_t Hlink = get_rpy_xyz_htmatrix(&xyz[i], &rpy[i]);
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
		mat4_t link = {
			{
				{1,		0,				0,				dh[i].a},
				{0,		cos_alpha,		-sin_alpha,		0},
				{0,		sin_alpha,		cos_alpha,		dh[i].d},
				{0,		0,				0,				1}
			}
		};
		m_mcpy(&j[i].h_link, &link, sizeof(mat4_t));
		copy_mat4_t(&j[i].him1_i, &j[i].h_link);	//htheta of 0 is the identity
		j[i - 1].child = &j[i];
	}
	copy_mat4_t(&j[0].h_link, &j[0].hb_i);
	j[0].q = 0;
	j[num_joints].child = NULL;	//initialize the last node in the linked list to NULL
	for (int i = 1; i <= num_joints; i++)
		mat4_t_mult_pbr(&j[i - 1].hb_i, &j[i].him1_i, &j[i].hb_i);
}

/*pre-loads all sin_q and cos_q for the chain*/
void load_q(joint* chain_start)
{
	joint* j = chain_start;
	while (j != NULL)
	{
		j->sin_q = sin_fast(j->q);
		j->cos_q = cos_fast(j->q);

		j = j->child;
	}
}

/*
Do forward kinematics on a singly linked list of joints!
Inputs:
		f1_joint: the first joint of the chain. I.e. the joint that relates frame 0 to frame 1
		hb_0: the matrix which relates a rigid base/reference frame to frame 0. Used for initial condition, and
				for robots with multiple chains.
Outputs:
		updates him1_i and hb_i of every element in the list

obviously singly linked list has same limitation as array. On
an embedded system, dynamic memory and recursion are not allowed
(both of which would be quite nice here; i.e. we can implement this as a dfs
type recursive run through the tree) so we'll have to operate on the tree
in a rigid structure (i.e. hard-coding what would normally be recursive)
*/
void forward_kinematics(mat4_t * hb_0, joint* f1_joint)
{
	if (f1_joint == NULL)
		return;

	joint * j = f1_joint;
	while(j != NULL)
	{
		//float sth = (float)sin((double)j->q);
		//float cth = (float)cos((double)j->q);
		float sth = j->sin_q;
		float cth = j->cos_q;

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
	mat4_t_mult_pbr(hb_0, &j->him1_i, &j->hb_i);	//load hb_1.		hb_0 * h0_1 = hb_1
	while (j->child != NULL)
	{
		j = j->child;
		mat4_t_mult_pbr(&parent->hb_i, &j->him1_i, &j->hb_i);
		parent = j;
	}
}

/*
	Traverse linked list and load torques into a list of equal size.

	Dangerous; could overrun if the linked list is not set up properly
*/
void calc_taulist(joint* chain_start, vect3_t* f)
{
	joint* j = chain_start;
	while (j != NULL)
	{
		j->tau_static = vect_dot(&(j->Si.v[3]), f->v, 3);	//remove Si radix to restore original 'f' radix
		j = j->child;
	}
}


/*
* Arguments:
*
* Si: the jacobian vector corresponding to the i'th joint
* p_b: the reference point, represented in the chain base frame
* hb_i1: the homogeneous transformation matrix
*
*/
void calc_rotational_jacobian_entry_f(vect6_t* Si, vect3_t* p_b, mat4_t* hb_im1)
{
	vect3_t z, d, res;
	for (int r = 0; r < 3; r++)
	{
		z.v[r] = hb_im1->m[r][2];
		d.v[r] = p_b->v[r] - hb_im1->m[r][3];
	}
	cross_pbr(&z, &d, &res);
	for (int r = 0; r < 3; r++)
	{
		Si->v[r] = z.v[r];
		Si->v[r + 3] = res.v[r];
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
void calc_J_point(mat4_t* hb_0, joint * chain_start, vect3_t * point_b)
{
	/*
	First, obtain the base case. This is contribution of the velocity from joint 1, and
	is found using the vector formed by the origin of joint 1 and the point, and the
	axis of rotation of joint 1 in the base frame.
*/
	joint* j = chain_start;	//i.e. Joint 1. joint 0 is NOT VALID, which is why hb_0 is passed as an argument and why we need to do a base case
	calc_rotational_jacobian_entry_f(&j->Si, point_b, hb_0);	//get jacobian entry for joint 1 from hb_0 and the reference point in base frame

	joint* parent = j;
	while (j->child != NULL)
	{
		j = j->child;
		calc_rotational_jacobian_entry_f(&j->Si, point_b, &parent->hb_i);
		parent = j;
	}
}


vect6_t calc_w_v(kinematic_chain * chain, vect3_t * w, vect3_t * v)
{
	vect6_t ret;
	int i;
	for (i = 0; i < 6; i++)
		ret.v[i] = 0;			//initialize the sum
	for (i = 1; i < chain->num_frames; i++)
		ret = vect6_add(ret, vect6_scale(chain->j[i].Si, chain->j[i].dq));
	w->v[0] = ret.v[0];
	w->v[1] = ret.v[1];
	w->v[2] = ret.v[2];
	v->v[0] = ret.v[3];
	v->v[1] = ret.v[4];
	v->v[2] = ret.v[5];
	return ret;
}

void calc_tau(joint * j, int num_joints, vect6_t f, float * tau)
{
	int i;
	for (i = 1; i <= num_joints; i++)
		tau[i] = vect_dot(j[i].Si.v, f.v, 6);
}

void calc_tau3(joint * j, int num_joints, vect3_t * f, float* tau)
{
	for (int i = 1; i <= num_joints; i++)
	{
		float dp = 0.f;
		for (int r = 0; r < 3; r++)
			dp += j[i].Si.v[r + 3] * f->v[r];
		tau[i] = dp;
	}
}

/*pass by reference htmatrix (special subset of mat4_t) and 3 vector)*/
void htmatrix_vect3_mult(mat4_t* m, vect3_t* v, vect3_t* ret)
{
	for (int r = 0; r < 3; r++)
	{
		float tmp = 0;
		for (int i = 0; i < 3; i++)
		{
			tmp += m->m[r][i] * v->v[i];
		}
		tmp += m->m[r][3];
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
*/
void h_origin_pbr(mat4_t * h, vect3_t * xyz)
{
	int i;
	for (i = 0; i < 3; i++)
		xyz->v[i] = h->m[i][3];
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


float abs_f(float v)
{
	if(v < 0.0f)
		v = -v;
	return v;
}


int gd_ik_single(mat4_t* hb_0, joint* start, joint* end, vect3_t* anchor_end, vect3_t* targ_b, vect3_t* anchor_b, float epsilon_divisor)	//num anchors?
{
	if (hb_0 == NULL || start == NULL || end == NULL || anchor_end == NULL || targ_b == NULL || anchor_b == NULL)
		return 0;	//blah

	joint* j;
	int solved = 0;
	int cycles = 0;
	while (solved == 0)
	{
		//do forward kinematics
		forward_kinematics(hb_0, start);
		htmatrix_vect3_mult(&end->hb_i, anchor_end, anchor_b);	//shift rotation out because only rotational components are added for a ht-multiply
		calc_J_point(hb_0, start, anchor_b);

		//printf("targ: [%d,%d,%d], ref: [%d,%d,%d]\r\n", o_targ_b->v[0], o_targ_b->v[1], o_targ_b->v[2], o_anchor_b->v[0], o_anchor_b->v[1], o_anchor_b->v[2]);
		//print_vect_mm("targ: ", o_targ_b, 16, "");
		//print_vect_mm("ref: ", o_anchor_b, 16, "\r\n");

		//get vector pointing from the anchor point on the robot to the target. call it 'f'. Unscaled.
		vect3_t f;
		for (int i = 0; i < 3; i++) //this can have much lower resolution than tau.  high res tau is important
			f.v[i] = (targ_b->v[i] - anchor_b->v[i]) / 16.f;	//step down from 16 to 12 bit reso for force vector
		calc_taulist(start, &f);	//removing an n_si (from f) yields tau in radix 16

		//apply a scaled torque vector to the chain structure via. sin and cosine vectors
		vect3_t z = {{ 0.f, 0.f, 1.f }};
		solved = 1;
		j = start;
		while (j != NULL)
		{
			vect3_t vq = {{ j->cos_q, j->sin_q, 0 }};	//create sin-cos structure

			vect3_t tangent;
			cross_pbr(&z, &vq, &tangent);		//obtain the tangent vector in the xy plane. it is normalized

			vect3_t vq_new;	//will contain the result of ~q+epsilon for our gradient descent

			//Scale the tangent vector and add it to the original vq vector
			for (int r = 0; r < 3; r++)
			{
				float tmp = (tangent.v[r] * j->tau_static) / epsilon_divisor;
				vq_new.v[r] = tmp + vq.v[r];

				if (abs_f(tmp) > 0.001f)
					solved = 0;
			}

			vect_normalize(vq_new.v, 3);

			j->cos_q = vq_new.v[0];
			j->sin_q = vq_new.v[1];

			j = j->child;
		}
		cycles++;
	}

	j = start;
	while (j != NULL)
	{
		j->q = atan2_approx(j->sin_q, j->cos_q);
		j = j->child;
	}
	return cycles;
}

#ifndef KINEMATICS_H
#define KINEMATICS_H

//#include "spatialAlgebra.h"
#include "vect.h"
#include "sin-math.h"

typedef struct dh_entry
{
	float d;
	float a;
	float alpha;

	//float sin_alpha;	//pre-store sin and cos of alpha, to avoid re-computing it every loop
	//float cos_alpha;
	//theta is stored separately
}dh_entry;

typedef struct joint
{
	float q;			//joint position
	float dq;			//joint velocity
	float ddq;			//joint acceleration
	float cos_q;
	float sin_q;
	float tau_static;	//helper variable which contains the result of J^T*f, for some arbitrary force f

	vect6_t Si;			//vector corresponding to the i'th column of the jacobian matrix. Si*q(i) = vi, where vi is the ith's joint's contribution to the total chain velocity in frame 0
	mat4_t hb_i;			//homogeneous transformation relating the BASE frame to the current frame (i). hb_0 = him1_0
	mat4_t him1_i;		//homogeneous transformation describing the rotation and translation from frame i-1 to the current frame (i).
	//dh_entry dh;	//dh parameter describing the offset and angles from frame i-1 to the current frame (i)

	mat4_t h_link;	//pre-computed, constant matrix. given that h_theta is the rotation matrix defined by q/theta, then him1_i = h_theta*h_link.

	mat6_t I_hat;			//6 matrix describing the spatial inertia of each frame with respect to the origin.

	struct joint * child;
}joint;

typedef struct kinematic_chain
{
	mat4_t hw_b;
	mat4_t hb_0;
	joint * j;	//list of joint variables, each containing the relevant information for that joint.
	int num_frames;
}kinematic_chain;


//void init_chain(kinematic_chain * chain, const dh_entry * table, const float * q_init, int num_frames);
//void chain_pre_allocate(kinematic_chain * chain, int num_frames);
//void chain_de_allocate(kinematic_chain * chain);
void init_forward_kinematics_urdf(joint* j, vect3_t* xyz, vect3_t* rpy, int num_joints);
mat4_t get_rpy_xyz_htmatrix(vect3_t* xyz, vect3_t* rpy);
void init_forward_kinematics_dh(joint* j, const dh_entry* dh, int num_joints);
void forward_kinematics(mat4_t* hb_0, joint* f1_joint);
void load_q(joint* chain_start);
void calc_J_point(mat4_t* hb_0, joint* chain_start, vect3_t* point_b);
void copy_mat4_t(mat4_t* dest, mat4_t* src);
vect6_t calc_w_v(kinematic_chain * chain, vect3_t * w, vect3_t * v);
void htmatrix_vect3_mult(mat4_t* m, vect3_t* v, vect3_t* ret);
vect3_t h_origin(mat4_t h);
void h_origin_pbr(mat4_t* h, vect3_t* xyz);
void get_xyz_rpy(mat4_t* M, vect3_t* xyz, vect3_t* rpy);
vect4_t h_origin_vect4(mat4_t h);
mat4_t quat_to_mat4_t(vect4_t quat, vect3_t origin);
void calc_tau(joint* j, int num_joints, vect6_t f, float* tau);
void calc_tau3(joint* j, int num_joints, vect3_t* f, float* tau);	//faster alt to calc_tau
void calc_taulist(joint* chain_start, vect3_t* f);
int gd_ik_single(mat4_t* hb_0, joint* start, joint* end, vect3_t* anchor_end, vect3_t* targ_b, vect3_t* anchor_b, float epsilon_divisor);	//num anchors?


#endif

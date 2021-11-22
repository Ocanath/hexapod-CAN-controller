#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "joint.h"

typedef struct dh_entry
{
	float d;
	float a;
	float alpha;

	//float sin_alpha;	//pre-store sin and cos of alpha, to avoid re-computing it every loop
	//float cos_alpha;
	//theta is stored separately
}dh_entry;

typedef struct kinematic_chain
{
	mat4_t hw_b;
	mat4_t hb_0;
	joint * j;	//list of joint variables, each containing the relevant information for that joint.
	int num_frames;
}kinematic_chain;


void init_forward_kinematics_urdf(joint* j, vect3_t* xyz, vect3_t* rpy, int num_joints);
void init_forward_kinematics_dh(joint* j, const dh_entry* dh, int num_joints);
void forward_kinematics(mat4_t * hb_0, joint* f1_joint);
void calc_J_point(joint* j, int num_joints, vect3_t point);
void copy_mat4_t(mat4_t* dest, mat4_t* src);
vect6_t calc_w_v(joint * chain, vect3_t * w, vect3_t * v);
void htmatrix_vect3_mult(mat4_t* m, vect3_t* v, vect3_t* ret);
vect3_t h_origin(mat4_t h);
vect4_t h_origin_vect4(mat4_t h);
mat4_t quat_to_mat4_t(vect4_t quat, vect3_t origin);
void calc_tau(joint* j, int num_joints, vect6_t f, float* tau);
void calc_tau3(joint* j, int num_joints, vect3_t* f, float* tau);	//faster alt to calc_tau
void rpy_to_mat4(mat4_t * m, vect3 rpy, vect3 xyz);


void dh_to_mat4(mat4_t * m, dh_entry * dh);
void rpy_to_mat4(mat4_t * m, vect3 rpy, vect3 xyz);

#endif

/*
 * vect.h
 *
 *  Created on: Nov 21, 2021
 *      Author: Ocanath Robotman
 */

#ifndef INC_VECT_H_
#define INC_VECT_H_
#include <stdint.h>

typedef float vect3[3];
typedef struct vect3_t
{
	vect3 v;
}vect3_t;

/*
homogeneous transformation matrix vectors (lol)
*/
typedef float vect4[4];
typedef struct vect4_t
{
	vect4 v;
}vect4_t;


typedef float vect6[6];
typedef struct vect6_t
{
	vect6 v;
}vect6_t;

typedef float mat3[3][3];	//	Array wrapper for unambiguous pointer to 2d array
typedef struct mat3_t		//	Rotation matrices, skew symmetric matrices take this type
{
	mat3 m;
}mat3_t;

typedef float mat4[4][4];		//Array wrapper for unambiguous pointer to 2d array
typedef struct mat4_t			//homogeneous transformation matrices take this type. Allows working with arrays as a type directly (including returns) instead of pointers
{
	mat4 m;
}mat4_t;

typedef float mat6[6][6];
typedef struct mat6_t
{
	mat6 m;
}mat6_t;

/*********************Fixed point matrix-vect types*****************************/
typedef int32_t mat4_32b[4][4];
typedef struct mat4_32b_t
{
	mat4_32b m;
}mat4_32b_t;

typedef int32_t vect3_32b[3];
typedef struct vect3_32b_t
{
	vect3_32b v;
}vect3_32b_t;
typedef int32_t vect6_32b[6];
typedef struct vect6_32b_t
{
	vect6_32b v;
}vect6_32b_t;

inline float asm_sqrt(float op1)
{
	float result;
	asm volatile("vsqrt.f32 %0, %1" : "=w" (result) : "w" (op1) );
	return result;
}

mat4_t mat4_t_mult(mat4_t m1, mat4_t m2);
void mat4_t_mult_pbr(mat4_t * m1, mat4_t * m2, mat4_t * ret);
void cross_pbr(vect3_t * v_a, vect3_t * v_b, vect3_t * ret);
mat4_t Hz(float angle);
mat4_t Hy(float angle);
mat4_t Hx(float angle);
void ht_mat4_mult_pbr(mat4_t * m1, mat4_t * m2, mat4_t * ret);

void ht32_mult_pbr(mat4_32b_t * m1, mat4_32b_t * m2, mat4_32b_t * ret);
void cross32_pbr(vect3_32b_t * v_a, vect3_32b_t * v_b, vect3_32b_t * ret, int n);
void ht32_mult64_pbr(mat4_32b_t * m1, mat4_32b_t * m2, mat4_32b_t * ret, int n);

float vect_dot(float* v1, float* v2, int n);
vect6_t vect6_add(vect6_t v_a, vect6_t v_b);
/*Multiples vector v_a by scalar scale*/
vect6_t vect6_scale(vect6_t v_a, float scale);
vect3_t vect3_scale(vect3_t v_a, float scale);
void vect_normalize(float* v, int n);

mat4_32b_t Hy_nb(int32_t angle, int n);
mat4_32b_t Hx_nb(int32_t angle, int n);
mat4_32b_t Hz_nb(int32_t angle, int n);

void ht_inverse_ptr(mat4_t * hin, mat4_t * hout);

float Q_rsqrt(float number);

#endif /* INC_VECT_H_ */

/*
 * vect.h
 *
 *  Created on: Nov 21, 2021
 *      Author: Ocanath Robotman
 */

#ifndef INC_VECT_H_
#define INC_VECT_H_

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

mat4_t mat4_t_mult(mat4_t m1, mat4_t m2);
void mat4_t_mult_pbr(mat4_t * m1, mat4_t * m2, mat4_t * ret);
void cross_pbr(vect3_t * v_a, vect3_t * v_b, vect3_t * ret);
mat4_t Hz(float angle);
mat4_t Hy(float angle);
mat4_t Hx(float angle);
void ht_mat4_mult_pbr(mat4_t * m1, mat4_t * m2, mat4_t * ret);

#endif /* INC_VECT_H_ */

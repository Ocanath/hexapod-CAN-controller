/*
 * vect.c
 *
 *  Created on: Nov 21, 2021
 *      Author: Ocanath Robotman
 */
#include "vect.h"

/*
 * Same as normalt mat4 multiplication with pass by pointer,
 * EXCEPT that it is assummed to be a homogeneous transformation matrix where
 * the bottom rows of all matrices are equal to 0,0,0,1
*/
void ht_mat4_mult_pbr(mat4_t * m1, mat4_t * m2, mat4_t * ret, int translation_order_n)
{
	int r, c, i;
	int32_t tmp;
	/*Do the rotation matrix part FIRST*/
	for(r = 0; r < 3; r++)
	{
		for(c = 0; c < 3; c++)
		{
			tmp = 0;
			for(int i = 0; i < 4; i++)
			{
				tmp += (m1->m[r][i] * m2->m[i][c]) >> 12;	//each term has 12bit binary decimal point
			}
			ret->m[r][c] = tmp;
		}
	}
	/*Do the translation part LAST. This part must be handled differently due to relative scaling*/


//	ret->m[3][0] = 0.f;
//	ret->m[3][1] = 0.f;
//	ret->m[3][2] = 0.f;
//	ret->m[3][3] = 1.f;	//commented because this function should be called on matrices where that relationship is assumed to be true
}

/*
	Returns vector cross product between 3 vectors A and B. Faster pass by pointer version
*/
void cross_pbr(vect3_t * v_a, vect3_t * v_b, vect3_t * ret)
{
	ret->v[0] = -v_a->v[2]*v_b->v[1] + v_a->v[1]*v_b->v[2];
	ret->v[1] = v_a->v[2]*v_b->v[0] - v_a->v[0]*v_b->v[2];
	ret->v[2] = -v_a->v[1]*v_b->v[0] + v_a->v[0]*v_b->v[1];
}

/*Loads rotation about coordinate. 0 = identity*/
mat4_t Hz(float angle)
{
	float cth = cos_fast(angle);
	float sth = sin_fast(angle);
	mat4_t r;
	r.m[0][0] = cth;		r.m[0][1] = -sth;		r.m[0][2] = 0;	r.m[0][3] = 0;
	r.m[1][0] = sth;		r.m[1][1] = cth;		r.m[1][2] = 0;	r.m[1][3] = 0;
	r.m[2][0] = 0;			r.m[2][1] = 0;			r.m[2][2] = 1;	r.m[2][3] = 0;
	r.m[3][0] = 0;			r.m[3][1] = 0;			r.m[3][2] = 0;	r.m[3][3] = 1;
	return r;
}

/*returns homogeneous transform mat4_t matrix which is the rotation 'analge' around the x axis */
mat4_t Hx(float angle)
{
	mat4_t ret;
	ret.m[0][0] = 1;	ret.m[0][1] = 0;				ret.m[0][2] = 0;				ret.m[0][3] = 0;
	ret.m[1][0] = 0;	ret.m[1][1] = cos_fast(angle);	ret.m[1][2] = -sin_fast(angle);	ret.m[1][3] = 0;
	ret.m[2][0] = 0;	ret.m[2][1] = sin_fast(angle);	ret.m[2][2] = cos_fast(angle);	ret.m[2][3] = 0;
	ret.m[3][0] = 0;	ret.m[3][1] = 0;				ret.m[3][2] = 0;				ret.m[3][3] = 1;
	return ret;
}

/*Returns rotation about coordinate. 0 = identity*/
mat4_t Hy(float angle)
{
	mat4_t ret;
	ret.m[0][0] = cos_fast(angle);	ret.m[0][1] = 0;	ret.m[0][2] = sin_fast(angle);	ret.m[0][3] = 0;
	ret.m[1][0] = 0;				ret.m[1][1] = 1;	ret.m[1][2] = 0;				ret.m[1][3] = 0;
	ret.m[2][0] = -sin_fast(angle);	ret.m[2][1] = 0;	ret.m[2][2] = cos_fast(angle);	ret.m[2][3] = 0;
	ret.m[3][0] = 0;				ret.m[3][1] = 0;	ret.m[3][2] = 0;				ret.m[3][3] = 1;
	return ret;
}

vect6_t vect6_add(vect6_t v_a, vect6_t v_b)
{
	vect6_t ret;
	int dim = 6; int i;
	for (i = 0; i < dim; i++)
		ret.v[i] = v_a.v[i] + v_b.v[i];
	return ret;
}

/*Multiples vector v_a by scalar scale*/
vect3_t vect3_scale(vect3_t v_a, float scale)
{
	vect3_t ret; int i;
	for (i = 0; i<3; i++)
		ret.v[i] = v_a.v[i] * scale;
	return ret;
}

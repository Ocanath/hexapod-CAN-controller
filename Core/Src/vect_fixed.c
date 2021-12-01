/*
 * vect.c
 *
 *  Created on: Nov 21, 2021
 *      Author: Ocanath Robotman
 */
#include "vect.h"
#include "trig_fixed.h"

#define ONE_12B 4096

/*
 * Same as normalt mat4 multiplication with pass by pointer,
 * EXCEPT that it is assummed to be a homogeneous transformation matrix where
 * the bottom rows of all matrices are equal to 0,0,0,1
*/
void ht32_mult_pbr(mat4_32b_t * m1, mat4_32b_t * m2, mat4_32b_t * ret)
{
	int r, c, i;
	int32_t tmp;
	/*Do the rotation matrix part FIRST*/
	for(r = 0; r < 3; r++)
	{
		for(c = 0; c < 3; c++)
		{
			tmp = 0;
			for(i = 0; i < 4; i++)
			{
				tmp += (m1->m[r][i] * m2->m[i][c]);
			}
			ret->m[r][c] = tmp >> 12;	//each term has 12bit binary decimal point. Remove the additional terms accumulated AFTER the sum.
			//the 3x3 portion is safe to do multiple accumulations because each term is 12bit limited (even ignoring constraints imposed by the
			//conditions required to be a homogeneous transformation matrix, such as column vector orthogonality, unit vector across row and column vectors)
		}
	}
	/*Do the translation part LAST. This part must be handled differently due to relative scaling*/

	for(int r = 0; r < 4; r++)
	{
		tmp = 0;
		for(int i = 0; i < 4; i++)
			tmp += m1->m[r][i] * m2->m[i][3];
		ret->m[r][3] = tmp >> 12;	//remove the order 12 component
	}

//	ret->m[3][0] = 0.f;
//	ret->m[3][1] = 0.f;
//	ret->m[3][2] = 0.f;
//	ret->m[3][3] = 1.f;	//commented because this function should be called on matrices where that relationship is assumed to be true
}

/*
 * Same as normalt mat4 multiplication with pass by pointer,
 * EXCEPT that it is assummed to be a homogeneous transformation matrix where
 * the bottom rows of all matrices are equal to 0,0,0,1
*/
void ht32_mult64_pbr(mat4_32b_t * m1, mat4_32b_t * m2, mat4_32b_t * ret, int n)
{
	int r, c, i;
	int64_t tmp;
	/*Do the rotation matrix part FIRST*/
	for(r = 0; r < 3; r++)
	{
		for(c = 0; c < 3; c++)
		{
			tmp = 0;
			for(i = 0; i < 4; i++)
			{
				tmp += ( ((int64_t)m1->m[r][i]) * ((int64_t)m2->m[i][c]) );
			}
			ret->m[r][c] = (int32_t)(tmp >> n);	//each term has 12bit binary decimal point. Remove the additional terms accumulated AFTER the sum.
			//the 3x3 portion is safe to do multiple accumulations because each term is 12bit limited (even ignoring constraints imposed by the
			//conditions required to be a homogeneous transformation matrix, such as column vector orthogonality, unit vector across row and column vectors)
		}
	}
	/*Do the translation part LAST. This part must be handled differently due to relative scaling*/
	for(int r = 0; r < 4; r++)
	{
		tmp = 0;
		for(int i = 0; i < 4; i++)
			tmp += ((int64_t)m1->m[r][i]) * ((int64_t)m2->m[i][3]);
		ret->m[r][3] = (int32_t)(tmp >> n);	//remove the order 12 component
	}

//	ret->m[3][0] = 0.f;
//	ret->m[3][1] = 0.f;
//	ret->m[3][2] = 0.f;
//	ret->m[3][3] = 1.f;	//commented because this function should be called on matrices where that relationship is assumed to be true
}

/*
	Returns vector cross product between 3 vectors A and B. Faster pass by pointer version
*/
void cross32_pbr(vect3_32b_t * v_a, vect3_32b_t * v_b, vect3_32b_t * ret, int n)
{
	ret->v[0] = (-v_a->v[2]*v_b->v[1] + v_a->v[1]*v_b->v[2]) >> n;
	ret->v[1] = (v_a->v[2]*v_b->v[0] - v_a->v[0]*v_b->v[2]) >> n;
	ret->v[2] = (-v_a->v[1]*v_b->v[0] + v_a->v[0]*v_b->v[1]) >> n;	//need to remove some of the scaling factor used to express these in fixed point
}


/*Loads rotation about coordinate. 0 = identity*/
mat4_32b_t Hz_nb(int32_t angle, int n)
{
	int32_t cth = cos_lookup(angle,n);
	int32_t sth = sin_lookup(angle,n);
	mat4_32b_t r;
	r.m[0][0] = cth;		r.m[0][1] = -sth;		r.m[0][2] = 0;			r.m[0][3] = 0;
	r.m[1][0] = sth;		r.m[1][1] = cth;		r.m[1][2] = 0;			r.m[1][3] = 0;
	r.m[2][0] = 0;			r.m[2][1] = 0;			r.m[2][2] = ONE_12B;	r.m[2][3] = 0;
	r.m[3][0] = 0;			r.m[3][1] = 0;			r.m[3][2] = 0;			r.m[3][3] = ONE_12B;
	return r;
}

/*returns homogeneous transform mat4_t matrix which is the rotation 'analge' around the x axis */
mat4_32b_t Hx_nb(int32_t angle, int n)
{
	int32_t cth = cos_lookup(angle,n);
	int32_t sth = sin_lookup(angle,n);

	mat4_32b_t ret;
	ret.m[0][0] = ONE_12B;	ret.m[0][1] = 0;	ret.m[0][2] = 0;		ret.m[0][3] = 0;
	ret.m[1][0] = 0;		ret.m[1][1] = cth;	ret.m[1][2] = -sth;		ret.m[1][3] = 0;
	ret.m[2][0] = 0;		ret.m[2][1] = sth;	ret.m[2][2] = cth;		ret.m[2][3] = 0;
	ret.m[3][0] = 0;		ret.m[3][1] = 0;	ret.m[3][2] = 0;		ret.m[3][3] = ONE_12B;
	return ret;
}

/*Returns rotation about coordinate. 0 = identity*/
mat4_32b_t Hy_nb(int32_t angle, int n)
{
	int32_t cth = cos_lookup(angle,n);
	int32_t sth = sin_lookup(angle,n);

	mat4_32b_t ret;
	ret.m[0][0] = cth;		ret.m[0][1] = 0;		ret.m[0][2] = sth;		ret.m[0][3] = 0;
	ret.m[1][0] = 0;		ret.m[1][1] = ONE_12B;	ret.m[1][2] = 0;		ret.m[1][3] = 0;
	ret.m[2][0] = -sth;		ret.m[2][1] = 0;		ret.m[2][2] = cth;		ret.m[2][3] = 0;
	ret.m[3][0] = 0;		ret.m[3][1] = 0;		ret.m[3][2] = 0;		ret.m[3][3] = ONE_12B;
	return ret;
}


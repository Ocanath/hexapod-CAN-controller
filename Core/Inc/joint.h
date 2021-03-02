/*
 * joint.h
 *
 *  Created on: Feb 12, 2021
 *      Author: Ocanath Robotman
 */

#ifndef INC_JOINT_H_
#define INC_JOINT_H_

typedef struct mat4
{
	float m[4][4];
}mat4;

typedef union
{
	uint32_t v;
	uint8_t d[sizeof(uint32_t)];
}uint32_fmt_t;

typedef union floatsend_t
{
	float v;
	uint8_t d[sizeof(float)];
}floatsend_t;
typedef struct joint
{
	uint16_t id;
	int frame;
	mat4 h0_i;
	mat4 him1_i;
	float q;
	float q_offset;	//track the phase offset present in the encoder signal
	floatsend_t tau;
	float qd;
	uint8_t misc_cmd;
}joint;


#endif /* INC_JOINT_H_ */

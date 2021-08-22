/*
 * joint.h
 *
 *  Created on: Feb 12, 2021
 *      Author: Ocanath Robotman
 */

#ifndef INC_JOINT_H_
#define INC_JOINT_H_
#include "init.h"
#include "sin-math.h"
#include "CAN.h"

#define RAD_TO_DEG	57.2957795
#define DEG_TO_RAD	0.0174532925f

#define NUM_JOINTS 9

enum {
	LED_ON = 0xDE,
	LED_OFF =0xFE,
	LED_BLINK= 0xAA,
	EN_UART_ENC = 0x34,
	DIS_UART_ENC = 0x35
};

typedef struct mat4
{
	float m[4][4];
}mat4;

typedef union
{
	uint32_t v;
	uint8_t d[sizeof(uint32_t)];
}uint32_fmt_t;

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

extern joint chain[NUM_JOINTS];

void joint_comm_misc(joint * chain);
void joint_comm_motor(joint * chain, int num_joints);

#endif /* INC_JOINT_H_ */

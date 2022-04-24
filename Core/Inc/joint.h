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
#include "vect.h"

#define RAD_TO_DEG	57.2957795
#define DEG_TO_RAD	0.0174532925f

#define NUM_JOINTS 9

enum {
	LED_ON = 0xDE,
	LED_OFF =0xFE,
	LED_BLINK= 0xAA,
	EN_UART_ENC = 0x34,
	DIS_UART_ENC = 0x35,
	SET_FOC_MODE = 0x36,
	SET_SINUSOIDAL_MODE = 0x37
};

typedef union
{
	uint32_t v;
	uint8_t d[sizeof(uint32_t)];
}uint32_fmt_t;

typedef struct ctl_params_t
{
	float kp;
	float ki_div;
	float kd;
	float x_pi;
	float x_sat;
	float tau_sat;
}ctl_params_t;

typedef struct joint
{
	uint16_t id;	//CAN node id. ID of the joint.
	int frame;		//kinematic frame joint belongs to. Not particularly necessary?

	//kinematics things
	mat4_t hb_i;
	mat4_t him1_i;
	mat4_t h_link;
	mat3_t InertiaTensor;
	vect6_t Si;			//vector corresponding to the i'th column of the jacobian matrix. Si*q(i) = vi, where vi is the ith's joint's contribution to the total chain velocity in frame 0
	struct joint * child;

	//fixedpoint kinematics things
	mat4_32b_t h32_b_i;
	mat4_32b_t h32_im1_i;
	mat4_32b_t h32_link;

	//Delivered to the motor. Velocity or torque.
	can_payload_t tau;

	//Measured quantites, delivered over can
	float q;
	int16_t q16;
	float dq_rotor;
	int16_t dq_rotor16;
	float iq_meas;		//motor torque, measured
	//Computed once per new value and saved in the wrapper structure for speed
	int32_t sin_q;
	int32_t cos_q;
	float sin_q_float;
	float cos_q_float;

	//Mapping from real to idealized for encoder
	float q_offset;	//track the phase offset present in the encoder signal
	float gear_ratio;	//rotor TO output. i.e. 16.f for our hexapod

	//calculation of measured output shaft angle velocity. We estimate not the motor
	float dq_output;	//estimated
	int16_t prev_q16;	//16bit loaded previous q, for fast unwrapping in fixed point angles
	uint32_t ts_dq;	//timestamp associated with last recieved sample of q

	//direct angular position control params. not necessarily used
	float qd;
	ctl_params_t ctl;

	uint8_t misc_cmd;

	uint8_t encoder_mode;
	uint8_t control_mode;


	uint8_t responsive;	//flag to indicate whether successful communication to this ID has been verified (through a 'heartbeat' motor instruction')
}joint;

extern joint chain[NUM_JOINTS];

float wrap(float in);
void joint_comm_misc(joint * chain);
int joint_comm(joint * j);
void chain_comm(joint * chain, int num_joints);
uint32_t get_ts_us(void);
int32_t wrap_fixed(int32_t in, uint32_t k);

#endif /* INC_JOINT_H_ */

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

#define NUM_JOINTS 18

enum {
	CALC_ALIGN_OFFSET = 0x1,
	CALC_ENC_MIDPOINTS = 0x2,


	LED_ON = 0xDE,
	LED_OFF =0xFE,
	LED_BLINK= 0xAA,

	EN_UART_ENC = 0x34,
	DIS_UART_ENC = 0x35,

	SET_FOC_MODE = 0x36,	//configure to foc mode. need to do once and it takes
	SET_SINUSOIDAL_MODE = 0x37,	//configure to velocity mode. send once and it takes
	CHANGE_V_RADIX = 0x38,	//radix for velocity expression setting
	CHANGE_IQ_RSHIFT = 0x39,	//rightshift for iq expression setting

	SET_PCTL_IQ_MODE = 0x40,	//configure to position control using FOC mode. send once and it takes. Needs all position settings written to do anything (stable)
	CHANGE_PCTL_IQ_KP_VALUE = 0x41,
	CHANGE_PCTL_IQ_KP_RADIX = 0x42,
	CHANGE_PCTL_IQ_KI_VALUE = 0x43,
	CHANGE_PCTL_IQ_KI_RADIX = 0x44,
	CHANGE_PCTL_IQ_KD_VALUE = 0x45,
	CHANGE_PCTL_IQ_KD_RADIX = 0x46,
	CHANGE_PCTL_IQ_XSAT = 0x47,
	CHANGE_PCTL_IQ_OUTSAT = 0x48,
	CHANGE_PCTL_IQ_OUT_RSHIFT = 0x49,


	SET_PCTL_VQ_MODE = 0x4A,	//configure to position control using sinusoidal mode. send once and it takes. Needs all position settings written to do anything (stable)
	CHANGE_PCTL_VQ_KP_VALUE = 0x4B,
	CHANGE_PCTL_VQ_KP_RADIX = 0x4C,
	CHANGE_PCTL_VQ_KI_VALUE = 0x4D,
	CHANGE_PCTL_VQ_KI_RADIX = 0x4E,
	CHANGE_PCTL_VQ_KD_VALUE = 0x4F,
	CHANGE_PCTL_VQ_KD_RADIX = 0x50,
	CHANGE_PCTL_VQ_XSAT = 0x51,
	CHANGE_PCTL_VQ_OUTSAT = 0x52,
	CHANGE_PCTL_VQ_OUT_RSHIFT = 0x53
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
	can_payload_t mtn16;

	//Measured quantites, delivered over can
	float q;
	int32_t q32_rotor; 	//POSSIBLE THAT THIS IS NOT UPDATDED: 32bit rotor position (overridden by aenc)
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
int joint_comm_misc(joint * chain);
int joint_comm(joint * j);
void chain_comm(joint * chain, int num_joints);
uint32_t get_ts_us(void);
int32_t wrap_fixed(int32_t in, uint32_t k);

#endif /* INC_JOINT_H_ */

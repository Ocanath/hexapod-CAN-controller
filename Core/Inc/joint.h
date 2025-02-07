/*
 * motor_t.h
 *
 *  Created on: Feb 12, 2021
 *      Author: Ocanath Robotman
 */

#ifndef INC_motor_t_H_
#define INC_motor_t_H_
#include "init.h"
#include "sin-math.h"
#include "CAN.h"
#include "vect.h"

#define RAD_TO_DEG	57.2957795
#define DEG_TO_RAD	0.0174532925f

#define NUM_MOTORS 18

enum {

	CALC_ALIGN_OFFSET = 0x1,
	CALC_ENC_MIDPOINTS = 0x2,

	CHANGE_AC_VQ = 0x4,
	CHANGE_AC_RUNTIME = 0x5,
	CHANGE_AC_GAIN = 0x6,	//ignore passthresh for now.

	CHANGE_ALIGN_OFFSET = 0x7,

	/*The following are settings that MUST be loaded which are for flash memory.
	 * Offloading records to the CAN master is not a bad idea honestly*/
	SET_ELEC_CONV_RATIO_FIXED = 0x8,
	SET_GL_PROP_DELAYLOOP_INTERVAL = 0x9,
	SET_GL_PROP_DELAY_CONST_12B = 0xA,
	SET_IQ_PI_KP_I32 = 0xB,
	SET_IQ_PI_KP_RADIX = 0xC,
	SET_IQ_PI_KI_I32 = 0xD,
	SET_IQ_PI_KI_RADIX = 0xE,
	SET_IQ_PI_X_SAT = 0xF,
	SET_IQ_PI_OUT_RSHIFT = 0x10,
	SET_ID_PI_KP_I32 = 0x11,
	SET_ID_PI_KP_RADIX = 0x12,
	SET_ID_PI_KI_I32 = 0x13,
	SET_ID_PI_KI_RADIX = 0x14,
	SET_ID_PI_X_SAT = 0x15,
	SET_ID_PI_OUT_RSHIFT = 0x16,

	CMD_WRITE_FLASH = 0x17,	//it's important this only triggers once
	CMD_RESTART = 0x18,	//restart microcontroller

	SET_CAN_ID = 0x19,	//change the CAN ID over CAN. in theory, this should only take effect on a power cycle. NEEDS TESTING

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
	CHANGE_PCTL_VQ_OUT_RSHIFT = 0x53,

	EN_REVERSE_DIRECTION = 0x54,
	DIS_REVERSE_DIRECTION = 0x55,
	CHANGE_PCTL_VQ_XINTEGRALDIV = 0x56,
	CHANGE_PCTL_IQ_XINTEGRALDIV = 0x57
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
	float x_pi;
	float x_sat;
	float tau_sat;
}ctl_params_t;

typedef struct motor_t
{
	uint16_t id;	//CAN node id. ID of the motor_t.
	struct motor_t * child;

	//Delivered to the motor. Velocity or torque.
	can_payload_t mtn16;

	//Measured quantites, delivered over can
	int32_t q32_rotor; 	//POSSIBLE THAT THIS IS NOT UPDATDED: 32bit rotor position (overridden by aenc)
	int16_t q16;
	int16_t dq_rotor16;

	//Computed once per new value and saved in the wrapper structure for speed
	int32_t sin_q;
	int32_t cos_q;

	//calculation of measured output shaft angle velocity. We estimate not the motor
	int16_t prev_q16;	//16bit loaded previous q, for fast unwrapping in fixed point angles
	uint32_t ts_dq;	//timestamp associated with last recieved sample of q

	ctl_params_t ctl;

	uint8_t misc_cmd;

	uint8_t encoder_mode;
	uint8_t control_mode;

	uint8_t responsive;	//flag to indicate whether successful communication to this ID has been verified (through a 'heartbeat' motor instruction')
	uint8_t reverse_dir; //flag to indicate whether the direction of the adapter encoder should be negated
	uint8_t negate_aenc; //flag to indicate whether we need to negate the aenc to match for qkinematic

	float q;
	float iq_meas;
	float dq_output;
	float q_offset;

}motor_t;

extern motor_t chain[NUM_MOTORS];

float wrap(float in);
int motor_t_comm_misc(motor_t * chain);
void motor_t_get(motor_t * gl_chain, int chain_size);
void motor_put_wait(void);
void motor_t_put(motor_t * j);
uint32_t get_ts_us(void);
int32_t wrap_fixed(int32_t in, uint32_t k);

#endif /* INC_motor_t_H_ */

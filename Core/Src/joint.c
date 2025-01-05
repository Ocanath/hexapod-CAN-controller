/*
 * motor_t.c
 *
 *  Created on: Aug 21, 2021
 *      Author: Ocanath
 */
#include "joint.h"
#include "trig_fixed.h"

motor_t chain[NUM_MOTORS] =
{
		{//0						//leg 4, q1
				.id = 32,
				.q_offset = -0.49f,
				.reverse_dir = 1,
				.negate_aenc = 1
		},
		{//1						//leg 4, q2
				.id = 33,
				.q_offset = -1.599934f,
				.reverse_dir = 0,
				.negate_aenc = 0
		},
		{//2						//leg 4, q3
				.id = 34,
				.q_offset = 1.681383f,
				.reverse_dir = 1,
				.negate_aenc = 0
		},
		{//3						//leg 3, q1
				.id = 21,
				.mtn16 = {{0}},
				.q_offset = -1.422363f,
				.misc_cmd = LED_OFF,
				.reverse_dir = 1,
				.negate_aenc = 1
		},
		{//4						//leg 3, q2
				.id = 20,
				.mtn16 = {{0}},
				.q_offset = 0.823162f,
				.misc_cmd = LED_OFF,
				.reverse_dir = 1,
				.negate_aenc = 0
		},
		{//5						//leg 3, q3
				.id = 22,
				.mtn16 = {{0}},
				.q_offset = 1.213853f,
				.misc_cmd = LED_OFF,
				.reverse_dir = 0,
				.negate_aenc = 0
		},
		{//6						//leg 5, q1
				.id = 29,
				.q_offset = -1.150635,//-1.480469f,
				.reverse_dir = 1,
				.negate_aenc = 1
		},
		{//7						//leg 5, q2
				.id = 30,
				.q_offset = 2.093181f-0.5f,
				.reverse_dir = 1,
				.negate_aenc = 0
		},
		{//8						//leg 5, q3
				.id = 31,
				.q_offset = -2.467299f,
				.reverse_dir = 1,
				.negate_aenc = 0
		},
		{//9						//leg 1, q1
				.id = 24,
				.q_offset = 1.317627f,
				.reverse_dir = 1,
				.negate_aenc = 1
		},
		{//10						//leg 1, q2
				.id = 25,
				.q_offset = 0.565593f,
				.reverse_dir = 1,
				.negate_aenc = 0
		},
		{//11						//leg 1, q3
				.id = 23,
				.mtn16 = {{0}},
				.q_offset = -0.220717f,
				.misc_cmd = LED_OFF,
				.reverse_dir = 1,
				.negate_aenc = 0
		},
		{//12						//leg 6, q1
				.id = 36,
				.q_offset = 0.978027f,
				.reverse_dir = 1,
				.negate_aenc = 1
		},
		{//13						//leg 6, q2
				.id = 35,
				.q_offset = 1.709880f,
				.reverse_dir = 1,
				.negate_aenc = 0
		},
		{//14						//leg 6, q3
				.id = 37,
				.q_offset = -0.921401f,
				.reverse_dir = 1,
				.negate_aenc = 0
		},
		{//15						//leg 2, q1
				.id = 26,
				.mtn16 = {{0}},
				.q_offset = -1.957520f,
				.misc_cmd = LED_OFF,
				.reverse_dir = 0,
				.negate_aenc = 1
		},
		{//16						//leg 2, q2
				.id = 27,
				.mtn16 = {{0}},
				.q_offset = 1.244548f,
				.misc_cmd = LED_OFF,
				.reverse_dir = 1,
				.negate_aenc = 0
		},
		{//17						//leg 2, q3
				.id = 28,
				.mtn16 = {{0}},
				.q_offset = 3.138902f,
				.misc_cmd = LED_OFF,
				.reverse_dir = 1,
				.negate_aenc = 0
		}
};


float wrap(float in)
{
	return fmod_2pi(in + PI) - PI;
}

/*
 * Performs misc mode commands. operates on a single motor_t variable, pass by reference.
 */
int motor_t_comm_misc(motor_t * chain)
{
	can_tx_header.StdId = 0x7FF - chain->id;
	chain->mtn16.d[0]=chain->misc_cmd;
	HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, chain->mtn16.d, &can_tx_mailbox);

	if(chain->misc_cmd == EN_UART_ENC || chain->misc_cmd == DIS_UART_ENC)
		chain->encoder_mode = chain->misc_cmd;
	if(chain->misc_cmd == SET_FOC_MODE || chain->misc_cmd == SET_SINUSOIDAL_MODE || chain->misc_cmd == SET_PCTL_IQ_MODE || chain->misc_cmd == SET_PCTL_VQ_MODE)
		chain->control_mode = chain->misc_cmd;

	for(uint32_t exp_ts = HAL_GetTick()+1; HAL_GetTick() < exp_ts;)
	{
		if(HAL_CAN_IsTxMessagePending(&hcan1,can_tx_mailbox) == 0)
		{
			break;
		}
	}
	for(uint32_t exp_ts = HAL_GetTick()+10;  HAL_GetTick() < exp_ts;)
	{
		if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) >= 1)
		{
			if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) != HAL_OK)
			{
				if(can_rx_header.StdId == can_tx_header.StdId)
				{
					return 1;
				}
			}
			break;
		}
	}
	return 0;
}


/*This is based on the fixed "wrap_2pi_12b" function, except that it
 * wraps fixed point numbers of arbitrary scaling/ reference points for 2pi.
 *
 * This can be used, after a fashion, to unwrap timestamps coming in from a
 * timer that wraps, as long as the timer wrap value is smaller than 32bits
 * range.
 */
int32_t wrap_fixed(int32_t in, uint32_t k)
{
	uint32_t half_k = k >> 1;	//it is necessary to get half value

	int32_t result = ((in + half_k) % k) - half_k;
	if (in < -half_k)
		return k + result;
	else
		return result;
}

/*
 * return time in us from our us timer, which has a large period register/arr value
 * we can use wrap_fixed with the period register value to prevent discontinuities,
 * assuming we sample the register faster than twice the overall timer period
 * */
uint32_t get_ts_us(void)
{
	return TIM2->CNT;
}
static int count = 0;
static int vdiv = 1;

void update_motor_t_from_can_data(can_payload_t * payload, motor_t * j)
{
	if(j->encoder_mode == EN_UART_ENC)
	{
		j->q16 = payload->i16[0];
		j->dq_rotor16 = payload->i16[1];

		if(count >= vdiv)
		{
			uint32_t t_elapsed = get_ts_us();
			TIM2->CNT = 0;	//you have 35 seconds until this wraps around
			j->ts_dq = t_elapsed;

			int32_t wrapped_dif = wrap_2pi_12b( (int32_t)(j->q16 - j->prev_q16) );
			j->prev_q16 = j->q16;

			wrapped_dif = (wrapped_dif*1000000) >> 12;
			j->dq_output = ((float)wrapped_dif) / ((float)t_elapsed);
			count = 0;
		}
		count++;

		float q12b = (float)(j->q16);
		q12b *= 0.015625f;
		q12b *= 0.015625f;
		j->q = wrap(q12b - j->q_offset);



		{	//precompute sin and cos theta for kinematics here.	 TODO: benchmark the two options

			int32_t sth = sin_lookup(j->q16, 30);
			int32_t cth = cos_lookup(j->q16, 30);

			j->sin_q = sth >> 1;	//load sin_q and cos_q radix 29, for 64bit multiplication stability
			j->cos_q = cth >> 1;
		}

		//j->dq_rotor = (float)(j->dq_rotor16) * 0.062500f;	//dividing by 16 expresses velocity in units of rad/sec
	}
	else
	{
		j->q32_rotor = payload->i32[0];
		j->q = (float)(j->q32_rotor/4096.f);
	}


	//j->iq_meas = ((float)payload->i16[3])/4096.f;
	float iq12b = ((float)payload->i16[2]);
	iq12b *= 0.015625f;
	iq12b *= 0.015625f;
	j->iq_meas = iq12b;
}

volatile can_msg_record_t rx_list[NUM_MOTORS];
can_payload_t can_tx_list[NUM_MOTORS];

void load_can_list(void)
{
}

/*
 * Performs normal mode torque/position commands to motors. Operates on a list of motor_ts,
 * stored in the chain pointer.
 *
 * TODO: make nonblocking timeout, or just do a straight up interrupt version
 */
void motor_t_put(motor_t * j)
{
	can_tx_header.StdId = j->id;
	if( (j->control_mode == SET_PCTL_VQ_MODE || j->control_mode == SET_PCTL_IQ_MODE) && j->encoder_mode == DIS_UART_ENC)
	{
		can_tx_header.DLC = sizeof(int32_t);
	}
	else
	{
		can_tx_header.DLC = sizeof(int16_t);
	}
	HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, j->mtn16.d, &can_tx_mailbox);


}
void motor_put_wait(void)
{
	for(uint32_t exp_ts = HAL_GetTick()+1; HAL_GetTick() < exp_ts;)
	{
		if(HAL_CAN_IsTxMessagePending(&hcan1, can_tx_mailbox) == 0)
			break;
	}
}



/*Make sure all joints in the robot are in one continuous
 * array structure.*/
void motor_t_get(motor_t * gl_chain, int chain_size)
{
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) >= 1)
	{
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) == HAL_OK)
		{
			int jid = can_rx_header.StdId;
			for(int i = 0; i < chain_size; i++)
			{
				motor_t * j = &gl_chain[i];
				if(jid == j->id)
				{
					update_motor_t_from_can_data(&can_rx_data, j);
					j->responsive = 1;
				}
			}
		}
	}
}



/*
 * motor_t.c
 *
 *  Created on: Aug 21, 2021
 *      Author: Ocanath
 */
#include "joint.h"
#include "trig_fixed.h"

motor_t chain[NUM_motor_tS] =
{
		{						//1
				.id = 24,
				.mtn16 = {{0}},
				.ctl = {
						.kp = 9.f,
						.ki_div = 377.f,
						.x_pi = 0,
						.x_sat = 1.5f,
						.tau_sat = 0.85f
				},
				.misc_cmd = LED_OFF
		},
		{						//2
				.id = 25,
				.mtn16 = {{0}},
				.ctl = {
						.kp = 9.f,
						.ki_div = 377.f,
						.x_pi = 0,
						.x_sat = 1.5f,
						.tau_sat = 0.85f
				},
				.misc_cmd = LED_OFF
		},
		{						//0
				.id = 23,
				.mtn16 = {{0}},
				.misc_cmd = LED_OFF
		},
		{						//3
				.id = 26,
				.mtn16 = {{0}},
				.misc_cmd = LED_OFF
		},
		{						//4
				.id = 27,
				.mtn16 = {{0}},
				.misc_cmd = LED_OFF
		},
		{						//5
				.id = 28,
				.mtn16 = {{0}},
				.misc_cmd = LED_OFF
		},
		{						//7
				.id = 21,
				.mtn16 = {{0}},
				.misc_cmd = LED_OFF
		},
		{						//6
				.id = 20,
				.mtn16 = {{0}},
				.misc_cmd = LED_OFF
		},
		{						//7
				.id = 22,
				.mtn16 = {{0}},
				.misc_cmd = LED_OFF
		},
		{
			.id = 32
		},
		{
			.id = 33
		},
		{
			.id = 34
		},
		{
			.id = 36
		},
		{
			.id = 35
		},
		{
			.id = 37
		},
		{
			.id = 29
		},
		{
			.id = 30
		},
		{
			.id = 31
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

	for(uint32_t exp_ts = HAL_GetTick()+1; HAL_GetTick() < exp_ts;)
	{
		if(HAL_CAN_IsTxMessagePending(&hcan1,can_tx_mailbox) == 0)
		{
			if(chain->misc_cmd == EN_UART_ENC || chain->misc_cmd == DIS_UART_ENC)
				chain->encoder_mode = chain->misc_cmd;
			if(chain->misc_cmd == SET_FOC_MODE || chain->misc_cmd == SET_SINUSOIDAL_MODE || chain->misc_cmd == SET_PCTL_IQ_MODE || chain->misc_cmd == SET_PCTL_VQ_MODE)
				chain->control_mode = chain->misc_cmd;
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



/*
 * Performs normal mode torque/position commands to motors. Operates on a list of motor_ts,
 * stored in the chain pointer.
 *
 * TODO: make nonblocking timeout, or just do a straight up interrupt version
 */
int motor_t_comm(motor_t * j)
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

	for(uint32_t exp_ts = HAL_GetTick()+1; HAL_GetTick() < exp_ts;)
	{
		if(HAL_CAN_IsTxMessagePending(&hcan1, can_tx_mailbox) == 0)
			break;
	}

	int node_responsive = 0;
	int wrong_node = 0;
	int timed_out = 1;
	for(uint32_t exp_ts = HAL_GetTick()+10;  HAL_GetTick() < exp_ts;)
	{
		if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) >= 1)
		{
			if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) == HAL_OK)
			{
				timed_out = 0;
				exp_ts = 0;
				if(can_rx_header.StdId == j->id)
				{
					node_responsive = 1;
					update_motor_t_from_can_data(&can_rx_data, j);
				}
				else
				{
					wrong_node = 1;
				}
			}
		}
	}
	j->responsive = node_responsive;
	return (node_responsive) | (timed_out << 1) | (wrong_node << 2);	//return msg of 1 is all good.
}

void chain_comm(motor_t * chain, int num_motor_ts)
{
	for(int i = 0; i < num_motor_ts; i++)
	{
		int rc = motor_t_comm(&chain[i]);
//		if(rc != 1)
//		{
//			uint8_t timed_out = (rc & (1 << 1)) >> 1;
//			if(chain[i].responsive == 0 && timed_out == 0)	//means a different node responded
//			{
//				for(int j = 0; j < num_motor_ts; j++)
//				{
//					int lki = (j + i) % num_motor_ts;
//					motor_t * pj = &chain[lki];
//					if(can_rx_header.StdId == pj->id)
//					{
//						update_motor_t_from_can_data(&can_rx_data,pj);
//					}
//				}
//			}
//		}
	}
}



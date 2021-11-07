/*
 * joint.c
 *
 *  Created on: Aug 21, 2021
 *      Author: Ocanath
 */
#include "joint.h"


joint chain[NUM_JOINTS] = {
//		{						//0
//				.id = 23,
//				.frame = 1,
//				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.q = 0,
//				.q_offset = -325.32539f*DEG_TO_RAD,
//				.tau = {.f32 = {0.f, 0.f} },
//				.qd = 0.f,
//				.misc_cmd = LED_OFF
//		},
//		{						//1
//				.id = 24,
//				.frame = 1,
//				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.q = 0,
//				.q_offset = -8.8f*DEG_TO_RAD,
//				.tau = {.f32 = {0.f, 0.f} },
//				.qd = 0.f,
//				.misc_cmd = LED_OFF
//		},
//		{						//2
//				.id = 25,
//				.frame = 1,
//				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.q = 0,
//				.q_offset = 27.275f*DEG_TO_RAD,
//				.tau = {.f32 = {0.f, 0.f} },
//				.qd = 0.f,
//				.misc_cmd = LED_OFF
//		},
//		{						//3
//				.id = 26,
//				.frame = 1,
//				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.q = 0,
//				.q_offset = -113.113f*DEG_TO_RAD,
//				.tau = {.f32 = {0.f, 0.f} },
//				.qd = 0.f,
//				.misc_cmd = LED_OFF
//		},
//		{						//4
//				.id = 27,
//				.frame = 1,
//				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.q = 0,
//				.q_offset = 62.621f*DEG_TO_RAD,
//				.tau = {.f32 = {0.f, 0.f} },
//				.qd = 0.f,
//				.misc_cmd = LED_OFF
//		},
//		{						//5
//				.id = 28,
//				.frame = 1,
//				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.q = 0,
//				.q_offset = -179.179f*DEG_TO_RAD,
//				.tau = {.f32 = {0.f, 0.f} },
//				.qd = 0.,
//				.misc_cmd = LED_OFF
//		},
//		{						//6
//				.id = 20,
//				.frame = 2,
//				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.q = 0,
//				.q_offset = 49.49*DEG_TO_RAD,
//				.tau = {.f32 = {0.f, 0.f} },
//				.qd = 0.f,
//				.misc_cmd = LED_OFF
//		},
//		{						//7
//				.id = 21,
//				.frame = 1,
//				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
//				.q = 0,
//				.q_offset = -27.27*DEG_TO_RAD,
//				.tau = {.f32 = {0.f, 0.f} },
//				.qd = 0.f,
//				.misc_cmd = LED_OFF
//		},
		{						//8
				.id = 32,
				.frame = 3,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = -275.275*DEG_TO_RAD,
				.tau = {.f32 = {0.f, 0.f} },
				.qd = 0.f,
				.misc_cmd = LED_OFF
		}
};


float wrap(float in)
{
	return fmod_2pi(in + PI) - PI;
}

/*
 * Performs misc mode commands. operates on a single joint variable, pass by reference.
 */
void joint_comm_misc(joint * chain)
{
	can_tx_header.StdId = 0x7FF - chain->id;
	can_tx_data.d[1]=chain->misc_cmd;
	HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, can_tx_data.d, &can_tx_mailbox);

	for(uint32_t exp_ts = HAL_GetTick()+1; HAL_GetTick() < exp_ts;)
	{
		if(HAL_CAN_IsTxMessagePending(&hcan1,can_tx_mailbox) == 0)
			break;
	}
	for(uint32_t exp_ts = HAL_GetTick()+10;  HAL_GetTick() < exp_ts;)
	{
		if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) >= 1)
		{
			if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) != HAL_OK)
			{}
			break;
		}
	}
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

void update_joint_from_can_data(can_payload_t * payload, joint * j)
{
	j->q32 = payload->i32[0];
	float q12b = ((float)(payload->i32[0]));

	if(count >= vdiv)
	{
		uint32_t t_elapsed = get_ts_us();
		TIM2->CNT = 0;	//you have 35 seconds until this wraps around
		j->ts_dq = t_elapsed;

		//1e6/4096 = 244.140625.
		j->dq = ((q12b - j->prev_q)*244.140625f) / ((float)t_elapsed);
		j->prev_q = q12b;

		count = 0;
	}
	count++;

	//j->iq_meas = ((float)payload->i16[3])/4096.f;
	float iq12b = ((float)payload->i16[2]);
	iq12b *= 0.015625f;
	iq12b *= 0.015625f;
	j->iq_meas = iq12b;

//	j->q = q12b/4096.f;
	q12b *= 0.015625f;
	q12b *= 0.015625f;
	j->q = wrap(q12b - j->q_offset);
}

/*
 * Performs normal mode torque/position commands to motors. Operates on a list of joints,
 * stored in the chain pointer.
 */
void joint_comm_motor(joint * chain, int num_joints)
{
	for(int i = 0; i < num_joints; i++)
	{
		can_tx_header.StdId = chain[i].id;
		HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, chain[i].tau.d, &can_tx_mailbox);

		for(uint32_t exp_ts = HAL_GetTick()+1; HAL_GetTick() < exp_ts;)
		{
			if(HAL_CAN_IsTxMessagePending(&hcan1,can_tx_mailbox) == 0)
				break;
		}

		for(uint32_t exp_ts = HAL_GetTick()+10;  HAL_GetTick() < exp_ts;)
		{
			if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) >= 1)
			{
				if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) == HAL_OK)
				{
					if(can_rx_header.StdId == chain[i].id)
					{
						update_joint_from_can_data(&can_rx_data, &chain[i]);
					}
					else
					{
						for(int sb = 0; sb < num_joints; sb++)	//sb = search base
						{
							int sidx = (sb + i) % num_joints;
							if(can_rx_header.StdId == chain[sidx].id)
							{
								update_joint_from_can_data(&can_rx_data, &chain[sidx]);
							}
						}
					}
				}
				break;
			}
		}
	}
}


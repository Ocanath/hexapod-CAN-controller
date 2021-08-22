/*
 * joint.c
 *
 *  Created on: Aug 21, 2021
 *      Author: Ocanath
 */
#include "joint.h"


joint chain[NUM_JOINTS] = {
		{						//0
				.id = 23,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = -325.32539f*DEG_TO_RAD,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{						//1
				.id = 24,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = -11.1128f*DEG_TO_RAD,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{						//2
				.id = 25,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = 27.275f*DEG_TO_RAD,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{						//3
				.id = 26,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = -57.574f*DEG_TO_RAD,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{						//4
				.id = 27,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = -154.154f*DEG_TO_RAD,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{						//5
				.id = 28,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = -179.179f*DEG_TO_RAD,
				.tau = {.v = 0.f},
				.qd = 0.,
				.misc_cmd = LED_OFF
		},
		{						//6
				.id = 20,
				.frame = 2,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = 49.49*DEG_TO_RAD,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{						//7
				.id = 21,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = -27.27*DEG_TO_RAD,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{						//8
				.id = 22,
				.frame = 3,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = -275.275*DEG_TO_RAD,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		}
};




/*
 * Performs misc mode commands. operates on a single joint variable, pass by reference.
 */
void joint_comm_misc(joint * chain)
{
	can_tx_header.StdId = 0x7FF - chain->id;
	can_tx_data.d[3]=chain->misc_cmd;
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
						chain[i].q = can_rx_data.v - chain[i].q_offset;
					else
					{
						for(int sb = 0; sb < num_joints; sb++)	//sb = search base
						{
							int sidx = (sb + i) % num_joints;
							if(can_rx_header.StdId == chain[sidx].id)
								chain[sidx].q = can_rx_data.v;
						}
					}
				}
				break;
			}
		}
	}
}


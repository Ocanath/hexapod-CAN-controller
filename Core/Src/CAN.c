/*
 * CAN.c
 *
 *  Created on: Oct 25, 2020
 *      Author: Ocanath Robotman
 *
 *      CAN Motor Protocol Breakdown:
 *
 *      For 'NORMAL OPERATION': (torque and position)
 *      	Message ID will be equal to the serial number of the motor (i.e. 23)
 *      	Master controller sends a floating point (ieee 32bit spec) number, parsed as a torque command
 *      	Upon receiving the matching message ID, the motor will put its current position on the bus
 *
 *      For 'EXTENDED COMMAND': (miscellaneous settings)
 *      	Message ID will be equal to MAX ID (0x7FF) minus the motor serial number (i.e. 0x7FF - 23)
 *      	The byte index 3 will contain a word that corresponds to setting or resetting a given binary setting
 *      	Upon receiving this message ID, the motor will also respond with its current position on the bus.
 *
 */
#include "CAN.h"

CAN_TxHeaderTypeDef		can_tx_header;
CAN_RxHeaderTypeDef		can_rx_header;
floatsend_t 			can_tx_data = {0x55555555};
floatsend_t 			can_rx_data = {0};
uint32_t				can_tx_mailbox;



void CAN_Init(void)
{
	CAN_FilterTypeDef  sFilterConfig;

	/*##-1- Configure the CAN peripheral #######################################*/
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 16;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;

	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	/*##-2- Configure the CAN Filter ###########################################*/
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;	//use id list
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;	//16 bit for idlist
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;	//deactivate incoming filter. we getting it all
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

//	/*##-4- Activate CAN RX notification #######################################*/
//	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
//	{
//		/* Notification Error */
//		Error_Handler();
//	}

	/*##-5- Configure Transmission process #####################################*/
	can_tx_header.StdId = 23;
	can_tx_header.ExtId = 0x00;
	can_tx_header.RTR = CAN_RTR_DATA;
	can_tx_header.IDE = CAN_ID_STD;
	can_tx_header.DLC = sizeof(float);	//4
	can_tx_header.TransmitGlobalTime = DISABLE;

	can_rx_header.StdId = 0x000;	//gets loaded by getmessage
	can_rx_header.ExtId = 0x00;
	can_rx_header.IDE = CAN_ID_STD;
	can_rx_header.RTR = CAN_RTR_DATA;
	can_rx_header.DLC = sizeof(float);	//4
	can_rx_header.Timestamp = 0;
	can_rx_header.FilterMatchIndex = 0;
}

/*
 * Performs misc mode commands. operates on a single joint variable, pass by reference.
 */
void CAN_comm_misc(joint * chain)
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
void CAN_comm_motor(joint * chain, int num_joints)
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


/*
 * CAN.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Ocanath Robotman
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_
#include "init.h"
#include "joint.h"

enum {
	LED_ON = 0xDE,
	LED_OFF =0xFE,
	LED_BLINK= 0xAA,
	EN_UART_ENC = 0x34,
	DIS_UART_ENC = 0x35
};

extern CAN_TxHeaderTypeDef   can_tx_header;
extern CAN_RxHeaderTypeDef   can_rx_header;
extern uint32_t				can_tx_mailbox;
extern floatsend_t 		can_tx_data;
extern floatsend_t 		can_rx_data;

void CAN_Init(void);
void CAN_comm_motor(joint * chain, int num_joints);
void CAN_comm_misc(joint * chain);

#endif /* INC_CAN_H_ */

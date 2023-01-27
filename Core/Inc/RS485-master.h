/*
 * RS485-master.h
 *
 *  Created on: Dec 13, 2021
 *      Author: Ocanath
 */

#ifndef INC_RS485_MASTER_H_
#define INC_RS485_MASTER_H_
#include "m_uart.h"

#define NETWORK_ID	0
#define NUM_TX_BYTES 2	//addr and checksum
#define NUM_RX_BYTES 4	//addr, 2 datas, checksum
#define NUM_SLAVES 3

typedef union
{
	int16_t v;
	uint8_t d[sizeof(int16_t)];
}int16_fmt_t;

typedef union {
	int8_t d8[sizeof(uint32_t)/sizeof(int8_t)];
	uint8_t u8[sizeof(uint32_t)/sizeof(uint8_t)];
	uint16_t u16[sizeof(uint32_t)/sizeof(uint16_t)];
	int16_t i16[sizeof(uint32_t)/sizeof(int16_t)];
	uint32_t u32;
	int32_t i32;
	float f32;	//sizeof(float) == sizeof(uint32_t) on this system
}u32_fmt_t;

typedef struct slave_node_t
{
	int16_fmt_t theta;
	uint8_t id;
}slave_node_t;

extern volatile slave_node_t gl_rs485_nodes[NUM_SLAVES];
extern volatile uint8_t gl_uart_rx_kb_activity_flag;

void handle_RS485_master_blocking(uint32_t timeout);
void handle_RS485_master(uint32_t timeout);

#endif /* INC_RS485_MASTER_H_ */

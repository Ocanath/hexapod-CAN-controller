/*
 * RS485-master.c
 *
 *  Created on: Dec 13, 2021
 *      Author: Ocanath
 */
#include "RS485-master.h"

static volatile int16_fmt_t theta;
static volatile uint8_t uart_activity_flag = 0;
static uint8_t active_idx = 0;
static uint32_t tx_ts = 0;
static uint8_t transmit_allowed = 0;
static uint32_t response_exp_ts = 0;

volatile slave_node_t gl_rs485_nodes[NUM_SLAVES] = {
		{
				.theta = {0},
				.id = 3
		},
		{
				.theta = {0},
				.id = 1
		},
		{
				.theta = {0},
				.id = 2
		}
};


void m_uart_rx_cplt_callback(uart_it_t * h)
{
	int8_t sum = 0;
	for(int i = 0; i < h->bytes_received; i++)
		sum += h->rx_buf[i];
	if(sum == 0)
	{
		uint8_t id = h->rx_buf[0];

		volatile slave_node_t * n = &gl_rs485_nodes[active_idx];
		if(id == n->id)
		{
			n->theta.d[0] = h->rx_buf[1];
			n->theta.d[1] = h->rx_buf[2];
		}
		else
		{
			for(int i = 0; i < NUM_SLAVES; i++)
			{
				n = &gl_rs485_nodes[i];
				if(id == n->id)
				{
					n->theta.d[0] = h->rx_buf[1];
					n->theta.d[1] = h->rx_buf[2];
					break;
				}
			}
		}
		uart_activity_flag = 1;
	}
}

void handle_RS485_master_blocking(uint32_t timeout)
{
	uint32_t tick = HAL_GetTick();
	if(tick > tx_ts)
	{
		tx_ts = tick+10;
		for(int i = 0; i < NUM_SLAVES; i++)
		{
			active_idx = i;
			uint8_t buf[NUM_TX_BYTES];
			buf[0] = gl_rs485_nodes[active_idx].id;
			buf[1] = get_checksum(buf,1);
			uart_activity_flag = 0;
			m_uart_tx_start(&m_huart2, buf, NUM_TX_BYTES);

			uint32_t exp_ts = HAL_GetTick()+2;
			while(uart_activity_flag == 0 && HAL_GetTick() < exp_ts);
		}
	}
}

void handle_RS485_master(uint32_t timeout)
{
	uint32_t tick = HAL_GetTick();
	if(tick > tx_ts)
	{
		/*queue transmission of current active idx,
		 * mark transaction in progress with activity flag,
		 * assign timeout timestamp from the time we began transmitting the request
		 * */
		if(transmit_allowed)
		{
			volatile slave_node_t * n = &gl_rs485_nodes[active_idx];
			uint8_t buf[NUM_TX_BYTES];
			buf[0] = n->id;
			buf[1] = get_checksum(buf,1);	//prep the buffer

			uart_activity_flag = 0;
			response_exp_ts = tick + timeout;
			transmit_allowed = 0;	//set the flags

			m_uart_tx_start(&m_huart2, buf, NUM_TX_BYTES);	//queue the _IT driven transmit
		}

		/*If you timeout or get a response, setup a for new transaction*/
		if(tick >= response_exp_ts || uart_activity_flag != 0)
		{
			transmit_allowed = 1;
			active_idx++;
			if(active_idx >= NUM_SLAVES)
			{
				active_idx = 0;
				tx_ts = tick+10;	//on the last node, give a gap before the next transaction
			}
		}
	}
}

/*
 * m_uart.c
 *
 *  Created on: May 17, 2021
 *      Author: Ocanath
 */
#include "m_uart.h"

/* Flag to clear ALL uart-associated interrupt requests, without clobbering reserved bits
 * (1 << 20) | (1 << 17) | (1 << 12) | (1 << 11) | (1 << 9) | (1 << 8) | (1 << 7) | (1 << 6) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0)
 */
#define ICR_CLEAR_ALL	0x00121BDF

/*ISR bits*/
#define RXNE_BIT 	(1 << 5)	//rx not empty interrupt flag mask
#define TXE_BIT		(1 << 7)	//tx empty interrupt flag mask
#define IDLE_BIT	(1 << 4)	//idle interrupt flag mask

/*CR1 bits*/
#define TXEIE		(1 << 7)	//tx interrupt enable flag


/*Initialize a baremetal uart handler structure for UART 1*/
uart_it_t m_huart2 =
{
		.Instance = USART2,
		.bytes_received = 0,
		.bytes_to_send = 0,
		.rx_buf = {0},
		.tx_buf = 0,
		.rx_idx = 0,
		.tx_idx = 0
};

/**
 * Weak prototype for external implementation of rx frame complete callback.
 * Gets called whenever a frame finishes, with an IDLE detection.
 *
 * h will contain the number of bytes actually received in the frame.
  */
__weak void m_uart_rx_cplt_callback(uart_it_t * h)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(h);
}

/*
Generic hex checksum calculation.
For lack of a better place to put this, it'll go here.
 */
uint8_t get_checksum(uint8_t * arr, int size)
{

	int8_t checksum = 0;
	for (int i = 0; i < size; i++)
		checksum += (int8_t)arr[i];
	return -checksum;
}

/*
 * Baremetal uart handler.
 *
 * Note: may require timer to trigger based on rx activity, to reset if partial frame detected. Depends on the behavior of the IDLE interrupt in
 * edge cases.
 * */
void m_uart_it_handler(uart_it_t * h)
{

	uint32_t isrflags   = h->Instance->ISR;	//read interrupt status register

	uint16_t rdr = (uint16_t)h->Instance->RDR;	//read RDR, thus clearing the associated interrupt flag

	int rxne = (isrflags & RXNE_BIT) != 0;		//check if there's bytes in the queue
	int txe = (isrflags & TXE_BIT) != 0;		//check if the tx queue is ready to receive
	int idle = (isrflags & IDLE_BIT) != 0;		//check if the rx frame has ended (idle)

	if(rxne != 0)	//if there's stuff in the buffer
	{
		if(h->rx_idx < UART_IT_BUF_SIZE)
			h->rx_buf[h->rx_idx++] = rdr & 0x00FF;
	}
	if(idle != 0)	//if idle line is detected
	{
		h->bytes_received = h->rx_idx;
		h->rx_idx = 0;	//end of receive frame behavior
		m_uart_rx_cplt_callback(h);
	}

	if(h->bytes_to_send == 0 && txe != 0)
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,0);//manually write out DE

	if(txe != 0 && h->tx_idx < h->bytes_to_send)	//if the TDR register is empty and we still have bytes to send
	{
		h->Instance->TDR = h->tx_buf[h->tx_idx++];
	}
	else if (h->tx_idx >= h->bytes_to_send)	//currently ALWAYS writes to CR1 masking tx interrupts. This is something a guard against interrupt storms. Likely unnecessary; only needs to be written once
	{
		h->tx_idx = 0;
		h->bytes_to_send = 0;
		h->Instance->CR1 &= ~TXEIE;	//be sure to cancel tx interrupts if you don't want to tx, otherwise they'll trigger an interrupt storm
	}

	h->Instance->ICR |=  ICR_CLEAR_ALL;	//clear all remaining interrupt flags to avoid a storm
}

void m_uart_tx_start(uart_it_t * h, uint8_t * buf, int size)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,1);
	h->tx_idx = 0;
	h->bytes_to_send = size;
	h->tx_buf = buf;
	h->Instance->TDR = h->tx_buf[h->tx_idx++];
	h->Instance->CR1 |= TXEIE;
}

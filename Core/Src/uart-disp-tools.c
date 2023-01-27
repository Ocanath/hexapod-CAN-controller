/*
 * uart-disp-tools.c
 *
 *  Created on: Aug 29, 2020
 *      Author: Ocanath Robotman
 */
#include "uart-disp-tools.h"
#include "joint.h"
#include "m_uart.h"

void uart_tx_blocking(uint8_t * str, int len, uint32_t timeout)
{
	uint32_t timeout_ts = HAL_GetTick() + timeout;
	m_uart_tx_start(&m_huart2, str, len);
	while(m_huart2.bytes_to_send > 0 && HAL_GetTick() < timeout_ts);
}

char gl_print_str[64] = {0};
void print_string(const char * str)
{
	int strlen;
	for(strlen = 0; str[strlen] != 0; strlen++);
	uart_tx_blocking((uint8_t*)str, strlen, 10);
}
/*cheap helper function to avoid needing to use sprintf...*/
int32_t int_to_str(int32_t arg, uint8_t* buf, int16_t buf_len)
{
	if(arg == 0)
	{
		buf[0] = '0';
		return 1;
	}
	uint8_t flag = 0;
	if (arg < 0)
	{
		flag = 1;
		arg = -arg;
	}

	int32_t argcpy = arg;
	int32_t val_len = (flag == 0) ? 0 : 1;
	while (argcpy != 0)
	{
		argcpy /= 10;
		val_len++;
	}
	if (val_len > buf_len)
		return -1;

	int16_t i = val_len - 1;
	while (arg != 0)
	{
		if (i < 0 || i >= buf_len)
		{
			return -1;
		}
		buf[i--] = (arg % 10) + '0';
		arg /= 10;
	}
	if (flag && i >= 0 && i < buf_len)
	{
		buf[i--] = '-';
	}
	if (i == -1)		//gets subtracted on last character load
		return val_len;
	else
		return -1;
}
void print_int_as_str(int32_t in)
{
	uint8_t istr[12] = {0};	//only 11 characters are needed actually
	int_to_str(in, istr, 12);
	print_string((const char *)istr);
}

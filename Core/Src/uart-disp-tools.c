/*
 * uart-disp-tools.c
 *
 *  Created on: Aug 29, 2020
 *      Author: Ocanath Robotman
 */
#include "uart-disp-tools.h"

char gl_print_str[64] = {0};
void print_string(const char * str)
{
	int strlen;
	for(strlen = 0; str[strlen] != 0; strlen++);
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen, 10);
}
void print_float(float f)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)(&f), 4, 10);
}

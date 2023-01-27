/*
 * uart-disp-tools.h
 *
 *  Created on: Aug 29, 2020
 *      Author: Ocanath Robotman
 */

#ifndef UART_DISP_TOOLS_H_
#define UART_DISP_TOOLS_H_
#include "init.h"
#include <string.h>

extern char gl_print_str[64];
void print_string(const char * str);
void print_float(float f);
int32_t int_to_str(int32_t arg, uint8_t* buf, int16_t buf_len);

#endif /* UART_DISP_TOOLS_H_ */

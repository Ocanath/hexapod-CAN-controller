/*
 * init.h
 *
 *  Created on: Oct 27, 2020
 *      Author: Ocanath Robotman
 */

#ifndef INC_INIT_H_
#define INC_INIT_H_
#include "main.h"

#define MICROSECOND_TIMER_PERIOD 0xFFFFFFFF	//32 bit timer. 24 bits

extern ADC_HandleTypeDef hadc1;

extern CAN_HandleTypeDef hcan1;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_CAN1_Init(void);
void MX_USART2_UART_Init(void);
void MX_SPI1_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);

#endif /* INC_INIT_H_ */

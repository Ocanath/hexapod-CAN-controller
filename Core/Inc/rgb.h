/*
 * rgb.h
 *
 *  Created on: Nov 1, 2020
 *      Author: Ocanath Robotman
 */

#ifndef RGB_H_
#define RGB_H_
#include "init.h"

#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;
typedef struct rgb_t
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
}rgb_t;

void rgb_play(rgb_t rgb);

#endif /* RGB_H_ */

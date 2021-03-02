/*
 * rgb.c
 *
 *  Created on: Nov 1, 2020
 *      Author: Ocanath Robotman
 */
#include "rgb.h"

void rgb_play(rgb_t rgb)
{
	TIMER_UPDATE_DUTY(rgb.b, rgb.g, rgb.r);
}


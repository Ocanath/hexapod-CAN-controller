/*
 * hexapod_params.h
 *
 *  Created on: Nov 21, 2021
 *      Author: Ocanath Robotman
 */

#ifndef INC_HEXAPOD_PARAMS_H_
#define INC_HEXAPOD_PARAMS_H_
#include "kinematics.h"

#define NUM_LEGS 6

typedef struct dynamic_hex_t
{
	mat4_t hb_0[NUM_LEGS];
	joint * p_joint[NUM_LEGS];
}dynamic_hex_t;

extern dynamic_hex_t gl_hex;

void setup_dynamic_hex(dynamic_hex_t * robot);


#endif /* INC_HEXAPOD_PARAMS_H_ */

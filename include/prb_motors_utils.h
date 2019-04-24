/*
 * prb_motors_utils.h
 *
 *  Created on: 10 mar 2019
 *      Author: andrea
 */
#include <prb_common.h>
#include <pru_mylinuxdrone.h>

#ifndef PRB_MOTORS_UTILS_H_
#define PRB_MOTORS_UTILS_H_

#define PRB_MOTORS_FRONT_LEFT    3
#define PRB_MOTORS_FRONT_RIGHT   1
#define PRB_MOTORS_REAR_LEFT     2
#define PRB_MOTORS_REAR_RIGHT    4

/*
 * Motors macros
 * range pwm [3125, 6250]
 */
#define SCALE_MOTORS_FACTOR      204800 // (3.125*2^16)
#define SCALE_MOTORS_VALUE(V)    (3125 + FXPOINT16_MULTIPLY(SCALE_MOTORS_FACTOR, V))


void prb_motors_calculate(PrbMessageType* received_pru1_data_struct);
uint16_t* prb_motors_get_motors_target();


#endif /* PRB_MOTORS_UTILS_H_ */

/*
 * prb_pid.h
 *
 *  Created on: 10 mar 2019
 *      Author: andrea
 */
#include <prb_common.h>
#ifndef PRB_PID_H_
#define PRB_PID_H_

#define KE_FIXPOINT_BITS 8

/*
 * PID macros
 */
#define PID_YPR(YPR,MX)  LIMIT(FXPOINT20_MULTIPLY(LSB_FIXPOINT_20, ((FXPOINT8_MULTIPLY(KE[YPR], pru_rpmsg_ang_acc_target[YPR]) >> 2) + (FXPOINT8_MULTIPLY(KEI[YPR], pru_rpmsg_ang_accI_target[YPR]) >> 2) + (FXPOINT8_MULTIPLY(KED[YPR], pru_rpmsg_ang_accD_target[YPR]) >> 2))),MX,-MX)
#define LSB_FIXPOINT_20   16009 // (1/65.5)*2^20

void prb_pid_calculate();
int16_t prb_pid_get(uint8_t axis, uint8_t limit);
uint16_t prb_pid_get_throttle();
#endif /* PRB_PID_H_ */



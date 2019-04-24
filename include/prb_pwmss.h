/*
 * prb_pwmss.h
 *
 *  Created on: 10 mar 2019
 *      Author: andrea
 */

#ifndef PRB_PWMSS_H_
#define PRB_PWMSS_H_

#include <stdint.h>
#include <sys_pwmss.h>

uint8_t pru_pwmss_lib_Init(uint8_t pwmssDevice);
void pru_pwmss_lib_SetData(uint8_t pwmssDevice, uint16_t period, uint16_t duA, uint16_t duB);
void pru_pwmss_lib_SetDuty(uint8_t pwmssDevice, uint16_t duA, uint16_t duB);
void pru_pwmss_lib_SetPeriod(uint8_t pwmssDevice, uint16_t period);
void pru_pwmss_lib_Stop(uint8_t pwmssDevice);
void pru_pwmss_lib_Start(uint8_t pwmssDevice);
uint8_t pru_pwmss_lib_IsRunning(uint8_t pwmssDevice);


#endif /* PRB_PWMSS_H_ */

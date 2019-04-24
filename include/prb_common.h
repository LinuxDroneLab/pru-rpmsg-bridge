/*
 * prb_common.h
 *
 *  Created on: 09 mar 2019
 *      Author: andrea
 */
#include <stdint.h>

#ifndef PRB_COMMON_H_
#define PRB_COMMON_H_

#define PRU_RPMSG_X                0
#define PRU_RPMSG_Y                1
#define PRU_RPMSG_Z                2
#define PRU_RPMSG_YAW              0
#define PRU_RPMSG_PITCH            1
#define PRU_RPMSG_ROLL             2
#define PRU_RPMSG_M1               0
#define PRU_RPMSG_M2               1
#define PRU_RPMSG_M3               2
#define PRU_RPMSG_M4               3

#define ROUND100(X)                           (X < 0 ? (X-55)/100 : (X+55)/100)
#define MAX(A,B)                              (A > B ? A : B)
#define MIN(A,B)                              (A < B ? A : B)
#define LIMIT(V,MX,MN)                        (MAX((MN),MIN((V),(MX))))
#define FXPOINT20_MULTIPLY(F,V)               ((F * V) >> 20)
#define FXPOINT16_MULTIPLY(F,V)               ((F * V) >> 16)
#define FXPOINT8_MULTIPLY(F,V)                ((F * V) >> 8)




#endif /* PRB_COMMON_H_ */

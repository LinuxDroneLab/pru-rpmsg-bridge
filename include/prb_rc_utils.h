/*
 * prb_rc_utils.h
 *
 *  Created on: 10 mar 2019
 *      Author: andrea
 */
#include <prb_common.h>
#include <pru_mylinuxdrone.h>

#ifndef PRB_RC_UTILS_H_
#define PRB_RC_UTILS_H_

#define PRU_RPMSG_THROTTLE_CHAN    2
#define PRU_RPMSG_YAW_CHAN         4
#define PRU_RPMSG_PITCH_CHAN       3
#define PRU_RPMSG_ROLL_CHAN        1
#define PRU_RPMSG_AUX1_CHAN        6
#define PRU_RPMSG_AUX2_CHAN        5
#define PRU_RPMSG_AUX3_CHAN        7
#define PRU_RPMSG_AUX4_CHAN        8


#define PRU_RPMSG_RC_CENTER_DEADBAND    20
#define PRU_RPMSG_RC_LEFT_RANGE         0
#define PRU_RPMSG_RC_RIGHT_RANGE        1
#define PRU_RPMSG_RC_MIN_RANGE_POS      0
#define PRU_RPMSG_RC_CENTER_RANGE_POS   1
#define PRU_RPMSG_RC_MAX_RANGE_POS      2


#define MAX_SCALED_THROTTLE  32750
#define MAX_SCALED_YAW       3930
#define MAX_SCALED_PITCH     16375
#define MAX_SCALED_ROLL      16375


/*
 * RC Channels Macros
 */
#define GET_MAX_RANGE(CHAN)                   (pru_rpmsg_rc_conf[CHAN - 1][PRU_RPMSG_RC_MAX_RANGE_POS])
#define GET_MIN_RANGE(CHAN)                   (pru_rpmsg_rc_conf[CHAN - 1][PRU_RPMSG_RC_MIN_RANGE_POS])
#define GET_CENTER_RANGE(CHAN)                (pru_rpmsg_rc_conf[CHAN - 1][PRU_RPMSG_RC_CENTER_RANGE_POS])
#define GET_CENTER_LEFT_LIMIT(CHAN)           ((GET_CENTER_RANGE(CHAN) - PRU_RPMSG_RC_CENTER_DEADBAND))
#define GET_CENTER_RIGHT_LIMIT(CHAN)          ((GET_CENTER_RANGE(CHAN) + PRU_RPMSG_RC_CENTER_DEADBAND))
#define IS_RANGE_LEFT(V,CHAN)                 (V < GET_CENTER_LEFT_LIMIT(CHAN))
#define IS_RANGE_RIGHT(V,CHAN)                (V > GET_CENTER_RIGHT_LIMIT(CHAN))
#define IS_RANGE_CENTER(V,CHAN)               (!IS_RANGE_RIGHT(V,CHAN) && !IS_RANGE_LEFT(V,CHAN))
#define CENTRALIZE_LEFT_VALUE(V,CHAN)         (V - GET_CENTER_LEFT_LIMIT(CHAN))
#define CENTRALIZE_RIGHT_VALUE(V,CHAN)        (V - GET_CENTER_RIGHT_LIMIT(CHAN))
#define GET_LEFT_FACTOR(CHAN)                 (pru_rpmsg_rc_factors[CHAN - 1][PRU_RPMSG_RC_LEFT_RANGE])
#define GET_RIGHT_FACTOR(CHAN)                (pru_rpmsg_rc_factors[CHAN - 1][PRU_RPMSG_RC_RIGHT_RANGE])
#define SCALE_CHAN_LEFT_VALUE(V,CHAN)         (FXPOINT16_MULTIPLY(GET_LEFT_FACTOR(CHAN),CENTRALIZE_LEFT_VALUE(V, CHAN)))
#define SCALE_CHAN_RIGHT_VALUE(V,CHAN)        (FXPOINT16_MULTIPLY(GET_RIGHT_FACTOR(CHAN),CENTRALIZE_RIGHT_VALUE(V, CHAN)))
#define SCALE_CHAN_VALUE(V,CHAN)              IS_RANGE_CENTER(V, CHAN) ? 0 :  ( IS_RANGE_LEFT(V, CHAN) ?  SCALE_CHAN_LEFT_VALUE(V, CHAN) :  ( IS_RANGE_RIGHT(V, CHAN) ?  SCALE_CHAN_RIGHT_VALUE(V, CHAN): 0))
#define GET_SCALED_CHAN_VALUE(CHAN)           prb_rc_utils_scaled[CHAN - 1]
#define ASSIGN_SCALED_CHAN_VALUE(V,CHAN) {\
                V = LIMIT(V,GET_MAX_RANGE(CHAN),GET_MIN_RANGE(CHAN));\
                prb_rc_utils_scaled[CHAN - 1] = SCALE_CHAN_VALUE(V, CHAN);\
        }
#define SHIFT_THROTTLE()  (prb_rc_utils_get_scaled_chan_value(PRU_RPMSG_THROTTLE_CHAN) + MAX_SCALED_THROTTLE)


void prb_init_rc_factors();
void prb_rc_utils_scale(PrbMessageType* received_pru1_data_struct);
int16_t prb_rc_utils_get_scaled_chan_value(uint8_t channel);
uint16_t prb_rc_utils_get_shifted_throttle();

#endif /* PRB_RC_UTILS_H_ */

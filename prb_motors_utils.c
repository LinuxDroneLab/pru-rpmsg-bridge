/*
 * prb_motors_utils.c
 *
 *  Created on: 10 mar 2019
 *      Author: andrea
 */

#include <prb_motors_utils.h>
#include <prb_pid.h>

int32_t pru_rpmsg_temp = 0;
uint16_t pru_rpmsg_acc_motors_target[4] = { 0 };
int8_t pru_rpmsg_matrix_A[4][3] = { {1,1,-1},{1,-1,1},{-1,1,1},{-1,-1,-1} };

void prb_motors_calculate(PrbMessageType* received_pru1_data_struct) {

    // motors in [0,1000] (costs 1084 bytes)
    for(pru_rpmsg_temp = 0; pru_rpmsg_temp < 4; pru_rpmsg_temp++) {
        pru_rpmsg_acc_motors_target[pru_rpmsg_temp] = SCALE_MOTORS_VALUE(LIMIT(FXPOINT20_MULTIPLY(LSB_FIXPOINT_20, prb_pid_get_throttle())
               + pru_rpmsg_matrix_A[pru_rpmsg_temp][PRU_RPMSG_YAW]*prb_pid_get(PRU_RPMSG_YAW,60)
               + pru_rpmsg_matrix_A[pru_rpmsg_temp][PRU_RPMSG_PITCH]*prb_pid_get(PRU_RPMSG_PITCH,250)
               + pru_rpmsg_matrix_A[pru_rpmsg_temp][PRU_RPMSG_ROLL]*prb_pid_get(PRU_RPMSG_ROLL,250),1000,0));
        received_pru1_data_struct->motors_vect.m[pru_rpmsg_temp] = pru_rpmsg_acc_motors_target[pru_rpmsg_temp];
    }

}

uint16_t* prb_motors_get_motors_target() {
    return pru_rpmsg_acc_motors_target;
}

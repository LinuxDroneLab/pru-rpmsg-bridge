/*
 * prb_pid.c
 *
 *  Created on: 10 mar 2019
 *      Author: andrea
 */
#include <prb_pid.h>
#include <prb_gyro_utils.h>
#include <prb_rc_utils.h>



int16_t* pru_rpmsg_gyro_sample_prev;
int16_t pru_rpmsg_ang_acc_target_prev[3] = { 0 };
int16_t pru_rpmsg_ang_acc_target[3] = { 0 };
int32_t pru_rpmsg_ang_accI_target[3] = { 0 };
int16_t pru_rpmsg_ang_accD_target[3] = { 0 };


/*
    keRoll   = 3.22f; 3.22*2^8 = 824
    keDRoll  = 7.98f; 7.98*2^8 = 2043
    keIRoll  = 0.00f;

    kePitch   = 3.22f;
    keDPitch  = 7.98f;
    keIPitch  = 0.0f;

    keYaw   = 43.3; 43.3*2^8 = 11085
    keDYaw  = 12.4f;12.4*2^8 = 3174
    keIYaw  = 1.0f; 1.0*2^8 = 256
 *
 */
//uint16_t KE[3] = {11085, 824, 824}; // yaw, pitch,roll
//uint16_t KEI[3] = {256, 0, 0};
//uint16_t KED[3] = {3174, 2043, 2043};
uint16_t KE[3] = {256, 0, 0}; // yaw, pitch,roll
uint16_t KEI[3] = {0, 0, 0};
uint16_t KED[3] = {0, 0, 0};


/* 3(cw) 1(ccw)
     \  /
      \/
      /\
     /  \
   2(ccw)4(cw)
 */
// F3 e F4 girano in senso orario (CW)
// F1 + F3 - F2 - F4 = pitch
// F2 + F3 - F1 - F4 = roll
// F3 + F4 - F1 - F2 = yaw
// F1 + F2 + F3 + F4 = thrust
// +--+--+--+--+   +--+   +--+
// | 1| 1|-1|-1|   |F1|   |Y |
// +--+--+--+--+   +--+   +--+
// | 1|-1| 1|-1|   |F2|   |P |
// +--+--+--+--+ * +--+ = +--+
// |-1| 1| 1|-1|   |F3|   |R |
// +--+--+--+--+   +--+   +--+
// | 1| 1| 1| 1|   |F4|   |T |
// +--+--+--+--+   +--+   +--+
/*
 -->inv(M)
        +---+---+---+---+   +---+   +----+
        | 1 | 1 |-1 | 1 |   | Y |   | F1 |
        +---+---+---+---+   +---+   +----+
        | 1 |-1 | 1 | 1 |   | P |   | F2 |
 0.25 * +---+---+---+---+ * +---+ = +----+
        |-1 | 1 | 1 | 1 |   | R |   | F3 |
        +---+---+---+---+   +---+   +----+
        |-1 |-1 |-1 | 1 |   | T |   | F4 |
        +---+---+---+---+   +---+   +----+
 per riportare tutto in termini di accelerazione e dati giroscopio
 gamma = 0.25*r/LSB; r=1; T=thrust/4/gamma; dgz,y,x angular acc
         +---+---+---+---+   +-----+   +----+
         | 1 | 1 |-1 | 1 |   | dgz |   | a1 |
         +---+---+---+---+   +-----+   +----+
         | 1 |-1 | 1 | 1 |   | dgy |   | a2 |
 gamma * +---+---+---+---+ * +-----+ = +----+
         |-1 | 1 | 1 | 1 |   | dgx |   | a3 |
         +---+---+---+---+   +-----+   +----+
         |-1 |-1 |-1 | 1 |   |  T  |   | a4 |
         +---+---+---+---+   +-----+   +----+
 */
void prb_pid_calculate() {
    pru_rpmsg_gyro_sample_prev = prb_gyro_get_sample_prev();

    // calculate new values (costs 52 bytes)
    pru_rpmsg_ang_acc_target[PRU_RPMSG_YAW]   =  prb_rc_utils_get_scaled_chan_value(PRU_RPMSG_YAW_CHAN) - pru_rpmsg_gyro_sample_prev[PRU_RPMSG_Z];
    // nota: lo stick aumenta per diminuire il pitch (lo stick viene negato)
    pru_rpmsg_ang_acc_target[PRU_RPMSG_PITCH] =  -prb_rc_utils_get_scaled_chan_value(PRU_RPMSG_PITCH_CHAN) - pru_rpmsg_gyro_sample_prev[PRU_RPMSG_Y];
    pru_rpmsg_ang_acc_target[PRU_RPMSG_ROLL]  =  prb_rc_utils_get_scaled_chan_value(PRU_RPMSG_ROLL_CHAN) - pru_rpmsg_gyro_sample_prev[PRU_RPMSG_X];

    // gamma*inv(A)*(dw/dt)=a
    // calculate integration (costs 180 bytes)
    pru_rpmsg_ang_accI_target[PRU_RPMSG_YAW] = LIMIT(pru_rpmsg_ang_accI_target[PRU_RPMSG_YAW] + pru_rpmsg_ang_acc_target[PRU_RPMSG_YAW], MAX_SCALED_YAW, -MAX_SCALED_YAW);
    pru_rpmsg_ang_accI_target[PRU_RPMSG_PITCH] = LIMIT(pru_rpmsg_ang_accI_target[PRU_RPMSG_PITCH] + pru_rpmsg_ang_acc_target[PRU_RPMSG_PITCH], MAX_SCALED_PITCH, -MAX_SCALED_PITCH);
    pru_rpmsg_ang_accI_target[PRU_RPMSG_ROLL] = LIMIT(pru_rpmsg_ang_accI_target[PRU_RPMSG_ROLL] + pru_rpmsg_ang_acc_target[PRU_RPMSG_ROLL], MAX_SCALED_ROLL, -MAX_SCALED_ROLL);

    // calculate derivative (costs 24 bytes)
    pru_rpmsg_ang_accD_target[PRU_RPMSG_YAW] = pru_rpmsg_ang_acc_target[PRU_RPMSG_YAW] - pru_rpmsg_ang_acc_target_prev[PRU_RPMSG_YAW];
    pru_rpmsg_ang_accD_target[PRU_RPMSG_PITCH] = pru_rpmsg_ang_acc_target[PRU_RPMSG_PITCH] - pru_rpmsg_ang_acc_target_prev[PRU_RPMSG_PITCH];
    pru_rpmsg_ang_accD_target[PRU_RPMSG_ROLL] = pru_rpmsg_ang_acc_target[PRU_RPMSG_ROLL] - pru_rpmsg_ang_acc_target_prev[PRU_RPMSG_ROLL];

    // save prev values
    pru_rpmsg_ang_acc_target_prev[PRU_RPMSG_YAW]   =  pru_rpmsg_ang_acc_target[PRU_RPMSG_YAW];
    pru_rpmsg_ang_acc_target_prev[PRU_RPMSG_PITCH] =  pru_rpmsg_ang_acc_target[PRU_RPMSG_PITCH];
    pru_rpmsg_ang_acc_target_prev[PRU_RPMSG_ROLL]  =  pru_rpmsg_ang_acc_target[PRU_RPMSG_ROLL];
}

int16_t prb_pid_get(uint8_t axis, uint8_t limit) {
    return PID_YPR(axis,limit);
}

uint16_t prb_pid_get_throttle() {
    return prb_rc_utils_get_shifted_throttle();
}

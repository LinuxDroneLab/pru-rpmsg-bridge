/*
 * prb_rc_utils.c
 *
 *  Created on: 10 mar 2019
 *      Author: andrea
 */
#include <prb_rc_utils.h>

int16_t prb_rc_utils_scaled[8] = { 0 };
uint32_t pru_rpmsg_rc_factors[8][2] = { { 0 } }; // pru_rpmsg_rc_factors[i][0] = factor left;  pru_rpmsg_rc_factors[i][1] = factor right;
int16_t pru_rpmsg_rc_conf[8][3] = { { 1455, 2409, 3368 }, //ROLL
        { 1419, 2380, 3368 }, //THROTTLE
        { 1557, 2405, 3253 }, //PITCH
        { 1386, 2367, 3315 }, //YAW
        { 1384, 2375, 3366 }, //AUX2
        { 1384, 2376, 3368 }, //AUX1
        { 1400, 2400, 3400 }, //AUX3
        { 1400, 2400, 3400 }  //AUX4
};

int16_t prb_rc_utils_get_scaled_chan_value(uint8_t channel) {
    return GET_SCALED_CHAN_VALUE(channel);
}
uint16_t prb_rc_utils_get_shifted_throttle() {
    return SHIFT_THROTTLE();
}
void prb_init_rc_factors()
{
    // fixed point numbers (factor of 16 bit)

    // Throttle trunc(32750*2^16/(center - min))
    pru_rpmsg_rc_factors[PRU_RPMSG_THROTTLE_CHAN - 1][PRU_RPMSG_RC_LEFT_RANGE] =
            (2146304000)
                    / (pru_rpmsg_rc_conf[PRU_RPMSG_THROTTLE_CHAN - 1][PRU_RPMSG_RC_CENTER_RANGE_POS]
                            - (PRU_RPMSG_RC_CENTER_DEADBAND
                                    + pru_rpmsg_rc_conf[PRU_RPMSG_THROTTLE_CHAN
                                            - 1][PRU_RPMSG_RC_MIN_RANGE_POS]));
    pru_rpmsg_rc_factors[PRU_RPMSG_THROTTLE_CHAN - 1][PRU_RPMSG_RC_RIGHT_RANGE] =
            (2146304000)
                    / (pru_rpmsg_rc_conf[PRU_RPMSG_THROTTLE_CHAN - 1][PRU_RPMSG_RC_MAX_RANGE_POS]
                            - (PRU_RPMSG_RC_CENTER_DEADBAND
                                    + pru_rpmsg_rc_conf[PRU_RPMSG_THROTTLE_CHAN
                                            - 1][PRU_RPMSG_RC_CENTER_RANGE_POS]));

    // Yaw channel max 60 deg/sec, 60*LSB=3930, LSB=65,5 => trunc(3930*2^16/(center - min))
    pru_rpmsg_rc_factors[PRU_RPMSG_YAW_CHAN - 1][PRU_RPMSG_RC_LEFT_RANGE] =
            (257556480)
                    / (pru_rpmsg_rc_conf[PRU_RPMSG_YAW_CHAN - 1][PRU_RPMSG_RC_CENTER_RANGE_POS]
                            - (PRU_RPMSG_RC_CENTER_DEADBAND
                                    + pru_rpmsg_rc_conf[PRU_RPMSG_YAW_CHAN - 1][PRU_RPMSG_RC_MIN_RANGE_POS]));
    pru_rpmsg_rc_factors[PRU_RPMSG_YAW_CHAN - 1][PRU_RPMSG_RC_RIGHT_RANGE] =
            (257556480)
                    / (pru_rpmsg_rc_conf[PRU_RPMSG_YAW_CHAN - 1][PRU_RPMSG_RC_MAX_RANGE_POS]
                            - (PRU_RPMSG_RC_CENTER_DEADBAND
                                    + pru_rpmsg_rc_conf[PRU_RPMSG_YAW_CHAN - 1][PRU_RPMSG_RC_CENTER_RANGE_POS]));

    // Roll/Pitch channels max 250 deg/sec, 250*LSB=16375 => trunc(16375*2^16/(center - min))
    pru_rpmsg_rc_factors[PRU_RPMSG_ROLL_CHAN - 1][PRU_RPMSG_RC_LEFT_RANGE] =
            (1073152000)
                    / (pru_rpmsg_rc_conf[PRU_RPMSG_ROLL_CHAN - 1][PRU_RPMSG_RC_CENTER_RANGE_POS]
                            - (PRU_RPMSG_RC_CENTER_DEADBAND
                                    + pru_rpmsg_rc_conf[PRU_RPMSG_ROLL_CHAN - 1][PRU_RPMSG_RC_MIN_RANGE_POS]));
    pru_rpmsg_rc_factors[PRU_RPMSG_ROLL_CHAN - 1][PRU_RPMSG_RC_RIGHT_RANGE] =
            (1073152000)
                    / (pru_rpmsg_rc_conf[PRU_RPMSG_ROLL_CHAN - 1][PRU_RPMSG_RC_MAX_RANGE_POS]
                            - (PRU_RPMSG_RC_CENTER_DEADBAND
                                    + pru_rpmsg_rc_conf[PRU_RPMSG_ROLL_CHAN - 1][PRU_RPMSG_RC_CENTER_RANGE_POS]));
    pru_rpmsg_rc_factors[PRU_RPMSG_PITCH_CHAN - 1][PRU_RPMSG_RC_LEFT_RANGE] =
            (1073152000)
                    / (pru_rpmsg_rc_conf[PRU_RPMSG_PITCH_CHAN - 1][PRU_RPMSG_RC_CENTER_RANGE_POS]
                            - (PRU_RPMSG_RC_CENTER_DEADBAND
                                    + pru_rpmsg_rc_conf[PRU_RPMSG_PITCH_CHAN - 1][PRU_RPMSG_RC_MIN_RANGE_POS]));
    pru_rpmsg_rc_factors[PRU_RPMSG_PITCH_CHAN - 1][PRU_RPMSG_RC_RIGHT_RANGE] =
            (1073152000)
                    / (pru_rpmsg_rc_conf[PRU_RPMSG_PITCH_CHAN - 1][PRU_RPMSG_RC_MAX_RANGE_POS]
                            - (PRU_RPMSG_RC_CENTER_DEADBAND
                                    + pru_rpmsg_rc_conf[PRU_RPMSG_PITCH_CHAN - 1][PRU_RPMSG_RC_CENTER_RANGE_POS]));

    pru_rpmsg_rc_factors[PRU_RPMSG_AUX1_CHAN - 1][PRU_RPMSG_RC_LEFT_RANGE] =
            (1073152000)
                    / (pru_rpmsg_rc_conf[PRU_RPMSG_AUX1_CHAN - 1][PRU_RPMSG_RC_CENTER_RANGE_POS]
                            - (PRU_RPMSG_RC_CENTER_DEADBAND
                                    + pru_rpmsg_rc_conf[PRU_RPMSG_AUX1_CHAN
                                            - 1][PRU_RPMSG_RC_MIN_RANGE_POS]));
    pru_rpmsg_rc_factors[PRU_RPMSG_AUX1_CHAN - 1][PRU_RPMSG_RC_RIGHT_RANGE] =
            (1073152000)
                    / (pru_rpmsg_rc_conf[PRU_RPMSG_AUX1_CHAN - 1][PRU_RPMSG_RC_MAX_RANGE_POS]
                            - (PRU_RPMSG_RC_CENTER_DEADBAND
                                    + pru_rpmsg_rc_conf[PRU_RPMSG_AUX1_CHAN
                                            - 1][PRU_RPMSG_RC_CENTER_RANGE_POS]));

    pru_rpmsg_rc_factors[PRU_RPMSG_AUX2_CHAN - 1][PRU_RPMSG_RC_LEFT_RANGE] =
            (1073152000)
                    / (pru_rpmsg_rc_conf[PRU_RPMSG_AUX2_CHAN - 1][PRU_RPMSG_RC_CENTER_RANGE_POS]
                            - (PRU_RPMSG_RC_CENTER_DEADBAND
                                    + pru_rpmsg_rc_conf[PRU_RPMSG_AUX2_CHAN
                                            - 1][PRU_RPMSG_RC_MIN_RANGE_POS]));
    pru_rpmsg_rc_factors[PRU_RPMSG_AUX2_CHAN - 1][PRU_RPMSG_RC_RIGHT_RANGE] =
            (1073152000)
                    / (pru_rpmsg_rc_conf[PRU_RPMSG_AUX2_CHAN - 1][PRU_RPMSG_RC_MAX_RANGE_POS]
                            - (PRU_RPMSG_RC_CENTER_DEADBAND
                                    + pru_rpmsg_rc_conf[PRU_RPMSG_AUX2_CHAN
                                            - 1][PRU_RPMSG_RC_CENTER_RANGE_POS]));
}

void prb_rc_utils_scale(PrbMessageType* received_pru1_data_struct) {
    ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.throttle, PRU_RPMSG_THROTTLE_CHAN);
    ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.yaw, PRU_RPMSG_YAW_CHAN);
    ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.pitch, PRU_RPMSG_PITCH_CHAN);
    ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.roll, PRU_RPMSG_ROLL_CHAN);
    ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.aux1, PRU_RPMSG_AUX1_CHAN);
    ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.aux2, PRU_RPMSG_AUX2_CHAN);
    received_pru1_data_struct->rc.aux3 = 0;
    received_pru1_data_struct->rc.aux4 = 0;
}

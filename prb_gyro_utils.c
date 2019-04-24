/*
 * prb_gyro_calibration.c
 *
 *  Created on: 09 mar 2019
 *      Author: andrea
 */
#include <prb_gyro_utils.h>
uint8_t prb_gyro_calibration_result = 0;
int16_t pru_rpmsg_calibration_samples = 10000;
int32_t pru_rpmsg_sumGyro[3] = { 0 };
int16_t prb_gyro_calibration_offset[3] = { 0 };
int16_t pru_gyro_sample_prev[3] = { 0 };
int32_t prb_gyro_temp = 0;


uint8_t prb_is_gyro_calibration_active() {
    return (pru_rpmsg_calibration_samples > 0);
}
int16_t* prb_gyro_calibration_get_offsets() {
    return prb_gyro_calibration_offset;
}
int16_t* prb_gyro_get_sample_prev() {
    return pru_gyro_sample_prev;
}

void prb_gyro_calibration_sample(PrbMessageType* received_pru1_data_struct) {
        pru_rpmsg_calibration_samples--;
        pru_rpmsg_sumGyro[0] +=
                received_pru1_data_struct->mpu_accel_gyro.gx;
        pru_rpmsg_sumGyro[1] +=
                received_pru1_data_struct->mpu_accel_gyro.gy;
        pru_rpmsg_sumGyro[2] +=
                received_pru1_data_struct->mpu_accel_gyro.gz;
        if (prb_is_gyro_calibration_active() == 0)
        {
            prb_gyro_calibration_offset[0] = ROUND100(
                    pru_rpmsg_sumGyro[0] / 100);
            prb_gyro_calibration_offset[1] = ROUND100(
                    pru_rpmsg_sumGyro[1] / 100);
            prb_gyro_calibration_offset[2] = ROUND100(
                    pru_rpmsg_sumGyro[2] / 100);

            pru_gyro_sample_prev[0] =
                    received_pru1_data_struct->mpu_accel_gyro.gx
                            - prb_gyro_calibration_offset[0];
            pru_gyro_sample_prev[1] =
                    received_pru1_data_struct->mpu_accel_gyro.gy
                            - prb_gyro_calibration_offset[1];
            pru_gyro_sample_prev[2] =
                    received_pru1_data_struct->mpu_accel_gyro.gz
                            - prb_gyro_calibration_offset[2];


            pru_rpmsg_sumGyro[0] = 0;
            pru_rpmsg_sumGyro[1] = 0;
            pru_rpmsg_sumGyro[2] = 0;
        }
}

void prb_gyro_filter_sample(PrbMessageType* received_pru1_data_struct) {
    if(prb_is_gyro_calibration_active()) {
        prb_gyro_calibration_sample(received_pru1_data_struct);
    } else {
        // TODO: verificare se assegnabili direttamente su Gyro della IMU
        // Applico Offset
        received_pru1_data_struct->mpu_accel_gyro.gx -=
                prb_gyro_calibration_offset[0];
        received_pru1_data_struct->mpu_accel_gyro.gy -=
                prb_gyro_calibration_offset[1];
        received_pru1_data_struct->mpu_accel_gyro.gz -=
                prb_gyro_calibration_offset[2];

        // TODO: da rivedere. Filtro gyro
        prb_gyro_temp = 55
                * (received_pru1_data_struct->mpu_accel_gyro.gx
                        - pru_gyro_sample_prev[0]);
        received_pru1_data_struct->mpu_accel_gyro.gx =
                pru_gyro_sample_prev[0]
                        + ROUND100(prb_gyro_temp);
        prb_gyro_temp = 55
                * (received_pru1_data_struct->mpu_accel_gyro.gy
                        - pru_gyro_sample_prev[1]);
        received_pru1_data_struct->mpu_accel_gyro.gy =
                pru_gyro_sample_prev[1]
                        + ROUND100(prb_gyro_temp);
        prb_gyro_temp = 55
                * (received_pru1_data_struct->mpu_accel_gyro.gz
                        - pru_gyro_sample_prev[2]);
        received_pru1_data_struct->mpu_accel_gyro.gz =
                pru_gyro_sample_prev[2]
                        + ROUND100(prb_gyro_temp);
        pru_gyro_sample_prev[0] =
                received_pru1_data_struct->mpu_accel_gyro.gx;
        pru_gyro_sample_prev[1] =
                received_pru1_data_struct->mpu_accel_gyro.gy;
        pru_gyro_sample_prev[2] =
                received_pru1_data_struct->mpu_accel_gyro.gz;
    }
}

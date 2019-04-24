/*
 * prb_gyro_calibration.h
 *
 *  Created on: 09 mar 2019
 *      Author: andrea
 */

#include <prb_common.h>
#include <pru_mylinuxdrone.h>
#ifndef PRB_GYRO_CALIBRATION_H_
#define PRB_GYRO_CALIBRATION_H_

uint8_t prb_is_gyro_calibration_active();
void prb_gyro_calibration_sample(PrbMessageType* received_pru1_data_struct);
int16_t* prb_gyro_get_sample_prev();
int16_t* prb_gyro_calibration_get_offsets();
void prb_gyro_filter_sample(PrbMessageType* received_pru1_data_struct);

#endif /* PRB_GYRO_CALIBRATION_H_ */

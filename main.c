#include <stdint.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <pru_rpmsg.h>
#include "resource_table.h"

#define PRU_RPMSG_THROTTLE_CHAN    2
#define PRU_RPMSG_YAW_CHAN         4
#define PRU_RPMSG_PITCH_CHAN       3
#define PRU_RPMSG_ROLL_CHAN        1
#define PRU_RPMSG_AUX1_CHAN        6
#define PRU_RPMSG_AUX2_CHAN        5
#define PRU_RPMSG_AUX3_CHAN        7
#define PRU_RPMSG_AUX4_CHAN        8

#define VIRTIO_CONFIG_S_DRIVER_OK  4
#define ROUND100(X)                           (X < 0 ? (X-55)/100 : (X+55)/100)
#define MAX(A,B)                              (A > B ? A : B)
#define MIN(A,B)                              (A < B ? A : B)
#define LIMIT(V,MX,MN)                        (MAX(MN,MIN(V,MX)));
/*
 * RC Channel Macros
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
#define FXPOINT16_MULTIPLY(F,V)               ((F * V) >> 16)
#define SCALE_CHAN_LEFT_VALUE(V,CHAN)         (FXPOINT16_MULTIPLY(GET_LEFT_FACTOR(CHAN),CENTRALIZE_LEFT_VALUE(V, CHAN)))
#define SCALE_CHAN_RIGHT_VALUE(V,CHAN)        (FXPOINT16_MULTIPLY(GET_RIGHT_FACTOR(CHAN),CENTRALIZE_RIGHT_VALUE(V, CHAN)))
#define SCALE_CHAN_VALUE(V,CHAN)              IS_RANGE_CENTER(V, CHAN) ? 0 :  ( IS_RANGE_LEFT(V, CHAN) ?  SCALE_CHAN_LEFT_VALUE(V, CHAN) :  ( IS_RANGE_RIGHT(V, CHAN) ?  SCALE_CHAN_RIGHT_VALUE(V, CHAN): 0))
#define GET_SCALED_CHAN_VALUE(CHAN)           pru_rpmsg_rc[CHAN - 1]
#define ASSIGN_SCALED_CHAN_VALUE(V,CHAN) {\
                V = LIMIT(V,GET_MAX_RANGE(CHAN),GET_MIN_RANGE(CHAN));\
                pru_rpmsg_rc[CHAN - 1] = SCALE_CHAN_VALUE(V, CHAN);\
        }

volatile register uint32_t __R30;
volatile register uint32_t __R31;

struct pru_rpmsg_transport transport;
unsigned short src, dst, len;
unsigned char received_arm_data[sizeof(PrbMessageType)] = { '\0' };
unsigned char received_pru1_data[sizeof(PrbMessageType)] = { '\0' };
PrbMessageType* received_pru1_data_struct = (PrbMessageType*) received_pru1_data;
int32_t pru_rpmsg_temp = 0;

int counter32 = 0;

int16_t pru_rpmsg_discard_samples = 5000;
int16_t pru_rpmsg_calibration_samples = 10000;
int32_t pru_rpmsg_sumGyro[3] = { 0 };
int16_t pru_rpmsg_gyro_offset[3] = { 0 };
int16_t pru_rpmsg_gyro_sample_prev[3] = { 0 };

#define PRU_RPMSG_RC_CENTER_DEADBAND    20
#define PRU_RPMSG_RC_LEFT_RANGE         0
#define PRU_RPMSG_RC_RIGHT_RANGE        1
#define PRU_RPMSG_RC_MIN_RANGE_POS      0
#define PRU_RPMSG_RC_CENTER_RANGE_POS   1
#define PRU_RPMSG_RC_MAX_RANGE_POS      2
int16_t pru_rpmsg_rc_conf[8][3] = { { 1455, 2409, 3368 }, //ROLL
        { 1419, 2380, 3368 }, //THROTTLE
        { 1557, 2405, 3253 }, //PITCH
        { 1386, 2367, 3315 }, //YAW
        { 1384, 2375, 3366 }, //AUX2
        { 1384, 2376, 3368 }, //AUX1
        { 1400, 2400, 3400 }, //AUX3
        { 1400, 2400, 3400 }  //AUX4
};

int16_t pru_rpmsg_rc[8] = { 0 };
uint32_t pru_rpmsg_rc_factors[8][2] = { { 0 } }; // pru_rpmsg_rc_factors[i][0] = factor left;  pru_rpmsg_rc_factors[i][1] = factor right;

static void prb_init_rc_factors()
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

}
static void prb_init_buffers()
{
    for (counter32 = 0; counter32 < sizeof(PrbMessageType); counter32++)
    {
        received_arm_data[counter32] = '\0';
        received_pru1_data[counter32] = '\0';
    }
}

static uint8_t prb_init_rpmsg()
{
    volatile unsigned char *status;

    /* Clear the status of the registers that will be used in this programs
     * As they have been un-serviced in the last software tun
     */
    CT_INTC.SICR_bit.STS_CLR_IDX = INT_ARM_TO_P0;
    CT_INTC.SICR_bit.STS_CLR_IDX = INT_P1_TO_P0;

    /*
     * Register initialization
     *      Writing to an INTC register or CGF register can be done with
     *      the help of predefined structures. These structures are :
     *      CT_INTC and CT_CFG. Writing to these registers basically
     *      accesse the peripheral through constant-table and loads the
     *      indicated register with appropriate value.
     *
     * CT_CFG.SYSCFG_bit.STANDBY_INIT : the object is used to write data to
     * the SYSCFG register. Writing 0 to the STANDBY_INIT section of the
     * register opens up the OCP master port that is used for PRU to ARM
     * communication.
     *
     * CT_INTC.EISR_bit.EN_SET_IDX : the object is used to write data to the
     * EISR register. Writing an index number to the EN_SET_IDX section of the
     * register results in enabling of that interrupt.
     */
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;
    CT_INTC.EISR_bit.EN_SET_IDX = INT_P1_TO_P0;
    CT_INTC.EISR_bit.EN_SET_IDX = INT_P0_TO_P1;

    /* Make sure the Linux drivers are ready for RPMsg communication */
    /* this is another place where a hang could occur */
    status = &resourceTable.rpmsg_vdev.status;
    while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK))
        ;

    /* Initialize the RPMsg transport structure */
    /* this function is defined in rpmsg_pru.c.  It's sole purpose is to call pru_virtqueue_init twice (once for
     vring0 and once for vring1).  pru_virtqueue_init is defined in pru_virtqueue.c.  It's sole purpose is to
     call vring_init.  Not sure yet where that's defined, but it appears to be part of the pru_rpmsg iface.*/
    /* should probably test for RPMSG_SUCCESS.  If not, then the interface is not working and code should halt */
    pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0,
                   &resourceTable.rpmsg_vring1, INT_P0_TO_ARM, INT_ARM_TO_P0);

    /* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
    // In a real-time environment, rather than waiting forever, this can probably be run loop-after-loop
    // until success is achieved.  At that point, set a flag and then enable the send/receive functionality
    while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, RPMSG_CHAN_NAME,
    RPMSG_CHAN_DESC,
                             RPMSG_CHAN_PORT) != PRU_RPMSG_SUCCESS)
        ;
    return 0;
}

/**
 * main.c for pru0
 */
int main(void)
{
    // fill payload for received messages to null
    prb_init_buffers();

    /*
     * Init rpmsg and create pru-mylinuxdrone channel
     */
    prb_init_rpmsg();

    // Init factors for RC ranges
    prb_init_rc_factors();

    while (1)
    {
        // receive message from ARM
        // receive data from PRU1
        if (CT_INTC.SECR0_bit.ENA_STS_31_0 & (1 << INT_P1_TO_P0))
        {
            CT_INTC.SICR_bit.STS_CLR_IDX = INT_P1_TO_P0;
            // send data from PRU1 to ARM
            __xin(SP_BANK_1, 6, 0, received_pru1_data);
            switch (received_pru1_data_struct->message_type)
            {
            case MPU_DATA_MSG_TYPE:
            {
                if (pru_rpmsg_calibration_samples == 0)
                {
                    // TODO: verificare se assegnabili direttamente su Gyro della IMU
                    // Applico Offset
                    received_pru1_data_struct->mpu_accel_gyro.gx -=
                            pru_rpmsg_gyro_offset[0];
                    received_pru1_data_struct->mpu_accel_gyro.gy -=
                            pru_rpmsg_gyro_offset[1];
                    received_pru1_data_struct->mpu_accel_gyro.gz -=
                            pru_rpmsg_gyro_offset[2];

                    // TODO: da rivedere. Filtro gyro
                    pru_rpmsg_temp = 55
                            * (received_pru1_data_struct->mpu_accel_gyro.gx
                                    - pru_rpmsg_gyro_sample_prev[0]);
                    received_pru1_data_struct->mpu_accel_gyro.gx =
                            pru_rpmsg_gyro_sample_prev[0]
                                    + ROUND100(pru_rpmsg_temp);
                    pru_rpmsg_temp = 55
                            * (received_pru1_data_struct->mpu_accel_gyro.gy
                                    - pru_rpmsg_gyro_sample_prev[1]);
                    received_pru1_data_struct->mpu_accel_gyro.gy =
                            pru_rpmsg_gyro_sample_prev[1]
                                    + ROUND100(pru_rpmsg_temp);
                    pru_rpmsg_temp = 55
                            * (received_pru1_data_struct->mpu_accel_gyro.gz
                                    - pru_rpmsg_gyro_sample_prev[2]);
                    received_pru1_data_struct->mpu_accel_gyro.gz =
                            pru_rpmsg_gyro_sample_prev[2]
                                    + ROUND100(pru_rpmsg_temp);
                    pru_rpmsg_gyro_sample_prev[0] =
                            received_pru1_data_struct->mpu_accel_gyro.gx;
                    pru_rpmsg_gyro_sample_prev[1] =
                            received_pru1_data_struct->mpu_accel_gyro.gy;
                    pru_rpmsg_gyro_sample_prev[2] =
                            received_pru1_data_struct->mpu_accel_gyro.gz;

                }
                else if (pru_rpmsg_discard_samples > 0)
                {
                    pru_rpmsg_discard_samples--;
                }
                else if (pru_rpmsg_calibration_samples > 0)
                {
                    pru_rpmsg_calibration_samples--;
                    pru_rpmsg_sumGyro[0] +=
                            received_pru1_data_struct->mpu_accel_gyro.gx;
                    pru_rpmsg_sumGyro[1] +=
                            received_pru1_data_struct->mpu_accel_gyro.gy;
                    pru_rpmsg_sumGyro[2] +=
                            received_pru1_data_struct->mpu_accel_gyro.gz;
                    if (pru_rpmsg_calibration_samples == 0)
                    {
                        pru_rpmsg_gyro_offset[0] = ROUND100(
                                pru_rpmsg_sumGyro[0] / 100);
                        pru_rpmsg_gyro_offset[1] = ROUND100(
                                pru_rpmsg_sumGyro[1] / 100);
                        pru_rpmsg_gyro_offset[2] = ROUND100(
                                pru_rpmsg_sumGyro[2] / 100);
                        pru_rpmsg_gyro_sample_prev[0] =
                                received_pru1_data_struct->mpu_accel_gyro.gx
                                        - pru_rpmsg_gyro_offset[0];
                        pru_rpmsg_gyro_sample_prev[1] =
                                received_pru1_data_struct->mpu_accel_gyro.gy
                                        - pru_rpmsg_gyro_offset[1];
                        pru_rpmsg_gyro_sample_prev[2] =
                                received_pru1_data_struct->mpu_accel_gyro.gz
                                        - pru_rpmsg_gyro_offset[2];

                        pru_rpmsg_sumGyro[0] = 0;
                        pru_rpmsg_sumGyro[1] = 0;
                        pru_rpmsg_sumGyro[2] = 0;
                    }
                }
                break;
            } // end case MPU_DATA_MSG_TYPE
            case RC_DATA_MSG_TYPE:
            {
                received_pru1_data_struct->rc.throttle /= 100;
                received_pru1_data_struct->rc.yaw      /= 100;
                received_pru1_data_struct->rc.pitch    /= 100;
                received_pru1_data_struct->rc.roll     /= 100;
                received_pru1_data_struct->rc.aux1     /= 100;
                received_pru1_data_struct->rc.aux2     /= 100;

                ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.throttle, PRU_RPMSG_THROTTLE_CHAN);
                ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.yaw, PRU_RPMSG_YAW_CHAN);
                ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.pitch, PRU_RPMSG_PITCH_CHAN);
                ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.roll, PRU_RPMSG_ROLL_CHAN);
                ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.aux1, PRU_RPMSG_AUX1_CHAN);
                ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.aux2, PRU_RPMSG_AUX2_CHAN);
                received_pru1_data_struct->rc.aux3 = 0;
                received_pru1_data_struct->rc.aux4 = 0;

                // I want to send scaled data
                received_pru1_data_struct->rc.throttle = GET_SCALED_CHAN_VALUE(PRU_RPMSG_THROTTLE_CHAN);
                received_pru1_data_struct->rc.yaw = GET_SCALED_CHAN_VALUE(PRU_RPMSG_YAW_CHAN);
                received_pru1_data_struct->rc.pitch = GET_SCALED_CHAN_VALUE(PRU_RPMSG_PITCH_CHAN);
                received_pru1_data_struct->rc.roll = GET_SCALED_CHAN_VALUE(PRU_RPMSG_ROLL_CHAN);
                received_pru1_data_struct->rc.aux1 = GET_SCALED_CHAN_VALUE(PRU_RPMSG_AUX1_CHAN);
                received_pru1_data_struct->rc.aux2 = GET_SCALED_CHAN_VALUE(PRU_RPMSG_AUX2_CHAN);
                break;
            }
                // end case RC_DATA_MSG_TYPE
            } // end switch
            pru_rpmsg_send(&transport, dst, src, received_pru1_data,
                           sizeof(PrbMessageType));
        } // end if received message from P1
        else if (CT_INTC.SECR0_bit.ENA_STS_31_0 & (1 << INT_ARM_TO_P0))
        {
            CT_INTC.SICR_bit.STS_CLR_IDX = INT_ARM_TO_P0;
            if (pru_rpmsg_receive(&transport, &src, &dst, received_arm_data,
                                  &len) == PRU_RPMSG_SUCCESS)
            {
                // send data from ARM to PRU1
                __xout(SP_BANK_0, 3, 0, received_arm_data);
                // send interrupt to P1
                CT_INTC.SRSR0_bit.RAW_STS_31_0 |= (1 << INT_P0_TO_P1);
            }
        } // end if received message from ARM
    }
}

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

#define MAX_SCALED_THROTTLE  32750
#define MAX_SCALED_YAW       3930
#define MAX_SCALED_PITCH     16375
#define MAX_SCALED_ROLL      16375

#define VIRTIO_CONFIG_S_DRIVER_OK  4
#define ROUND100(X)                           (X < 0 ? (X-55)/100 : (X+55)/100)
#define MAX(A,B)                              (A > B ? A : B)
#define MIN(A,B)                              (A < B ? A : B)
#define LIMIT(V,MX,MN)                        (MAX((MN),MIN((V),(MX))))
#define FXPOINT20_MULTIPLY(F,V)               ((F * V) >> 20)
#define FXPOINT16_MULTIPLY(F,V)               ((F * V) >> 16)
#define FXPOINT8_MULTIPLY(F,V)                ((F * V) >> 8)
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
#define GET_SCALED_CHAN_VALUE(CHAN)           pru_rpmsg_rc[CHAN - 1]
#define ASSIGN_SCALED_CHAN_VALUE(V,CHAN) {\
                V = LIMIT(V,GET_MAX_RANGE(CHAN),GET_MIN_RANGE(CHAN));\
                pru_rpmsg_rc[CHAN - 1] = SCALE_CHAN_VALUE(V, CHAN);\
        }

/*
 * PID macros
 */
#define PID_YPR(YPR,MX)  LIMIT(FXPOINT20_MULTIPLY(LSB_FIXPOINT_20, ((FXPOINT8_MULTIPLY(KE[YPR], pru_rpmsg_ang_acc_target[YPR]) >> 2) + (FXPOINT8_MULTIPLY(KEI[YPR], pru_rpmsg_ang_accI_target[YPR]) >> 2) + (FXPOINT8_MULTIPLY(KED[YPR], pru_rpmsg_ang_accD_target[YPR]) >> 2))),MX,-MX)
#define SHIFT_THROTTLE()  (GET_SCALED_CHAN_VALUE(PRU_RPMSG_THROTTLE_CHAN) + MAX_SCALED_THROTTLE)
#define LSB_FIXPOINT_20   16009 // (1/65.5)*2^20

/*
 * Motors macros
 * range pwm [3125, 6250]
 */
#define SCALE_MOTORS_FACTOR      204800 // (3.125*2^16)
#define SCALE_MOTORS_VALUE(V)    (3125 + FXPOINT16_MULTIPLY(SCALE_MOTORS_FACTOR, V))

volatile register uint32_t __R30;
volatile register uint32_t __R31;

struct pru_rpmsg_transport transport;
unsigned short src, dst, len;
unsigned char received_arm_data[sizeof(PrbMessageType)] = { '\0' };
unsigned char received_pru1_data[sizeof(PrbMessageType)] = { '\0' };
PrbMessageType* received_pru1_data_struct = (PrbMessageType*) received_pru1_data;
int32_t pru_rpmsg_temp = 0;
// 0,015267372

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
int16_t pru_rpmsg_ang_acc_target_prev[3] = { 0 };
int16_t pru_rpmsg_ang_acc_target[3] = { 0 };
int32_t pru_rpmsg_ang_accI_target[3] = { 0 };
int16_t pru_rpmsg_ang_accD_target[3] = { 0 };
uint8_t pru_rpmsg_imu_data_changed_flag = 0;
uint8_t pru_rpmsg_rc_data_changed_flag = 0;
uint16_t pru_rpmsg_acc_motors_target[4] = { 0 };
uint8_t pru_rpmsg_acc_motors_changed_flag = 0;
int8_t pru_rpmsg_matrix_A[4][3] = { {1,1,-1},{1,-1,1},{-1,1,1},{-1,-1,-1} };
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
#define KE_FIXPOINT_BITS 8
//uint16_t KE[3] = {11085, 824, 824}; // yaw, pitch,roll
//uint16_t KEI[3] = {256, 0, 0};
//uint16_t KED[3] = {3174, 2043, 2043};
uint16_t KE[3] = {256, 0, 0}; // yaw, pitch,roll
uint16_t KEI[3] = {0, 0, 0};
uint16_t KED[3] = {0, 0, 0};

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
                // calibration costs 1000 bytes
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
                    pru_rpmsg_imu_data_changed_flag++;
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
            case RC_DATA_MSG_TYPE: // 1936 bytes di istruzioni
            {
                ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.throttle, PRU_RPMSG_THROTTLE_CHAN);
                ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.yaw, PRU_RPMSG_YAW_CHAN);
                ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.pitch, PRU_RPMSG_PITCH_CHAN);
                ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.roll, PRU_RPMSG_ROLL_CHAN);
                ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.aux1, PRU_RPMSG_AUX1_CHAN);
                ASSIGN_SCALED_CHAN_VALUE(received_pru1_data_struct->rc.aux2, PRU_RPMSG_AUX2_CHAN);
                received_pru1_data_struct->rc.aux3 = 0;
                received_pru1_data_struct->rc.aux4 = 0;

                pru_rpmsg_rc_data_changed_flag++;
                break;
            }
                // end case RC_DATA_MSG_TYPE
            } // end switch
            pru_rpmsg_send(&transport, dst, src, received_pru1_data,
                           sizeof(PrbMessageType));
        } // end if received message from P1
        // received message from ARM
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
        // PID costs 1476 bytes
        else if (pru_rpmsg_imu_data_changed_flag) {
            pru_rpmsg_imu_data_changed_flag = 0;
            pru_rpmsg_rc_data_changed_flag = 0;

            // calculate new values (costs 52 bytes)
            pru_rpmsg_ang_acc_target[PRU_RPMSG_YAW]   =  GET_SCALED_CHAN_VALUE(PRU_RPMSG_YAW_CHAN) - pru_rpmsg_gyro_sample_prev[PRU_RPMSG_Z];
            // nota: lo stick aumenta per diminuire il pitch (lo stick viene negato)
            pru_rpmsg_ang_acc_target[PRU_RPMSG_PITCH] =  -GET_SCALED_CHAN_VALUE(PRU_RPMSG_PITCH_CHAN) - pru_rpmsg_gyro_sample_prev[PRU_RPMSG_Y];
            pru_rpmsg_ang_acc_target[PRU_RPMSG_ROLL]  =  GET_SCALED_CHAN_VALUE(PRU_RPMSG_ROLL_CHAN) - pru_rpmsg_gyro_sample_prev[PRU_RPMSG_X];

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

            // motors in [0,1000] (costs 1084 bytes)
            for(pru_rpmsg_temp = 0; pru_rpmsg_temp < 4; pru_rpmsg_temp++) {
                pru_rpmsg_acc_motors_target[pru_rpmsg_temp] = SCALE_MOTORS_VALUE(LIMIT(FXPOINT20_MULTIPLY(LSB_FIXPOINT_20, SHIFT_THROTTLE())
                       + pru_rpmsg_matrix_A[pru_rpmsg_temp][PRU_RPMSG_YAW]*PID_YPR(PRU_RPMSG_YAW,60)
                       + pru_rpmsg_matrix_A[pru_rpmsg_temp][PRU_RPMSG_PITCH]*PID_YPR(PRU_RPMSG_PITCH,250)
                       + pru_rpmsg_matrix_A[pru_rpmsg_temp][PRU_RPMSG_ROLL]*PID_YPR(PRU_RPMSG_ROLL,250),1000,0));
                received_pru1_data_struct->motors_vect.m[pru_rpmsg_temp] = pru_rpmsg_acc_motors_target[pru_rpmsg_temp];
            }

            received_pru1_data_struct->message_type = MOTORS_DATA_MSG_TYPE;
            pru_rpmsg_send(&transport, dst, src, received_pru1_data,
                           sizeof(PrbMessageType));

            // TODO: verificare il calcolo

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


        }
    }
}

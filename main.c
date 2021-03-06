#include <prb_gyro_utils.h>
#include <prb_rc_utils.h>
#include <prb_motors_utils.h>
#include <prb_pid.h>
#include <prb_pwmss.h>
#include <stdint.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <pru_rpmsg.h>
#include "resource_table.h"

#define VIRTIO_CONFIG_S_DRIVER_OK  4
#define PWMSS_DEVICE_FRONT 0
#define PWMSS_DEVICE_REAR 2

volatile register uint32_t __R30;
volatile register uint32_t __R31;

struct pru_rpmsg_transport transport;
unsigned short src, dst, len;
unsigned char received_arm_data[sizeof(PrbMessageType)] = { '\0' };
unsigned char received_pru1_data[sizeof(PrbMessageType)] = { '\0' };
PrbMessageType* received_pru1_data_struct = (PrbMessageType*) received_pru1_data;

// 0,015267372

int counter32 = 0;
int16_t pru_rpmsg_discard_samples = 5000;
uint8_t pru_rpmsg_imu_data_changed_flag = 0;
uint8_t pru_rpmsg_rc_data_changed_flag = 0;
uint8_t pru_rpmsg_acc_motors_changed_flag = 0;
uint16_t* pru_rpmsg_motors_ptr;

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

    pru_pwmss_lib_Init(PWMSS_DEVICE_FRONT);
    pru_pwmss_lib_Init(PWMSS_DEVICE_REAR);

    pru_pwmss_lib_Start(PWMSS_DEVICE_FRONT);
    pru_pwmss_lib_Start(PWMSS_DEVICE_REAR);

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
                if (pru_rpmsg_discard_samples > 0)
                {
                    pru_rpmsg_discard_samples--;
                } else {
                    prb_gyro_filter_sample(received_pru1_data_struct);
                    if(prb_is_gyro_calibration_active() == 0) {
                        pru_rpmsg_imu_data_changed_flag = 1;
                    }
                }
                break;
            } // end case MPU_DATA_MSG_TYPE
            case RC_DATA_MSG_TYPE: // 1936 bytes di istruzioni
            {
                prb_rc_utils_scale(received_pru1_data_struct);
                pru_rpmsg_rc_data_changed_flag=1;
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
            prb_pid_calculate();
            prb_motors_calculate(received_pru1_data_struct);
            received_pru1_data_struct->message_type = MOTORS_DATA_MSG_TYPE;
            pru_rpmsg_motors_ptr = prb_motors_get_motors_target();

            pru_pwmss_lib_SetDuty(PWMSS_DEVICE_FRONT, pru_rpmsg_motors_ptr[PRB_MOTORS_FRONT_LEFT-1], pru_rpmsg_motors_ptr[PRB_MOTORS_FRONT_RIGHT-1]);
            pru_pwmss_lib_SetDuty(PWMSS_DEVICE_REAR, pru_rpmsg_motors_ptr[PRB_MOTORS_REAR_LEFT-1], pru_rpmsg_motors_ptr[PRB_MOTORS_REAR_RIGHT-1]);
            pru_rpmsg_send(&transport, dst, src, received_pru1_data, sizeof(PrbMessageType));

        }
    }
}

/*
 * Source Modified by Andrea Lambruschini < andrea.lambruschini@gmail.com >
 * Based on the examples distributed by TI
 * and from BeagleScope sources (by Zubeen Tolani < ZeekHuge - zeekhuge@gmail.com >)
 */
#ifndef _RSC_TABLE_PRU_H_
#define _RSC_TABLE_PRU_H_

#include <stddef.h> //needed for offset_of
#include <rsc_types.h>
#include "pru_virtio_ids.h"
#include <pru_mylinuxdrone.h>

/*
 * Sizes of the virtqueues (expressed in number of buffers supported,
 * and must be power of 2)
 */
#define PRU_RPMSG_VQ0_SIZE	16
#define PRU_RPMSG_VQ1_SIZE	16

/*
 * The feature bitmap for virtio rpmsg
 */
#define VIRTIO_RPMSG_F_NS	0		//name service notifications

/* This firmware supports name service notifications as one of its features */
#define RPMSG_PRU_C0_FEATURES	(1 << VIRTIO_RPMSG_F_NS)

/* Definition for unused interrupts */
#define HOST_UNUSED		        255

/* Interrupt mappings
 * CHNL_PRU0_TO_PRU1 : The INTC channel to which the INT_P0_to_P1 interrupt
 *              is be mapped.
 * HOST_PRU0_TO_PRU1 : The Host to which CHNL_PRU0_TO_PRU1 channel is
 *              mapped. PRU1 will be checking status of this host to check
 *              the occurence of INT_P0_to_P1 interrupt. It is recommended
 *              to map channel N to Host N.
 * CHNL_ARM_TO_PRU0 : The INTC channel to which the INT_ARM_to_P0 interrupt
 *              is be mapped.
 * HOST_ARM_TO_PRU0 : The Host to which CHNL_ARM_TO_PRU0 channel is
 *              mapped. PRU0 will be checking status of this host to check
 *              the occurence of INT_ARM_to_P0 interrupt. It is recommended
 *              to map channel N to Host N.
 * CHNL_PRU0_TO_ARM : The INTC channel to which the INT_P0_to_ARM interrupt
 *              is be mapped.
 * HOST_PRU0_TO_ARM : The Host to which CHNL_PRU0_TO_ARM channel is
 *              mapped. This is to generate INT_P0_to_ARM interrupt. The
 *              host is internally mapped to the ARM INTC, and hence generates
 *              an ARM interrupt.
 */
#define CHNL_PRU0_TO_PRU1       1
#define HOST_PRU0_TO_PRU1       1
#define CHNL_ARM_TO_PRU0        0
#define HOST_ARM_TO_PRU0        0
#define CHNL_PRU0_TO_ARM        2
#define HOST_PRU0_TO_ARM        2

#define INT_P0_TO_ARM           16
#define INT_ARM_TO_P0           17

/*
 * HOST0 and HOST1 check bits
 * HOST_ARM_TO_PRU0_CB : The check bit to check HOST_ARM_TO_PRU0 interrupt
 *              Since the value of HOST_ARM_TO_PRU0 is 0, HOST_ARM_TO_PRU0_CB
 *              has the value 30
 */
#define HOST_ARM_TO_PRU0_CB     30 // host channel 0

/* Mapping sysevts to a channel. Each pair contains a sysevt, channel. */
struct ch_map pru_intc_map[] = {{INT_P0_TO_ARM, CHNL_PRU0_TO_ARM},
                                {INT_ARM_TO_P0, CHNL_ARM_TO_PRU0}

};

 /* RPMsg channel details
  * Settings for the  RPMsg channel that is represented by PRU0
  *
  * RPMSG_CHAN_NAME : The name of the channel. Each channel has s
  * specific name, that decides the controlling driver on the
  * kernel side.
  *
  * RPMSG_CHAN_PORT : Port number of the rpmsg channel.
  *
  * RPMSG_CHAN_DESC : description of the rpmsg channel.
  */
 #define RPMSG_CHAN_NAME                 "pru-control"
 #define RPMSG_CHAN_PORT                 30
 #define RPMSG_CHAN_DESC                 "Channel 30"
 #define RPMSG_MPU_CHAN_NAME             "pru-imu"
 #define RPMSG_MPU_CHAN_PORT             32
 #define RPMSG_MPU_CHAN_DESC             "Channel 32"



struct my_resource_table {
        struct resource_table base;

        uint32_t offset[2]; /* Should match 'num' in actual definition */

        /* rpmsg vdev entry */
        struct fw_rsc_vdev rpmsg_vdev;
        struct fw_rsc_vdev_vring rpmsg_vring0;
        struct fw_rsc_vdev_vring rpmsg_vring1;

        /* intc definition */
        struct fw_rsc_custom pru_ints;
};


#pragma DATA_SECTION(resourceTable, ".resource_table")
#pragma RETAIN(resourceTable)
struct my_resource_table resourceTable = {
        1,      /* Resource table version: only version 1 is supported by the current driver */
        2,      /* number of entries in the table */
        0, 0,   /* reserved, must be zero */
        /* offsets to entries */
        {
                offsetof(struct my_resource_table, rpmsg_vdev),
                offsetof(struct my_resource_table, pru_ints),
        },

        /* rpmsg vdev entry */
        {
                (uint32_t)TYPE_VDEV,                    //type
                (uint32_t)VIRTIO_ID_RPMSG,              //id
                (uint32_t)0,                            //notifyid
                (uint32_t)RPMSG_PRU_C0_FEATURES,        //dfeatures
                (uint32_t)0,                            //gfeatures
                (uint32_t)0,                            //config_len
                (uint8_t)0,                             //status
                (uint8_t)2,                             //num_of_vrings, only two is supported
                { (uint8_t)0, (uint8_t)0 },             //reserved
                /* no config data */
        },
        /* the two vrings */
        {
                0,                      //da, will be populated by host, can't pass it in
                16,                     //align (bytes),
                PRU_RPMSG_VQ0_SIZE,     //num of descriptors
                0,                      //notifyid, will be populated, can't pass right now
                0                       //reserved
        },
        {
                0,                      //da, will be populated by host, can't pass it in
                16,                     //align (bytes),
                PRU_RPMSG_VQ1_SIZE,     //num of descriptors
                0,                      //notifyid, will be populated, can't pass right now
                0                       //reserved
        },

        {
                TYPE_CUSTOM, TYPE_PRU_INTS,
                sizeof(struct fw_rsc_custom_ints),
                { /* PRU_INTS version */
                        0x0000,
                        /* Channel-to-host mapping, 255 for unused */
                        CHNL_ARM_TO_PRU0, HOST_UNUSED, CHNL_PRU0_TO_ARM, HOST_UNUSED, HOST_UNUSED,
                        HOST_UNUSED, HOST_UNUSED, HOST_UNUSED, HOST_UNUSED, HOST_UNUSED,
                        /* Number of evts being mapped to channels */
                        (sizeof(pru_intc_map) / sizeof(struct ch_map)),
                        /* Pointer to the structure containing mapped events */
                        pru_intc_map,
                },
        },
};

#endif /* _RSC_TABLE_PRU_H_ */

/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

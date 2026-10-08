/*
 *   @file  mmw_link_config.h
 *
 *   @brief
 *      Header file for mmwavelink based structures
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

#ifndef MMW_LINK_CONFIG_H
#define MMW_LINK_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

/* SDK Control Layer Include files */
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/mmwave/include/mmwave_internal.h>

    /**
     * @brief
     *  Mmwave Link Master Control Block
     *
     * @details
     *  The structure is used to hold all the relevant information for the
     *  Mmwave Link.
     */
    typedef struct MmwaveLink_MCB
    {

        /**
         * @brief   Semaphore handle for the mmWave Link
         */
        SemaphoreP_Object linkSemaphore;

        /**
         * @brief   mmWave Link Spawning function
         */
        RL_P_OSI_SPAWN_ENTRY spawnFxn;

        /**
         * @brief   mmWave Link Spawning function table
         */
        RL_P_OSI_SPAWN_ENTRY spawnTable[MMWAVE_MAX_NUM_SPAWN_LIST];

        /**
         * @brief   mmWave Link Spawning function counter
         */
        volatile uint8_t spawnFxnCnt;

        /**
         * @brief   Status of the BSS:
         */
        volatile uint32_t bssStatus;

        /**
         * @brief   Counter which tracks of the number of times the spawn function was
         * overrun.
         */
        uint32_t spawnOverrun;
        /**
         * @brief   Handle to the CRC Channel
         */
        // CRC_Handle                  crcHandle;

        /**
         * @brief   Debug stats.
         */
        MMWave_debugStats debugStats;

    } MmwaveLink_MCB;

/**
 * @brief   Status flag to indicate the open state
 */
#define MMWAVE_STATUS_OPENED 1U

/**
 * @brief   Status flag to indicate the configured state
 */
#define MMWAVE_STATUS_CONFIGURED 2U


/**
 * @brief   Status flag to indicate the started state
 */
#define MMWAVE_STATUS_STARTED 8U

/**
 * @brief   Link Status flag used to indicate that the RF was initialized successfully
 */
#define MMWAVE_RF_INITIALIZED 1U

/**
 * @brief   Link Status flag used to indicate that the RF initialization failed.
 */
#define MMWAVE_RF_INIT_FAILED 2U

/**
 * @brief   Link Status flag used to indicate that the RF Calibration completed
 */
#define MMWAVE_RF_CALIBRATION_DONE 4U

/**
 * @brief   Link Status flag used to indicate that the Calibration failed
 */
#define MMWAVE_RF_CALIBRATION_FAILED 8U

/**
 * @brief   Link Status flag used to indicate that the ESM fault was detected
 */
#define MMWAVE_RF_ESM_FAULT 16U

/**
 * @brief   Link Status flag used to indicate that the CPU fault was detected
 */
#define MMWAVE_RF_CPU_FAULT 32U

/**
 * @brief   Link Status flag used to indicate that the Analog fault was detected
 */
#define MMWAVE_RF_ANALOG_FAULT 64U

/**
 * @brief   Link Status flag used to indicate that the mmwavelink detected protocol
 *          error in async report
 */
#define MMWAVE_LINK_ASYNC_EVENT_MISMATCH_ERROR 128U

/**
 * @brief   Link Status flag used to indicate that the mmwavelink detected internal
 *          error in async report
 */
#define MMWAVE_LINK_ASYNC_EVENT_INTERNAL_ERROR 256U

/**
 * @brief   Front End MSS Status flag used to indicate that MSS power up is done
 */
#define MMWAVE_FRONT_END_MSS_POWER_UP_DONE 1U
/**
 * @brief   Front End MSS Status flag used to indicate CPU fault
 */
#define MMWAVE_FRONT_END_MSS_CPU_FAULT 2U

/**
 * @brief   Front End MSS Status flag used to indicate RF power up done
 */
#define MMWAVE_FRONT_END_MSS_RF_POWER_UP_DONE 4U

/**
 * @brief   Front End MSS Status flag used to indicate ESM fault
 */
#define MMWAVE_FRONT_END_MSS_ESM_FAULT 8U

/**
 * @brief   Front End MSS Status flag used to indicate boot error
 */
#define MMWAVE_FRONT_END_MSS_BOOT_ERROR 16U

/**
 * @brief   Status flag to indicate the synchronization state
 */
#define MMWAVE_STATUS_SYNCHRONIZED 4U

#ifdef __cplusplus
}
#endif

#endif

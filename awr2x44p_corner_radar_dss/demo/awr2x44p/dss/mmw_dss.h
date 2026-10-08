/**
 *   @file  mmw_dss.h
 *
 *   @brief
 *      This is the main header file for the Millimeter Wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2020 - 2021 Texas Instruments, Inc.
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
#ifndef MMW_DSS_H
#define MMW_DSS_H

/* MCU + SDK Include Files*/
#include <drivers/mailbox.h>
#include <drivers/hwa.h>
#include <drivers/edma.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HeapP.h>
#include "FreeRTOS.h"
#include "task.h"

/* mmWave SDK Include Files*/
#include <ti/common/mmwave_error.h>

/* Demo Include Files*/
#include "../mmw_resDDM.h"
#include "../../datapath/dpc/objdethwaDDMA/objectdetection.h"
#include "../common/mmw_output.h"
#include "../common/mmw_common.h"

#include "../../utils/mmwdemo_adcconfig.h"
#include "../common/mmw_config.h"

/* This is used to resolve RL_MAX_SUBFRAMES */
#include <ti/control/mmwavelink/mmwavelink.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================================================================== */
    /*                           Macros & Typedefs                              */
    /* ======================================================================== */

    /**
     * @brief
     *  Millimeter Wave Demo Data Path Object
     *
     * @details
     *  The structure is used to hold all the relevant information for the
     *  Millimeter Wave demo's data path
     */
    typedef struct MmwDemo_DataPathObj_t
    {
        /*! @brief Handle to hardware accelerator driver. */
        HWA_Handle hwaHandle;

        /*! @brief dpm Handle */
        DPM_Handle objDetDpmHandle;

        /*! @brief dpc handle */
        DPM_DPCHandle objDetDpcHandle;

        /*! @brief   Handle of the EDMA driver. */
        EDMA_Handle edmaHandle[EDMA_NUM_CC];

        /*! @brief          Processing Stats */
        MmwDemo_output_message_stats subFrameStats[RL_MAX_SUBFRAMES];
    } MmwDemo_DataPathObj;

    /*!
     * @brief
     * Structure holds calibration save configuration used during sensor open.
     *
     * @details
     *  The structure holds calibration save configuration.
     */
    typedef struct MmwDemo_calibDataHeader_t
    {
        /*! @brief      Magic word for calibration data header */
        uint32_t magic;

        /*! @brief      Header length */
        uint32_t hdrLen;

        /*! @brief      mmwLink version */
        rlSwVersionParam_t linkVer;

        /*! @brief      RadarSS version */
        rlFwVersionParam_t radarSSVer;

        /*! @brief      Data length */
        uint32_t dataLen;

        /*! @brief      data padding to make sure calib data is 8 bytes aligned */
        uint32_t padding;
    } MmwDemo_calibDataHeader;

    /*!
     * @brief
     * Structure holds calibration save configuration used during sensor open.
     *
     * @details
     *  The structure holds calibration save configuration.
     */
    typedef struct MmwDemo_calibCfg_t
    {
        /*! @brief      Calibration data header for validation read from flash */
        MmwDemo_calibDataHeader calibDataHdr;

        /*! @brief      Size of Calibraton data size includng header */
        uint32_t sizeOfCalibDataStorage;

        /*! @brief      Enable/Disable calibration save process  */
        uint32_t saveEnable;

        /*! @brief      Enable/Disable calibration restore process  */
        uint32_t restoreEnable;

        /*! @brief      Flash Offset to restore the data from */
        uint32_t flashOffset;
    } MmwDemo_calibCfg;

    /*!
     * @brief
     * Structure holds calibration restore configuration used during sensor open.
     *
     * @details
     *  The structure holds calibration restore configuration.
     */
    typedef struct MmwDemo_calibData_t
    {
        /*! @brief      Calibration data header for validation read from flash */
        MmwDemo_calibDataHeader calibDataHdr;

        /*! @brief      Calibration data */
        rlCalibrationData_t calibData;

        /*! @brief      Phase shift Calibration data */
        rlPhShiftCalibrationData_t phaseShiftCalibData;

        /* Future: If more fields are added to this structure or RL definitions
            are changed, please add dummy padding bytes here if size of
            MmwDemo_calibData is not multiple of 8 bytes. */
    } MmwDemo_calibData;


    /**
     * @brief
     *  Millimeter Wave Demo MCB
     *
     * @details
     *  The structure is used to hold all the relevant information for the
     *  Millimeter Wave demo
     */
    typedef struct MmwDemo_DSS_MCB_t
    {
#ifndef MMWAVELINK_ON_MSS
        /*! @brief      Configuration which is used to execute the demo */
        MmwDemo_Cfg cfg;

        /*! @brief      Object Detection DPC subFrame configuration */
        MmwDemo_SubFrameCfg subFrameCfg[RL_MAX_SUBFRAMES];

        /*! @brief   Calibration cofiguration for save/restore */
        MmwDemo_calibCfg calibCfg;

#ifdef LVDS_STREAM
        /*! @brief   Handle of the EDMA driver, used for CBUFF */
        EDMA_Handle edmaHandle;
        /*! @brief   this structure is used to hold all the relevant information
             for the mmw demo LVDS stream*/
        MmwDemo_LVDSStream_MCB_t lvdsStream;
#endif

        /*! @brief  Number of empty subbands */
        uint8_t numEmptySubBands;

        /*! @brief antenna indices in increasing order of phase shift value. */
        uint8_t ddmPhaseShiftOrder[SYS_COMMON_NUM_TX_ANTENNAS];

        /*! @brief      Object Detection DPC common configuration */
        MmwDemo_DPC_ObjDet_CommonCfg objDetCommonCfg;

        /*! @brief      ADCBuf driver handle */
        ADCBuf_Handle adcBufHandle;

        /*! @brief   Rf frequency scale factor, = 2.7 for 60GHz device, = 3.6 for 76GHz device */
        double rfFreqScaleFactor;
#endif

        /*! @brief   Semaphore Object to signal DPM ioctl from DPM report function. */
        SemaphoreP_Object DPMioctlSemHandle;

        /*! @brief     DPM Handle */
        TaskHandle_t objDetDpmTaskHandle;

        /*! @brief     DPM task object */
        StaticTask_t objDetDpmTaskObj;

        /*! @brief     init Task Handle */
        TaskHandle_t initTaskHandle;

        /*! @brief     init Task object */
        StaticTask_t initTaskObj;

        /*! @brief   Semaphore Object to pend main task */
        SemaphoreP_Object demoInitTaskCompleteSemHandle;

        /**
         * @brief   This is the semaphore handle which is used to handle the messages exchanged
         * between the MSS/DSS and BSS.
         */
        SemaphoreP_Object linkSemHandle;

        /*! @brief     Data Path object */
        MmwDemo_DataPathObj dataPathObj;
    } MmwDemo_DSS_MCB;


    /**************************************************************************
     *************************** Extern Definitions ***************************
     **************************************************************************/
    extern void MmwDemo_dataPathOpen(MmwDemo_DataPathObj *obj);
    extern void MmwDemo_dataPathClose(MmwDemo_DataPathObj *obj);


    /* Sensor Management Module Exported API */
    extern void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line);
#define MmwDemo_debugAssert(expression) \
    { \
        DebugP_assert(expression); \
    }

#ifdef __cplusplus
}
#endif

#endif /* MMW_DSS_H */

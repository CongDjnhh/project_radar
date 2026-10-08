/**
 *   @file  dss_main.c
 *
 *   @brief
 *      This is the main file which implements the millimeter wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2020 - 2024 Texas Instruments, Inc.
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

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* MCU PLUS SDK Include Files */
#include <ti_drivers_config.h>
#include <ti_board_config.h>
#include <ti_drivers_open_close.h>
#include <ti_board_open_close.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>

/* mmWave SDK Include Files: */
#include <ti/common/syscommon.h>
#include <ti/common/mmwavesdk_version.h>
#include "datapath/dpc/objdethwaDDMA/objectdetection.h"
#include <ti/utils/mathutils/mathutils.h>

/* Demo Include Files */
#include "../common/mmw_config.h"
#include "dss_cm4/mmw_dss_cm4.h"


/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/
/*Size of calibration data*/ //check
//#define CALIB_DATA_SIZE  (0x500)

/*! L3 RAM buffer for object detection DPC */
uint8_t *gMmwL3 = (uint8_t *)((uint32_t)0x88000000);

#if defined(ENET_STREAM)
/*
 * 48 KB used for ENET Memory Pool
 * 30 KB is used for BSS section
 */
#define DSS_L3_U_SIZE (0x2EC800)
#else
/* 30 KB is used for BSS section */
#define DSS_L3_U_SIZE (0x2F8800)
#endif

/*! L2 RAM buffer for object detection DPC */
#define MMWDEMO_OBJDET_L2RAM_SIZE (96U * 1024U)

uint8_t gDPC_ObjDetL2Heap[MMWDEMO_OBJDET_L2RAM_SIZE] __attribute__((aligned(4096U), section(".dpc_l2Heap")));

/*! \brief Frame Start Hardware Interrupt Object */
HwiP_Object MMwDemo_FrameStartHwiObject;

/*! @brief   Semaphore Object to handle DPC state */
SemaphoreP_Object gDPCStateSemHandle;

uint8_t gMsgBuf[DPM_BUFFER_MAX_SIZE] = { 0 };

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_DSS_MCB gMmwDssCM4MCB;

/**
 * @brief
 *  Global Variable for DPM result buffer
 */
DPM_Buffer resultBuffer;

uint32_t gDPCState;


/**************************************************************************
 ******************* Millimeter Wave Demo Functions Prototype *******************
 **************************************************************************/
static void MmwDemo_dssInitTask(void *args);
void        MmwDemo_DPC_ObjectDetection_reportFxn(void *data, uint16_t dataLen);
static void MmwDemo_DPC_ObjectDetection_processFrameBeginCallBackFxn(uint8_t subFrameIndx);
static void MmwDemo_DPC_ObjectDetection_processInterFrameBeginCallBackFxn(uint8_t subFrameIndx);
static void MmwDemo_updateObjectDetStats(
    DPC_ObjectDetection_Stats    *currDpcStats,
    MmwDemo_output_message_stats *outputMsgStats);

static int32_t MmwDemo_copyResultToHSRAM(
    MmwDemo_HSRAM                     *ptrHsramBuffer,
    DPC_ObjectDetection_ExecuteResult *result,
    MmwDemo_output_message_stats      *outStats);
static void MmwDemo_DPC_ObjectDetection_dpmTask(void *args);
static void MmwDemo_sensorStopEpilog(void);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Epilog processing after sensor has stopped
 *
 *  @retval None
 */
static void MmwDemo_sensorStopEpilog(void)
{

    DebugP_log("Data Path Stopped (last frame processing done)\n");
}

void MmwDemo_DPC_ObjectDetection_reportFxn(void *data, uint16_t dataLen)
{
    memcpy((void *)gMsgBuf, (void *)data, dataLen);
    SemaphoreP_post(&gDPCStateSemHandle);
}

/**
 *  @b Description
 *  @n
 *      DPC Execution Finite State Machine (FSM).
 *      Following DPC States are valid and handled in this function:
 *      1. MMWDEMO_DPC_COMMONCFG:
 *          a. DPC waits for the commong cfg to be available from MSS.
 *          b. Store the commonCfg in DPC Object.
 *          c. Update the DPC State, and sends the ack back to MSS.
 *      2. MMWDEMO_DPC_CFG:
 *          a. DPC waits for the preStart cfg to be available from MSS.
 *          b. Configure the DPUs amd start the DPC.
 *          c. Update the DPC State, and sends the ack back to MSS.
 *      3. MMWDEMO_DPC_EXECUTE:
 *          a. DPU process functions are executed in this state.
 *          b. MSS is notified after the results of each frame/subrame are ready.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_DPC_ObjectDetection(void)
{
    int32_t  retVal;
    uint32_t msg = 0;

    while (1)
    {
        switch (gDPCState)
        {
            case MMWDEMO_DPC_COMMONCFG:
            {
                /* Wait for the common cfg to be available. */
                SemaphoreP_pend(&gDPCStateSemHandle, SystemP_WAIT_FOREVER);
                retVal = DPC_ObjectDetection_ioctl(gMmwDssCM4MCB.dataPathObj.objDetDpcHandle, DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG, gMsgBuf, sizeof(DPC_ObjectDetection_PreStartCommonCfg));
                if (retVal < 0)
                {
                    DebugP_log("DPC Error:%d\n", retVal);
                    DebugP_assert(0);
                }

                /* Update the sensor state */
                gDPCState = MMWDEMO_DPC_CFG;

                /* Send the acknowledgement */
                msg    = MMWDEMO_DPC_COMMONCFG;
                retVal = DPM_send(gMmwDssCM4MCB.dataPathObj.objDetDpmHandle, (void *)(&msg), sizeof(uint32_t), CSL_CORE_ID_C66SS0);
                DebugP_assert(0U == retVal);
                break;
            }

            case MMWDEMO_DPC_CFG:
            {
                /* Wait for the preStart Cfg to be available */
                SemaphoreP_pend(&gDPCStateSemHandle, SystemP_WAIT_FOREVER);
                retVal = DPC_ObjectDetection_ioctl(gMmwDssCM4MCB.dataPathObj.objDetDpcHandle,
                                                   DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG,
                                                   gMsgBuf,
                                                   sizeof(DPC_ObjectDetection_PreStartCfg));
                if (retVal < 0)
                {
                    DebugP_log("DPC Error:%d\n", retVal);
                    DebugP_assert(0);
                }

                /* config done, start the DPC */
                retVal = DPC_ObjectDetection_start(gMmwDssCM4MCB.dataPathObj.objDetDpcHandle);
                if (retVal < 0)
                {
                    DebugP_log("DPC Error:%d\n", retVal);
                    DebugP_assert(0);
                }

                /* Update the sensor state */
                gDPCState = MMWDEMO_DPC_EXECUTE;

                /* Send the acknowledgement */
                msg    = MMWDEMO_DPC_CFG;
                retVal = DPM_send(gMmwDssCM4MCB.dataPathObj.objDetDpmHandle, (void *)(gMsgBuf), sizeof(DPC_ObjectDetection_PreStartCfg), CSL_CORE_ID_C66SS0);
                DebugP_assert(0U == retVal);
                break;
            }

            case MMWDEMO_DPC_EXECUTE:
                /* Process the ADC Data and get the detected objects. */
                retVal = DPC_ObjectDetection_execute(gMmwDssCM4MCB.dataPathObj.objDetDpcHandle, &resultBuffer);
                if (retVal < 0)
                {
                    DebugP_log("DPC Error:%d\n", retVal);
                    DebugP_assert(0);
                }

                /* Notify the DSS that M4 Results are ready and it can start angle estimation. */
                msg    = MMWDEMO_DPC_RESULT;
                retVal = DPM_send(gMmwDssCM4MCB.dataPathObj.objDetDpmHandle, (void *)(&msg), sizeof(uint32_t), CSL_CORE_ID_C66SS0);
                DebugP_assert(0U == retVal);

                break;
            default:
                DebugP_assert(0);
        }
    }
}


/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main(void)
{
    int32_t                        errCode;
    DPM_InitCfg                    dpmInitCfg;
    DPC_ObjectDetection_InitParams objDetInitParams;
    HwiP_Params                    hwiPrms;

    /* init SOC specific modules */
    System_init();
    Board_init();

    /* Initialize the demo MCB */
    memset((void *)&gMmwDssCM4MCB, 0, sizeof(gMmwDssCM4MCB));

    CycleCounterP_reset();

    Drivers_open();
    /* Open the HWA Instance */
    gMmwDssCM4MCB.dataPathObj.hwaHandle = HWA_open(0, NULL, &errCode);
    if (gMmwDssCM4MCB.dataPathObj.hwaHandle == NULL)
    {
        DebugP_log("Error: Unable to open the HWA [Error: %d]\n", errCode);
        MmwDemo_debugAssert(0);
    }
    gMmwDssCM4MCB.dataPathObj.edmaHandle = gEdmaHandle[0];

    /* Initialization of the DPM Module: */

    memset((void *)&dpmInitCfg, 0, sizeof(DPM_InitCfg));
    /* Setup the configuration: */
    dpmInitCfg.localEndPt = gRemoteCoreEndPt[CSL_CORE_ID_M4SS0_1];
    dpmInitCfg.reportFxn  = MmwDemo_DPC_ObjectDetection_reportFxn;
    dpmInitCfg.setBitPos  = DPM_DSS_CM4_BOOT_INFO_BIT_POS;

    gMmwDssCM4MCB.dataPathObj.objDetDpmHandle = DPM_init(&dpmInitCfg, &errCode);
    if (gMmwDssCM4MCB.dataPathObj.objDetDpmHandle == NULL)
    {
        DebugP_log("Error: Unable to initialize the DPM Module [Error: %d]\n", errCode);
        MmwDemo_debugAssert(0);
        return errCode;
    }

    /* Initialization of the DPC Module */
    memset((void *)&objDetInitParams, 0, sizeof(DPC_ObjectDetection_InitParams));

    /* Note this must be after MmwDemo_dataPathOpen() above which opens the hwa */
    objDetInitParams.hwaHandle            = gMmwDssCM4MCB.dataPathObj.hwaHandle;
    objDetInitParams.L3ramCfg.addr        = (void *)gMmwL3;
    objDetInitParams.L3ramCfg.size        = DSS_L3_U_SIZE;
    objDetInitParams.CoreLocalRamCfg.addr = &gDPC_ObjDetL2Heap[0];
    objDetInitParams.CoreLocalRamCfg.size = sizeof(gDPC_ObjDetL2Heap);
    objDetInitParams.edmaHandle[0]        = gMmwDssCM4MCB.dataPathObj.edmaHandle;

    gMmwDssCM4MCB.dataPathObj.objDetDpcHandle = DPC_ObjectDetection_init(&objDetInitParams, &errCode);
    if (gMmwDssCM4MCB.dataPathObj.objDetDpcHandle == NULL)
    {
        DebugP_log("Error: Unable to initialize the DPC [Error: %d]\n", errCode);
        MmwDemo_debugAssert(0);
        return errCode;
    }

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = DPM_synch(gMmwDssCM4MCB.dataPathObj.objDetDpmHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the framework */
            DebugP_log("Error: DPM Synchronization failed [Error code %d]\n", errCode);
            MmwDemo_debugAssert(0);
            return errCode;
        }
        if (syncStatus == CSL_FMKR(DPM_DSS_BOOT_INFO_BIT_POS, DPM_MSS_BOOT_INFO_BIT_POS, 7))
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        ClockP_usleep(1 * 1000U);
    }

    /* Register Frame Start Interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = CSL_CM4_INTR_DFE_FRAME_START_TO_MSS;
    hwiPrms.callback = &DPC_ObjectDetection_frameStart;
    hwiPrms.args     = (void *)gMmwDssCM4MCB.dataPathObj.objDetDpcHandle;
    errCode          = HwiP_construct(&MMwDemo_FrameStartHwiObject, &hwiPrms);
    if (SystemP_SUCCESS != errCode)
    {
        DebugP_log("Error: Frame Start Interrupt Registration failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert(0);
        return errCode;
    }
    else
    {
        HwiP_enableInt((uint32_t)hwiPrms.intNum);
    }

    /* Create the Semaphore to wait in a particular Sensor State */
    SemaphoreP_constructBinary(&gDPCStateSemHandle, 0);

    /* Initial DPC state is to wait for common configurations from MSS. */
    gDPCState = MMWDEMO_DPC_COMMONCFG;

    /* Program should never return from this function call. */
    MmwDemo_DPC_ObjectDetection();

    return 0;
}

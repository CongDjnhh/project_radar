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
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include "FreeRTOS.h"
#include "task.h"

/* Sysconfig include files*/
#include "dssgenerated/ti_drivers_config.h"
#include "dssgenerated/ti_board_config.h"
#include "dssgenerated/ti_drivers_open_close.h"
#include "dssgenerated/ti_board_open_close.h"

/* mmWave SDK Include Files: */
#include <ti/common/syscommon.h>
#include <ti/common/mmwavesdk_version.h>
#include <ti/utils/mathutils/mathutils.h>

/* Demo Include Files */
#include "../../datapath/dpc/objdethwaDDMA/objectdetection.h"
#include "../common/mmw_config.h"
#include "mmw_dss.h"
#include "../common/mmw_common.h"

#include "../../utils/mmwdemo_rfparser.h"
#include "../../utils/mmwdemo_adcconfig.h"
#include "../../utils/mmwdemo_monitor.h"
#include "../common/chirp_design_HECR.h"
#include "../common/mmw_link_mon.h"


/* Task declarations */
#define MMWDEMO_DSS_INIT_TASK_PRI        (1U)
#define MMWDEMO_DSS_INIT_TASK_STACK_SIZE (1 * 1024u)

/* Stack for tasks */
StackType_t gMmwDemo_dssInitTaskStack[MMWDEMO_DSS_INIT_TASK_STACK_SIZE] __attribute__((aligned(64)));

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/*! @brief   Semaphore Object to trigger Elevation Estimation Execution */
SemaphoreP_Object gDPCExecSemHandle;

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
//#pragma DATA_SECTION(gMmw_MCB, ".customCode");
MmwDemo_DSS_MCB gMmw_MCB;

/**
 * @brief
 *  Global Variable for DPM result buffer
 */
DPM_Buffer resultBuffer;


#ifndef MMWAVELINK_ON_MSS
extern int32_t MmwDemo_initLink(void);
extern int32_t MmwDemo_openLink(void);
extern int32_t MmwDemo_configLink(void);
extern int32_t MmwDemo_sensorStart(void);
extern int32_t MmwDemo_startLink(void);

extern int32_t MmwDemo_openSensor(bool isFirstTimeOpen);
extern int32_t MmwDemo_configSensor(void);
extern int32_t MmwDemo_startSensor(void);

extern int32_t MmwDemo_loadPhaseShiftConfig(void);
extern int32_t MmwDemo_DataOutputMode(void);
extern int32_t MmwDemo_ChannelCfg(void);
extern int32_t MmwDemo_ADCCfg(void);
extern int32_t MmwDemo_loadLowPowerConfig(void);
extern int32_t MmwDemo_loadDatapathConfig(void);
#endif

/**
 * @brief
 *  Global Variable for DPC result buffer
 */
extern DPC_ObjectDetection_ExecuteResult *gExecuteResults;

DPC_ObjectDetection_ElevEstCfg gElevEstCfg = { 0 };

extern uint16_t gRemoteCoreEndPt[CSL_CORE_ID_MAX];

MmwDemo_calibData gCalibDataStoragetoMss __attribute__((aligned(8), section(".calibSharedMem")));

/*! @brief   Semaphore Object to handle Calibration Restore */
SemaphoreP_Object gCalibRestoreSemHandle;

/*! @brief   Semaphore Object to handle Calibration Save Acknowledgement */
SemaphoreP_Object gCalibSaveAckSemHandle;

/**************************************************************************
 ******************* Millimeter Wave Demo Functions Prototype *******************
 **************************************************************************/
static void MmwDemo_dssInitTask(void *args);


/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/


/**
 *  @b Description
 *  @n
 *      Help function to make DPM_ioctl blocking until response is reported
 *
 *  @retval
 *      Success         -0
 *      Failed          <0
 */
int32_t MmwDemo_DPM_ioctl_blocking(
    DPM_Handle handle,
    uint32_t   cmd,
    void      *arg,
    uint32_t   argLen)
{
    int32_t retVal = 0;
    switch (cmd)
    {
        case DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG:
        case DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG:
            retVal = DPM_send(handle, (void *)arg, (uint16_t)argLen, CSL_CORE_ID_M4SS0_1);
            break;
    }
    if (retVal == 0)
    {
        /* Wait until ioctl completed */
        SemaphoreP_pend(&gMmw_MCB.DPMioctlSemHandle, SystemP_WAIT_FOREVER);
    }

    return (retVal);
}


#if 1 // def (!defined(MMWAVELINK_ON_MSS) && defined(SUBSYS_DSS))
int32_t MmwDemo_Send_Cfg_data_MSS()
{
    mmwDemo_cfgToMss cfgToMss = { 0 };
    int32_t          retVal;

    cfgToMss.txAntMask            = gMmw_MCB.cfg.openCfg.chCfg.txChannelEn;
    cfgToMss.numTxAnt             = mathUtils_countSetBits(gMmw_MCB.cfg.openCfg.chCfg.txChannelEn);
    cfgToMss.numRangeBins[0]      = gMmw_MCB.subFrameCfg[0].numRangeBins;
    cfgToMss.numRangeBins[1]      = gMmw_MCB.subFrameCfg[1].numRangeBins;
    cfgToMss.numDopplerBins[0]    = gMmw_MCB.subFrameCfg[0].numDopplerBins;
    cfgToMss.numDopplerBins[1]    = gMmw_MCB.subFrameCfg[1].numDopplerBins;
    cfgToMss.numDopFFTSubBins[0]  = cfgToMss.numDopplerBins[0] / (cfgToMss.numTxAnt + gMmw_MCB.numEmptySubBands);
    cfgToMss.numDopFFTSubBins[1]  = cfgToMss.numDopplerBins[1] / (cfgToMss.numTxAnt + gMmw_MCB.numEmptySubBands);
    cfgToMss.numSubFrames         = gMmw_MCB.objDetCommonCfg.preStartCommonCfg.numSubFrames;
    cfgToMss.measureRxChBiasCfgEn = gMmw_MCB.objDetCommonCfg.preStartCommonCfg.measureRxChannelBiasCfg.enabled;
    cfgToMss.numSubFrames         = gMmw_MCB.objDetCommonCfg.preStartCommonCfg.numSubFrames;
    cfgToMss.guiMonSel            = gMmw_MCB.subFrameCfg[0U].guiMonSel;

    /* send cfgToMss to MSS over Mbox */
    retVal = DPM_send(gMmw_MCB.dataPathObj.objDetDpmHandle, (void *)(&cfgToMss), sizeof(mmwDemo_cfgToMss), CSL_CORE_ID_R5FSS0_0);

    return retVal;
}
#endif


/**
 *  @b Description
 *  @n
 *      DPM Registered Report Handler. The DPM Module uses this registered function to notify
 *      the application about DPM reports.
 *
 *  @param[in]  reportType
 *      Report Type
 *  @param[in]  instanceId
 *      Instance Identifier which generated the report
 *  @param[in]  errCode
 *      Error code if any.
 *  @param[in] arg0
 *      Argument 0 interpreted with the report type
 *  @param[in] arg1
 *      Argument 1 interpreted with the report type
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_DPC_ObjectDetection_reportFxn(void *data, uint16_t dataLen)
{
    uint32_t msg = *((uint32_t *)data);
    if (msg == MMWDEMO_DPC_COMMONCFG || msg == MMWDEMO_DPC_CFG)
    {
        SemaphoreP_post(&gMmw_MCB.DPMioctlSemHandle);
    }
    else if (dataLen == sizeof(DPC_ObjectDetection_PreStartCfg))
    {
        DPC_ObjectDetection_PreStartCfg *dataCfg;
        dataCfg                                       = (DPC_ObjectDetection_PreStartCfg *)data;
        DPC_ObjectDetection_ElevEstCfg *ptrElevEstCfg = (DPC_ObjectDetection_ElevEstCfg *)(&dataCfg->elevEstCfg);
        memcpy((void *)&gElevEstCfg, (void *)ptrElevEstCfg, sizeof(DPC_ObjectDetection_ElevEstCfg));
        SemaphoreP_post(&gMmw_MCB.DPMioctlSemHandle);
    }
    else if (dataLen == sizeof(MMWave_CalibrationData))
    {
        gMmw_MCB.cfg.calibrationDataCfg = *((MMWave_CalibrationData *)data);
        SemaphoreP_post(&gCalibRestoreSemHandle);
    }
    else if (msg == MMWDEMO_CALIBSAVEACK)
    {
        SemaphoreP_post(&gCalibSaveAckSemHandle);
    }
    else if (msg == MMWDEMO_DPC_RESULT)
    {
        SemaphoreP_post(&gDPCExecSemHandle);
    }
    else
    {
    }
}

volatile uint32_t gFrameCnt = 0;
// volatile uint32_t timeMsr1, timeMsr2;
/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dssInitTask(void *args)
{
    int32_t     errCode;
    DPM_InitCfg dpmInitCfg;
    uint32_t    i;

    CycleCounterP_reset();

    /*****************************************************************************
     * Open EDMA drivers instances
     *****************************************************************************/
    Drivers_open();
    Board_driversOpen();

    for (i = 0U; i < CONFIG_EDMA_NUM_INSTANCES; i++)
    {
        gMmw_MCB.dataPathObj.edmaHandle[i] = gEdmaHandle[i];
    }

    /* Create binary semaphore to pend Main task, */
    SemaphoreP_constructBinary(&gMmw_MCB.demoInitTaskCompleteSemHandle, 0);

    /*! @brief   Semaphore Object to handle DPC state */
    SemaphoreP_constructBinary(&gDPCExecSemHandle, 0);

    SemaphoreP_constructBinary(&gMmw_MCB.DPMioctlSemHandle, 0);

    /*****************************************************************************
     * Initialization of the DPM Module:
     *****************************************************************************/
    memset((void *)&dpmInitCfg, 0, sizeof(DPM_InitCfg));

    /* Setup the configuration: */
    dpmInitCfg.localEndPt = gRemoteCoreEndPt[CSL_CORE_ID_C66SS0];
    dpmInitCfg.reportFxn  = MmwDemo_DPC_ObjectDetection_reportFxn;
    dpmInitCfg.setBitPos  = DPM_DSS_BOOT_INFO_BIT_POS;

    /* Initialize the DPM Module: */
    gMmw_MCB.dataPathObj.objDetDpmHandle = DPM_init(&dpmInitCfg, &errCode);
    if (gMmw_MCB.dataPathObj.objDetDpmHandle == NULL)
    {
        DebugP_log("Error: Unable to initialize the DPM Module [Error: %d]\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = DPM_synch(gMmw_MCB.dataPathObj.objDetDpmHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the framework */
            DebugP_log("Error: DPM Synchronization failed [Error code %d]\n", errCode);
            MmwDemo_debugAssert(0);
            return;
        }
        if (syncStatus == CSL_FMKR(DPM_DSS_BOOT_INFO_BIT_POS, DPM_MSS_BOOT_INFO_BIT_POS, 7))
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        ClockP_usleep(1 * 1000U);
    }

#ifndef MMWAVELINK_ON_MSS
    /*****************************************************************************
     * mmWave: Initialization of the high level module
     *****************************************************************************/
    MmwDemo_initLink();

    /* while running mmwavelink on DSS, recieve following from MSS
       1. gMmw_MCB : contain whole configuration
       2. calibrationDataCfg : calibration data read from sFlash
       3.
    */
    MmwDemo_loadDatapathConfig();

    /* Set the calibration parameters */
    gMmw_MCB.calibCfg.restoreEnable = FACTORY_CALIB_RESTORE_ENABLE;
    gMmw_MCB.calibCfg.saveEnable    = FACTORY_CALIB_SAVE_ENABLE;
    gMmw_MCB.calibCfg.flashOffset   = FACTORY_CALIB_FLASH_OFFSET;

    if (gMmw_MCB.calibCfg.restoreEnable != 0)
    {
        SemaphoreP_constructBinary(&gCalibRestoreSemHandle, 0);

        SemaphoreP_pend(&gCalibRestoreSemHandle, SystemP_WAIT_FOREVER);
    }

    MmwDemo_openSensor(true);

    MmwDemo_configSensor();

    if (gMmw_MCB.calibCfg.saveEnable != 0)
    {
        SemaphoreP_constructBinary(&gCalibSaveAckSemHandle, 0);

        SemaphoreP_pend(&gCalibSaveAckSemHandle, SystemP_WAIT_FOREVER);
    }

    /* send required config to MSS */
    MmwDemo_Send_Cfg_data_MSS();

    if (ENABLE_MONITORS != 0)
    {
        MmwaveLink_monitoring(RL_AR_DEVICETYPE_294X);
    }

    /* trigger the frame */
    MmwDemo_startSensor();
#endif

    /* Translate the detObjList address (DSS_L2) from M4 core view to DSS core view. */
    gElevEstCfg.detObjList = (DetObjParams *)SOC_phyToVirt((uint32_t)gElevEstCfg.detObjList);

    while (1)
    {
        SemaphoreP_pend(&gDPCExecSemHandle, SystemP_WAIT_FOREVER);
        DPC_ObjectDetection_ExecuteResult *result = gElevEstCfg.result;
        DPC_ObjDet_estimateXYZ(&gElevEstCfg, gElevEstCfg.detObjList, result->objOut, result->dopNumObjOut, &result->numObjOut, gElevEstCfg.result->pointCloudRadialCompactList);
        gFrameCnt++;
        /* send result data to MSS */
        errCode = DPM_send(gMmw_MCB.dataPathObj.objDetDpmHandle, (void *)(result), sizeof(DPC_ObjectDetection_ExecuteResult), CSL_CORE_ID_R5FSS0_0);
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

    /* init SOC specific modules */
    System_init();
    Board_init();

    /* Initialize and populate the demo MCB */
    memset((void *)&gMmw_MCB, 0, sizeof(gMmw_MCB));

    /* This task is created at highest priority, it should create more tasks and then delete itself */
    gMmw_MCB.initTaskHandle = xTaskCreateStatic(MmwDemo_dssInitTask,
                                                "MmwDemo_dssInitTask",
                                                MMWDEMO_DSS_INIT_TASK_STACK_SIZE,
                                                NULL,
                                                MMWDEMO_DSS_INIT_TASK_PRI,
                                                gMmwDemo_dssInitTaskStack,
                                                &gMmw_MCB.initTaskObj);
    configASSERT(gMmw_MCB.initTaskHandle != NULL);

    /* Start the scheduler to start the tasks executing. */
    vTaskStartScheduler();

    /* The following line should never be reached because vTaskStartScheduler()
    will only return if there was not enough FreeRTOS heap memory available to
    create the Idle and (if configured) Timer tasks.  Heap management, and
    techniques for trapping heap exhaustion, are described in the book text. */
    DebugP_assertNoLog(0);
}

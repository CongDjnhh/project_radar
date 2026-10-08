/*
 *   @file  mmw_link_config.c
 *
 *   @brief
 *      The file contains functions which call mmWave Link APIs to
 *      configure the front end of the device.
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

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>


/* mmWave SDK Include Files: */
#include <ti/common/syscommon.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/edma.h>
#include <drivers/adcbuf.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/crc.h>

#include "FreeRTOS.h"
#include "task.h"
#include <drivers/mailbox.h>
#include <drivers/adcbuf.h>

#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwavelink/include/rl_driver.h>

/* Demo layer header inclusions */
#ifdef SUBSYS_MSS
#include "../mss/mmw_mss.h"
#include "../mss/mssgenerated/ti_drivers_config.h"
#include "../mss/mssgenerated/ti_drivers_open_close.h"
#include "../mss/mssgenerated/ti_board_open_close.h"
#elif defined(SUBSYS_DSS)
#include "../dss/mmw_dss.h"
#include "../dss/dssgenerated/ti_drivers_config.h"
#include "../dss/dssgenerated/ti_drivers_open_close.h"
#include "../dss/dssgenerated/ti_board_open_close.h"
#endif
#include "mmw_link_config.h"
#include "chirp_design_HECR.h"
#include "mmw_link_mon.h"

#define MMWAVELINK_CRC_TYPE RL_CRC_TYPE_16BIT_CCITT

#define MMW_LINK_MGMNT_TASK_STACK_SIZE (5 * 1024U)
StackType_t linkMgmntStack[MMW_LINK_MGMNT_TASK_STACK_SIZE] __attribute__((aligned(32)));

// #define LOGGING

/**
 * @brief   This is the bit mask derived from the mmWave Link \ref rlRfInitComplete_t
 * to indicate that the RF Initialization procedure was successful. As per the
 * documentation the bit mask is derived as follows:-
 *
 *  0       [Reserved]
 *  1       APLL tuning
 *  2       SYNTH VCO1 tuning
 *  3       SYNTH VCO2 tuning
 *  4       LODIST calibration
 *  5       RX ADC DC offset calibration
 *  6       HPF cutoff calibration
 *  7       LPF cutoff calibration
 *  8       Peak detector calibration
 *  9       TX Power calibration
 * 10       RX gain calibration
 * 11       TX Phase calibration
 * 12       RX IQMM calibration
 * 31:13    [Reserved]
 */
#define MMWAVE_INIT_CALIB_SUCCESS 0x1FFEU

#if 0
/* User defined heap memory and handle */
#define BPM_HEAP_MEM_SIZE (2 * 1024u)

static uint8_t gBpmHeapMem[BPM_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));
static HeapP_Object gBpmHeapObj;
#endif

// #define DEBUG_LINK

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
#ifdef SUBSYS_MSS
extern MmwDemo_MSS_MCB gMmw_MCB;
#elif defined(SUBSYS_DSS)
extern MmwDemo_DSS_MCB gMmw_MCB;
#endif

/* Link Layer function definitions */
int32_t MmwDemo_initLink(void) __attribute__((section(".customCode")));
int32_t MmwDemo_openLink(void) __attribute__((section(".customCode")));
int32_t MmwDemo_configLink(void); // __attribute__((section (".customCode")));
int32_t MmwDemo_sensorStart(void);

/* Application layer datapath config function */
extern int32_t MmwDemo_loadOpenChConfig(void);
extern int32_t MmwDemo_loadProfileConfig(void);
extern int32_t MmwDemo_loadChirpConfig(void);
extern int32_t MmwDemo_loadPhaseShiftConfig(void);
extern int32_t MmwDemo_loadFrameConfig(void);
extern int32_t MmwDemo_loadGUIMonConfig(void);
extern int32_t MmwDemo_loadADCBUFConfig(void);
extern void    MmwDemo_printError(int32_t mmWaveError, int32_t subSysError);
#ifdef HECR_ENABLE_ADV_CHIRP
extern int32_t MmwDemo_loadAdvChirpConfig(void);
#endif
/**************************************************************************
 *************************** Local Structures *****************************
 **************************************************************************/

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern ADCBuf_Handle adcbufHandle;
extern bool          gLinkContModeTest;
extern uint16_t      TxEnLutOffset;

#if 0 // Currently not required
extern rlUInt16_t lutOffsetInNBytes;
extern rlInt8_t AdvChirpLUTData[LUT_ADVCHIRP_TABLE_SIZE];
#endif

extern uint16_t           monAeCnt[MMWAVELINK_TEST_MAX_NUM_MON];
extern uint16_t           monFailRepCheck[MMWAVELINK_TEST_MAX_NUM_MON];
extern monitoringDataAe_t gMonDataAeStrct;

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/* Global Variable for tracking information required by the mmWave Link */
MmwaveLink_MCB     gMmwaveLinkMCB;
uint32_t           gMonitorHdrCnt              = 0U;
uint32_t           gInitTimeCalibStatus        = 0U;
uint32_t           gRunTimeCalibStatus         = 0U;
volatile uint32_t  gFrameStartStatus           = 0U;
volatile uint32_t  gMmwlChecksumFailAsyncEvent = 0U;
volatile uint32_t  gMmwlCrcFailAsyncEvent      = 0U;
rlUInt16_t         gMonitoringStatus           = 0U;
rlUInt8_t          isGetGpAdcMeasData          = 0U;
rlCpuFault_t       cpufault;
RL_P_EVENT_HANDLER g_CommInterruptFunc  = NULL;
volatile uint8_t   eDMADataTransferDone = 0U;
uint32_t           baseAddr, regionId;
int32_t            Status = SystemP_SUCCESS;
uint32_t           dmaCh0, tcc0, param0;
volatile int8_t    cpTestStatus = -10;
volatile int8_t    cqTestStatus = -10;

/* received GPAdc Data over Async Event */
rlRecvdGpAdcData_t rcvGpAdcData = { 0 };

/* Async Event Enable and Direction configuration */
rlRfDevCfg_t rfDevCfg = { 0 };

/* Calibration Data storage */
rlCalibrationData_t calibData = { 0 };

/* Dummy mbox handle */
uint32_t mailbox_dummy_handle = 0;
/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern rlInt32_t Osal_mutexCreate(rlOsiMutexHdl_t *mutexHdl, rlInt8_t *name);
extern rlInt32_t Osal_mutexLock(rlOsiMutexHdl_t *mutexHdl, rlOsiTime_t timeout);
extern rlInt32_t Osal_mutexUnlock(rlOsiMutexHdl_t *mutexHdl);
extern rlInt32_t Osal_mutexDelete(rlOsiMutexHdl_t *mutexHdl);
extern rlInt32_t Osal_semCreate(rlOsiSemHdl_t *semHdl, rlInt8_t *name);
extern rlInt32_t Osal_semWait(rlOsiSemHdl_t *semHdl, rlOsiTime_t timeout);
extern rlInt32_t Osal_semSignal(rlOsiSemHdl_t *semHdl);
extern rlInt32_t Osal_semDelete(rlOsiSemHdl_t *semHdl);

/**
 *  @b Description
 *  @n
 *  @param[in] time
 *          Sleep time in 1us units
 *
 *   Sleep function. 1LSB = 1uS.
 */
void MMWave_sleep(uint32_t time)
{
    ClockP_usleep(time);
}

/**
 *  @b Description
 *  @n
 *      The function is used to perform an internal synchronization waiting
 *      for the specific flag to be set. The function will loop around until
 *      the condition is met
 *  @param[in]  flag
 *      Flag for which we are synchronizing
 *
 *  @retval
 *      Not applicable
 */
void MMWaveLink_internalSync(
    uint32_t flag)
{
    while ((gMmw_MCB.cfg.linkStatus & flag) == 0U)
    {
        /* Nothing to do, just wait */
    }
}

/**
 *  @b Description
 *  @n
 *      Mailbox registered function which is invoked on the reception of data
 *
 *  @retval
 *      Success - Communicate Interface Channel Handle
 *  @retval
 *      Error   - NULL
 */
static void MmwaveLink_mboxCallbackFxn(uint32_t remoteCoreId, void *args)
{
    /* Indicate to the Radar Link that a message has been received. */
    g_CommInterruptFunc(0, NULL);
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to open the communication
 *      interface channel
 *
 *  @retval
 *      Success - Communicate Interface Channel Handle
 *  @retval
 *      Error   - NULL
 */
static rlComIfHdl_t MmwaveLink_commOpen(rlUInt8_t deviceIndex, uint32_t flags)
{
    /*Developer notes: DSS mailbox memory needs to be initialized.
     * This needs to be taken care in SBL.
     * MSS mailbox memory is initialized bu IPC driver. implies not required
     * to initialized.
     */
#ifdef SUBSYS_DSS
    /* Clear init Done status. */
    HW_WR_REG32((CSL_DSS_CTRL_U_BASE + CSL_DSS_CTRL_DSS_MAILBOX_MEMINIT_DONE), 1U);

    /* Initialize the memory for the mailbox */
    HW_WR_REG32((CSL_DSS_CTRL_U_BASE + CSL_DSS_CTRL_DSS_MAILBOX_MEMINIT_START), 1U);

    while ((HW_RD_REG32(CSL_DSS_CTRL_U_BASE + CSL_DSS_CTRL_DSS_MAILBOX_MEMINIT_DONE) & 0x01U) != 0x01)
    {
        /* Wait till init is complete. */
    }

    /* Clear init Done status. */
    HW_WR_REG32((CSL_DSS_CTRL_U_BASE + CSL_DSS_CTRL_DSS_MAILBOX_MEMINIT_DONE), 1U);
#endif
    Mailbox_setReadCallback(MmwaveLink_mboxCallbackFxn, NULL);

    return (rlComIfHdl_t)&mailbox_dummy_handle;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to close the communication
 *      interface channel
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t MmwaveLink_commClose(rlComIfHdl_t fd)
{
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to read data from the communication
 *      interface channel
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t MmwaveLink_commRead(rlComIfHdl_t fd, uint8_t *pBuff, uint16_t len)
{
    int32_t status;
    status = Mailbox_read(CSL_CORE_ID_RSS_R4, pBuff, len, SystemP_WAIT_FOREVER);

    if (status != SystemP_SUCCESS)
    {
        DebugP_logInfo("Mailbox Read Error. \n");
    }
    else
    {
        status = len;
    }

    return status;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to write data to the communication
 *      interface channel
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t MmwaveLink_commWrite(rlComIfHdl_t fd, uint8_t *pBuff, uint16_t len)
{
    int32_t status;

    status = Mailbox_write(CSL_CORE_ID_RSS_R4, pBuff, len, SystemP_WAIT_FOREVER);

    if (status != SystemP_SUCCESS)
    {
        DebugP_logInfo("Mailbox Write Error. \n");
    }
    else
    {
        status = len;
    }

    return status;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to power on the AR1XX Device
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_enableDevice(rlUInt8_t deviceIndex)
{
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to power off the AR1XX Device
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_disableDevice(rlUInt8_t deviceIndex)
{
    /* Note: halting of the BSS is not currently supported */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to mask the interrupts.
 *      In the case of Mailbox communication interface the driver will
 *      handle the interrupt management. This function is a dummy stub
 *
 *  @retval
 *      Not applicable
 */
static void MmwaveLink_maskHostIRQ(rlComIfHdl_t fd)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to umask the interrupts.
 *      In the case of the mailbox driver we will flush out and close the
 *      read buffer.
 *
 *  @retval
 *      Not applicable
 */
void MmwaveLink_unmaskHostIRQ(rlComIfHdl_t fd)
{
    /* The Mailbox driver indicates to the remote endpoint that the message
     * have been processed. The mmWave is now capable of receiving another message */
    Mailbox_readDone(CSL_CORE_ID_RSS_R4);
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to poll the
 *      host interrupt status. In the case of the Mailbox the driver is
 *      responsible for the interrupt management. This function is a
 *      dummy stub
 *
 *  @param[in]  fd
 *      Handle to the communication interface
 *  @param[in]  Level
 *      Wait for IRQ Level(high/low)
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Always returns 0 indicating that the IRQ Line is low
 */
static rlInt32_t MMWaveLink_waitIRQStatus(rlComIfHdl_t fd, rlUInt8_t Level)
{
    return 0;
}

#ifdef LOGGING
static rlInt32_t MMWaveLink_print(const rlInt8_t *format, ...)
{
    // va_list     arg;
    // char        logMessage[256];
    // int32_t     sizeMessage;
    // UART_Transaction trans;

    // UART_Transaction_init(&trans);

    // /* Format the message: */
    // va_start (arg, format);
    // sizeMessage = vsnprintf (&logMessage[0], sizeof(logMessage), format, arg);
    // va_end (arg);

    // /* If CLI_write is called before CLI init has happened, return */
    // if (gMmw_MCB.loggingUartHandle == NULL)
    // {
    //     return -1;
    // }

    // trans.buf   = &logMessage[0U];
    // trans.count = sizeMessage;

    // /* Log the message on the UART CLI console: */
    // /* Blocking Mode: */
    // UART_write (gMmw_MCB.loggingUartHandle, &trans);

    // va_list     arg;
    // va_start (arg, format);
    // vprintf_(format, arg);
    // va_end (arg);

    va_list arg;
    char    logMessage[256] = { 0 };

    uint32_t key = HwiP_disable();

    /* Format the message: */
    va_start(arg, format);
    vsnprintf(&logMessage[0], sizeof(logMessage), format, arg);
    va_end(arg);

    printf("%s", &logMessage[0]);

    HwiP_restore(key);

    return 0;
}
#endif

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to register the Interrupt Handler.
 *      In the case of the Mailbox the driver does the interrupt registeration and
 *      so this function is a dummy stub.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_registerInterruptHandler(rlUInt8_t deviceIndex, RL_P_EVENT_HANDLER pHandler, void *pValue)
{
    g_CommInterruptFunc = pHandler;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to handle asynchronous events
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
uint16_t errorStatusCnt[5] = { 0 };

static void MmwaveLink_asyncEventHandler(uint8_t devIndex, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);
    uint16_t msgId   = RL_GET_MSGID_FROM_SBID(sbId);

    gMmwaveLinkMCB.debugStats.numAsyncEvents++;

    DebugP_logInfo("mmwavelink async event handler: BSS Event MsgId: %d [Device Index: 0x%x Sub Block Id: 0x%x Sub Block Length: %d]\n",
                   msgId,
                   devIndex,
                   sbId,
                   sbLen);

    /* Process the received message: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /*******************************************************************
             * Received Asychronous Message
             *******************************************************************/
            switch (asyncSB)
            {
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    rlRfInitComplete_t *ptrRFInitCompleteMessage;
                    uint32_t            calibrationStatus;

                    /* Get the RF-Init completion message: */
                    ptrRFInitCompleteMessage = (rlRfInitComplete_t *)payload;

                    /* Were we successful?  */
                    calibrationStatus = ptrRFInitCompleteMessage->calibStatus & MMWAVE_INIT_CALIB_SUCCESS;
                    if (calibrationStatus != 0)
                    {
                        /* YES: Mark the link as operational */
                        gMmw_MCB.cfg.linkStatus = gMmw_MCB.cfg.linkStatus | MMWAVE_RF_INITIALIZED;
                    }
                    else
                    {
                        /* NO: Link initialization failed. */
                        gMmw_MCB.cfg.linkStatus = gMmw_MCB.cfg.linkStatus | MMWAVE_RF_INIT_FAILED;
                    }
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    /* Calibration has been completed. */
                    gMmw_MCB.cfg.linkStatus = gMmw_MCB.cfg.linkStatus | MMWAVE_RF_CALIBRATION_DONE;
                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    /* Calibration has failed. */
                    gMmw_MCB.cfg.linkStatus = gMmw_MCB.cfg.linkStatus | MMWAVE_RF_CALIBRATION_FAILED;
                    break;
                }
                case RL_RF_AE_CPUFAULT_SB:
                {
                    /* BSS reports CPU Fault: */
                    gMmw_MCB.cfg.linkStatus = gMmw_MCB.cfg.linkStatus | MMWAVE_RF_CPU_FAULT;
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    /* BSS reports ESM Fault: */
                    gMmw_MCB.cfg.linkStatus = gMmw_MCB.cfg.linkStatus | MMWAVE_RF_ESM_FAULT;
                    break;
                }
                case RL_RF_AE_ANALOG_FAULT_SB:
                {
                    /* BSS reports Analog Fault: */
                    gMmw_MCB.cfg.linkStatus = gMmw_MCB.cfg.linkStatus | MMWAVE_RF_ANALOG_FAULT;
                    break;
                }
                case RL_RF_AE_MON_DIG_PERIODIC_REPORT_SB:
                {
                    monAeCnt[29]++;
                    // extern volatile uint32_t gLinkFrameCnt;
                    // errorStatusCnt[2] = gLinkFrameCnt;
                    gMonitoringStatus = ((rlDigPeriodicReportData_t *)payload)->digMonPeriodicStatus;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[29]++;
                    }
                    break;
                }
                case RL_RF_AE_MON_TEMPERATURE_REPORT_SB:
                {
                    // extern volatile uint32_t gLinkFrameCnt;
                    // errorStatusCnt[1] = gLinkFrameCnt;
                    memcpy(&gMonDataAeStrct.monTempReport, payload, sizeof(rlMonTempReportData_t));

                    gMonitoringStatus = ((rlMonTempReportData_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[MMWAVELINK_TEST_MON_TEMP]++;
                    }
                    monAeCnt[MMWAVELINK_TEST_MON_TEMP]++;
                    break;
                }
                case RL_RF_AE_MON_RX_GAIN_PHASE_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monRxGainPhRep, payload, sizeof(rlMonRxGainPhRep_t));
                    monAeCnt[MMWAVELINK_TEST_MON_RX_GAIN_PH]++;
                    break;
                }
                case RL_RF_AE_MON_RX_NOISE_FIG_REPORT:
                {
                    ;
                }
                case RL_RF_AE_MON_RX_IF_STAGE_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monRxIfStageRep, payload, sizeof(rlMonRxIfStageRep_t));
                    gMonitoringStatus = ((rlMonRxIfStageRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[3]++;
                    }
                    monAeCnt[3]++;
                    break;
                }
                case RL_RF_AE_MON_TX0_POWER_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monTx0powRep, payload, sizeof(rlMonTxPowRep_t));
                    gMonitoringStatus = ((rlMonTxPowRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[4]++;
                    }
                    monAeCnt[4]++;
                    break;
                }
                case RL_RF_AE_MON_TX1_POWER_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monTx1powRep, payload, sizeof(rlMonTxPowRep_t));
                    gMonitoringStatus = ((rlMonTxPowRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[5]++;
                    }
                    monAeCnt[5]++;
                    break;
                }
                case RL_RF_AE_MON_TX2_POWER_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monTx2powRep, payload, sizeof(rlMonTxPowRep_t));
                    gMonitoringStatus = ((rlMonTxPowRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[6]++;
                    }
                    monAeCnt[6]++;
                    break;
                }
                case RL_RF_AE_MON_TX0_BALLBREAK_REPORT:
                {
                    /* stop updating the global cached value after first error so that we can dump the first failed case valus */
                    if (monFailRepCheck[7] == 0)
                    {
                        memcpy(&gMonDataAeStrct.monTx0BallbreakRep, payload, sizeof(rlMonTxBallBreakRep_t));
                    }
                    gMonitoringStatus = ((rlMonTxBallBreakRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[7]++;
                    }
                    monAeCnt[7]++;
                    break;
                }
                case RL_RF_AE_MON_TX1_BALLBREAK_REPORT:
                {
                    /* stop updating the global cached value after first error so that we can dump the first failed case valus */
                    if (monFailRepCheck[8] == 0)
                    {
                        memcpy(&gMonDataAeStrct.monTx1BallbreakRep, payload, sizeof(rlMonTxBallBreakRep_t));
                    }
                    gMonitoringStatus = ((rlMonTxBallBreakRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[8]++;
                    }
                    monAeCnt[8]++;
                    break;
                }
                case RL_RF_AE_MON_REPORT_HEADER_SB:
                {
                    gMonitorHdrCnt++;
                    monAeCnt[27]++;
                    break;
                }
                case RL_RF_AE_GPADC_MEAS_DATA_SB:
                {
                    isGetGpAdcMeasData = 1U;
                    memcpy(&rcvGpAdcData, payload, sizeof(rlRecvdGpAdcData_t));
                    break;
                }
                case RL_RF_AE_DIG_LATENTFAULT_REPORT_SB:
                {
                    // printf ("Debug:  Digital Monitoring Latent Fault Report Received \n");
                    break;
                }
                case RL_RF_AE_FRAME_END_SB:
                {
                    gFrameStartStatus = 0U;
                    printf("Debug:  Frame Stop Async Event \n");
                    break;
                }
                default:
                {
                    /* Catch condition for any other sub block which the mmWave is NOT interested
                     * in. This is just a fall through. Pass the event to the application registered
                     * event function. */
                    break;
                }
            }
            break;
        }

        case RL_RF_ASYNC_EVENT_1_MSG:
        {
            /*Just fwd message to application*/
            switch (asyncSB)
            {
                case RL_RF_AE_MON_TX2_BALLBREAK_REPORT:
                {
                    /* stop updating the global cached value after first error so that we can dump the first failed case valus */
                    if (monFailRepCheck[9] == 0)
                    {
                        memcpy(&gMonDataAeStrct.monTx2BallbreakRep, payload, sizeof(rlMonTxBallBreakRep_t));
                    }
                    gMonitoringStatus = ((rlMonTxBallBreakRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[9]++;
                    }
                    monAeCnt[9]++;
                    break;
                }
                case RL_RF_AE_MON_TX3_BALLBREAK_REPORT:
                {
                    /* stop updating the global cached value after first error so that we can dump the first failed case valus */
                    if (monFailRepCheck[30] == 0)
                    {
                        memcpy(&gMonDataAeStrct.monTx3BallbreakRep, payload, sizeof(rlMonTxBallBreakRep_t));
                    }
                    gMonitoringStatus = ((rlMonTxBallBreakRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[30]++;
                    }
                    monAeCnt[30]++;
                    break;
                }
                case RL_RF_AE_MON_TX3_POWER_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monTx3powRep, payload, sizeof(rlMonTxPowRep_t));
                    gMonitoringStatus = ((rlMonTxPowRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[29]++;
                    }
                    monAeCnt[29]++;
                    break;
                }
                case RL_RF_AE_MON_TX_GAIN_MISMATCH_REPORT:
                {
                    ;
                }
                case RL_RF_AE_MON_ADV_TX_GAIN_PHASE_MIS_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monAdvTxGainPhMisMatchRep, payload, sizeof(rlMonAdvTxGainPhaMisRep_t));
                    gMonitoringStatus = ((rlMonAdvTxGainPhaMisRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[10]++;
                    }
                    monAeCnt[10]++;
                    break;
                }
                case RL_RF_AE_MON_SYNTHESIZER_FREQ_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monSynthFreqRep, payload, sizeof(rlMonSynthFreqRep_t));
                    gMonitoringStatus = ((rlMonSynthFreqRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[14]++;
                    }
                    monAeCnt[14]++;
                    break;
                }
                case RL_RF_AE_MON_EXT_ANALOG_SIG_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monExtAnaSigRep, payload, sizeof(rlMonExtAnaSigRep_t));
                    gMonitoringStatus = ((rlMonExtAnaSigRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[15]++;
                    }
                    monAeCnt[15]++;
                    break;
                }
                case RL_RF_AE_MON_TX0_INT_ANA_SIG_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monTx0IntAnaSigRep, payload, sizeof(rlMonTxIntAnaSigRep_t));
                    gMonitoringStatus = ((rlMonTxIntAnaSigRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[16]++;
                    }
                    monAeCnt[16]++;
                    break;
                }
                case RL_RF_AE_MON_TX1_INT_ANA_SIG_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monTx1IntAnaSigRep, payload, sizeof(rlMonTxIntAnaSigRep_t));
                    gMonitoringStatus = ((rlMonTxIntAnaSigRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[17]++;
                    }
                    monAeCnt[17]++;
                    break;
                }
                case RL_RF_AE_MON_TX2_INT_ANA_SIG_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monTx2IntAnaSigRep, payload, sizeof(rlMonTxIntAnaSigRep_t));
                    gMonitoringStatus = ((rlMonTxIntAnaSigRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[18]++;
                    }
                    monAeCnt[18]++;
                    break;
                }
                case RL_RF_AE_MON_TX3_INT_ANA_SIG_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monTx3IntAnaSigRep, payload, sizeof(rlMonTxIntAnaSigRep_t));
                    gMonitoringStatus = ((rlMonTxIntAnaSigRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[31]++;
                    }
                    monAeCnt[31]++;
                    break;
                }
                case RL_RF_AE_MON_RX_INT_ANALOG_SIG_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monRxIntAnaSigRep, payload, sizeof(rlMonRxIntAnaSigRep_t));
                    gMonitoringStatus = ((rlMonRxIntAnaSigRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[19]++;
                    }
                    monAeCnt[19]++;
                    break;
                }
                case RL_RF_AE_MON_PMCLKLO_INT_ANA_SIG_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monPmClkIntAnaSigRep, payload, sizeof(rlMonPmclkloIntAnaSigRep_t));
                    gMonitoringStatus = ((rlMonPmclkloIntAnaSigRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[20]++;
                    }
                    monAeCnt[20]++;
                    break;
                }
                case RL_RF_AE_MON_GPADC_INT_ANA_SIG_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monGpadcIntAnaSigRep, payload, sizeof(rlMonGpadcIntAnaSigRep_t));
                    gMonitoringStatus = ((rlMonGpadcIntAnaSigRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[21]++;
                    }
                    monAeCnt[21]++;
                    break;
                }
                case RL_RF_AE_MON_PLL_CONTROL_VOLT_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monPllConvVoltRep, payload, sizeof(rlMonPllConVoltRep_t));
                    gMonitoringStatus = ((rlMonPllConVoltRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[22]++;
                    }
                    monAeCnt[22]++;
                    break;
                }
                case RL_RF_AE_MON_DCC_CLK_FREQ_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monDccClkFreqRep, payload, sizeof(rlMonDccClkFreqRep_t));
                    gMonitoringStatus = ((rlMonDccClkFreqRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[23]++;
                    }
                    monAeCnt[23]++;
                    break;
                }
                case RL_RF_AE_MON_RX_MIXER_IN_PWR_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monRxMixrInPwrRep, payload, sizeof(rlMonRxMixrInPwrRep_t));
                    gMonitoringStatus = ((rlMonRxMixrInPwrRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[26]++;
                    }
                    monAeCnt[26]++;
                    break;
                }
                case RL_RF_AE_MON_SYNTH_FREQ_NONLIVE_REPORT:
                {
                    memcpy(&gMonDataAeStrct.monSynthFreqNonLiveRep, payload, sizeof(rlMonSynthFreqNonLiveRep_t));
                    gMonitoringStatus = ((rlMonSynthFreqNonLiveRep_t *)payload)->statusFlags;
                    if (gMonitoringStatus == 0U)
                    {
                        monFailRepCheck[28]++;
                    }
                    monAeCnt[28]++;
                    break;
                }
                default:
                {
                    printf("Error: Asynchronous Event SB Id %d not handled with msg ID 0x%x\n", asyncSB, msgId);
                    break;
                }
            }
            break;
        }

        /* Async Event from MMWL */
        case RL_MMWL_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
                case RL_MMWL_AE_MISMATCH_REPORT:
                {
                    /* link reports protocol error in the async report from BSS */
                    gMmw_MCB.cfg.linkStatus = gMmw_MCB.cfg.linkStatus | MMWAVE_LINK_ASYNC_EVENT_MISMATCH_ERROR;
                    gMmwaveLinkMCB.debugStats.numErrAsyncEvents++;

                    DebugP_logInfo("RL_MMWL_AE_MISMATCH_REPORT\n");

                    int errTemp = *(int32_t *)payload;
                    /* CRC mismatched in the received Async-Event msg */
                    if (errTemp == RL_RET_CODE_CRC_FAILED)
                    {
                        DebugP_logInfo("CRC failed \n");
                    }
                    /* Checksum mismatched in the received msg */
                    else if (errTemp == RL_RET_CODE_CHKSUM_FAILED)
                    {

                        DebugP_logInfo("Checksum failed \n");
                    }
                    /* Polling to HostIRQ timed out,
                    i.e. Device didn't respond to CNYS from the Host */
                    else if (errTemp == RL_RET_CODE_HOSTIRQ_TIMEOUT)
                    {
                        DebugP_logInfo("HostIRQ polling timed out \n");
                    }
                    else
                    {
                        DebugP_logInfo("mmWaveLink error \n");
                    }
                    break;
                }
                case RL_MMWL_AE_INTERNALERR_REPORT:
                {
                    gMmwaveLinkMCB.debugStats.numErrAsyncEvents++;
                    DebugP_logInfo("RL_MMWL_AE_INTERNALERR_REPORT\n");
                    /* link reports internal error during BSS communication */
                    gMmw_MCB.cfg.linkStatus = gMmw_MCB.cfg.linkStatus | MMWAVE_LINK_ASYNC_EVENT_INTERNAL_ERROR;
                    break;
                }
                default:
                {
                    DebugP_logInfo("Unhandled Async Event msgId: 0x%x, asyncSB:0x%x  \r\n", msgId, asyncSB);
                    break;
                }
            }
            break;
        }
        default:
        {
            /* Error: Received an invalid message identifier in the ASYNC Handler */
            DebugP_logInfo("Error: Invalid message %d is received in the async handler\n", msgId);
            DebugP_assert(0);
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to call the function in a different context
 *      This function is invoked from the Interrupt context.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_spawn(RL_P_OSI_SPAWN_ENTRY pEntry, const void *pValue, uint32_t flags)
{
    /* Record the entry to be spawned. */
    gMmwaveLinkMCB.spawnTable[gMmwaveLinkMCB.spawnFxnCnt++] = pEntry;

    /* Post the semaphore and wake up the link management task */
    SemaphoreP_post(&gMmwaveLinkMCB.linkSemaphore);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to compute the CRC.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_computeCRC(rlUInt8_t *data, rlUInt32_t dataLen, rlUInt8_t crcType, rlUInt8_t *crc)
{
    uint64_t             signature;
    uint32_t             index;
    uint8_t             *ptrSignature;
    int32_t              retVal    = 0;
    uint32_t             crcLength = 0U;
    uint32_t             loopCnt;
    uint32_t             patternCnt = 0;
    CRC_SignatureRegAddr psaSignRegAddr;
    uint16_t            *ptrData16 = (uint16_t *)data;
    uint32_t            *ptrData32 = (uint32_t *)data;
    CRC_Config           config;

    /* Sanity Check: Validate the parameters; determine the length of the CRC */
    switch (crcType)
    {
        case RL_CRC_TYPE_16BIT_CCITT:
        {
            /* CRC is 2 bytes */
            crcLength  = 2U;
            patternCnt = dataLen / 2;
            break;
        }
        case RL_CRC_TYPE_32BIT:
        {
            /* CRC is 4 bytes */
            crcLength  = 4U;
            patternCnt = dataLen / 4;
            break;
        }
        case RL_CRC_TYPE_64BIT_ISO:
        {
            /* CRC is 8 bytes */
            crcLength  = 8U;
            patternCnt = dataLen / 4;
            break;
        }
        default:
        {
            /* Error: The mmWave link has passed an invalid CRC Type */
            retVal = MINUS_ONE;
            break;
        }
    }

    /* Sanity Check: Did the link pass a valid CRC Type? */
    if (retVal == MINUS_ONE)
    {
        /* NO: Invalid CRC Type has been detected */
        DebugP_assert(0);
    }

    config.mode         = CRC_OPERATION_MODE_FULLCPU;
    config.type         = CRC_TYPE_16BIT;
    config.dataLen      = CRC_DATALENGTH_16BIT;
    config.bitSwap      = CRC_BITSWAP_MSB;
    config.byteSwap     = CRC_BYTESWAP_ENABLE;
    config.patternCount = patternCnt;
    config.sectorCount  = 1;

    retVal = CRC_configure((uint32_t)AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR),
                           CRC_CHANNEL_1,
                           &config);

    if (retVal < 0)
    {
        /* Error: Unable to configure CRC moduel. */
        DebugP_logInfo("Error: Configure of CRC failed [Error code %d]\n", retVal);
        DebugP_assert(0);
    }

    /* Get CRC PSA signature register address */
    CRC_getPSASigRegAddr((uint32_t)AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR),
                         CRC_CHANNEL_1,
                         &psaSignRegAddr);

    /* Reset the CRC channel*/
    CRC_channelReset((uint32_t)AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR),
                     CRC_CHANNEL_1);

    /* compute the CRC by writing the data buffer on which CRC computation is needed */
    for (loopCnt = 0; loopCnt < patternCnt; loopCnt++)
    {
        if (crcType == RL_CRC_TYPE_16BIT_CCITT)
        {
            HW_WR_REG16(psaSignRegAddr.regL, (uint16_t)ptrData16[loopCnt]);
        }
        else
        {
            HW_WR_REG32(psaSignRegAddr.regL, (uint32_t)ptrData32[loopCnt]);
        }
    }

    /* Fetch CRC signature value */
    retVal = CRC_getPSASig((uint32_t)AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR),
                           CRC_CHANNEL_1,
                           (CRC_Signature *)&signature);

    /* Get the pointer to the CRC Signature: */
    ptrSignature = (uint8_t *)&signature;

    /* Copy the CRC signature into CRC output array*/
    for (index = 0U; index < crcLength; index++)
    {
        *(crc + index) = *(ptrSignature + index);
    }

    /* CRC signature has been computed correctly */
    retVal = 0;

    return (rlInt32_t)retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the task which handles the mmWave Link communication
 *      messages between the BSS and MSS.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwaveLink_mmwaveLinkMgmtTask(void *arg)
{
    RL_P_OSI_SPAWN_ENTRY spawnFxn;
    uintptr_t            key;

    /* Initialize the mmWave Link Semaphore: */
    SemaphoreP_constructBinary(&gMmwaveLinkMCB.linkSemaphore, 0);

    /* Execute forever: */
    while (1)
    {
        /* Pending on the link semaphore */
        SemaphoreP_pend(&gMmwaveLinkMCB.linkSemaphore, SystemP_WAIT_FOREVER);

        /* Critical Section: We record the spawn function which is to be executed */
        key                     = HwiP_disable();
        spawnFxn                = gMmwaveLinkMCB.spawnTable[gMmwaveLinkMCB.spawnFxnCnt - 1];
        gMmwaveLinkMCB.spawnFxn = NULL;
        gMmwaveLinkMCB.spawnFxnCnt--;
        HwiP_restore(key);

        /* Execute the spawn function: */
        spawnFxn(NULL);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the asynchronous events. On Dual core
 *      either the DSS or the MSS could be configured to be the recepient of
 *      asynchronous events.
 *      It also configures if the start/stop asynchronous events are
 *      enabled or disabled.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the mmWave control block
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup MMWAVE_CTRL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_deviceCfgAsyncEvent()
{
    int32_t      retVal;
    rlRfDevCfg_t asyncEventCfg;

    /* Sanity Check: Validate the arguments */
    if ((gMmw_MCB.cfg.openCfg.defaultAsyncEventHandler != MMWave_DefaultAsyncEventHandler_MSS) &&
        (gMmw_MCB.cfg.openCfg.defaultAsyncEventHandler != MMWave_DefaultAsyncEventHandler_DSS))
    {
        /* Error: Invalid argument. */
        MmwDemo_printError(MMWAVE_EINVAL, 0);
    }

    /* Initialize the configuration: */
    memset((void *)&asyncEventCfg, 0, sizeof(rlRfDevCfg_t));

#ifdef SUBSYS_DSS
    gMmw_MCB.cfg.openCfg.defaultAsyncEventHandler = MMWave_DefaultAsyncEventHandler_DSS;
#else
    gMmw_MCB.cfg.openCfg.defaultAsyncEventHandler = MMWave_DefaultAsyncEventHandler_MSS;
#endif

    /* Which domain are we executing on? */
    if (gMmw_MCB.cfg.openCfg.defaultAsyncEventHandler == MMWave_DefaultAsyncEventHandler_MSS)
    {
        /* MSS: */
        asyncEventCfg.aeDirection = 0U;
    }
    else
    {
        /* DSS: */
        asyncEventCfg.aeDirection = 0xAU;
    }

    /*Disable Frame Start async event? */
    if (gMmw_MCB.cfg.openCfg.disableFrameStartAsyncEvent)
    {
        asyncEventCfg.aeControl |= 0x1U;
    }

    /*Disable Frame Stop async event? */
    if (gMmw_MCB.cfg.openCfg.disableFrameStopAsyncEvent)
    {
        asyncEventCfg.aeControl |= 0x2U;
    }

    /* Set the configuration in the link: */
    retVal = rlRfSetDeviceCfg(RL_DEVICE_MAP_CASCADED_1, (rlRfDevCfg_t *)&asyncEventCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Set the Async Event Direction Failed */
        MmwDemo_printError(MMWAVE_EASYNCEVENT, retVal);
    }

    /* Control comes here implies either of the following:-
     * - Asynchronous event direction was configured successfully
     * - We are not the default event handler.
     * Either case we report success */
    retVal = 0;

    return retVal;
}

#ifdef DEBUG_LINK
/**
 *  @b Description
 *  @n
 *      The function is used to get and display the version information
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_getVersion(void)
{
    rlVersion_t verArgs;
    int32_t     retVal;
    /* currently patch binaries are available for AWR16 ES2.0 & IWR16 ES2.0 only*/
    int8_t rfPatchBuildVer, rfPatchDebugVer;

    /* Get the version string: */
    retVal = rlDeviceGetVersion(RL_DEVICE_MAP_INTERNAL_BSS, &verArgs);
    if (retVal != 0)
    {
        printf("Error: Unable to get the device version from mmWave link [Error %d]\n", retVal);
        return -1;
    }

    /* Display the version information */
    printf("RF H/W Version    : %02d.%02d\n",
           verArgs.rf.hwMajor,
           verArgs.rf.hwMinor);
    printf("RF F/W Version    : %02d.%02d.%02d.%02d.%02d.%02d.%02d\n",
           verArgs.rf.fwMajor,
           verArgs.rf.fwMinor,
           verArgs.rf.fwBuild,
           verArgs.rf.fwDebug,
           verArgs.rf.fwYear,
           verArgs.rf.fwMonth,
           verArgs.rf.fwDay);
    rfPatchDebugVer = ((verArgs.rf.patchBuildDebug) & 0x0F);
    rfPatchBuildVer = (((verArgs.rf.patchBuildDebug) & 0xF0) >> 4);

    printf("RF F/W Patch Version : %02d.%02d.%02d.%02d.%02d.%02d.%02d\n",
           verArgs.rf.patchMajor,
           verArgs.rf.patchMinor,
           rfPatchBuildVer,
           rfPatchDebugVer,
           verArgs.rf.patchYear,
           verArgs.rf.patchMonth,
           verArgs.rf.patchDay);
    printf("mmWaveLink Version: %02d.%02d.%02d.%02d\n",
           verArgs.mmWaveLink.major,
           verArgs.mmWaveLink.minor,
           verArgs.mmWaveLink.build,
           verArgs.mmWaveLink.debug);
    return 0;
}
#endif

StaticTask_t gStaticTask;

/**
 *  @b Description
 *  @n
 *      The function is used to initialize and setup the mmWave link
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t MmwDemo_initLink()
{
    rlClientCbs_t RlApp_ClientCtx = { 0 };
    TaskHandle_t  taskHandle;
    int32_t       retVal;

    /* Initialize and populate the Mmwave Link MCB */
    memset((void *)&gMmwaveLinkMCB, 0, sizeof(MmwaveLink_MCB));

    // status = SemaphoreP_constructBinary(&(gMmw_MCB.linkSemHandle), 0);
    // DebugP_assert(SystemP_SUCCESS == status);

    /*****************************************************************************
     * Start CRC driver:
     *****************************************************************************/

    /* Configure CRC Module */
    retVal = CRC_channelReset((uint32_t)AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR),
                              CRC_CHANNEL_1);
    DebugP_assert(retVal == SystemP_SUCCESS);

    retVal = CRC_initialize((uint32_t)AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR),
                            CRC_CHANNEL_1,
                            0,
                            0);

    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log("Error: Unable to open the CRC Channel [Error Code %d]\n", retVal);
        return -1;
    }
#ifdef DEBUG_LINK
    DebugP_log("Debug: CRC Channel %d has been opened successfully\n", CRC_CHANNEL_1);
#endif
    /*****************************************************************************
     * Launch the Mmwave Link Tasks:
     *****************************************************************************/

    /* Initialize and Launch the mmWave Link Management Task: */

    taskHandle = xTaskCreateStatic(MmwaveLink_mmwaveLinkMgmtTask,
                                   "link_mgnt_task",
                                   MMW_LINK_MGMNT_TASK_STACK_SIZE,
                                   NULL,
                                   4,
                                   linkMgmntStack,
                                   &gStaticTask);

    configASSERT(taskHandle != NULL);

    /*****************************************************************************
     * Initialize the mmWave Link: We need to have the link management task
     * operational to be able to process the SPAWN function.
     *****************************************************************************/

    /* Reset the client context: */
    memset((void *)&RlApp_ClientCtx, 0, sizeof(rlClientCbs_t));

    RlApp_ClientCtx.ackTimeout = 1000U;

    /* Setup the crc Type in the mmWave link and synchronize this with the
     * created CRC Channel. */
    RlApp_ClientCtx.crcType = RL_CRC_TYPE_16BIT_CCITT;

    /* Setup the platform on which the mmWave Link executes */
#ifdef SUBSYS_MSS
    RlApp_ClientCtx.platform = RL_PLATFORM_MSS;
#endif
#ifdef SUBSYS_DSS
    RlApp_ClientCtx.platform = RL_PLATFORM_DSS;
#endif
    RlApp_ClientCtx.arDevType = RL_AR_DEVICETYPE_22XX;

    /* Initialize the Communication Interface API: */
    RlApp_ClientCtx.comIfCb.rlComIfOpen  = MmwaveLink_commOpen;
    RlApp_ClientCtx.comIfCb.rlComIfClose = MmwaveLink_commClose;
    RlApp_ClientCtx.comIfCb.rlComIfRead  = MmwaveLink_commRead;
    RlApp_ClientCtx.comIfCb.rlComIfWrite = MmwaveLink_commWrite;


    /* Initialize OSI Mutex Interface */
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexCreate = Osal_mutexCreate;
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexLock   = Osal_mutexLock;
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexUnLock = Osal_mutexUnlock;
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexDelete = Osal_mutexDelete;

    /* Initialize OSI Semaphore Interface */
    RlApp_ClientCtx.osiCb.sem.rlOsiSemCreate = Osal_semCreate;
    RlApp_ClientCtx.osiCb.sem.rlOsiSemWait   = Osal_semWait;
    RlApp_ClientCtx.osiCb.sem.rlOsiSemSignal = Osal_semSignal;
    RlApp_ClientCtx.osiCb.sem.rlOsiSemDelete = Osal_semDelete;

    /* Initialize OSI Queue Interface */
    RlApp_ClientCtx.osiCb.queue.rlOsiSpawn = MmwaveLink_spawn;

    /* Initialize OSI Timer Interface */
    RlApp_ClientCtx.timerCb.rlDelay = NULL;

    /* Initialize the CRC Interface */
    RlApp_ClientCtx.crcCb.rlComputeCRC = MmwaveLink_computeCRC;

    /* Initialize Device Control Interface */
    RlApp_ClientCtx.devCtrlCb.rlDeviceDisable            = MmwaveLink_disableDevice;
    RlApp_ClientCtx.devCtrlCb.rlDeviceEnable             = MmwaveLink_enableDevice;
    RlApp_ClientCtx.devCtrlCb.rlDeviceMaskHostIrq        = MmwaveLink_maskHostIRQ;
    RlApp_ClientCtx.devCtrlCb.rlDeviceUnMaskHostIrq      = MmwaveLink_unmaskHostIRQ;
    RlApp_ClientCtx.devCtrlCb.rlDeviceWaitIrqStatus      = MMWaveLink_waitIRQStatus;
    RlApp_ClientCtx.devCtrlCb.rlRegisterInterruptHandler = MmwaveLink_registerInterruptHandler;

    /* Initialize the Asynchronous Event Handler: */
    RlApp_ClientCtx.eventCb.rlAsyncEvent = MmwaveLink_asyncEventHandler;

#ifdef LOGGING
    RlApp_ClientCtx.dbgCb.dbgLevel = RL_DBG_LEVEL_DEBUG;
    RlApp_ClientCtx.dbgCb.rlPrint  = MMWaveLink_print;
#endif

    /* Power on the Device */
    if (rlDevicePowerOn(RL_DEVICE_MAP_INTERNAL_BSS, RlApp_ClientCtx) != 0)
    {
        DebugP_log("Error: Power on request to the BSS failed\n");
        return -1;
    }

    /* Check for BSS powerup */
    /* Check if BSS powerup is done */
    /* Bit 18 - The bit will be set once the BSS boots up before entering the idle task loop */
    if ((CSL_REG32_RD(CSL_RSS_PROC_CTRL_U_BASE + CSL_RSS_PROC_CTRL_RSS_CR4_BOOT_INFO_REG0) & 0x00040000) == 0x00040000)
    {
        DebugP_log("BSS is powered up...\r\n");
    }
    else
    {
        DebugP_log("BSS is not powered up...\r\n");
        DebugP_assert(0);
    }

    rlVersion_t verArgs;
    retVal = rlDeviceGetVersion(RL_DEVICE_MAP_INTERNAL_BSS, &verArgs);
    if (retVal != 0)
    {
        DebugP_log("Error: Unable to get the device version from mmWave link [Error %d]\n", retVal);
        return -1;
    }

    /* Link has been setup successfully. */
    retVal = 0;

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to open the mmWave Link module.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[in] ptrCalibrationData
 *      Optional pointer to the calibration data which needs to be
 *      restored.
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_openLink()
{
    int32_t                   retVal = MINUS_ONE;
    rlRfCalMonFreqLimitConf_t freqLimit;
    rlRfInitCalConf_t         rfInitCalib;
    rlRfCalMonTimeUntConf_t   timeCfg;
    // rlRfDevCfg_t                asyncEventCfg;

    /* Initialize the configurations: */
    memset((void *)&freqLimit, 0, sizeof(rlRfCalMonFreqLimitConf_t));
    memset((void *)&rfInitCalib, 0, sizeof(rlRfInitCalConf_t));
    memset((void *)&timeCfg, 0, sizeof(rlRfCalMonTimeUntConf_t));

    /* Link is not operational: */
    gMmw_MCB.cfg.linkStatus = 0U;

    /****************************************************************************************
     * Setup the RF Calibration Time unit:
     ****************************************************************************************/
    if (gMmw_MCB.cfg.openCfg.calibMonTimeUnit != 0)
    {
        timeCfg.numOfCascadeDev  = 1;
        timeCfg.calibMonTimeUnit = gMmw_MCB.cfg.openCfg.calibMonTimeUnit;
        retVal                   = rlRfSetCalMonTimeUnitConfig(RL_DEVICE_MAP_INTERNAL_BSS, &timeCfg);
        if (retVal != RL_RET_CODE_OK)
        {
            /* Error: Unable to set the calibration time unit */
            MmwDemo_printError(MMWAVE_ECALPERIOD, retVal);
        }
    }
    else
    {
        // Error: invalid value provided for calibMonTimeUnit
        MmwDemo_printError(MMWAVE_EINVALIDCALMONUNIT, 0);
    }

    /****************************************************************************************
     * Setup the RF Calibration Frequency limit:
     ****************************************************************************************/
    freqLimit.freqLimitLow  = gMmw_MCB.cfg.openCfg.freqLimitLow;
    freqLimit.freqLimitHigh = gMmw_MCB.cfg.openCfg.freqLimitHigh;
    retVal                  = rlRfSetCalMonFreqLimitConfig(RL_DEVICE_MAP_INTERNAL_BSS, &freqLimit);
    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Frequency Limit Calibration  */
        MmwDemo_printError(MMWAVE_ECALCFG, retVal);
    }

    /* Are we supporting custom calibration or not? */
    if (gMmw_MCB.cfg.openCfg.useCustomCalibration == true)
    {
        /*********************************************************************
         * Custom Calibration: Use the application provided calibration mask
         *********************************************************************/
        rfInitCalib.calibEnMask = gMmw_MCB.cfg.openCfg.customCalibrationEnableMask;
    }
    else
    {
        /*********************************************************************
         * Default: Enable all calibrations
         *********************************************************************/
        rfInitCalib.calibEnMask = CSL_FMKR(4U, 4U, 1U) | /* LODIST calibration            */
            CSL_FMKR(5U, 5U, 1U) | /* RX ADC DC offset calibration  */
            CSL_FMKR(6U, 6U, 1U) | /* HPF cutoff calibration        */
            CSL_FMKR(7U, 7U, 1U) | /* LPF cutoff calibration        */
            CSL_FMKR(8U, 8U, 1U) | /* Peak detector calibration     */
            CSL_FMKR(9U, 9U, 1U) | /* TX Power calibration          */
            CSL_FMKR(10U, 10U, 1U) | /* RX gain calibration           */
            CSL_FMKR(11U, 11U, 1U) | /* TX phase calibration          */
            CSL_FMKR(12U, 12U, 1U); /* RX IQMM calibration           */
    }
    retVal = rlRfInitCalibConfig(RL_DEVICE_MAP_INTERNAL_BSS, &rfInitCalib);
    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Calibration init failed */
        MmwDemo_printError(MMWAVE_ECALINIT, retVal);
    }

    /****************************************************************************************
     * Set the channel configuration:
     ****************************************************************************************/
    retVal = rlSetChannelConfig(RL_DEVICE_MAP_INTERNAL_BSS, &gMmw_MCB.cfg.openCfg.chCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        // Error: mmWave link channel configuration failed
        MmwDemo_printError(MMWAVE_ECHCFG, retVal);
    }

    /****************************************************************************************
     * Set the ADC Output configuration:
     ****************************************************************************************/
    retVal = rlSetAdcOutConfig(RL_DEVICE_MAP_INTERNAL_BSS, &gMmw_MCB.cfg.openCfg.adcOutCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Set the ADC configuration failed */
        MmwDemo_printError(MMWAVE_EADCCFG, retVal);
    }

    /****************************************************************************************
     * Set the low power mode configuration:
     ****************************************************************************************/
    retVal = rlSetLowPowerModeConfig(RL_DEVICE_MAP_INTERNAL_BSS, &gMmw_MCB.cfg.openCfg.lowPowerMode);
    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Set the Low power configuration failed */
        MmwDemo_printError(MMWAVE_EPOWERCFG, retVal);
    }

    /****************************************************************************************
     * Asynchronous event configuration:
     ****************************************************************************************/
    retVal = MMWave_deviceCfgAsyncEvent();
    if (retVal < 0)
    {
        /* Error: Asynchronous Event configuration failed; error code is already setup */
        goto exit;
    }

    /****************************************************************************************
     * Do we need to restore calibration data?
     ****************************************************************************************/
    if (gMmw_MCB.cfg.calibrationDataCfg.ptrCalibData != NULL)
    {
        /* YES: Restore the calibration data */
        retVal = rlRfCalibDataRestore(RL_DEVICE_MAP_INTERNAL_BSS, gMmw_MCB.cfg.calibrationDataCfg.ptrCalibData);
        if (retVal != RL_RET_CODE_OK)
        {
            // Error: Calibration data restore failed
            MmwDemo_printError(MMWAVE_ECALFAIL, retVal);
        }

        /* As per the Link documentation; radarSS will consume the data and will send an
         * asynchronous message to indicate the result of the calibration based on the
         * calibration data which has been restored. So we need to wait on the status of
         * the operation. */
        MMWaveLink_internalSync(MMWAVE_RF_INITIALIZED | MMWAVE_RF_INIT_FAILED);

        /* Was the calibration restored succesfully? */
        if (gMmw_MCB.cfg.linkStatus & MMWAVE_RF_INIT_FAILED)
        {
            /* NO: Calibration was not restored. The asynchronous message has been
             * passed to the application too. Setup the error code */
            goto exit;
        }

        /* Calibration was restored successfully so we need to reset the link operational
         * status again. */
        gMmw_MCB.cfg.linkStatus = 0U;
    }

    /* Re-store Phase shift calibration. */
    if (gMmw_MCB.cfg.calibrationDataCfg.ptrPhaseShiftCalibData != NULL)
    {
        /* User should have set txIndex equal to PhShiftcalibChunk array index when calling
            rlRfPhShiftCalibDataStore() so that data for all Tx are obtained regardless of
            channels enabled in chCfg. However, phase shift data will be non-zero only
            for Tx channels that are enabled in chCfg */
        if ((gMmw_MCB.cfg.calibrationDataCfg.ptrPhaseShiftCalibData->PhShiftcalibChunk[0].txIndex != 0) ||
            (gMmw_MCB.cfg.calibrationDataCfg.ptrPhaseShiftCalibData->PhShiftcalibChunk[1].txIndex != 1) ||
            (gMmw_MCB.cfg.calibrationDataCfg.ptrPhaseShiftCalibData->PhShiftcalibChunk[2].txIndex != 2) ||
            (gMmw_MCB.cfg.calibrationDataCfg.ptrPhaseShiftCalibData->PhShiftcalibChunk[3].txIndex != 3))
        {
            MmwDemo_printError(MMWAVE_EPHASESHIFTCALDATARESTOREFAIL, 0);
        }
        /* User should not set calibApply to 1 for any Tx. rlRfPhShiftCalibDataRestore() will take care of
            it internally */
        if ((gMmw_MCB.cfg.calibrationDataCfg.ptrPhaseShiftCalibData->PhShiftcalibChunk[0].calibApply != 0) ||
            (gMmw_MCB.cfg.calibrationDataCfg.ptrPhaseShiftCalibData->PhShiftcalibChunk[1].calibApply != 0) ||
            (gMmw_MCB.cfg.calibrationDataCfg.ptrPhaseShiftCalibData->PhShiftcalibChunk[2].calibApply != 0) ||
            (gMmw_MCB.cfg.calibrationDataCfg.ptrPhaseShiftCalibData->PhShiftcalibChunk[3].calibApply != 0))
        {
            MmwDemo_printError(MMWAVE_EPHASESHIFTCALDATARESTOREFAIL, retVal);
        }
        /* Basic validation passed: Restore the phase shift calibration data */
        retVal = rlRfPhShiftCalibDataRestore(RL_DEVICE_MAP_INTERNAL_BSS, gMmw_MCB.cfg.calibrationDataCfg.ptrPhaseShiftCalibData);
        if (retVal != RL_RET_CODE_OK)
        {
            /* Error: Phase shift Calibration data restore failed */
            MmwDemo_printError(MMWAVE_EPHASESHIFTCALDATARESTOREFAIL, retVal);
        }
    }

    /****************************************************************************************
     * Initialize the RF:
     ****************************************************************************************/
    retVal = rlRfInit(RL_DEVICE_MAP_INTERNAL_BSS);
    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: RF Initialization failed */
        MmwDemo_printError(MMWAVE_ERFINIT, retVal);
    }

    /***************************************************************************
     * SYNCHRONIZATION: We need to loop around till the BSS has performed the
     * RF Initialization; either a success or failure.
     ***************************************************************************/
    MMWaveLink_internalSync(MMWAVE_RF_INITIALIZED | MMWAVE_RF_INIT_FAILED);

    /* Was the RF Initialized succesfully? */
    if (gMmw_MCB.cfg.linkStatus & MMWAVE_RF_INIT_FAILED)
    {
        /* NO: The RF initialization failed. The asynchronous event was passed to the application
         * The application can decode the exact reason for the failure. We simply set the error
         * code and report failure */
        MmwDemo_printError(MMWAVE_ECALFAIL, 0);
    }

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the mmWave link with the supplied
 *      configuration
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[in]  ptrControlCfg
 *      Pointer to the control configuration
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_configLink()
{
    int32_t retVal;

    /* Determine the DFE Output mode? */
    switch (gMmw_MCB.cfg.ctrlCfg.dfeDataOutputMode)
    {
        case MMWave_DFEDataOutputMode_FRAME:
        {
            /**************************************************************************
             * Frame Mode:
             * Order of operations as specified by the mmWave Link are
             *  - Profile configuration
             *  - Chirp configuration
             *  - Frame configuration
             **************************************************************************/
            retVal = MmwDemo_loadProfileConfig();

            if (retVal < 0)
            {
                goto exit;
            }

            retVal = MmwDemo_loadChirpConfig();

            if (retVal < 0)
            {
                goto exit;
            }

            /* BPM configuration is for all devices. */
#if 0
            retVal = MMWave_configBPM();

            if(retVal < 0)
            {
                retVal = -1;
                goto end;
            }
#endif

            retVal = MmwDemo_loadFrameConfig();

            /* Phase Shifter configuration is for all devices. */
            retVal = MmwDemo_loadPhaseShiftConfig();

            if (retVal < 0)
            {
                retVal = -1;
                goto exit;
            }

            /* Set the frame configuration: */
            DebugP_logInfo("rlSetFrameConfig...\n");
            retVal = rlSetFrameConfig(RL_DEVICE_MAP_INTERNAL_BSS, &gMmw_MCB.cfg.ctrlCfg.u.frameCfg[0].frameCfg);
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Setting the frame configuration failed */
                DebugP_logInfo("rlSetFrameConfig error %d\n", retVal);

                // *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
                retVal = MINUS_ONE;
                goto exit;
            }
            break;
        }
#ifdef HECR_ENABLE_ADV_CHIRP
        case MMWave_DFEDataOutputMode_ADVANCED_CHIRP_LEGACY_FRAME:
        {
            /**************************************************************************
             * Advanced Chirp with normal frame Configuration:
             * Order of operations as specified by the mmWave Link are
             *  - Profile configuration
             *  - Advanced Chirp configuration
             *  - LUT Configuration
             *  - Frame configuration
             **************************************************************************/
            retVal = MmwDemo_loadProfileConfig();
            if (retVal < 0)
            {
                goto exit;
            }

            retVal = MmwDemo_loadAdvChirpConfig();
            if (retVal < 0)
            {
                goto exit;
            }

            retVal = MmwDemo_loadChirpConfig();
            if (retVal < 0)
            {
                goto exit;
            }

            /* Set the frame configuration: */
            MmwDemo_loadFrameConfig();
            retVal = rlSetFrameConfig(RL_DEVICE_MAP_INTERNAL_BSS, &gMmw_MCB.cfg.ctrlCfg.u.frameCfg[0].frameCfg);
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Setting the frame configuration failed */
                MmwDemo_printError(MMWAVE_EFRAMECFG, retVal);
            }
            break;
        }
#endif
        default:
        {
            /* Error: This should never occur and the user seems to have ignored a warning. */
            MmwDemo_printError(MMWAVE_EINVAL, 0);
        }
    }

    /* Set the return value to be success. */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to start the mmWave link. This function can only be
 *      invoked once the configuration has been completed successfully.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_startLink()
{
    rlRunTimeCalibConf_t runTimeCalib;
    // rlContModeEn_t          contModeEnable;

    int32_t retVal;

    /* Sensor is being started; reset the link status to get rid of the calibration flags. */
    gMmw_MCB.cfg.linkStatus &= ~(uint32_t)MMWAVE_RF_CALIBRATION_DONE;
    gMmw_MCB.cfg.linkStatus &= ~(uint32_t)MMWAVE_RF_CALIBRATION_FAILED;

    /* Initialize the runtime calibration configuration: */
    memset((void *)&runTimeCalib, 0, sizeof(rlRunTimeCalibConf_t));

    /* Enable calibration: */
    runTimeCalib.reportEn           = gMmw_MCB.cfg.calibrationCfg.u.chirpCalibrationCfg.reportEn;
    runTimeCalib.oneTimeCalibEnMask = CSL_FMKR(4U, 4U, 1U) | /* LODIST calibration   */
        CSL_FMKR(8U, 8U, 1U) | /* PD calibration */
        CSL_FMKR(9U, 9U, 1U) | /* TX Power calibration */
        CSL_FMKR(10U, 10U, 1U); /* RX gain calibration  */

    /* Are we operating in Chirp or Continuous mode? */
    if ((gMmw_MCB.cfg.calibrationCfg.dfeDataOutputMode == MMWave_DFEDataOutputMode_FRAME) ||
        (gMmw_MCB.cfg.calibrationCfg.dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME) ||
        (gMmw_MCB.cfg.calibrationCfg.dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_CHIRP_LEGACY_FRAME) ||
        (gMmw_MCB.cfg.calibrationCfg.dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_CHIRP_ADVANCED_FRAME))
    {
        /******************************************************************************
         * FRAME Mode: Is calibration enabled?
         ******************************************************************************/
        if (gMmw_MCB.cfg.calibrationCfg.u.chirpCalibrationCfg.enableCalibration == true)
        {
            /* Do we need to configure periodic calibration? */
            if (gMmw_MCB.cfg.calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity == true)
            {
                /* YES: Enable all calibrations with the configured periodicity */
                runTimeCalib.periodicCalibEnMask = runTimeCalib.oneTimeCalibEnMask;
                runTimeCalib.calibPeriodicity    = gMmw_MCB.cfg.calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames;
            }

            /* Configure the calibrations: */
            retVal = rlRfRunTimeCalibConfig(RL_DEVICE_MAP_INTERNAL_BSS, &runTimeCalib);
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Runtime calibration failed */
                MmwDemo_printError(MMWAVE_ECALTRIG, retVal);
            }

            /* Check if application requested for checking run-time calibration report. */
            if (gMmw_MCB.cfg.calibrationCfg.u.chirpCalibrationCfg.reportEn == 1)
            {
                /***************************************************************************
                 * SYNCHRONIZATION: We need to loop around till the BSS has completed the
                 * ONE SHOT calibration [Done or Failed]. The application would have
                 * received an asynchronous event through the callback function about the
                 * failure.
                 ***************************************************************************/
                MMWaveLink_internalSync(MMWAVE_RF_CALIBRATION_DONE | MMWAVE_RF_CALIBRATION_FAILED);

                /* Did the calibration fail? */
                if (gMmw_MCB.cfg.linkStatus & MMWAVE_RF_CALIBRATION_FAILED)
                {
                    /* YES: Calibration failed. Abort and don't start the sensor. */
                    MmwDemo_printError(MMWAVE_ECALFAIL, 0);
                }
            }
        }

        /* Start device */
        retVal = rlSensorStart(RL_DEVICE_MAP_INTERNAL_BSS);

        if (retVal != RL_RET_CODE_OK)
        {
            /* Error: Starting the sensor failed */
            MmwDemo_printError(MMWAVE_ESENSOR, retVal);
        }
    }

    /* Control comes here indicates that the sensor has been started successfully */
    retVal = 0;

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test RF Init/Calibration API.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_rfCalibration(void)
{
    int32_t retVal;

    /* RF Initialization */
    retVal = rlRfInit(RL_DEVICE_MAP_INTERNAL_BSS);
    if (retVal != 0)
    {
        DebugP_log("Error: Unable to start RF [Error %d]\n", retVal);
        return -1;
    }
    while (gInitTimeCalibStatus == 0U)
    {
        /* Sleep and poll again: */
        ClockP_usleep(1);
    }
    gInitTimeCalibStatus = 0U;

#ifdef DEBUG_LINK
    printf("Debug: RF start successfully\n");
#endif

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test start sensor API.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_sensorStart(void)
{
    int32_t retVal;

    /* Trigger the frame */
    retVal = rlSensorStart(RL_DEVICE_MAP_INTERNAL_BSS);
    if (retVal != 0)
    {
        DebugP_log("Error: Unable to start Sensor [Error %d]\n", retVal);
        return -1;
    }
    /* if frame start async event is enable by rlRfSetDeviceCfg API,
       then wait for Frame start async event */
    if ((rfDevCfg.aeControl & 0x1) == 0x0)
    {
        while (gFrameStartStatus == 0U)
        {
            /* Sleep and poll again: */
            ClockP_usleep(1);
        }
    }
    return 0;
}


/**
 *  @b Description
 *  @n
 *      The function is used to test stop sensor API.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_sensorStop(void)
{
    int32_t retVal;

    /* Stop the frame */
    retVal = rlSensorStop(RL_DEVICE_MAP_INTERNAL_BSS);
    if (retVal != 0)
    {
        if (retVal == RL_RET_CODE_FRAME_ALREADY_ENDED)
        {
            DebugP_log("Debug: Frames are already stopped  [%d]\n", retVal);
            return 0;
        }
        DebugP_log("Error: Unable to stop Sensor [Error %d]\n", retVal);
        return -1;
    }

    /* if frame stop async event is enable by rlRfSetDeviceCfg API,
        then wait for Frame stop async event */
    if ((rfDevCfg.aeControl & 0x2) == 0x0)
    {
        while (gFrameStartStatus == 1U)
        {
            /* Sleep and poll again: */
            ClockP_usleep(1);
        }
    }
    DebugP_log("Debug: Sensor stop successfully\n");

    return 0;
}

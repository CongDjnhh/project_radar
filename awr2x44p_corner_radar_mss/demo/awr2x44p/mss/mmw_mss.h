/**
 *   @file  mmw_mss.h
 *
 *   @brief
 *      This is the main header file for the Millimeter Wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016-2021 Texas Instruments, Inc.
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
#ifndef MMW_MSS_H
#define MMW_MSS_H

/* MCU + SDK Files*/
#include <drivers/uart.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include "FreeRTOS.h"
#include "task.h"

/* mmWave SDK Include Files*/
#include <ti/common/syscommon.h>
#include <ti/common/mmwave_error.h>
#include <ti/control/mmwave/mmwave.h>

/* Demo Datapath Include Files*/
#include "../../datapath/dpc/objdethwaDDMA/objectdetection.h"

/* Demo Include Files*/
#include "../../utils/mmwdemo_adcconfig.h"
#include "../../utils/mmwdemo_monitor.h"
#include "../common/mmw_output.h"
#include "../common/mmw_config.h"
#include "../common/mmw_lvds_stream.h"
#include "../common/mmw_common.h"

#ifdef ENET_STREAM
#include <networking/lwip/lwip-stack/src/include/lwip/tcpip.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/*! @brief For advanced frame config, below define means the configuration given is
 * global at frame level and therefore it is broadcast to all sub-frames.
 */
#define MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG (-1)

/*! @brief CFAR threshold encoding factor
 */
#define MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR (100.0)

/**
 * @defgroup configStoreOffsets     Offsets for storing CLI configuration
 * @brief    Offsets of config fields within the parent structures, note these offsets will be
 *           unique and hence can be used to differentiate the commands for processing purposes.
 * @{
 */
#define MMWDEMO_GUIMONSEL_OFFSET (offsetof(MmwDemo_SubFrameCfg, guiMonSel))
#define MMWDEMO_ADCBUFCFG_OFFSET (offsetof(MmwDemo_SubFrameCfg, adcBufCfg))
#ifdef LVDS_STREAM
#define MMWDEMO_LVDSSTREAMCFG_OFFSET (offsetof(MmwDemo_SubFrameCfg, lvdsStreamCfg))
#endif
#define MMWDEMO_SUBFRAME_STATICCFG_OFFSET (offsetof(MmwDemo_SubFrameCfg, datapathStaticCfg))
#define MMWDEMO_CFARDOPPLERCFG_OFFSET     MMWDEMO_SUBFRAME_STATICCFG_OFFSET + \
    (offsetof(MmwDemo_datapathCfg, cfarCfg)) + \
    (offsetof(DPC_ObjectDetection_CfarCfg, cfg))
#define MMWDEMO_CFARCFGRANGE_OFFSET MMWDEMO_SUBFRAME_STATICCFG_OFFSET + \
    (offsetof(MmwDemo_datapathCfg, rangeCfarCfg)) + \
    (offsetof(DPC_ObjectDetection_CfarCfg, cfg))
#define MMWDEMO_COMPRESSIONCFG_OFFSET MMWDEMO_SUBFRAME_STATICCFG_OFFSET + \
    (offsetof(MmwDemo_datapathCfg, compressionCfg))
#define MMWDEMO_INTFMITIGCFG_OFFSET MMWDEMO_SUBFRAME_STATICCFG_OFFSET + \
    (offsetof(MmwDemo_datapathCfg, intfStatsdBCfg))
#define MMWDEMO_FASTRANGEPROCCFG_OFFSET MMWDEMO_SUBFRAME_STATICCFG_OFFSET + \
    (offsetof(MmwDemo_datapathCfg, fastRangeProcCfg))
#define MMWDEMO_LOCALMAXCFG_OFFSET MMWDEMO_SUBFRAME_STATICCFG_OFFSET + \
    (offsetof(MmwDemo_datapathCfg, localMaxCfg))
#define MMWDEMO_FOVAOA_OFFSET (MMWDEMO_SUBFRAME_STATICCFG_OFFSET + \
                               offsetof(MmwDemo_datapathCfg, aoaFovCfg))

    /** @}*/ /* configStoreOffsets */

    /**
     * @brief
     *  Millimeter Wave Demo Sensor State
     *
     * @details
     *  The enumeration is used to define the sensor states used in mmwDemo
     */
    typedef enum MmwDemo_SensorState_e
    {
        /*!  @brief Inital state after sensor is initialized.
         */
        MmwDemo_SensorState_INIT = 0,

        /*!  @brief Inital state after sensor is post RF init.
         */
        MmwDemo_SensorState_OPENED,

        /*!  @brief Indicates sensor is started */
        MmwDemo_SensorState_STARTED,

        /*!  @brief  State after sensor has completely stopped */
        MmwDemo_SensorState_STOPPED
    } MmwDemo_SensorState;

    /**
     * @brief
     *  Millimeter Wave Demo statistics
     *
     * @details
     *  The structure is used to hold the statistics information for the
     *  Millimeter Wave demo
     */
    typedef struct MmwDemo_MSS_Stats_t
    {
        /*! @brief   Counter which tracks the number of frame trigger events from BSS */
        uint64_t frameTriggerReady;

        /*! @brief   Counter which tracks the number of failed calibration reports
         *           The event is triggered by an asynchronous event from the BSS */
        uint32_t failedTimingReports;

        /*! @brief   Counter which tracks the number of calibration reports received
         *           The event is triggered by an asynchronous event from the BSS */
        uint32_t calibrationReports;

        /*! @brief   Counter which tracks the number of sensor stop events received
         *           The event is triggered by an asynchronous event from the BSS */
        uint32_t sensorStopped;

        /*! @brief   Flag to indicate status of previous Frame object data sent over
         *           UART is complete.
         *           True  - Object data sending over UART completed
         *           False - Object data sending over UART is pending*/
        volatile Bool isLastFrameDataProcessed;
    } MmwDemo_MSS_Stats;

    /*!
     * @brief
     * Structure holds message stats information from data path.
     *
     * @details
     *  The structure holds stats information. This is a payload of the TLV message item
     *  that holds stats information.
     */
    typedef struct MmwDemo_SubFrameStats_t
    {
        /*! @brief   Frame processing stats */
        MmwDemo_output_message_stats outputStats;

        /*! @brief   Dynamic CLI configuration time in usec */
        uint32_t pendingConfigProcTime;

        /*! @brief   SubFrame Preparation time on MSS in usec */
        uint32_t subFramePreparationTime;
    } MmwDemo_SubFrameStats;

    /**
     * @brief Task handles storage structure
     */
    typedef struct MmwDemo_TaskHandles_t
    {
        /*! @brief   MMWAVE Control Task Handle */
        TaskHandle_t mmwCtrlTask;
        StaticTask_t mmwCtrlTaskObj;

        /*! @brief   ObjectDetection DPC related dpmTask */
        TaskHandle_t mmwObjDetDpmTask;
        StaticTask_t mmwObjDetDpmTaskObj;

        /*! @brief   UART Data Export related Task */
        TaskHandle_t uartDataExportTask;
        StaticTask_t uartDataExportTaskObj;

        /*! @brief   Demo init task */
        TaskHandle_t initTask;
        StaticTask_t initTaskObj;

        /*! @brief   Task for ethernet streaming of object data */
        TaskHandle_t enetTask;
        StaticTask_t enetTaskObj;

    } MmwDemo_taskHandles;

    /*!
     * @brief
     * Structure holds temperature information from Radar front end.
     *
     * @details
     *  The structure holds temperature stats information.
     */
    typedef struct MmwDemo_temperatureStats_t
    {

        /*! @brief   retVal from API rlRfTempData_t - can be used to know
                     if values in temperatureReport are valid */
        int32_t tempReportValid;

        /*! @brief   detailed temperature report - snapshot taken just
                     before shipping data over UART */
        rlRfTempData_t temperatureReport;

    } MmwDemo_temperatureStats;

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

#ifdef ENET_STREAM
    /*!
     * @brief
     * Structure holds ethernet related configuration
     *
     * @details
     *  Structure holds ethernet related configuration for detected objects related data streaming
     */
    typedef struct MmwDemo_enetCfg_t
    {

        /*! @brief      Enable/Disable calibration save process  */
        bool streamEnable;

        /*! @brief      Calibration data header for validation read from flash */
        ip_addr_t localIp;

        /*! @brief      Calibration data header for validation read from flash */
        ip_addr_t remoteIp;

        /*! @brief      Status */
        bool status;

        /*! @brief      Semaphore for enet configuration */
        SemaphoreP_Object EnetCfgDoneSemHandle;

    } MmwDemo_enetCfg;
#endif

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

    /*!
     * @brief
     * Structure holds Spread Spectrum configuration.
     *
     * @details
     *  The structure holds Spread Spectrum configuration.
     */
    typedef struct MmwDemo_spreadSpectrumConfig_t
    {
        /*! @brief      Moduldation depth in percentage */
        float modDepth;

        /*! @brief      flag to check the valid Config */
        bool isEnable;

        /*! @brief      Moduldation rate in KHz */
        uint8_t modRate;

        /*! @brief      downSpread */
        uint8_t downSpread;
    } MmwDemo_spreadSpectrumConfig;

    typedef struct MmwDemo_trackerRansacConfig_t
    {
        bool                    ransacEnabled;
        uint16_t                ransacIterations;
        float                   ransacThresh;
        void                   *gHTrackModule;
        bool                    gtrackEnabled;
        bool                    gtrackInstanceCreated;
        uint16_t                maxNumPoints;
        uint16_t                maxNumTracks;
        float                   deltaT;
        GTRACK_sceneryParams    appSceneryParams;
        GTRACK_gatingParams     appGatingParams;
        GTRACK_stateParams      appStateParams;
        GTRACK_allocationParams appAllocationParams;
    } MmwDemo_trackerRansacConfig;

    /* Power optimization configuration */
    typedef struct MmwDemo_powerOptConfig_t
    {
        uint32_t dspStateAfterFrameProc;
        uint32_t isHwaDynamicClkGate;
        uint32_t hwaStateAfterFrameProc;
    } MmwDemo_powerOptConfig;

    /* Config for Errata ANA#46: Spurs caused due to data transfer activity */
    typedef struct MmwDemo_adcDataDithDelayCfg_t
    {
        uint8_t     isDitherEn;
        uint16_t    ditherVal;
        HwiP_Object chirpAvailHwiObject;
    } MmwDemo_adcDataDithDelayCfg;
    /**
     * @brief
     *  Millimeter Wave Demo MCB
     *
     * @details
     *  The structure is used to hold all the relevant information for the
     *  Millimeter Wave demo.
     */
    typedef struct MmwDemo_MSS_MCB_t
    {
        /*! @brief      Configuration which is used to execute the demo */
        MmwDemo_Cfg cfg;

#ifdef MMWAVELINK_ON_MSS
        /*! @brief      Object Detection DPC subFrame configuration */
        MmwDemo_SubFrameCfg subFrameCfg[RL_MAX_SUBFRAMES];

#ifdef LVDS_STREAM
        /*! @brief   this structure is used to hold all the relevant information
             for the mmw demo LVDS stream*/
        MmwDemo_LVDSStream_MCB_t lvdsStream;
#endif

#ifdef MMWDEMO_DDM
        /*! @brief  Number of empty subbands */
        uint8_t numEmptySubBands;

        /*! @brief antenna indices in increasing order of phase shift value. */
        uint8_t ddmPhaseShiftOrder[SYS_COMMON_NUM_TX_ANTENNAS];
#endif

        /*! @brief      Object Detection DPC common configuration */
        MmwDemo_DPC_ObjDet_CommonCfg objDetCommonCfg;

        /*! @brief      ADCBuf driver handle */
        ADCBuf_Handle adcBufHandle;

        /*! @brief   Rf frequency scale factor, = 2.7 for 60GHz device, = 3.6 for 76GHz device */
        double rfFreqScaleFactor;
#else
    /*! @brief      Config sent from DSS when mmWaveLink runs on DSS */
    mmwDemo_cfgToMss cfgFromDss;


#endif

        /*! @brief      UART Logging Handle */
        UART_Handle loggingUartHandle;

        /*! @brief      UART Command Rx/Tx Handle */
        UART_Handle commandUartHandle;

        /*! @brief      This is the mmWave control handle which is used
         * to configure the BSS. */
        MMWave_Handle ctrlHandle;

        /*! @brief   Handle of the EDMA driver, used for CBUFF */
        EDMA_Handle edmaHandle;

        /*! @brief   Number of EDMA event Queues (tc) */
        uint8_t numEdmaEventQueues;

        /*! @brief   True if need to poll for edma error (because error interrupt is not
         *           connected to the CPU on the device */
        bool isPollEdmaError;

        /*! @brief   True if need to poll for edma transfer controller error
         *           because at least one of the transfer controller error interrupts
         *           are not connected to the CPU on the device */
        bool isPollEdmaTransferControllerError;

        /*! @brief      DPM Handle */
        DPM_Handle objDetDpmHandle;

        /*! @brief      sub-frame stats */
        MmwDemo_SubFrameStats subFrameStats[RL_MAX_SUBFRAMES];

        /*! @brief      Demo Stats */
        MmwDemo_MSS_Stats stats;

        /*! @brief      Task handle storage */
        MmwDemo_taskHandles taskHandles;

        /*! @brief   Semaphore Object to signal DPM started from DPM report function */
        SemaphoreP_Object DPMstartSemHandle;

        /*! @brief   Semaphore Object to signal DPM stopped from DPM report function. */
        SemaphoreP_Object DPMstopSemHandle;

        /*! @brief   Semaphore Object to signal DPM ioctl from DPM report function. */
        SemaphoreP_Object DPMioctlSemHandle;

        /*! @brief   Semaphore Object to signal UART export function. */
        SemaphoreP_Object UartExportSemHandle;

        /*! @brief   Semaphore Object to pend main task */
        SemaphoreP_Object demoInitTaskCompleteSemHandle;

        /**
         * @brief   This is the semaphore handle which is used to handle the messages exchanged
         * between the MSS/DSS and BSS.
         */
        SemaphoreP_Object linkSemHandle;

        /*! @brief    Sensor state */
        MmwDemo_SensorState sensorState;

        /*! @brief   Tracks the number of sensor start */
        uint32_t sensorStartCount;

        /*! @brief   Tracks the number of sensor sop */
        uint32_t sensorStopCount;

        /*! @brief   CQ monitor configuration - Signal Image band data */
        rlSigImgMonConf_t cqSigImgMonCfg[RL_MAX_PROFILES_CNT];

        /*! @brief   CQ monitor configuration - Saturation data */
        rlRxSatMonConf_t cqSatMonCfg[RL_MAX_PROFILES_CNT];

        /*! @brief   Analog monitor bit mask */
        MmwDemo_AnaMonitorCfg anaMonCfg;

        /*! @brief   this structure is used to hold all the relevant information
         for the temperature report*/
        MmwDemo_temperatureStats temperatureStats;

        /*! @brief   Calibration cofiguration for save/restore */
        MmwDemo_calibCfg calibCfg;

#ifdef ENET_STREAM
        /*! @brief   Ethernet related cofiguration for object data streaming */
        MmwDemo_enetCfg enetCfg;
#endif

        /*! @brief Flag indicating if @ref anaMonCfg is pending processing. */
        uint8_t isAnaMonCfgPending : 1;

        /*! @brief Flag indicating if @ref calibCfg is pending processing. */
        uint8_t isCalibCfgPending : 1;

        /*! @brief ptr to Result Buffer. */
        volatile DPM_Buffer ptrResult;

        /*! @brief   this structure is used to hold all the relevant information
         for the Core ADPLL SSC Configuration*/
        MmwDemo_spreadSpectrumConfig coreAdpllSscCfg;

        /*! @brief   this structure is used to hold all the relevant information
         for the DSP ADPLL SSC Configuration*/
        MmwDemo_spreadSpectrumConfig dspAdpllSscCfg;

        /*! @brief   this structure is used to hold all the relevant information
         for the PER ADPLL SSC Configuration*/
        MmwDemo_spreadSpectrumConfig perAdpllSscCfg;

        DPC_ObjectDetection_ExecuteResult *result;

        /*! @brief this structure is used to store tracker configuration */
        MmwDemo_trackerRansacConfig trackerCfg;

        /*! @brief Power optimization config */
        MmwDemo_powerOptConfig powerOptCfg;

        /*! @brief ADC data dithering config */
        MmwDemo_adcDataDithDelayCfg adcDataDithDelayCfg;

    } MmwDemo_MSS_MCB;

    #define gMmwMssMCB gMmw_MCB

    /*!
     * @brief
     * Structure holds object data for ethernet streaming.
     *
     * @details
     *  Structure holds object data for ethernet streaming.
     */
    typedef struct MmwDemo_enetStreamObjData_t
    {
        /*! @brief      Object data */
        uint8_t objData[DPC_OBJDET_MAX_NUM_OBJECTS * sizeof(DPIF_PointCloudCartesian)];

        /*! @brief      Number of objects */
        uint32_t numObj;

    } MmwDemo_enetStreamObjData;

    /**************************************************************************
     *************************** Extern Definitions ***************************
     **************************************************************************/

#ifdef ENET_STREAM
    extern int32_t MmwDemo_mssEnetCfgDone(void);
#endif

    /* CLI and Sensor Control APIs */
    extern void    MmwDemo_CLIInit(uint8_t taskPriority);
    extern int32_t MmwDemo_openSensor(bool isFirstTimeOpen);
    extern int32_t MmwDemo_configSensor(void);
    extern int32_t MmwDemo_startSensor(void);
    extern void    MmwDemo_stopSensor(void);
    extern int32_t MmwDemo_dataPathConfig(void);
    extern int32_t MmwDemo_trackerConfig(void);

    /* functions to manage the dynamic configuration */
    extern uint8_t MmwDemo_isAllCfgInPendingState(void);
    extern uint8_t MmwDemo_isAllCfgInNonPendingState(void);
    extern void    MmwDemo_resetStaticCfgPendingState(void);
    extern void    MmwDemo_CfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum);
    extern void    MMWDemo_configSSC(void);

    /* Debug Functions */
    extern void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line);
#define MmwDemo_debugAssert(expression) \
    { \
        _MmwDemo_debugAssert(expression, \
                             __FILE__, \
                             __LINE__); \
        DebugP_assert(expression); \
    }

#ifdef __cplusplus
}
#endif

#endif /* MMW_MSS_H */

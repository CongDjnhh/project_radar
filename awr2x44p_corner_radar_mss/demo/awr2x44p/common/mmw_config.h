/**
 *   @file  mmw_config.h
 *
 *   @brief
 *      This is the header file that describes configurations for the Millimeter
 *   Wave Demo.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018-2021 Texas Instruments, Inc.
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
#ifndef MMW_CONFIG_H
#define MMW_CONFIG_H

/* MMWAVE library Include Files */
#include <ti/control/mmwave/mmwave.h>
#include <ti/common/syscommon.h>

/* Demo Include Files*/
#include "../../utils/mmwdemo_adcconfig.h"
#include "../../../datapath/dpc/objdethwaDDMA/objectdetection.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief
     *  Millimeter Wave Demo Gui Monitor Selection
     *
     * @details
     *  The structure contains the selection for what information is placed to
     *  the output packet, and sent out to GUI. Unless otherwise specified,
     *  if the flag is set to 1, information
     *  is sent out. If the flag is set to 0, information is not sent out.
     *
     */
    typedef struct MmwDemo_GuiMonSel_t
    {
        /*! @brief   if 1: Send list of detected objects (see @ref DPIF_PointCloudCartesian) and
         *                 side info (@ref DPIF_PointCloudSideInfo).\n
         *           if 2: Send list of detected objects only (no side info)\n
         *           if 0: Don't send anything */
        uint8_t detectedObjects;

        /*! @brief   Send log magnitude range array  */
        uint8_t logMagRange;

        /*! @brief   Send noise floor profile */
        uint8_t noiseProfile;

        /*! @brief   Send complex range bins at zero doppler, all antenna symbols for range-azimuth heat map */
        uint8_t rangeAzimuthHeatMap;

        /*! @brief   Send complex range bins at zero doppler, (all antenna symbols), for range-azimuth heat map */
        uint8_t rangeDopplerHeatMap;

        /*! @brief   Send stats */
        uint8_t statsInfo;

        /*! @brief   Send ransac filter mask */
        uint8_t ransacFilterMask;

        /*! @brief   Send trackers */
        uint8_t trackers;
    } MmwDemo_GuiMonSel;

    /**
     * @brief
     *  Millimeter Wave Demo CLI Config Structure
     *
     * @details
     *  The structure contains the profile and chirp configuration
     *  related information that will be used by the RF Parser
     *
     */
    typedef struct MmwDemo_WaveformCfg_t
    {
        /*! @brief   Number of profiles in use */
        rlUInt8_t numProfiles;

        /*! @brief   Number of chirps used for the configuration */
        rlUInt8_t numChirps;

        /*! @brief   Full profileCfg */
        rlProfileCfg_t profileCfg[6];

        /*! @brief   Full chirpCfg */
        rlChirpCfg_t chirpCfg[2];

#ifndef HECR_ENABLE_ADV_FRAME
        /*! @brief   Full frameCfg */
        rlFrameCfg_t frameCfg;
#else
    /**
     * @brief   Advanced Frame configuration
     */
    rlAdvFrameCfg_t frameCfg;
#endif

    } MmwDemo_WaveformCfg;

    /**
     * @brief
     *  LVDS streaming configuration
     *
     * @details
     *  The structure is used to hold all the relevant configuration
     *  for the LVDS streaming.
     */
    typedef struct MmwDemo_LvdsStreamCfg_t
    {
        /**
         * @brief  HSI Header enabled/disabled flag. Only applicable for HW streaming.
         *         Will be ignored for SW streaming which will always have HSI header.
         */
        bool isHeaderEnabled;

        /*! HW STREAMING DISABLED */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED 0

        /*! ADC */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_ADC 1

        /*! CP_ADC */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_CP_ADC 2

        /*! CP_ADC_CQ */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_CP_ADC_CQ 4

        /*! HW streaming data format:
            0-HW STREAMING DISABLED
            1-ADC
            2-Reserved
            3-Reserved
            4-CP_ADC_CQ
        */
        uint8_t dataFmt;

        /**
         * @brief  SW enabled/disabled flag
         */
        bool isSwEnabled;
    } MmwDemo_LvdsStreamCfg;

    /**
     * @brief
     *  Millimeter Wave Demo Data Path Information.
     *
     * @details
     *  The structure is used to hold all the relevant information for
     *  the data path.
     */
    typedef struct MmwDemo_platformCfg_t
    {
        /*! @brief   GPIO index for sensor status */
        uint32_t SensorStatusGPIO;

        /*! @brief   CPU Clock Frequency. */
        uint32_t sysClockFrequency;

        /*! @brief   UART Logging Baud Rate. */
        uint32_t loggingBaudRate;

        /*! @brief   UART Command Baud Rate. */
        uint32_t commandBaudRate;
    } MmwDemo_platformCfg;

    /**
     * @brief
     *  Millimeter Wave Demo configuration
     *
     * @details
     *  The structure is used to hold all the relevant configuration
     *  which is used to execute the Millimeter Wave Demo.
     */
    typedef struct MmwDemo_Cfg_t
    {
        /*! @brief   mmWave Control Configuration. */
        MMWave_CtrlCfg ctrlCfg;

        /*! @brief   mmWave Open Configuration. */
        MMWave_OpenCfg openCfg;

        /*! @brief   Platform specific configuration. */
        MmwDemo_platformCfg platformCfg;

        /*! @brief   Status of the mmWave Link: This can be changed from multiple execution contexts */
        volatile uint32_t linkStatus;


        /*! @brief   Datapath output loggerSetting
                        0 (default): MSS UART logger
                        1: DSS UART logger
            */
        uint8_t dataLogger;

        MMWave_CalibrationCfg  calibrationCfg;
        MMWave_CalibrationData calibrationDataCfg;
    } MmwDemo_Cfg;

#if defined(MMWDEMO_DDM)
    typedef struct MmwDemo_datapathCfg_t
    {
        /*! @brief     Doppler CFAR Cfg */
        DPC_ObjectDetection_CfarCfg cfarCfg;

        /*! @brief     Range CFAR Cfg */
        DPC_ObjectDetection_CfarCfg rangeCfarCfg;

        /*! @brief     Compression Cfg */
        DPU_RangeProcHWA_CompressionCfg compressionCfg;

        /*! @brief Shift/Scale config for Interf Stats Mag Diff in range DPU */
        // TODO: Also add isenabled
        DPU_RangeProcHWADDMA_intfStatsdBCfg intfStatsdBCfg;

        /*! @brief      LocalMax thresholds in doppler and azimuth dimensions */
        DPU_DopplerProc_LocalMaxCfg localMaxCfg;

        /*! @brief      Field of view config used in AoA estimation */
        DPC_ObjectDetection_FovAoaCfg aoaFovCfg;

    } MmwDemo_datapathCfg;
#endif

    typedef struct MmwDemo_DPC_ObjDet_CommonCfg_t
    {

        /*! @brief Flag indicating if new MeasureRxChannelBiasCfg configuration is
         *         pending issuance to DPC */
        uint8_t isMeasureRxChannelBiasCfgPending : 1;

#ifdef MMWDEMO_DDM
        /*! @brief Flag indicating if antennaCalib configuration is
         *         pending issuance to DPC */
        uint8_t isAntennaCalibParamCfgPending : 1;

        /*! @brief Flag indicating if antennaGeometry configuration is
         *         pending issuance to DPC */
        uint8_t isAntennaGeometryCfgPending : 1;
#endif

        /*! @brief pre start common config */
        DPC_ObjectDetection_PreStartCommonCfg preStartCommonCfg;
    } MmwDemo_DPC_ObjDet_CommonCfg;

    typedef struct Element_t
    {
        int value;
        int index;
    } Element;


    /**
     * @brief
     *  Millimeter Wave Demo Data Path Information.
     *
     * @details
     *  The structure is used to hold all the relevant information for
     *  the data path.
     */
    typedef struct MmwDemo_SubFrameCfg_t
    {
        /*! @brief ADC buffer configuration storage */
        MmwDemo_ADCBufCfg adcBufCfg;

        /*! @brief Flag indicating if @ref adcBufCfg is pending processing. */
        uint8_t isAdcBufCfgPending : 1;

#ifdef LVDS_STREAM
        /*! @brief  LVDS stream configuration */
        MmwDemo_LvdsStreamCfg lvdsStreamCfg;

        /*! @brief Flag indicating if @ref lvdsStreamCfg is pending processing. */
        uint8_t isLvdsStreamCfgPending : 1;
#endif

        /*! @brief GUI Monitor selection configuration storage from CLI */
        MmwDemo_GuiMonSel guiMonSel;

        /*! @brief  Number of range FFT bins, this is at a minimum the next power of 2 of
                    numAdcSamples. If range zoom is supported, this can be bigger than
                    the minimum. */
        uint16_t numRangeBins;

        /*! @brief  Number of Doppler FFT bins, this is at a minimum the next power of 2 of
                    numDopplerChirps. If Doppler zoom is supported, this can be bigger
                    than the minimum. */
        uint16_t numDopplerBins;

        /*! @brief  ADCBUF will generate chirp interrupt event every this many chirps - chirpthreshold */
        uint8_t numChirpsPerChirpEvent;

        /*! @brief  Number of bytes per RX channel, it is aligned to 16 bytes as required by ADCBuf driver  */
        uint32_t adcBufChanDataSize;

        /*! @brief CQ signal & image band monitor buffer size */
        uint32_t sigImgMonTotalSize;

        /*! @brief CQ RX Saturation monitor buffer size */
        uint32_t satMonTotalSize;

        /*! @brief  Number of ADC samples */
        uint16_t numAdcSamples;

        /*! @brief  Number of chirps per sub-frame */
        uint16_t numChirpsPerSubFrame;

        /*! @brief  Number of virtual antennas */
        uint8_t numVirtualAntennas;

#ifdef MMWDEMO_DDM
        /*! @brief   Datapath static configuration */
        MmwDemo_datapathCfg datapathStaticCfg;
#endif
    } MmwDemo_SubFrameCfg;

    /* this contains all the parameters which need to send to MSS,
        while mmWaveLink is running on DSS. */
    typedef struct mmwDemo_cfgToMss_t
    {
        /*! @brief  Number of range FFT bins, this is at a minimum the next power of 2 of
                    numAdcSamples. If range zoom is supported, this can be bigger than
                    the minimum. For two Subframe Configs */
        uint16_t numRangeBins[2];
        /*! @brief  Number of Doppler FFT bins, this is at a minimum the next power of 2 of
                    numDopplerChirps. If Doppler zoom is supported, this can be bigger
                    than the minimum. For two Subframe Configs */
        uint16_t numDopplerBins[2];

        uint16_t numDopFFTSubBins[2];
        uint8_t  txAntMask;
        uint8_t  numTxAnt;
        uint8_t  numSubFrames;
        uint8_t  measureRxChBiasCfgEn;
        /*! @brief GUI Monitor selection configuration storage from header file */
        MmwDemo_GuiMonSel guiMonSel;
    } mmwDemo_cfgToMss;

#ifdef __cplusplus
}
#endif

#endif /* MMW_CONFIG_H */

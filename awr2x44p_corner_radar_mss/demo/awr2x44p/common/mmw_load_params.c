/**
 *   @file  mmw_load_params.c
 *
 *   @brief
 *      Defines functions to load the configuration need for the demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2024 Texas Instruments, Inc.
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

#ifndef MMWAVELINK_ON_MSS

/* Standard Include Files*/
#include <stdbool.h>

/* SDK Control Layer Include Files*/
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/mathutils/mathutils.h>

/* Application layer Include Files */
#ifdef SUBSYS_MSS
#include "../mss/mmw_mss.h"
#elif defined(SUBSYS_DSS)
#include "../dss/mmw_dss.h"
#endif

#include "../common/mmw_config.h"
#include "../../utils/mmwdemo_adcconfig.h"
#include "../../utils/mmwdemo_rfparser.h"
#include "chirp_design_HECR.h"
#include <kernel/dpl/DebugP.h>


extern int32_t MmwDemo_openLink(void);
extern int32_t MmwDemo_configLink(void);
extern int32_t MmwDemo_sensorStart(void);
extern int32_t MmwDemo_startLink(void);
extern int32_t MmwDemo_calibSave(MmwDemo_calibDataHeader *ptrCalibDataHdr, MmwDemo_calibData *ptrCalibrationData);
extern int32_t MmwDemo_calibRestore(MmwDemo_calibData *calibrationData);

#if (defined(MMWAVELINK_ON_MSS) && defined(SUBSYS_MSS)) || (defined(SOC_AWR2X44P) && !defined(MMWAVELINK_ON_MSS) && defined(SUBSYS_DSS))
extern int32_t MmwDemo_DPM_ioctl_blocking(DPM_Handle handle, uint32_t cmd, void *arg, uint32_t argLen);
#endif

/* Global variables to hold datapath/demo parameters and configuration */
// extern uint8_t MmwDemo_RFParserPhaseShiftOrder[SYS_COMMON_NUM_TX_ANTENNAS];
extern MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg;

/*extern global for factory calibration*/
extern MmwDemo_calibData gCalibDataStoragetoMss;
extern MmwDemo_calibData gCalibDataStoragelinkMss;

/**
 * @brief
 *  Global Variable for holding profile and chirp cfg required by the mmw Demo RF parser
 */
#if (defined(MMWAVELINK_ON_MSS) && defined(SUBSYS_MSS)) || (!defined(MMWAVELINK_ON_MSS) && defined(SUBSYS_DSS))
//#pragma DATA_SECTION(gMmwWFCfg, ".customCode");
MmwDemo_WaveformCfg gMmwWFCfg = { 0 };
#endif

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
#if defined(SUBSYS_MSS)
extern MmwDemo_MSS_MCB gMmw_MCB;
#elif defined(SUBSYS_DSS)
extern MmwDemo_DSS_MCB gMmw_MCB;
#endif

#ifdef TRACKER_ON_MSS
int32_t MmwDemo_loadGtrackCfg(void);
void    MmwDemo_loadRansac(void);
void    MmwDemo_GAppAllocParams(void);
void    MmwDemo_GAppGatingParams(void);
void    MmwDemo_GAppStateParams(void);
void    MmwDemo_GAppSceneryParams(void);
#endif

/* ddmPhaseShiftAntOrder <Tx0> <Tx1> ... <TxN> */
#ifdef SOC_AWR2X44P
uint32_t ddm_phase_shift_order[4] = { 0, 1, 2, 3 };
#else
uint32_t               ddm_phase_shift_order[4] = { 0, 2, 3, 1 };
#endif

/* Application layer datapath config function */
#if defined(SUBSYS_DSS) && !defined(MMWAVELINK_ON_MSS)
int32_t MmwDemo_loadDatapathConfig(void) __attribute__((section(".customCode")));
int32_t MmwDemo_loadOpenChConfig(void) __attribute__((section(".customCode")));
int32_t MmwDemo_loadProfileConfig(void) __attribute__((section(".customCode")));
int32_t MmwDemo_loadChirpConfig(void) __attribute__((section(".customCode")));
int32_t MmwDemo_loadPhaseShiftConfig(void) __attribute__((section(".customCode")));
void    MmwDemo_loadFrameConfig(void) __attribute__((section(".customCode")));
void    MmwDemo_loadGUIMonConfig(void) __attribute__((section(".customCode")));
int32_t MmwDemo_loadADCBUFConfig(void) __attribute__((section(".customCode")));
int32_t MmwDemo_getNumEmptySubBands(uint32_t numTxAntennas) __attribute__((section(".customCode")));
void    MmwDemo_DataOutputMode(void) __attribute__((section(".customCode")));
void    MmwDemo_ChannelCfg(void) __attribute__((section(".customCode")));
void    MmwDemo_ADCCfg(void) __attribute__((section(".customCode")));
void    MmwDemo_loadFastRangeProcCfg(void) __attribute__((section(".customCode")));
int32_t MmwDemo_loadAntGeometryCfg(void) __attribute__((section(".customCode")));
int32_t MmwDemo_loadDDMPhaseShiftOrder(void) __attribute__((section(".customCode")));
#else
int32_t                MmwDemo_loadDatapathConfig(void); //__attribute__((section (".customCode")));
int32_t                MmwDemo_loadOpenChConfig(void); //__attribute__((section (".customCode")));
int32_t                MmwDemo_loadProfileConfig(void); //__attribute__((section (".customCode")));
int32_t                MmwDemo_loadChirpConfig(void); // __attribute__((section (".customCode")));
int32_t                MmwDemo_loadPhaseShiftConfig(void); //__attribute__((section (".customCode")));
void                   MmwDemo_loadFrameConfig(void); //__attribute__((section (".customCode")));
void                   MmwDemo_loadGUIMonConfig(void); //__attribute__((section (".customCode")));
int32_t                MmwDemo_loadADCBUFConfig(void); // __attribute__((section (".customCode")));
int32_t                MmwDemo_getNumEmptySubBands(uint32_t numTxAntennas); //__attribute__((section (".customCode")));
void                   MmwDemo_DataOutputMode(void);
void                   MmwDemo_ChannelCfg(void);
void                   MmwDemo_ADCCfg(void);
void                   MmwDemo_loadFastRangeProcCfg(void);
int32_t                MmwDemo_loadAntGeometryCfg(void); //__attribute__((section (".customCode")));
int32_t                MmwDemo_loadDDMPhaseShiftOrder(void); //__attribute__((section (".customCode")));
#endif

#ifdef LVDS_STREAM
int32_t MmwDemo_loadLVDSStreamConfig(void) __attribute__((section(".customCode")));
#endif

#ifdef HECR_ENABLE_ADV_CHIRP
int32_t MmwDemo_loadAdvChirpConfig(void) __attribute__((section(".customCode")));
#endif

/* Function to compare two elements for qsort */
static int compare(const void *a, const void *b)
{
    Element *element1 = (Element *)a;
    Element *element2 = (Element *)b;
    return element1->value - element2->value;
}

/**
 *  @b Description
 *  @n
 *      Returns number of empty subbands
 *
 *  @retval
 *      Success     - Number of empty subbands
 *  @retval
 *      Error       - <0
 */
int32_t MmwDemo_getNumEmptySubBands(uint32_t numTxAntennas)
{

    int32_t numBandsEmpty;
    /* Empty subbands */
    switch (numTxAntennas)
    {
        case 2:
            numBandsEmpty = 1;
            break;
        case 3:
            numBandsEmpty = 1;
            break;
        case 4:
            numBandsEmpty = 2;
            break;
        default:
            numBandsEmpty = -1;
            goto exit;
    }

exit:
    return numBandsEmpty;
}

#if defined(TRACKER_ON_MSS) && !defined(MMWAVELINK_ON_MSS)

int32_t MmwDemo_trackerConfig()
{
    int32_t retVal;

    MmwDemo_loadRansac();
    MmwDemo_GAppSceneryParams();
    MmwDemo_GAppAllocParams();
    MmwDemo_GAppGatingParams();
    MmwDemo_GAppStateParams();
    retVal = MmwDemo_loadGtrackCfg();

    return retVal;
}

/* Tracker is running on MSS for now */
void MmwDemo_loadRansac(void)
{
    gMmw_MCB.trackerCfg.ransacEnabled    = HECR_RANSAC_ENABLE;
    gMmw_MCB.trackerCfg.ransacIterations = HECR_RANSAC_INTERATIONS;
    gMmw_MCB.trackerCfg.ransacThresh     = HECR_RANSAC_THRESHOLD;
}

int32_t MmwDemo_loadGtrackCfg(void)
{
    int32_t                   retVal = 0;
    int32_t                   errCode;
    GTRACK_moduleConfig       config;
    GTRACK_advancedParameters advParams;

    gMmw_MCB.trackerCfg.gtrackEnabled = HECR_GTRACK_ENABLE;

    // delete gtrack instance if it already exists
    if (gMmw_MCB.trackerCfg.gtrackInstanceCreated)
    {
        gtrack_delete(gMmw_MCB.trackerCfg.gHTrackModule);
        gMmw_MCB.trackerCfg.gHTrackModule         = NULL;
        gMmw_MCB.trackerCfg.gtrackInstanceCreated = false;
    }

    if (!gMmw_MCB.trackerCfg.gtrackEnabled)
    {
        return retVal;
    }
    else
    {
        // create gtrack instance
        memset((void *)&config, 0, sizeof(GTRACK_moduleConfig));
        memset((void *)&advParams, 0, sizeof(GTRACK_advancedParameters));

        config.stateVectorType = GTRACK_STATE_VECTORS_3DA;
        config.verbose         = GTRACK_VERBOSE_NONE;

        config.maxNumPoints             = HECR_GTRACK_MAX_NUM_PT;
        config.maxNumTracks             = HECR_GTRACK_MAX_NUM_TRCK;
        config.initialRadialVelocity    = HECR_GTRACK_INIT_RAD_VEL;
        config.maxRadialVelocity        = HECR_GTRACK_MAX_RAD_VEL;
        config.radialVelocityResolution = HECR_GTRACK_RAD_VEL_RESO;
        config.maxAcceleration[0]       = HECR_GTRACK_MAX_ACC_0;
        config.maxAcceleration[1]       = HECR_GTRACK_MAX_ACC_1;
        config.maxAcceleration[2]       = HECR_GTRACK_MAX_ACC_2;
        config.deltaT                   = HECR_GTRACK_DELTA_T;


        advParams.allocationParams = &gMmw_MCB.trackerCfg.appAllocationParams;
        advParams.gatingParams     = &gMmw_MCB.trackerCfg.appGatingParams;
        advParams.stateParams      = &gMmw_MCB.trackerCfg.appStateParams;
        advParams.sceneryParams    = &gMmw_MCB.trackerCfg.appSceneryParams;

        config.advParams                  = &advParams;
        gMmw_MCB.trackerCfg.gHTrackModule = gtrack_create(&config, &errCode);
        if (gMmw_MCB.trackerCfg.gHTrackModule == NULL)
        {
            return -1;
        }

        gMmw_MCB.trackerCfg.gtrackInstanceCreated = true;
        gMmw_MCB.trackerCfg.gtrackEnabled         = true;
        return retVal;
    }
}

void MmwDemo_GAppAllocParams(void)
{
    GTRACK_allocationParams *allocationParams;
    allocationParams = (GTRACK_allocationParams *)&gMmw_MCB.trackerCfg.appAllocationParams;

    allocationParams->snrThre         = HECR_APP_ALLOC_SNR_THRE;
    allocationParams->snrThreObscured = HECR_APP_ALLOC_SNR_THRE_OBSC;
    allocationParams->velocityThre    = HECR_APP_ALLOC_VEL_THRE;
    allocationParams->pointsThre      = HECR_APP_ALLOC_PTS_THRE;
    allocationParams->maxDistanceThre = HECR_APP_ALLOC_MX_DIS_THRE;
    allocationParams->maxVelThre      = HECR_APP_ALLOC_MX_VEL_THRE;
}

void MmwDemo_GAppGatingParams(void)
{
    GTRACK_gatingParams *gatingParams;
    gatingParams = (GTRACK_gatingParams *)&gMmw_MCB.trackerCfg.appGatingParams;

    gatingParams->gain           = HECR_APP_GATING_GAIN;
    gatingParams->limitsArray[0] = HECR_APP_GATING_LIM_ARR_0;
    gatingParams->limitsArray[1] = HECR_APP_GATING_LIM_ARR_1;
    gatingParams->limitsArray[2] = HECR_APP_GATING_LIM_ARR_2;
    gatingParams->limitsArray[3] = HECR_APP_GATING_LIM_ARR_3;
}

void MmwDemo_GAppStateParams(void)
{
    GTRACK_stateParams *stateParams;
    stateParams = (GTRACK_stateParams *)&gMmw_MCB.trackerCfg.appStateParams;

    stateParams->det2actThre     = HECR_APP_STATE_DET2_ACT_THRE;
    stateParams->det2freeThre    = HECR_APP_STATE_DET2_FREE_THRE;
    stateParams->active2freeThre = HECR_APP_STATE_ACT2_FREE_THRE;
    stateParams->static2freeThre = HECR_APP_STATE_STATIC2_FREE_THRE;
    stateParams->exit2freeThre   = HECR_APP_STATE_EXIT2_FREE_THRE;
    stateParams->sleep2freeThre  = HECR_APP_STATE_SLEEP2_FREE_THRE;
}

void MmwDemo_GAppSceneryParams(void)
{
    GTRACK_sceneryParams *sceneryParams;
    sceneryParams = (GTRACK_sceneryParams *)&gMmw_MCB.trackerCfg.appSceneryParams;

    sceneryParams->sensorPosition.x = HECR_SCENE_SENSOR_POS_X;
    sceneryParams->sensorPosition.y = HECR_SCENE_SENSOR_POS_Y;
    sceneryParams->sensorPosition.z = HECR_SCENE_SENSOR_POS_Z;

    sceneryParams->sensorOrientation.azimTilt = HECR_SCENE_SENSOR_ORIN_AZIM_TILT;
    sceneryParams->sensorOrientation.elevTilt = HECR_SCENE_SENSOR_ORIN_ELEV_TILT;

    sceneryParams->numBoundaryBoxes  = HECR_SCENE_NUM_BOUND_BOX;
    sceneryParams->boundaryBox[0].x1 = HECR_SCENE_BOUND_BOX_X1;
    sceneryParams->boundaryBox[0].x2 = HECR_SCENE_BOUND_BOX_X2;
    sceneryParams->boundaryBox[0].y1 = HECR_SCENE_BOUND_BOX_Y1;
    sceneryParams->boundaryBox[0].y2 = HECR_SCENE_BOUND_BOX_Y2;
    sceneryParams->boundaryBox[0].z1 = HECR_SCENE_BOUND_BOX_Z1;
    sceneryParams->boundaryBox[0].z2 = HECR_SCENE_BOUND_BOX_Z2;

    sceneryParams->numStaticBoxes = HECR_SCENE_NUM_STATIC_BOX;
}
#endif


#if (defined(MMWAVELINK_ON_MSS) && defined(SUBSYS_MSS)) || (!defined(MMWAVELINK_ON_MSS) && defined(SUBSYS_DSS))
/**
 *  @b Description
 *  @n
 *      The function is used to encode the error code. This encoded error
 *      will be returned back to the application.
 *
 *  @param[in]  mmWaveError
 *      Error code for the mmWave
 *  @param[in]  subSysError
 *      Subsystem error code
 *
 *  @retval
 *      Encoded error code
 */
void MmwDemo_printError(
    int32_t mmWaveError,
    int32_t subSysError)
{
    DebugP_log("[Error code: %d, Subsystem: %d]\r\n", mmWaveError, subSysError);
    DebugP_assert(0);
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the DFE Data Output mode.
 */
void MmwDemo_DataOutputMode(void)
{
    gMmw_MCB.cfg.ctrlCfg.dfeDataOutputMode = HECR_DFE_DATA_OUTPUT_MODE;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the channel configuration command
 */
void MmwDemo_ChannelCfg(void)
{
    gMmw_MCB.cfg.openCfg.chCfg.rxChannelEn        = HECR_RX_CHAN_EN;
    gMmw_MCB.cfg.openCfg.chCfg.txChannelEn        = HECR_TX_CHAN_EN;
    gMmw_MCB.cfg.openCfg.chCfg.cascading          = 0U;
    gMmw_MCB.cfg.openCfg.chCfg.cascadingPinoutCfg = (1U << 5U); /* Disbale OSC_CLKOUT */
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the ADC configuration command
 */
void MmwDemo_ADCCfg(void)
{
    gMmw_MCB.cfg.openCfg.adcOutCfg.fmt.b2AdcBits   = HECR_ADC_NUM_BIT;
    gMmw_MCB.cfg.openCfg.adcOutCfg.fmt.b2AdcOutFmt = HECR_ADC_OUT_FORMAT;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for ADC buffer configuration
 */
int32_t MmwDemo_loadADCBUFConfig(void)
{
    MmwDemo_ADCBufCfg *adcBufCfg;

    /* Get the address of the ADCBUF cfg in the global DSS handle */
    adcBufCfg = (MmwDemo_ADCBufCfg *)&gMmw_MCB.subFrameCfg[0].adcBufCfg;

    /* Set the ADC buf configuration */
    adcBufCfg->adcFmt         = HECR_ADCBUF_FORMAT;
    adcBufCfg->iqSwapSel      = HECR_IQ_SWAP_SEL;
    adcBufCfg->chInterleave   = HECR_CHAN_INTERLEAVE;
    adcBufCfg->chirpThreshold = HECR_CHIRP_THRESHOLD;

    /* This demo is using HWA for 1D processing which does not allow multi-chirp
     * processing */
    if (adcBufCfg->chirpThreshold != 1)
    {
        DebugP_log("Error: chirpThreshold must be 1, multi-chirp is not allowed\n");
        DebugP_assert(0);
    }

    /* Set the state of the configuration for the ADC buffer */
    gMmw_MCB.subFrameCfg[0].isAdcBufCfgPending = 1;

    return 0;
}

#if 0
int32_t MmwaveLink_getProfileConfig (void)
{
    int32_t         retVal, i;
    rlProfileCfg_t  getProfileData = { 0 };

    for(i = 0; i< gMmwWFCfg.numProfiles; i++)
    {
        /* Get profile configuration */
        retVal = rlGetProfileConfig(RL_DEVICE_MAP_INTERNAL_BSS, i, (rlProfileCfg_t*)&getProfileData);

        /* Check for mmWaveLink API call status */
        if(retVal != 0)
        {
            /* Error: Link reported an issue. */
            printf("Error: rlGetProfileConfig retVal=%d\n", retVal);
            return -1;
        }
        if((getProfileData.adcStartTimeConst != gMmwWFCfg.profileCfg[i].adcStartTimeConst) || \
           (getProfileData.digOutSampleRate != gMmwWFCfg.profileCfg[i].digOutSampleRate) || \
           (getProfileData.freqSlopeConst != gMmwWFCfg.profileCfg[i].freqSlopeConst) || \
           (getProfileData.hpfCornerFreq1 != gMmwWFCfg.profileCfg[i].hpfCornerFreq1) || \
           (getProfileData.hpfCornerFreq2 != gMmwWFCfg.profileCfg[i].hpfCornerFreq2) || \
           (getProfileData.idleTimeConst != gMmwWFCfg.profileCfg[i].idleTimeConst) || \
           (getProfileData.numAdcSamples != gMmwWFCfg.profileCfg[i].numAdcSamples) || \
           (getProfileData.profileId != gMmwWFCfg.profileCfg[i].profileId) || \
           (getProfileData.rampEndTime != gMmwWFCfg.profileCfg[i].rampEndTime) || \
           (getProfileData.rxGain != gMmwWFCfg.profileCfg[i].rxGain) || \
           (getProfileData.startFreqConst != gMmwWFCfg.profileCfg[i].startFreqConst) || \
           (getProfileData.txOutPowerBackoffCode != gMmwWFCfg.profileCfg[i].txOutPowerBackoffCode) || \
           (getProfileData.txPhaseShifter != gMmwWFCfg.profileCfg[i].txPhaseShifter) || \
           (getProfileData.txStartTime != gMmwWFCfg.profileCfg[i].txStartTime))
        {
            printf("Error: rlGetProfileConfig parameters not matched with configured parameters [%d]\n", i);
            retVal = -1;
        }
    }
    if(retVal == 0)
    {
        printf("Debug: rlGetProfileConfig parameters matched with configured parameters\n");
    }
    return 0;
}
#endif

int32_t MmwDemo_loadProfileConfig()
{
    int32_t         retVal = 0;
    rlProfileCfg_t *profileCfg;

    /* How many profiles need to be configured? */
    gMmwWFCfg.numProfiles = HECR_NUM_PROFILES;

    /* Frequency scale factor */
    double gCLI_mmwave_freq_scale_factor = 3.6;

    profileCfg = (rlProfileCfg_t *)&gMmwWFCfg.profileCfg;

    /********************************************************************************************************************/
    /*                                       Populate the profile configuration:                                        */
    /********************************************************************************************************************/
    switch (gMmwWFCfg.numProfiles)
    {
        case 4:
        {
            /************************************************  PROFILE-3 ********************************************************/

            profileCfg[3].profileId             = HECR_PROFILE_3_PROFILE_ID;
            profileCfg[3].pfVcoSelect           = HECR_PROFILE_3_VCO_SELECT; /* VCO_SEL : VCO2 for xWR294x to use 76-81 GHz */
            profileCfg[3].startFreqConst        = (uint32_t)((float)HECR_PROFILE_3_START_FREQ_GHZ * (1U << 26) / 3.6); // 77GHz
            profileCfg[3].idleTimeConst         = (uint32_t)((float)HECR_PROFILE_3_IDLE_TIME_VAL * 1000 / 10);
            profileCfg[3].adcStartTimeConst     = (uint32_t)((float)HECR_PROFILE_3_ADC_START_TIME_VAL * 1000 / 10);
            profileCfg[3].rampEndTime           = (uint32_t)((float)HECR_PROFILE_3_RAMP_END_TIME_VAL * 1000 / 10);
            profileCfg[3].txOutPowerBackoffCode = HECR_PROFILE_3_TXOUT_POWER_BACKOFF;
            profileCfg[3].txPhaseShifter        = 0;
            /* Translate from MHz/us to [1 LSB = (gCLI_mmwave_freq_scale_factor * 1e6 * 900) / 2^26 kHz/uS]
             * units of mmwavelink format */
            profileCfg[3].freqSlopeConst = (int16_t)(HECR_PROFILE_3_FREQ_SLOPE_MHZ_PER_US * (1U << 26) /
                                                     ((gCLI_mmwave_freq_scale_factor * 1e3) * 900.0));
            /* Translate from us to [1 LSB = 10 ns] units of mmwavelink format */
            profileCfg[3].txStartTime      = (int32_t)((float)HECR_PROFILE_3_TX_START_TIME_VAL * 1000 / 10);
            profileCfg[3].numAdcSamples    = HECR_PROFILE_3_ADC_SAMPLE_VAL;
            profileCfg[3].digOutSampleRate = HECR_PROFILE_3_DIGOUT_SAMPLERATE_VAL;
            profileCfg[3].hpfCornerFreq1   = HECR_PROFILE_3_HPFCORNER_FREQ1_VAL;
            profileCfg[3].hpfCornerFreq2   = HECR_PROFILE_3_HPFCORNER_FREQ2_VAL;
            profileCfg[3].rxGain           = HECR_PROFILE_3_RX_GAIN_VAL;
            profileCfg[3].pfCalLutUpdate   = HECR_PROFILE_3_PF_CAL_UPDATE;

            /********************************************************************************************************************/
        }
        case 3:
        {
            /************************************************  PROFILE-2 ********************************************************/

            profileCfg[2].profileId             = HECR_PROFILE_2_PROFILE_ID;
            profileCfg[2].pfVcoSelect           = HECR_PROFILE_2_VCO_SELECT; /* VCO_SEL : VCO2 for xWR294x to use 76-81 GHz */
            profileCfg[2].startFreqConst        = (uint32_t)((float)HECR_PROFILE_2_START_FREQ_GHZ * (1U << 26) / 3.6); // 77GHz
            profileCfg[2].idleTimeConst         = (uint32_t)((float)HECR_PROFILE_2_IDLE_TIME_VAL * 1000 / 10);
            profileCfg[2].adcStartTimeConst     = (uint32_t)((float)HECR_PROFILE_2_ADC_START_TIME_VAL * 1000 / 10);
            profileCfg[2].rampEndTime           = (uint32_t)((float)HECR_PROFILE_2_RAMP_END_TIME_VAL * 1000 / 10);
            profileCfg[2].txOutPowerBackoffCode = HECR_PROFILE_2_TXOUT_POWER_BACKOFF;
            profileCfg[2].txPhaseShifter        = 0;
            /* Translate from MHz/us to [1 LSB = (gCLI_mmwave_freq_scale_factor * 1e6 * 900) / 2^26 kHz/uS]
             * units of mmwavelink format */
            profileCfg[2].freqSlopeConst = (int16_t)(HECR_PROFILE_2_FREQ_SLOPE_MHZ_PER_US * (1U << 26) /
                                                     ((gCLI_mmwave_freq_scale_factor * 1e3) * 900.0));
            /* Translate from us to [1 LSB = 10 ns] units of mmwavelink format */
            profileCfg[2].txStartTime      = (int32_t)((float)HECR_PROFILE_2_TX_START_TIME_VAL * 1000 / 10);
            profileCfg[2].numAdcSamples    = HECR_PROFILE_2_ADC_SAMPLE_VAL;
            profileCfg[2].digOutSampleRate = HECR_PROFILE_2_DIGOUT_SAMPLERATE_VAL;
            profileCfg[2].hpfCornerFreq1   = HECR_PROFILE_2_HPFCORNER_FREQ1_VAL;
            profileCfg[2].hpfCornerFreq2   = HECR_PROFILE_2_HPFCORNER_FREQ2_VAL;
            profileCfg[2].rxGain           = HECR_PROFILE_2_RX_GAIN_VAL;
            profileCfg[2].pfCalLutUpdate   = HECR_PROFILE_2_PF_CAL_UPDATE;

            /********************************************************************************************************************/
        }
        case 2:
        {
            /************************************************  PROFILE-1 ********************************************************/

            profileCfg[1].profileId             = HECR_PROFILE_1_PROFILE_ID;
            profileCfg[1].pfVcoSelect           = HECR_PROFILE_1_VCO_SELECT; /* VCO_SEL : VCO2 for xWR294x to use 76-81 GHz */
            profileCfg[1].startFreqConst        = (uint32_t)((float)HECR_PROFILE_1_START_FREQ_GHZ * (1U << 26) / 3.6); // 77GHz
            profileCfg[1].idleTimeConst         = (uint32_t)((float)HECR_PROFILE_1_IDLE_TIME_VAL * 1000 / 10);
            profileCfg[1].adcStartTimeConst     = (uint32_t)((float)HECR_PROFILE_1_ADC_START_TIME_VAL * 1000 / 10);
            profileCfg[1].rampEndTime           = (uint32_t)((float)HECR_PROFILE_1_RAMP_END_TIME_VAL * 1000 / 10);
            profileCfg[1].txOutPowerBackoffCode = HECR_PROFILE_1_TXOUT_POWER_BACKOFF;
            profileCfg[1].txPhaseShifter        = 0;
            /* Translate from MHz/us to [1 LSB = (gCLI_mmwave_freq_scale_factor * 1e6 * 900) / 2^26 kHz/uS]
             * units of mmwavelink format */
            profileCfg[1].freqSlopeConst = (int16_t)(HECR_PROFILE_1_FREQ_SLOPE_MHZ_PER_US * (1U << 26) /
                                                     ((gCLI_mmwave_freq_scale_factor * 1e3) * 900.0));
            /* Translate from us to [1 LSB = 10 ns] units of mmwavelink format */
            profileCfg[1].txStartTime      = (int32_t)((float)HECR_PROFILE_1_TX_START_TIME_VAL * 1000 / 10);
            profileCfg[1].numAdcSamples    = HECR_PROFILE_1_ADC_SAMPLE_VAL;
            profileCfg[1].digOutSampleRate = HECR_PROFILE_1_DIGOUT_SAMPLERATE_VAL;
            profileCfg[1].hpfCornerFreq1   = HECR_PROFILE_1_HPFCORNER_FREQ1_VAL;
            profileCfg[1].hpfCornerFreq2   = HECR_PROFILE_1_HPFCORNER_FREQ2_VAL;
            profileCfg[1].rxGain           = HECR_PROFILE_1_RX_GAIN_VAL;
            profileCfg[1].pfCalLutUpdate   = HECR_PROFILE_1_PF_CAL_UPDATE;

            /********************************************************************************************************************/
        }
        case 1:
        {
            /************************************************  PROFILE-0 ********************************************************/

            profileCfg[0].profileId   = HECR_PROFILE_0_PROFILE_ID;
            profileCfg[0].pfVcoSelect = HECR_PROFILE_0_VCO_SELECT; /* VCO_SEL : VCO2 for xWR294x to use 76-81 GHz */
            /* Translate from GHz to [1 LSB = gCLI_mmwave_freq_scale_factor * 1e9 / 2^26 Hz] units
             * of mmwavelink format */
            profileCfg[0].startFreqConst = (uint32_t)((float)HECR_PROFILE_0_START_FREQ_GHZ * (1U << 26) / 3.6); // 77GHz
            /* Translate below times from us to [1 LSB = 10 ns] units of mmwavelink format */
            profileCfg[0].idleTimeConst         = (uint32_t)((float)HECR_PROFILE_0_IDLE_TIME_VAL * 1000 / 10);
            profileCfg[0].adcStartTimeConst     = (uint32_t)((float)HECR_PROFILE_0_ADC_START_TIME_VAL * 1000 / 10);
            profileCfg[0].rampEndTime           = (uint32_t)((float)HECR_PROFILE_0_RAMP_END_TIME_VAL * 1000 / 10);
            profileCfg[0].txOutPowerBackoffCode = HECR_PROFILE_0_TXOUT_POWER_BACKOFF;
            profileCfg[0].txPhaseShifter        = 0;
            /* Translate from MHz/us to [1 LSB = (gCLI_mmwave_freq_scale_factor * 1e6 * 900) / 2^26 kHz/uS]
             * units of mmwavelink format */
            profileCfg[0].freqSlopeConst = (int16_t)(HECR_PROFILE_0_FREQ_SLOPE_MHZ_PER_US * (1U << 26) /
                                                     ((gCLI_mmwave_freq_scale_factor * 1e3) * 900.0));
            /* Translate from us to [1 LSB = 10 ns] units of mmwavelink format */
            profileCfg[0].txStartTime      = (int32_t)((float)HECR_PROFILE_0_TX_START_TIME_VAL * 1000 / 10);
            profileCfg[0].numAdcSamples    = HECR_PROFILE_0_ADC_SAMPLE_VAL;
            profileCfg[0].digOutSampleRate = HECR_PROFILE_0_DIGOUT_SAMPLERATE_VAL;
            profileCfg[0].hpfCornerFreq1   = HECR_PROFILE_0_HPFCORNER_FREQ1_VAL;
            profileCfg[0].hpfCornerFreq2   = HECR_PROFILE_0_HPFCORNER_FREQ2_VAL;
            profileCfg[0].rxGain           = HECR_PROFILE_0_RX_GAIN_VAL;
            profileCfg[0].pfCalLutUpdate   = HECR_PROFILE_0_PF_CAL_UPDATE;

            /********************************************************************************************************************/
            break;
        }
        default:
        {
            DebugP_log("No profiles configured!\n");
            break;
        }
    }

    /* Send all four profiles to the BSS */
    retVal = rlSetProfileConfig(RL_DEVICE_MAP_INTERNAL_BSS, gMmwWFCfg.numProfiles, profileCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        // Error: mmWave link profile configuration failed
        MmwDemo_printError(MMWAVE_EPROFILECFG, retVal);
    }

    // MmwaveLink_getProfileConfig();
    return retVal;
}

int32_t MmwDemo_loadChirpConfig(void)
{
    int32_t       retVal = 0;
    rlChirpCfg_t *chirpCfg;

    chirpCfg = (rlChirpCfg_t *)&gMmwWFCfg.chirpCfg[0];

    /* Load the DDM chirp configuration for the chirps 0 to 5 */
    chirpCfg->chirpStartIdx   = HECR_CHIRP_0_START_INDEX;
    chirpCfg->chirpEndIdx     = HECR_CHIRP_0_END_INDEX;
    chirpCfg->profileId       = HECR_CHIRP_0_PROFILE_ID;
    chirpCfg->startFreqVar    = HECR_CHIRP_0_START_FREQ_VAL;
    chirpCfg->freqSlopeVar    = HECR_CHIRP_0_FREQ_SLOPE_VAL;
    chirpCfg->idleTimeVar     = HECR_CHIRP_0_IDLE_TIME_VAL;
    chirpCfg->adcStartTimeVar = HECR_CHIRP_0_ADC_START_TIME_VAL;
    chirpCfg->txEnable        = HECR_CHIRP_0_TX_CHANNEL;

    gMmwWFCfg.numChirps = HECR_CHIRP_0_END_INDEX - HECR_CHIRP_0_START_INDEX + 1;

#ifndef HECR_ENABLE_ADV_CHIRP
    /* Set the complete DDM chirp configuration at once */
    retVal = rlSetChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, 1U, chirpCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        // Error: mmWave link chirp configuration failed
        MmwDemo_printError(MMWAVE_ECHIRPCFG, retVal);
        retVal = MINUS_ONE;
    }
#endif

    return retVal;
}

void MmwDemo_loadFrameConfig(void)
{
    rlFrameCfg_t *frameCfg;

    frameCfg = (rlFrameCfg_t *)&gMmwWFCfg.frameCfg;

    /* Load the DDM chirp configuration for the chirps 0 to 5 */
    frameCfg->chirpStartIdx = HECR_FRAME_CHIRP_START_IDX;
    frameCfg->chirpEndIdx   = HECR_FRAME_CHIRP_END_IDX;
#ifdef HECR_ENABLE_ADV_CHIRP
    frameCfg->numLoops = ((HECR_CHIRP_0_END_INDEX - HECR_CHIRP_0_START_INDEX + 1) * HECR_FRAME_CHIRP_NUM_LOOP);
#else
    frameCfg->numLoops        = HECR_FRAME_CHIRP_NUM_LOOP;
#endif
    frameCfg->numFrames         = HECR_FRAME_NUM_FRAMES;
    frameCfg->numAdcSamples     = HECR_FRAME_NUM_SAMPLES;
    frameCfg->framePeriodicity  = (uint32_t)(HECR_FRAME_FRAME_PERIODICITY * 1000000 / 5);
    frameCfg->triggerSelect     = HECR_FRAME_FRAME_TRIGGER_SELECT;
    frameCfg->frameTriggerDelay = (uint32_t)(HECR_FRAME_FRAME_TRIGGER_DELAY * 1000000 / 5);

    /* Copy the frame config in the ctrlCfg and update the DFE mode */
    memcpy((void *)&gMmw_MCB.cfg.ctrlCfg.u.frameCfg[0].frameCfg, (void *)frameCfg, sizeof(rlFrameCfg_t));

    gMmw_MCB.cfg.ctrlCfg.dfeDataOutputMode = HECR_DFE_DATA_OUTPUT_MODE;

    /* Set frame configuration */
    // retVal = rlSetFrameConfig(RL_DEVICE_MAP_INTERNAL_BSS, frameCfg);
    /* Set this back for datapth configuration reference */
    frameCfg->numLoops = HECR_FRAME_CHIRP_NUM_LOOP;
}

#ifdef HECR_ENABLE_ADV_CHIRP
/* Advanced Chirp LUT data */
/* Max size of the LUT is 12KB.
   Maximum of 212 bytes per chunk can be present per SPI message. */
/* This array is created to store the LUT RAM values from the user programmed parameters or config file.
   The populated array is sent over SPI to populate the RadarSS LUT RAM.
 * The chirp paramters start address offset should be 4 byte aligned */
rlInt8_t   AdvChirpLUTData[HECR_ADV_CHIRP_LUT_ADVCHIRP_TABLE_SIZE] = { 0 };
rlUInt16_t lutOffsetInNBytes                                       = 0;

int32_t MmwDemo_loadAdvChirpConfig(void)
{
    int32_t retVal = 0;

    rlRfMiscConf_t     MiscCfg              = { 0 };
    rlAdvChirpCfg_t    AdvChirpCfgArgs      = { 0 };
    rlAdvChirpLUTCfg_t rlAdvChirpLUTCfgArgs = { 0 };
    rlFillLUTParams_t  rlFillLUTParamsArgs  = { 0 };

    /* LUT */
    // 0 -> Dummy
    // 4 -> Profile
    // uint32_t LUTWordSizeOffset = 4;

    /* Enable Adv chirp feature
    b0: PERCHIRP_PHASESHIFTER_EN
    b1: ADVANCE_CHIRP_CONFIG_EN
    */
    MiscCfg.miscCtl = 0x3U;
    /* Enable advChirp in miscConfig */
    retVal = rlRfSetMiscConfig(RL_DEVICE_MAP_INTERNAL_BSS, &MiscCfg);

    /********************************************************************************************************************************************/
    /* Configure Profile (Param Index = 0) */
    AdvChirpCfgArgs.chirpParamIdx           = RL_LUT_CHIRP_PROFILE_VAR;
    AdvChirpCfgArgs.lutResetPeriod          = 1;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 0;
    AdvChirpCfgArgs.lutPatternAddressOffset = lutOffsetInNBytes; /* Fixed LUT pattern to offset */
    AdvChirpCfgArgs.lutChirpParamSize       = 0; // Default size
    AdvChirpCfgArgs.numOfPatterns           = 4;

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 0) %d*** \r\n", retVal);
        return retVal;
    }
    /* Update LUT for param = 0 () To pick from profile 0 */
    // AdvChirpLUTData[LUTWordSizeOffset] = (uint32_t) 0x0;
    // LUTWordSizeOffset = LUTWordSizeOffset + 4; // LUTWordSizeOffset = 8;

    uint8_t profileIdxCfg[4];
    profileIdxCfg[0] = 0;
    profileIdxCfg[1] = 1;
    profileIdxCfg[2] = 2;
    profileIdxCfg[3] = 3;

    /* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
    rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_PROFILE_VAR;
    rlFillLUTParamsArgs.chirpParamSize  = AdvChirpCfgArgs.lutChirpParamSize;
    rlFillLUTParamsArgs.inputSize       = AdvChirpCfgArgs.numOfPatterns;
    rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
    retVal                              = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&profileIdxCfg[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);
    // while(lutOffsetInNBytes%4 != 0)
    // {
    //     lutOffsetInNBytes+=1;
    // }

    /********************************************************************************************************************************************/
    /* Start Frequency (Param Index = 1) */
    memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
    AdvChirpCfgArgs.chirpParamIdx           = RL_LUT_CHIRP_FREQ_START_VAR;
    AdvChirpCfgArgs.lutResetPeriod          = 1;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 0;
    AdvChirpCfgArgs.lutPatternAddressOffset = HECR_ADV_CHIRP_DUMMY_LUT_ADDR_OFFSET; /* Fixed LUT pattern to offset */
    AdvChirpCfgArgs.numOfPatterns           = 1;

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 1) %d*** \r\n", retVal);
        return retVal;
    }

    /********************************************************************************************************************************************/
    /* Frequency Slope (Param Index = 2) */
    /* LUT Dither to this chirp param type */
    /* LUT start address offset for the slope chirp parameter is made 2056 */
    memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
    AdvChirpCfgArgs.chirpParamIdx           = RL_LUT_CHIRP_FREQ_SLOPE_VAR;
    AdvChirpCfgArgs.lutResetPeriod          = 1;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 0;
    AdvChirpCfgArgs.lutPatternAddressOffset = HECR_ADV_CHIRP_DUMMY_LUT_ADDR_OFFSET; /* Fixed LUT pattern to offset */
    AdvChirpCfgArgs.numOfPatterns           = 1;

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 2) %d*** \r\n", retVal);
        return retVal;
    }

    /********************************************************************************************************************************************/
    /* Idle time (Param Index = 3) */
    memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
    AdvChirpCfgArgs.chirpParamIdx           = RL_LUT_CHIRP_IDLE_TIME_VAR;
    AdvChirpCfgArgs.lutResetPeriod          = 1;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 0;
    AdvChirpCfgArgs.lutPatternAddressOffset = HECR_ADV_CHIRP_DUMMY_LUT_ADDR_OFFSET; /* Fixed LUT pattern to offset */
    AdvChirpCfgArgs.numOfPatterns           = 1;

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 3) %d*** \r\n", retVal);
        return retVal;
    }

    /********************************************************************************************************************************************/
    /* ADC start time (Param Index = 4) */
    /* No Dither to this chirp param type */
    memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
    AdvChirpCfgArgs.chirpParamIdx           = RL_LUT_CHIRP_ADC_START_TIME_VAR;
    AdvChirpCfgArgs.lutResetPeriod          = 1;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 0;
    AdvChirpCfgArgs.lutPatternAddressOffset = HECR_ADV_CHIRP_DUMMY_LUT_ADDR_OFFSET; /* Fixed LUT pattern to zero offset */
    AdvChirpCfgArgs.numOfPatterns           = 1;

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code %d*** (Param - 4) \r\n", retVal);
        return retVal;
    }

    /********************************************************************************************************************************************/
    /* BPM Enable (Param Index = 6) */
    /* No LUT or Delta Dither for BPM Enable parameter */
    memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
    AdvChirpCfgArgs.chirpParamIdx           = RL_LUT_CHIRP_BPM_VAL_VAR;
    AdvChirpCfgArgs.lutResetPeriod          = 1;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 0;
    AdvChirpCfgArgs.lutPatternAddressOffset = HECR_ADV_CHIRP_DUMMY_LUT_ADDR_OFFSET; /* Fixed LUT pattern to offset */
    AdvChirpCfgArgs.numOfPatterns           = 1;

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 6) %d*** \r\n", retVal);
        return retVal;
    }

    /********************************************************************************************************************************************/
    /* TX0 Phase shifter (Param Index = 7) */
    /* No Delta or LUT dither for TX0 phase shifter */
    memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
    AdvChirpCfgArgs.chirpParamIdx = RL_LUT_TX0_PHASE_SHIFT_VAR;
    /* Update LUT for param = 7 (45 deg) */
    // AdvChirpLUTData[LUTWordSizeOffset] = (uint32_t) 0x8;
    // LUTWordSizeOffset = LUTWordSizeOffset + 4; // LUTWordSizeOffset = 12;

#ifdef FIXED_PS
    AdvChirpCfgArgs.lutResetPeriod          = 1;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 0;
    AdvChirpCfgArgs.lutPatternAddressOffset = lutOffsetInNBytes; /* Fixed LUT pattern to offset */
    AdvChirpCfgArgs.numOfPatterns           = 1;
    AdvChirpCfgArgs.lutChirpParamSize       = 0; // Default size

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 7) %d*** \r\n", retVal);
        return retVal;
    }

    // uint8_t tx0PhShift = 0x8;
    uint8_t tx0PhShift = 0x0;
    /* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
    rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_TX1_PHASE_SHIFT_VAR;
    rlFillLUTParamsArgs.chirpParamSize  = AdvChirpCfgArgs.lutChirpParamSize;
    rlFillLUTParamsArgs.inputSize       = AdvChirpCfgArgs.numOfPatterns;
    rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
    retVal                              = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&tx0PhShift, &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);
#else
    AdvChirpCfgArgs.lutResetPeriod          = 6;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 1;
    AdvChirpCfgArgs.lutPatternAddressOffset = lutOffsetInNBytes; /* Fixed LUT pattern to offset */
    AdvChirpCfgArgs.numOfPatterns           = 6;
    AdvChirpCfgArgs.lutChirpParamSize       = 0; // Default size

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 7) %d*** \r\n", retVal);
        return retVal;
    }

    uint8_t tx0PhShift[6];
    tx0PhShift[0] = 0; // Unique Chirp 0
    tx0PhShift[1] = 0; // Unique Chirp 1
    tx0PhShift[2] = 0; // Unique Chirp 2
    tx0PhShift[3] = 0; // Unique Chirp 3
    tx0PhShift[4] = 0; // Unique Chirp 4
    tx0PhShift[5] = 0; // Unique Chirp 5

    /* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
    rlFillLUTParamsArgs.chirpParamIndex     = RL_LUT_TX0_PHASE_SHIFT_VAR;
    rlFillLUTParamsArgs.chirpParamSize      = AdvChirpCfgArgs.lutChirpParamSize;
    rlFillLUTParamsArgs.inputSize           = AdvChirpCfgArgs.numOfPatterns;
    rlFillLUTParamsArgs.lutGlobalOffset     = lutOffsetInNBytes;
    retVal                                  = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&tx0PhShift[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);
#endif

    while (lutOffsetInNBytes % 4 != 0)
    {
        lutOffsetInNBytes += 1;
    }

    /********************************************************************************************************************************************/
    /* TX1 Phase shifter (Param Index = 8) */
    /* No Delta or LUT dither for TX0 phase shifter */
    memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
    AdvChirpCfgArgs.chirpParamIdx = RL_LUT_TX1_PHASE_SHIFT_VAR;

    /* Update LUT for param = 8 (90 deg) */
    // AdvChirpLUTData[LUTWordSizeOffset] = (uint32_t) 0x10;
    // LUTWordSizeOffset = LUTWordSizeOffset + 4; // LUTWordSizeOffset = 16;

#ifdef FIXED_PS
    AdvChirpCfgArgs.lutResetPeriod          = 1;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 0;
    AdvChirpCfgArgs.lutPatternAddressOffset = lutOffsetInNBytes; /* Fixed LUT pattern to offset */
    AdvChirpCfgArgs.numOfPatterns           = 1;
    AdvChirpCfgArgs.lutChirpParamSize       = 0; // Default size

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 8) %d*** \r\n", retVal);
        return retVal;
    }

    // uint8_t tx1PhShift = 0x10;
    uint8_t tx1PhShift = 0x0;
    /* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
    rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_TX1_PHASE_SHIFT_VAR;
    rlFillLUTParamsArgs.chirpParamSize  = AdvChirpCfgArgs.lutChirpParamSize;
    rlFillLUTParamsArgs.inputSize       = AdvChirpCfgArgs.numOfPatterns;
    rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
    retVal                              = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&tx1PhShift, &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);
#else
    AdvChirpCfgArgs.lutResetPeriod          = 6;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 1;
    AdvChirpCfgArgs.lutPatternAddressOffset = lutOffsetInNBytes; /* Fixed LUT pattern to offset */
    AdvChirpCfgArgs.numOfPatterns           = 6;
    AdvChirpCfgArgs.lutChirpParamSize       = 0; // Default size

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 8) %d*** \r\n", retVal);
        return retVal;
    }

    uint8_t tx1PhShift[6];
    tx1PhShift[0] = 0; // Unique Chirp 0
    tx1PhShift[1] = 128 >> 2; // Unique Chirp 1
    tx1PhShift[2] = 0; // Unique Chirp 2
    tx1PhShift[3] = 128 >> 2; // Unique Chirp 3
    tx1PhShift[4] = 0; // Unique Chirp 4
    tx1PhShift[5] = 128 >> 2; // Unique Chirp 5

    /* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
    rlFillLUTParamsArgs.chirpParamIndex     = RL_LUT_TX1_PHASE_SHIFT_VAR;
    rlFillLUTParamsArgs.chirpParamSize      = AdvChirpCfgArgs.lutChirpParamSize;
    rlFillLUTParamsArgs.inputSize           = AdvChirpCfgArgs.numOfPatterns;
    rlFillLUTParamsArgs.lutGlobalOffset     = lutOffsetInNBytes;
    retVal                                  = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&tx1PhShift[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);
#endif
    while (lutOffsetInNBytes % 4 != 0)
    {
        lutOffsetInNBytes += 1;
    }

    /********************************************************************************************************************************************/
    /* TX2 Phase shifter (Param Index = 9) */
    /* No Delta or LUT dither for TX2 phase shifter */
    memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
    AdvChirpCfgArgs.chirpParamIdx = RL_LUT_TX2_PHASE_SHIFT_VAR;

    /* Update LUT for param = 9 (135 deg) */
    // AdvChirpLUTData[LUTWordSizeOffset] = (uint32_t) 0x20;
    // LUTWordSizeOffset = LUTWordSizeOffset + 4; // LUTWordSizeOffset = 20;

#ifdef FIXED_PS
    AdvChirpCfgArgs.lutResetPeriod          = 1;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 0;
    AdvChirpCfgArgs.lutPatternAddressOffset = lutOffsetInNBytes; /* Fixed LUT pattern to offset */
    AdvChirpCfgArgs.numOfPatterns           = 1;
    AdvChirpCfgArgs.lutChirpParamSize       = 0; // Default size

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 9) %d*** \r\n", retVal);
        return retVal;
    }

    // uint8_t tx2PhShift = 0x20;
    uint8_t tx2PhShift = 0x0;
    /* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
    rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_TX2_PHASE_SHIFT_VAR;
    rlFillLUTParamsArgs.chirpParamSize  = AdvChirpCfgArgs.lutChirpParamSize;
    rlFillLUTParamsArgs.inputSize       = AdvChirpCfgArgs.numOfPatterns;
    rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
    retVal                              = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&tx2PhShift, &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);
#else
    AdvChirpCfgArgs.lutResetPeriod          = 6;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 1;
    AdvChirpCfgArgs.lutPatternAddressOffset = lutOffsetInNBytes; /* Fixed LUT pattern to offset */
    AdvChirpCfgArgs.numOfPatterns           = 6;
    AdvChirpCfgArgs.lutChirpParamSize       = 0; // Default size

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 9) %d*** \r\n", retVal);
        return retVal;
    }

    uint8_t tx2PhShift[6];
    tx2PhShift[0] = 0; // Unique Chirp 0
    tx2PhShift[1] = 44 >> 2; // Unique Chirp 1
    tx2PhShift[2] = 84 >> 2; // Unique Chirp 2
    tx2PhShift[3] = 128 >> 2; // Unique Chirp 3
    tx2PhShift[4] = 172 >> 2; // Unique Chirp 4
    tx2PhShift[5] = 212 >> 2; // Unique Chirp 5

    /* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
    rlFillLUTParamsArgs.chirpParamIndex     = RL_LUT_TX2_PHASE_SHIFT_VAR;
    rlFillLUTParamsArgs.chirpParamSize      = AdvChirpCfgArgs.lutChirpParamSize;
    rlFillLUTParamsArgs.inputSize           = AdvChirpCfgArgs.numOfPatterns;
    rlFillLUTParamsArgs.lutGlobalOffset     = lutOffsetInNBytes;
    retVal                                  = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&tx2PhShift[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);
#endif
    while (lutOffsetInNBytes % 4 != 0)
    {
        lutOffsetInNBytes += 1;
    }

    /********************************************************************************************************************************************/
    /* TX3 Phase shifter (Param Index = 10) */
    /* No Delta or LUT dither for TX0 phase shifter */
    memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
    AdvChirpCfgArgs.chirpParamIdx = RL_LUT_TX3_PHASE_SHIFT_VAR;

    /* Update LUT for param = 10 (180 deg) */
    // AdvChirpLUTData[LUTWordSizeOffset] = (uint32_t) 0x40;
    // LUTWordSizeOffset = LUTWordSizeOffset + 4; // LUTWordSizeOffset = 24;
#ifdef FIXED_PS
    AdvChirpCfgArgs.lutResetPeriod          = 1;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 0;
    AdvChirpCfgArgs.lutPatternAddressOffset = lutOffsetInNBytes; /* Fixed LUT pattern to zero offset */
    AdvChirpCfgArgs.numOfPatterns           = 1;
    AdvChirpCfgArgs.lutChirpParamSize       = 0; // Default size

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 10) %d*** \r\n", retVal);
        return retVal;
    }

    // uint8_t tx3PhShift = 0x40;
    uint8_t tx3PhShift = 0x0;
    /* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
    rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_TX3_PHASE_SHIFT_VAR;
    rlFillLUTParamsArgs.chirpParamSize  = AdvChirpCfgArgs.lutChirpParamSize;
    rlFillLUTParamsArgs.inputSize       = AdvChirpCfgArgs.numOfPatterns;
    rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
    retVal                              = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&tx3PhShift, &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);
#else
    AdvChirpCfgArgs.lutResetPeriod          = 6;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 1;
    AdvChirpCfgArgs.lutPatternAddressOffset = lutOffsetInNBytes; /* Fixed LUT pattern to offset */
    AdvChirpCfgArgs.numOfPatterns           = 6;
    AdvChirpCfgArgs.lutChirpParamSize       = 0; // Default size

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 10) %d*** \r\n", retVal);
        return retVal;
    }

    uint8_t tx3PhShift[6];
    tx3PhShift[0] = 0; // Unique Chirp 0
    tx3PhShift[1] = 84 >> 2; // Unique Chirp 1
    tx3PhShift[2] = 172 >> 2; // Unique Chirp 2
    tx3PhShift[3] = 0; // Unique Chirp 3
    tx3PhShift[4] = 84 >> 2; // Unique Chirp 4
    tx3PhShift[5] = 172 >> 2; // Unique Chirp 5

    /* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
    rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_TX3_PHASE_SHIFT_VAR;
    rlFillLUTParamsArgs.chirpParamSize  = AdvChirpCfgArgs.lutChirpParamSize;
    rlFillLUTParamsArgs.inputSize       = AdvChirpCfgArgs.numOfPatterns;
    rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
    retVal                              = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&tx3PhShift[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);
#endif
    while (lutOffsetInNBytes % 4 != 0)
    {
        lutOffsetInNBytes += 1;
    }

    /********************************************************************************************************************************************/
    /* Tx Enable (Param Index = 5) */
    /* Configuring 4 unique Tx enable mask (7,3,1,2) in the generic SW LUT - LUT Reset period (4) */
    /* The new Tx enable mask is picked every chirp (update period = 1) and it will reset every 4 chirps */
    /* By default all the Tx should be enabled as LUT update period is 0 which is what is needed for DDM */
    /*   Chirp (burst)   Tx enable mask
          0-511          1
          512-1023       2
          1024-1535      4
          1536-2047      8 and then reset */
    /* Each data parameter is 4 bits */
    memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
    AdvChirpCfgArgs.chirpParamIdx           = RL_LUT_CHIRP_TX_EN_VAR;
    AdvChirpCfgArgs.lutResetPeriod          = 1;
    AdvChirpCfgArgs.lutParamUpdatePeriod    = 0;
    AdvChirpCfgArgs.lutPatternAddressOffset = lutOffsetInNBytes;
    AdvChirpCfgArgs.numOfPatterns           = 1;
    AdvChirpCfgArgs.lutChirpParamSize       = 0; // Default size

    retVal = rlSetAdvChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, &AdvChirpCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("*** Failed - rlSetAdvChirpConfig failed with error code (Param - 5) %d*** \r\n", retVal);
        return retVal;
    }
    /* Update LUT for param = 5 (All Tx enable) */
    // AdvChirpLUTData[LUTWordSizeOffset] = (uint32_t) 0xFFFFFFFF; // Enable all
    // AdvChirpLUTData[LUTWordSizeOffset] = (uint32_t) 0x00000000; // Disable all
    // LUTWordSizeOffset = LUTWordSizeOffset + 4; // LUTWordSizeOffset = 24;

    uint8_t txEnableCfg = 0xF;
    /* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
    rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_TX_EN_VAR;
    rlFillLUTParamsArgs.chirpParamSize  = AdvChirpCfgArgs.lutChirpParamSize;
    rlFillLUTParamsArgs.inputSize       = AdvChirpCfgArgs.numOfPatterns;
    rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
    retVal                              = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&txEnableCfg, &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);
    while (lutOffsetInNBytes % 4 != 0)
    {
        lutOffsetInNBytes += 1;
    }

    /********************************************************************************************************************************************/
    /* Send the locally programmed LUT data to the device */
    rlAdvChirpLUTCfgArgs.lutAddressOffset = 0;
    rlAdvChirpLUTCfgArgs.numBytes         = lutOffsetInNBytes;

    retVal = rlSetMultiAdvChirpLUTConfig(RL_DEVICE_MAP_INTERNAL_BSS, &rlAdvChirpLUTCfgArgs, &AdvChirpLUTData[0]);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("rlSetMultiAdvChirpLUTConfig for deviceMap %u failed with error code %d \r\n", RL_DEVICE_MAP_INTERNAL_BSS, retVal);
        return retVal;
    }

    /********************************************************************************************************************************************/
    return retVal;
}
#endif


void MmwDemo_loadGUIMonConfig(void)
{
    uint8_t i = 0;

    for (i = 0; i < RL_MAX_SUBFRAMES; i++)
    {
        MmwDemo_GuiMonSel *guiMonSel = (MmwDemo_GuiMonSel *)&gMmw_MCB.subFrameCfg[i].guiMonSel;

        /* Set the selection of which GUI monitors need to be enabled */
        guiMonSel->detectedObjects     = HECR_GUI_DETECT_OBJ;
        guiMonSel->logMagRange         = HECR_GUI_LOG_MAG_RANGE;
        guiMonSel->noiseProfile        = HECR_GUI_NOISE_PROFILE;
        guiMonSel->rangeAzimuthHeatMap = HECR_GUI_RANGE_AZI_HEAT_MAP;
        guiMonSel->rangeDopplerHeatMap = HECR_GUI_RANGE_DOP_HEAR_MAP;
        guiMonSel->statsInfo           = HECR_GUI_STATS_INFO;
        guiMonSel->ransacFilterMask    = HECR_GUI_RANSAC_FILTER_MSK;
        guiMonSel->trackers            = HECR_GUI_TRACKERS;
    }
}

#ifdef LVDS_STREAM
int32_t MmwDemo_loadLVDSStreamConfig()
{
    int32_t                retVal = 0;
    MmwDemo_LvdsStreamCfg *cfg;

    cfg = (MmwDemo_LvdsStreamCfg *)&gMmw_MCB.subFrameCfg[0].lvdsStreamCfg;

    /* Set the ADC buf configuration */
    cfg->isHeaderEnabled = HECR_LVDS_IS_HEADER_ENABLED;
    cfg->dataFmt         = HECR_LVDS_DATA_FMT;
    cfg->isSwEnabled     = HECR_LVDS_IS_SW_ENABLED;

    /* If both h/w and s/w are enabled, HSI header must be enabled, because
     * we don't allow mixed h/w session without HSI header
     * simultaneously with s/w session with HSI header (s/w session always
     * streams HSI header) */
    if ((cfg->isSwEnabled == true) && (cfg->dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED))
    {
        if (cfg->isHeaderEnabled == false)
        {
            DebugP_log("Error: header must be enabled when both h/w and s/w streaming are enabled\r\n");
            retVal = -1;
        }
    }

    /* Set the state of the configuration for the ADC buffer */
    gMmw_MCB.subFrameCfg[0].isLvdsStreamCfgPending = 1;

    return retVal;
}
#endif

int32_t MmwDemo_loadPhaseShiftConfig()
{
    int32_t  retVal = 0;
    uint16_t chirpIdx, chirpStartIdx, chirpEndIdx, numTxAntAzim, numTxTotalDivisor,
        numTxAntElev, txAntMask, txOrderIdx, activeTxCnt, chirpPhaseMultiplier;
    /* xth value of this array corresponds to the phase shift multiplier for the xth Tx antenna */
    uint16_t            phaseShiftMultiplier[SYS_COMMON_NUM_TX_ANTENNAS];
    rlRfPhaseShiftCfg_t phaseShiftCfg;

/* In advanced chirp mode, this will be updated in the advanced chirp load function */
#ifndef HECR_ENABLE_ADV_CHIRP
    rlRfMiscConf_t stRFMiscCnf;

    /* Set the miscellaneous configuration to enable the perchirp phase shifter only
     * if the number of phase shift chirps is non zero */
    memset((void *)&stRFMiscCnf, 0, sizeof(stRFMiscCnf));

    stRFMiscCnf.miscCtl          = 1U;
    stRFMiscCnf.crdNSlopeMag     = 0U;
    stRFMiscCnf.fastResetEndTime = 0U;

    retVal = rlRfSetMiscConfig(RL_DEVICE_MAP_INTERNAL_BSS, &stRFMiscCnf);
    if (retVal != RL_RET_CODE_OK)
    {
        DebugP_log("rlRfSetMiscConfig failed for deviceMap %u with error code %d\r\n", RL_DEVICE_MAP_INTERNAL_BSS, retVal);
        retVal = -1;
        goto exit;
    }
#endif

    /* Calculate and program the perchirp phase shift across the Tx */
    memset((void *)&phaseShiftCfg, 0, sizeof(phaseShiftCfg));

    txAntMask    = gMmw_MCB.cfg.openCfg.chCfg.txChannelEn;
    numTxAntAzim = mathUtils_countSetBits(txAntMask & MmwDemo_RFParserHwCfg.azimTxAntMask);
    numTxAntElev = mathUtils_countSetBits(txAntMask & MmwDemo_RFParserHwCfg.elevTxAntMask);

    gMmw_MCB.numEmptySubBands = MmwDemo_getNumEmptySubBands(numTxAntAzim + numTxAntElev);
    numTxTotalDivisor         = numTxAntAzim + numTxAntElev + gMmw_MCB.numEmptySubBands;

    activeTxCnt = 0;
    /* Get the phase multiplier factor for each Tx antenna */
    /* Loop over Tx Antenna Phase order index */
    for (txOrderIdx = 0; txOrderIdx < SYS_COMMON_NUM_TX_ANTENNAS; txOrderIdx++)
    {
        /* Check if the Tx antenna corresponding to the xth index in the phase order is enabled */
        if (1 << gMmw_MCB.ddmPhaseShiftOrder[txOrderIdx] & txAntMask)
        {
            /* Antenna is enabled, hence compute the phase shift value */
            phaseShiftMultiplier[gMmw_MCB.ddmPhaseShiftOrder[txOrderIdx]] = activeTxCnt;
            activeTxCnt++;
        }
        else
        {
            /* Antenna is disabled */
            phaseShiftMultiplier[gMmw_MCB.ddmPhaseShiftOrder[txOrderIdx]] = 0;
        }
    }

    chirpStartIdx = gMmw_MCB.cfg.ctrlCfg.u.frameCfg[0].frameCfg.chirpStartIdx;
    chirpEndIdx   = gMmw_MCB.cfg.ctrlCfg.u.frameCfg[0].frameCfg.chirpEndIdx;

    /* Phase multipliers have been computed; phase shift for xth chirp = (x-1) * phase multipler */
    for (chirpIdx = chirpStartIdx; chirpIdx <= chirpEndIdx; chirpIdx++)
    {
        chirpPhaseMultiplier = (chirpIdx - chirpStartIdx);

        /* Populate the chirp configuration: */
        phaseShiftCfg.chirpStartIdx = (chirpEndIdx + 1 - chirpPhaseMultiplier) % numTxTotalDivisor;
        phaseShiftCfg.chirpEndIdx   = (chirpEndIdx + 1 - chirpPhaseMultiplier) % numTxTotalDivisor;

        /* 1 LSB of phaseShiftCfg.txPhaseShift = 360/2^6 = 5.625 degrees Valid range: 0 to 63 */
        phaseShiftCfg.tx0PhaseShift =
            ((uint32_t)MATHUTILS_ROUND_FLOAT(((float)((chirpPhaseMultiplier * phaseShiftMultiplier[0]) % numTxTotalDivisor) / numTxTotalDivisor) * (1U << 6))) << 2;

        /* 1 LSB of phaseShiftCfg.txPhaseShift = 360/2^6 = 5.625 degrees Valid range: 0 to 63 */
        phaseShiftCfg.tx1PhaseShift =
            ((uint32_t)MATHUTILS_ROUND_FLOAT(((float)((chirpPhaseMultiplier * phaseShiftMultiplier[1]) % numTxTotalDivisor) / numTxTotalDivisor) * (1U << 6))) << 2;

        /* 1 LSB of phaseShiftCfg.txPhaseShift = 360/2^6 = 5.625 degrees Valid range: 0 to 63 */
        phaseShiftCfg.tx2PhaseShift =
            ((uint32_t)MATHUTILS_ROUND_FLOAT(((float)((chirpPhaseMultiplier * phaseShiftMultiplier[2]) % numTxTotalDivisor) / numTxTotalDivisor) * (1U << 6))) << 2;

#if defined(SOC_AWR2944) || defined(SOC_AWR2X44P)
        /* 1 LSB of phaseShiftCfg.txPhaseShift = 360/2^6 = 5.625 degrees Valid range: 0 to 63 */
        phaseShiftCfg.tx3PhaseShift =
            ((uint32_t)MATHUTILS_ROUND_FLOAT(((float)((chirpPhaseMultiplier * phaseShiftMultiplier[3]) % numTxTotalDivisor) / numTxTotalDivisor) * (1U << 6))) << 2;
#endif

        /* Add the chirp to the profile by setting the chirp level phase shifter configuration in the mmWave link */
        retVal = rlRfSetPhaseShiftConfig(RL_DEVICE_MAP_INTERNAL_BSS, 1U, &phaseShiftCfg);
        if (retVal != RL_RET_CODE_OK)
        {
            /* Error: Setting the chirp level phase shifter configuration failed */
            MmwDemo_printError(MMWAVE_EPHASESHIFTCFG, retVal);
        }
    }

exit:
    return retVal;
}


void MmwDemo_loadFastRangeProcCfg(void)
{
}

int32_t MmwDemo_loadDDMPhaseShiftOrder(void)
{
    uint32_t i = 0;
    for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS; i++)
    {
        gMmw_MCB.ddmPhaseShiftOrder[i] = ddm_phase_shift_order[i];
    }

    return 0;
}

int32_t MmwDemo_loadAntGeometryCfg(void)
{
    /* antGeometryCfg <Tx0Row> <Tx0Col> .... <TxNRow> <TxNCol> <xSpacebylambda> <zSpacebylambda>*/
#ifdef SOC_AWR2X44P
    uint32_t ant_geometry[33] = { 1, 0, 1, 2, 1, 7, 1, 9, 1, 3, 1, 5, 1, 10, 1, 12, 1, 6, 1, 8, 1, 13, 1, 15, 0, 9, 0, 11, 0, 16, 0, 18 };
#else
    uint32_t ant_geometry[33] = { 1, 0, 1, 1, 1, 2, 1, 3, 0, 2, 0, 3, 0, 4, 0, 5, 1, 4, 1, 5, 1, 6, 1, 7, 1, 8, 1, 9, 1, 10, 1, 11 };
#endif

    /*! @brief Antenna Geometry   */
    uint32_t i = 0, azimElemIdx = 0, elevElemIdx = 0;
    uint64_t zeroInsrtMaskAzim = 0, zeroInsrtMaskElev = 0;
    Element  antArr[MAX_NUM_VIRT_ANT];
    Element *elevAntArr = &antArr[MAX_NUM_AZIM_VIRT_ANT];

    /* Assumption: row1: azimuth antenna array, row0: elevation antenna array */
    for (i = 0; i < MAX_NUM_VIRT_ANT * 2; i += 2)
    {
        if (ant_geometry[i] == 0)
        {
            /* Elevation Antenna Sample */
            antArr[MAX_NUM_AZIM_VIRT_ANT + elevElemIdx].value = ant_geometry[i + 1];
            antArr[MAX_NUM_AZIM_VIRT_ANT + elevElemIdx].index = elevElemIdx;
            elevElemIdx++;
        }
        else if (ant_geometry[i] == 1)
        {
            /* Azimuth Antenna Sample */
            antArr[azimElemIdx].value = ant_geometry[i + 1];
            antArr[azimElemIdx].index = azimElemIdx;
            azimElemIdx++;
        }
        else
        {
            DebugP_log("Error: Invalid usage of the CLI command. Assumption - Row1: Azimuth Array (12 elements), Row0: Elevation Array (4 elements) \n");
            return -1;
        }
    }

    /* Sort the azimuth array of structures based on value */
    qsort(antArr, MAX_NUM_AZIM_VIRT_ANT, sizeof(antArr[0]), compare);

    /* Sort the elevation array of structures based on value */
    qsort(elevAntArr, MAX_NUM_ELEV_VIRT_ANT, sizeof(elevAntArr[0]), compare);

    /* Store the rearrangement Order */
    for (i = 0; i < MAX_NUM_VIRT_ANT; i++)
    {
        gMmw_MCB.objDetCommonCfg.preStartCommonCfg.antennaGeometryCfg[i] = (uint16_t)antArr[i].index;
    }

    /* Find the zero insertion mask */
    for (i = 0; i < MAX_NUM_AZIM_VIRT_ANT; i++)
    {
        zeroInsrtMaskAzim |= (uint64_t)1 << antArr[i].value;
    }
    gMmw_MCB.objDetCommonCfg.preStartCommonCfg.zeroInsrtMaskCfg.zeroInsrtMaskAzim = zeroInsrtMaskAzim;
    /* Check the number of set bits in azimuth array zero insertion mask */
    if (mathUtils_countSetBits(gMmw_MCB.objDetCommonCfg.preStartCommonCfg.zeroInsrtMaskCfg.zeroInsrtMaskAzim) != MAX_NUM_AZIM_VIRT_ANT)
    {
        DebugP_log("Error: Invalid input. Azimuth Zero Insertion Mask is incorrect. \n");
        return -1;
    }

    for (i = 0; i < MAX_NUM_ELEV_VIRT_ANT; i++)
    {
        zeroInsrtMaskElev |= (uint64_t)1 << elevAntArr[i].value;
    }
    gMmw_MCB.objDetCommonCfg.preStartCommonCfg.zeroInsrtMaskCfg.zeroInsrtMaskElev = zeroInsrtMaskElev;
    /* Check the number of set bits in elevation array zero insertion mask */
    if (mathUtils_countSetBits(gMmw_MCB.objDetCommonCfg.preStartCommonCfg.zeroInsrtMaskCfg.zeroInsrtMaskElev) != MAX_NUM_ELEV_VIRT_ANT)
    {
        DebugP_log("Error: Invalid input. Elevation Zero Insertion Mask is incorrect. \n");
        return -1;
    }

    gMmw_MCB.objDetCommonCfg.preStartCommonCfg.antennaSpacing.xSpacingByLambda = HECR_ANT_GEOMETRY_X_SPACING_BY_LAMBDA;
    gMmw_MCB.objDetCommonCfg.preStartCommonCfg.antennaSpacing.zSpacingByLambda = HECR_ANT_GEOMETRY_Z_SPACING_BY_LAMBDA;

    gMmw_MCB.objDetCommonCfg.isAntennaGeometryCfgPending = 1;

    return 0;
}

void MmwDemo_loadFOVCfg(void)
{
    uint8_t i = 0;
    for (i = 0; i < RL_MAX_SUBFRAMES; i++)
    {
        DPC_ObjectDetection_FovAoaCfg *fovCfg;
        fovCfg = &gMmw_MCB.subFrameCfg[i].datapathStaticCfg.aoaFovCfg;

        fovCfg->minAzimuthDeg   = HECR_FOVCFG_MIN_AZI_DEG;
        fovCfg->maxAzimuthDeg   = HECR_FOVCFG_MAX_AZI_DEG;
        fovCfg->minElevationDeg = HECR_FOVCFG_MIN_ELV_DEG;
        fovCfg->maxElevationDeg = HECR_FOVCFG_MAX_ELV_DEG;
    }
}

int32_t MmwDemo_loadCFARCfg(void)
{
    uint8_t i = 0;
    for (i = 0; i < RL_MAX_SUBFRAMES; i++)
    {
        DPU_DopplerProc_CfarCfg *dopCfarCfg;
        DPU_DopplerProc_CfarCfg *rngCfarCfg;

        dopCfarCfg = &gMmw_MCB.subFrameCfg[i].datapathStaticCfg.cfarCfg.cfg;
        rngCfarCfg = &gMmw_MCB.subFrameCfg[i].datapathStaticCfg.rangeCfarCfg.cfg;

        rngCfarCfg->isEnabled      = HECR_CFAR_RANGE_IS_ENABLED;
        rngCfarCfg->averageMode    = HECR_CFAR_RANGE_AVG_MODE;
        rngCfarCfg->winLen         = HECR_CFAR_RANGE_WIN_LEN;
        rngCfarCfg->guardLen       = HECR_CFAR_RANGE_GUARD_LEN;
        rngCfarCfg->noiseDivShift  = HECR_CFAR_RANGE_NOISE_DIV_SHIFT;
        rngCfarCfg->cyclicMode     = HECR_CFAR_RANGE_CYCLIC_MODE;
        rngCfarCfg->peakGroupingEn = HECR_CFAR_RANGE_PEAK_GRP_EN;
        rngCfarCfg->osKvalue       = HECR_CFAR_RANGE_OS_K_VAL;
        rngCfarCfg->osEdgeKscaleEn = HECR_CFAR_RANGE_OS_EDGE_KSCL_EN;
        rngCfarCfg->thresholdScale = HECR_CFAR_RANGE_THRESHOLD;

        /* Populate the configuration for the CFAR in the doppler dimensions */
        dopCfarCfg->isEnabled      = HECR_CFAR_DOPPLER_IS_ENABLED;
        dopCfarCfg->averageMode    = HECR_CFAR_DOPPLER_AVG_MODE;
        dopCfarCfg->winLen         = HECR_CFAR_DOPPLER_WIN_LEN;
        dopCfarCfg->guardLen       = HECR_CFAR_DOPPLER_GUARD_LEN;
        dopCfarCfg->noiseDivShift  = HECR_CFAR_DOPPLER_NOISE_DIV_SHIFT;
        dopCfarCfg->cyclicMode     = HECR_CFAR_DOPPLER_CYCLIC_MODE;
        dopCfarCfg->peakGroupingEn = HECR_CFAR_DOPPLER_PEAK_GRP_EN;
        dopCfarCfg->osKvalue       = HECR_CFAR_DOPPLER_OS_K_VAL;
        dopCfarCfg->osEdgeKscaleEn = HECR_CFAR_DOPPLER_OS_EDGE_KSCL_EN;
        dopCfarCfg->thresholdScale = HECR_CFAR_DOPPLER_THRESHOLD;

        if ((dopCfarCfg->thresholdScale / MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR < 0) ||
            (dopCfarCfg->thresholdScale / MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR > 100U) ||
            (rngCfarCfg->thresholdScale / MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR < 0) ||
            (rngCfarCfg->thresholdScale / MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR > 100U))
        {
            DebugP_log("Error: threshold configured should be from 0-100 dB in Subframe: %d \n", i);
            return -1;
        }
    }
    return 0;
}

int32_t MmwDemo_loadCompressionCfg(void)
{
    uint8_t i = 0;

    for (i = 0; i < RL_MAX_SUBFRAMES; i++)
    {
        DPU_RangeProcHWA_CompressionCfg *compressionCfg;
        compressionCfg = &gMmw_MCB.subFrameCfg[i].datapathStaticCfg.compressionCfg;

        compressionCfg->isEnabled         = HECR_CMPR_CFG_ENABLED;
        compressionCfg->compressionMethod = HECR_CMPR_CFG_METHOD;
        compressionCfg->compressionRatio  = HECR_CMPR_CFG_RATIO;
        compressionCfg->rangeBinsPerBlock = HECR_CMPR_CFG_BINS_PER_BLK;

        if (!((compressionCfg->rangeBinsPerBlock & (compressionCfg->rangeBinsPerBlock - 1)) == 0)) /* is it a power of 2? */
        {
            DebugP_log("Error: rangeBinsPerBlock should be a power of 2 for Subframe %d\n", i);
            return -1;
        }
    }

    return 0;
}

int32_t MmwDemo_loadDatapathConfig(void)
{
    int32_t                                     retVal = 0, idx;
    int32_t                                     i, j = 0;
    DPC_ObjectDetection_MeasureRxChannelBiasCfg cfg;

    /* Antenna Calibration Params */
    float antennaCalibParams[SYS_COMMON_NUM_RX_CHANNEL * SYS_COMMON_NUM_TX_ANTENNAS * 2] = { 0.6718, 0.3970, 0.5714, 0.5259, -0.0907, 0.8665, 0.0989, -0.9754, 0.8083, 0.0366, 0.7769, 0.2025, 0.3331, 0.8275, -0.3769, -0.9263, -0.6290, -0.4124, -0.5234, -0.5298, 0.1282, -0.8125, -0.1418, 0.9100, 0.5869, 0.3367, 0.4966, 0.4459, -0.0732, 0.7262, 0.0808, -0.8108 };
    /* When proper values measured, replace the following values and uncomment parameters below. Comment line above*/
    // float antennaCalibParams[SYS_COMMON_NUM_RX_CHANNEL * SYS_COMMON_NUM_TX_ANTENNAS * 2] = { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 };


    int32_t numTx             = mathUtils_countSetBits(gMmw_MCB.cfg.openCfg.chCfg.txChannelEn);
    gMmw_MCB.numEmptySubBands = MmwDemo_getNumEmptySubBands(numTx);

    retVal = MmwDemo_loadDDMPhaseShiftOrder();
    if (retVal < 0)
    {
        goto exit;
    }

    retVal = MmwDemo_loadAntGeometryCfg();
    if (retVal < 0)
    {
        goto exit;
    }

    /****************************************************************************************************************************/
    /*                                     Populate the configuration for ADC Buffer                                            */
    /****************************************************************************************************************************/
    retVal = MmwDemo_loadADCBUFConfig();
    if (retVal < 0)
    {
        goto exit;
    }

#ifdef LVDS_STREAM
    /****************************************************************************************************************************/
    /*                                     Populate the configuration for LVDS streaming                                           */
    /****************************************************************************************************************************/
    retVal = MmwDemo_loadLVDSStreamConfig();
    if (retVal < 0)
    {
        goto exit;
    }
#endif

    /****************************************************************************************************************************/
    /*                                     Populate the configuration for GUI Selection                                         */
    /****************************************************************************************************************************/
    MmwDemo_loadGUIMonConfig();

    /****************************************************************************************************************************/
    /*                                     Populate the configuration for FastRangeProc                                       */
    /****************************************************************************************************************************/
    MmwDemo_loadFastRangeProcCfg();

    /****************************************************************************************************************************/
    /*                                     Populate the configuration for AoA FoV                                               */
    /****************************************************************************************************************************/
    MmwDemo_loadFOVCfg();

    /****************************************************************************************************************************/
    /*                               Populate the configuration for the CFAR in the range dimensions                            */
    /****************************************************************************************************************************/
    retVal = MmwDemo_loadCFARCfg();
    if (retVal != 0)
    {
        goto exit;
    }

    /****************************************************************************************************************************/
    /*                                      Populate configuration for compression stage                                        */
    /****************************************************************************************************************************/
    retVal = MmwDemo_loadCompressionCfg();
    if (retVal != 0)
    {
        goto exit;
    }

    /****************************************************************************************************************************/
    /*                                Populate configuration for the 2D local max algorithm                                     */
    /****************************************************************************************************************************/
    for (i = 0; i < RL_MAX_SUBFRAMES; i++)
    {
        DPU_DopplerProc_LocalMaxCfg *localMaxCfg;
        localMaxCfg = &gMmw_MCB.subFrameCfg[i].datapathStaticCfg.localMaxCfg;

        localMaxCfg->azimThreshold    = HECR_LOCAL_MAX_AZIM_THRES;
        localMaxCfg->dopplerThreshold = HECR_LOCAL_MAX_DOPPLER_THRES;
    }

    /****************************************************************************************************************************/
    /*                                 Populate configuration for interference mitigation                                       */
    /****************************************************************************************************************************/
    for (i = 0; i < RL_MAX_SUBFRAMES; i++)
    {
        DPU_RangeProcHWADDMA_intfStatsdBCfg *intfStatsdBCfg;
        intfStatsdBCfg = &gMmw_MCB.subFrameCfg[i].datapathStaticCfg.intfStatsdBCfg;

        intfStatsdBCfg->intfMitgMagSNRdB     = HECR_INTF_MITG_MAG_SNR_DB;
        intfStatsdBCfg->intfMitgMagDiffSNRdB = HECR_INTF_MITG_MAG_DIFF_SNR_DB;
    }

    /****************************************************************************************************************************/
    /*                                 Populate configuration for measuring antenna calibration                                 */
    /****************************************************************************************************************************/
    // enable this parameter to measure range & phase bias
    cfg.enabled        = (uint8_t)ANTENNA_CALIB_ENABLE;
    cfg.targetDistance = (float)5.0;
    cfg.searchWinSize  = (float)1.0;

    /* Save Configuration to use later */
    memcpy((void *)&gMmw_MCB.objDetCommonCfg.preStartCommonCfg.measureRxChannelBiasCfg,
           &cfg,
           sizeof(cfg));

    /****************************************************************************************************************************/
    /*                                     Load the calibration parameters (Default)                                            */
    /****************************************************************************************************************************/
#ifndef SOC_AWR2X44P
    memset((void *)&antennaCalibParams, 0, sizeof(antennaCalibParams)); // TODO : Add mechanism to hold valid calibration params

    for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL * 2; i = i + 2)
    {
        antennaCalibParams[i]     = 0U; // Update the imag part
        antennaCalibParams[i + 1] = 1U; // Update the real part
    }
#endif

    /* Save Configuration to use later */
    memcpy((void *)&gMmw_MCB.objDetCommonCfg.preStartCommonCfg.antennaCalibParams,
           &antennaCalibParams,
           sizeof(antennaCalibParams));

    /* rearrange the antenna calibration parameters based on the antGeometryCfg */
    if (gMmw_MCB.objDetCommonCfg.isAntennaGeometryCfgPending)
    {
        for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL * 2; i += 2)
        {
            if (i < MAX_NUM_AZIM_VIRT_ANT * 2)
                idx = 2 * gMmw_MCB.objDetCommonCfg.preStartCommonCfg.antennaGeometryCfg[j++];
            else
                idx = 2 * (MAX_NUM_AZIM_VIRT_ANT + gMmw_MCB.objDetCommonCfg.preStartCommonCfg.antennaGeometryCfg[j++]);
            gMmw_MCB.objDetCommonCfg.preStartCommonCfg.antennaCalibParams[i]     = antennaCalibParams[idx];
            gMmw_MCB.objDetCommonCfg.preStartCommonCfg.antennaCalibParams[i + 1] = antennaCalibParams[idx + 1];
        }
    }
    else
    {
        // CLI_write ("Error: antGeometryCfg should be provided before antennaCalibParams.\n");
        return -1;
    }

    gMmw_MCB.objDetCommonCfg.isAntennaCalibParamCfgPending = 1;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Perform Data path driver open
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_AdcPathOpen(void)
{
    gMmw_MCB.adcBufHandle = MmwDemo_ADCBufOpen();
    if (gMmw_MCB.adcBufHandle == NULL)
    {
        MmwDemo_debugAssert(0);
    }
}

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to do one time sensor initialization.
 *      User need to fill gMmw_MCB.cfg.openCfg before calling this function
 *
 *  @param[in]  isFirstTimeOpen     If true then issues MMwave_open
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t MmwDemo_openSensor(bool isFirstTimeOpen)
{
#ifndef MMWAVELINK_ON_MSS
    int32_t                 retVal;
    MMWave_CalibrationData  calibrationDataCfg;
    MMWave_CalibrationData *ptrCalibrationDataCfg;
#endif
    /**********************************************************
     **********************************************************/

    /* Open mmWave module, this is only done once */
    /* Setup the calibration frequency:*/
    gMmw_MCB.cfg.openCfg.freqLimitLow  = 760U;
    gMmw_MCB.cfg.openCfg.freqLimitHigh = 810U;

    /* start/stop async events */
    gMmw_MCB.cfg.openCfg.disableFrameStartAsyncEvent = false;
    gMmw_MCB.cfg.openCfg.disableFrameStopAsyncEvent  = false;

    /* No custom calibration: */
    gMmw_MCB.cfg.openCfg.useCustomCalibration        = false;
    gMmw_MCB.cfg.openCfg.customCalibrationEnableMask = 0x0;

    MmwDemo_DataOutputMode();
    MmwDemo_ChannelCfg();
    MmwDemo_ADCCfg();

    // Low power mode configuration
    gMmw_MCB.cfg.openCfg.lowPowerMode.lpAdcMode = HECR_LOW_PWR_ADC;

    /* calibration monitoring base time unit
     * setting it to one frame duration as the demo doesnt support any
     * monitoring related functionality
     */
    gMmw_MCB.cfg.openCfg.calibMonTimeUnit = 4;

#ifndef MMWAVELINK_ON_MSS
    if ((gMmw_MCB.calibCfg.saveEnable != 0) &&
        (gMmw_MCB.calibCfg.restoreEnable != 0))
    {
        /* Error: only one can be enabled at at time */
        DebugP_log("Error: MmwDemo failed with both save and restore enabled.\n");
        return -1;
    }

    if (gMmw_MCB.calibCfg.restoreEnable != 0)
    {
        /*  Boot calibration during restore: Disable calibration for:
                - Rx gain,
                - Rx IQMM,
                - Tx phase shifer,
                - Tx Power

                The above calibration data will be restored from flash. Since they are calibrated in a control
                way to avoid interfaerence and spec violations.
                In this demo, other bit fields(except the above) are enabled as indicated in customCalibrationEnableMask to perform boot time
                calibration. The boot time calibration will overwrite the restored calibration data from flash.
                However other bit fields can be disabled and calibration data can be restored from flash as well.

                Note: In this demo, calibration masks are enabled for all bit fields when "saving" the data.
        */
        gMmw_MCB.cfg.openCfg.useCustomCalibration        = true;
        gMmw_MCB.cfg.openCfg.customCalibrationEnableMask = 0x1F0U;
    }
    else
    {
        ptrCalibrationDataCfg = NULL;
    }
#endif

    MmwDemo_openLink();

#ifndef MMWAVELINK_ON_MSS
    /* Save calibration data in flash */
    if (gMmw_MCB.calibCfg.saveEnable != 0)
    {
        retVal = rlRfCalibDataStore(RL_DEVICE_MAP_INTERNAL_BSS, &gCalibDataStoragetoMss.calibData);
        if (retVal != RL_RET_CODE_OK)
        {
            /* Error: Calibration data restore failed */
            DebugP_log("MSS demo failed rlRfCalibDataStore with Error[%d]\n", retVal);
            return -1;
        }

        /* update txIndex in all chunks to get data from all Tx.
        This should be done regardless of num TX channels enabled in MMWave_OpenCfg_t::chCfg or number of Tx
        application is interested in. Data for all existing Tx channels should be retrieved
        from RadarSS and in the order as shown below.
        RadarSS will return non-zero phase shift values for all the channels enabled via
        MMWave_OpenCfg_t::chCfg and zero phase shift values for channels disabled in MMWave_OpenCfg_t::chCfg */
        gCalibDataStoragetoMss.phaseShiftCalibData.PhShiftcalibChunk[0].txIndex = 0;
        gCalibDataStoragetoMss.phaseShiftCalibData.PhShiftcalibChunk[1].txIndex = 1;
        gCalibDataStoragetoMss.phaseShiftCalibData.PhShiftcalibChunk[2].txIndex = 2;

        /* Basic validation passed: Restore the phase shift calibration data */
        retVal = rlRfPhShiftCalibDataStore(RL_DEVICE_MAP_INTERNAL_BSS, &(gCalibDataStoragetoMss.phaseShiftCalibData));
        if (retVal != RL_RET_CODE_OK)
        {
            /* Error: Phase shift Calibration data restore failed */
            DebugP_log("MSS demo failed rlRfPhShiftCalibDataStore with Error[%d]\n", retVal);
            return retVal;
        }

        calibrationDataCfg.ptrCalibData           = &gCalibDataStoragetoMss.calibData;
        calibrationDataCfg.ptrPhaseShiftCalibData = &gCalibDataStoragetoMss.phaseShiftCalibData;
        ptrCalibrationDataCfg                     = &calibrationDataCfg;

        retVal = DPM_send(gMmw_MCB.dataPathObj.objDetDpmHandle, (void *)(ptrCalibrationDataCfg), sizeof(MMWave_CalibrationData), CSL_CORE_ID_R5FSS0_0);
        if (retVal != 0)
        {
            /* Error: Phase shift Calibration data restore failed */
            printf("DPM_send failed with Error[%d]\n", retVal);
            return retVal;
        }
    }
#endif

    /* Open the datapath modules that runs on MSS */
    MmwDemo_AdcPathOpen();
    return 0;
}


/**
 *  @b Description
 *  @n
 *      Utility function to convert the CFAR threshold
 *      from a CLI encoded dB value to a log2 value
 *      as expected by the doppler CFAR DPU
 *
 *  @param[in] codedCfarVal CFAR SNR threshold in dB as encoded in the CLI
 *  @param[in] numBands Total number of subbands
 *
 *  @retval
 *      CFAR threshold in dB format
 */
#define CONST_LOG2_10 (3.3219)
static uint16_t MmwDemo_convertDopplerCfarToThresh(uint16_t codedCfarVal)
{
    uint16_t linearVal;
    float    dbVal, linVal;

    /* dbVal is a float value from 0-100dB. It needs to
    be converted to linear scale..
    First, recover float dbVal that was encoded in CLI. */
    dbVal = (float)(codedCfarVal / MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);

    /* Now convert it to linear value */
    linVal = (uint32_t)(dbVal / 20.0 * CONST_LOG2_10 * 2048.0);

    linearVal = (uint16_t)linVal;
    return (linearVal);
}

/**
 *  @b Description
 *  @n
 *      Utility function to convert the CFAR threshold
 *      from a CLI encoded dB value to a log2 value
 *      as expected by the Range CFAR DPU
 *
 *  @param[in] codedCfarVal CFAR SNR threshold in dB as encoded in the CLI
 *  @param[in] numBands Total number of subbands
 *
 *  @retval
 *      CFAR threshold in dB format
 */
static uint16_t MmwDemo_convertRangeCfarToThresh(uint16_t codedCfarVal, uint8_t numBands)
{
    uint16_t linearVal;
    float    dbVal, linVal;
    uint32_t defaultScaling    = 1 << 11;
    float    additionalScaling = numBands / (float)(1 << mathUtils_ceilLog2(numBands));

    /* dbVal is a float value from 0-100dB. It needs to
    be converted to linear scale..
    First, recover float dbVal that was encoded in CLI. */
    dbVal = (float)(codedCfarVal / MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);

    /* Now convert it to linear value */
    linVal = (uint32_t)(log2f(pow(10, (float)dbVal / 20.0)) * additionalScaling * defaultScaling + 0.5);

    linearVal = (uint16_t)linVal;
    return (linearVal);
}

/**
 *  @b Description
 *  @n
 *      Resets (clears) all pending common configuration of Object Detection DPC
 *
 *  @param[in] cfg Object Detection DPC common configuration
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_resetDynObjDetCommonCfgPendingState(MmwDemo_DPC_ObjDet_CommonCfg *cfg)
{
    cfg->isAntennaCalibParamCfgPending = 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the data path based on the chirp profile.
 *      After this function is executed, the data path processing will ready to go
 *      when the ADC buffer starts receiving samples corresponding to the chirps.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t MmwDemo_dataPathConfig(void)
{
    int32_t                         errCode;
    MMWave_CtrlCfg                 *ptrCtrlCfg;
    MmwDemo_DPC_ObjDet_CommonCfg   *objDetCommonCfg;
    MmwDemo_SubFrameCfg            *subFrameCfg;
    int8_t                          subFrameIndx;
    MmwDemo_RFParserOutParams       RFparserOutParams;
    DPC_ObjectDetection_PreStartCfg objDetPreStartCfg;
    DPC_ObjectDetection_StaticCfg  *staticCfg;
    bool                            procChain = 1;
    /* Get data path object and control configuration */
    ptrCtrlCfg = &gMmw_MCB.cfg.ctrlCfg;

    objDetCommonCfg = &gMmw_MCB.objDetCommonCfg;
    staticCfg       = &objDetPreStartCfg.staticCfg;

    gMmw_MCB.objDetCommonCfg.preStartCommonCfg.numSubFrames =
        MmwDemo_RFParser_getNumSubFrames(ptrCtrlCfg);

    DebugP_logInfo("App: Issuing Pre-start Common Config IOCTL\n");

    /* Get RF frequency scale factor */
    gMmw_MCB.rfFreqScaleFactor = 3.6f;

#ifdef SUBSYS_MSS /* if running on MSS */
    /* DPC pre-start common config */
    errCode = MmwDemo_DPM_ioctl_blocking(gMmw_MCB.objDetDpmHandle,
                                         DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG,
                                         &objDetCommonCfg->preStartCommonCfg,
                                         sizeof(DPC_ObjectDetection_PreStartCommonCfg));

    if (errCode < 0)
    {
        DebugP_log("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG [Error:%d]\n", errCode);
        goto exit;
    }
#elif (defined(SOC_AWR2X44P) && defined(SUBSYS_DSS)) /* if running on DSS for AWR2E44P */
    /* DPC pre-start common config */
    errCode = MmwDemo_DPM_ioctl_blocking(gMmw_MCB.dataPathObj.objDetDpmHandle,
                                         DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG,
                                         &objDetCommonCfg->preStartCommonCfg,
                                         sizeof(DPC_ObjectDetection_PreStartCommonCfg));

    if (errCode < 0)
    {
        DebugP_log("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG [Error:%d]\n", errCode);
        goto exit;
    }
#else

    objDetCommonCfg = objDetCommonCfg; // TODO JIT: take action
#endif
    MmwDemo_resetDynObjDetCommonCfgPendingState(&gMmw_MCB.objDetCommonCfg);

    /* Reason for reverse loop is that when sensor is started, the first sub-frame
     * will be active and the ADC configuration needs to be done for that sub-frame
     * before starting (ADC buf hardware does not have notion of sub-frame, it will
     * be reconfigured every sub-frame). This cannot be alternatively done by calling
     * the MmwDemo_ADCBufConfig function only for the first sub-frame because this is
     * a utility API that computes the rxChanOffset that is part of ADC dataProperty
     * which will be used by range DPU and therefore this computation is required for
     * all sub-frames.
     */
    for (subFrameIndx = gMmw_MCB.objDetCommonCfg.preStartCommonCfg.numSubFrames - 1; subFrameIndx >= 0;
         subFrameIndx--)
    {
        subFrameCfg = &gMmw_MCB.subFrameCfg[subFrameIndx];

        /*****************************************************************************
         * Data path :: Algorithm Configuration
         *****************************************************************************/

        /* Parse the profile and chirp configs and get the valid number of TX Antennas */
        errCode = MmwDemo_RFParser_parseConfig(&RFparserOutParams, subFrameIndx, &gMmw_MCB.cfg.openCfg, ptrCtrlCfg, &subFrameCfg->adcBufCfg, gMmw_MCB.rfFreqScaleFactor, false, procChain);

        /* if number of doppler chirps is too low, interpolate to be able to detect
         * better with CFAR tuning. E.g. a 2-pt FFT will be problematic in terms
         * of distinguishing direction of motion */
        if (RFparserOutParams.numDopplerChirps <= 4)
        {
            RFparserOutParams.dopplerStep    = RFparserOutParams.dopplerStep / (8 / RFparserOutParams.numDopplerBins);
            RFparserOutParams.numDopplerBins = 8;
        }

        if (errCode != 0)
        {
            DebugP_log("Error: MmwDemo_RFParser_parseConfig [Error:%d]\n", errCode);
            goto exit;
        }

        subFrameCfg->numRangeBins = RFparserOutParams.numRangeBins;
        /* Workaround for range DPU limitation for FFT size 1024 and 12 virtual antennas case*/
        if ((RFparserOutParams.numVirtualAntennas == 12) && (RFparserOutParams.numRangeBins == 1024))
        {
            subFrameCfg->numRangeBins      = 1022;
            RFparserOutParams.numRangeBins = 1022;
        }

        subFrameCfg->datapathStaticCfg.compressionCfg.numRxAntennaPerBlock = RFparserOutParams.numRxAntennas;
        if (subFrameCfg->datapathStaticCfg.compressionCfg.compressionMethod == 1)
        { /* BFP Compression */
            subFrameCfg->datapathStaticCfg.compressionCfg.bfpCompExtraParamSets = 2 * (RFparserOutParams.numRxAntennas - 1);
        }
        else
        {
            subFrameCfg->datapathStaticCfg.compressionCfg.bfpCompExtraParamSets = 0;
        }

        subFrameCfg->numDopplerBins         = RFparserOutParams.numDopplerBins;
        subFrameCfg->numChirpsPerChirpEvent = RFparserOutParams.numChirpsPerChirpEvent;
        subFrameCfg->adcBufChanDataSize     = RFparserOutParams.adcBufChanDataSize;

        subFrameCfg->numAdcSamples        = RFparserOutParams.numAdcSamples;
        subFrameCfg->numChirpsPerSubFrame = RFparserOutParams.numChirpsPerFrame;
        subFrameCfg->numVirtualAntennas   = RFparserOutParams.numVirtualAntennas;

        errCode = MmwDemo_ADCBufConfig(gMmw_MCB.adcBufHandle,
                                       gMmw_MCB.cfg.openCfg.chCfg.rxChannelEn,
                                       subFrameCfg->numChirpsPerChirpEvent,
                                       subFrameCfg->adcBufChanDataSize,
                                       &subFrameCfg->adcBufCfg,
                                       &staticCfg->ADCBufData.dataProperty.rxChanOffset[0]);
        if (errCode < 0)
        {
            DebugP_log("Error: ADCBuf config failed with error[%d]\n", errCode);
            MmwDemo_debugAssert(0);
        }
#if 0 // TODO : disable CQ
        errCode = MmwDemo_configCQ(subFrameCfg, subFrameCfg->numChirpsPerChirpEvent,
                                   RFparserOutParams.validProfileIdx);
        if (errCode < 0)
        {
            DebugP_log("Error: CQ config failed with error[%d]\n", errCode);
            MmwDemo_debugAssert(0);
        }
#endif
        /* DPC pre-start config */
        {
            int32_t idx;

            objDetPreStartCfg.subFrameNum = subFrameIndx;

            /* Fill static configuration */
            staticCfg->ADCBufData.data                 = (void *)CSL_RSS_ADCBUF_READ_U_BASE;
            staticCfg->ADCBufData.dataProperty.adcBits = ADCBUF_DATA_PROPERTY_ADCBITS_16BIT; /* 16-bit */

            /* only real format supported */
            MmwDemo_debugAssert(subFrameCfg->adcBufCfg.adcFmt == 1);

            staticCfg->ADCBufData.dataProperty.dataFmt = DPIF_DATAFORMAT_REAL16;

            if (subFrameCfg->adcBufCfg.chInterleave == 0)
            {
                staticCfg->ADCBufData.dataProperty.interleave = DPIF_RXCHAN_INTERLEAVE_MODE;
            }
            else
            {
                staticCfg->ADCBufData.dataProperty.interleave = DPIF_RXCHAN_NON_INTERLEAVE_MODE;
            }
            staticCfg->ADCBufData.dataProperty.numAdcSamples          = RFparserOutParams.numAdcSamples;
            staticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent = RFparserOutParams.numChirpsPerChirpEvent;
            staticCfg->ADCBufData.dataProperty.numRxAntennas          = RFparserOutParams.numRxAntennas;
            staticCfg->ADCBufData.dataSize                            = RFparserOutParams.numRxAntennas * RFparserOutParams.numAdcSamples * sizeof(cmplx16ImRe_t);
            staticCfg->dopplerStep                                    = RFparserOutParams.dopplerStep;
            staticCfg->isValidProfileHasOneTxPerChirp                 = RFparserOutParams.validProfileHasOneTxPerChirp;
            staticCfg->numChirpsPerFrame                              = RFparserOutParams.numChirpsPerFrame;
            staticCfg->numDopplerBins                                 = RFparserOutParams.numDopplerBins;

            staticCfg->ADCBufConfig.rxChannelEn        = gMmw_MCB.cfg.openCfg.chCfg.rxChannelEn;
            staticCfg->ADCBufConfig.adcBufChanDataSize = RFparserOutParams.adcBufChanDataSize;

            staticCfg->numChirps = RFparserOutParams.numDopplerChirps;
            /* Sum Tx must be enabled if range profile is to be sent out or
                 range CFAR is enabled */

            staticCfg->isSumTxEnabled = (subFrameCfg->datapathStaticCfg.rangeCfarCfg.cfg.isEnabled) ||
                (subFrameCfg->guiMonSel.logMagRange) ||
                (subFrameCfg->guiMonSel.noiseProfile) ||
                (subFrameCfg->guiMonSel.rangeDopplerHeatMap) ||
                (gMmw_MCB.objDetCommonCfg.preStartCommonCfg.measureRxChannelBiasCfg.enabled);

            staticCfg->numRangeBins = RFparserOutParams.numRangeBins;
            /* Number of range bins are half the number of FFT Bins in case of real only chirp data */
            if (RFparserOutParams.adcDataFmtIsReal)
            {
                staticCfg->numRangeFFTBins = (RFparserOutParams.numRangeBins) * (2);
            }
            else
            {
                staticCfg->numRangeFFTBins = (RFparserOutParams.numRangeBins);
            }
            staticCfg->numTxAntennas      = RFparserOutParams.numTxAntennas;
            staticCfg->numVirtualAntAzim  = RFparserOutParams.numVirtualAntAzim;
            staticCfg->numVirtualAntElev  = RFparserOutParams.numVirtualAntElev;
            staticCfg->numVirtualAntennas = RFparserOutParams.numVirtualAntennas;
            staticCfg->rangeStep          = RFparserOutParams.rangeStep;
            staticCfg->numBandsTotal      = staticCfg->numTxAntennas + MmwDemo_getNumEmptySubBands(staticCfg->numTxAntennas);

            if (staticCfg->numRangeFFTBins > 1024)
            {
                staticCfg->rangeFFTtuning.fftOutputDivShift             = 0;
                staticCfg->rangeFFTtuning.numLastButterflyStagesToScale = 3; /* scale only 3 stages */
            }
            else if (staticCfg->numRangeFFTBins >= 1022)
            {
                staticCfg->rangeFFTtuning.fftOutputDivShift             = 0;
                staticCfg->rangeFFTtuning.numLastButterflyStagesToScale = 2; /* scale only 2 stages */
            }
            else if (staticCfg->numRangeFFTBins == 512)
            {
                staticCfg->rangeFFTtuning.fftOutputDivShift             = 1;
                staticCfg->rangeFFTtuning.numLastButterflyStagesToScale = 1; /* scale last stage */
            }
            else
            {
                staticCfg->rangeFFTtuning.fftOutputDivShift             = 2;
                staticCfg->rangeFFTtuning.numLastButterflyStagesToScale = 0; /* no scaling needed as ADC data is 16-bit and we have 8 bits to grow */
            }

            for (idx = 0; idx < RFparserOutParams.numRxAntennas; idx++)
            {
                staticCfg->rxAntOrder[idx] = RFparserOutParams.rxAntOrder[idx];
            }
            for (idx = 0; idx < RFparserOutParams.numTxAntennas; idx++)
            {
                staticCfg->txAntOrder[idx] = RFparserOutParams.txAntOrder[idx];
            }

            DebugP_logInfo("App: Issuing Pre-start Config IOCTL (subFrameIndx = %d)\n", subFrameIndx);

            /* Copy out the DPC Static cfg params */

            staticCfg->cfarCfg.subFrameNum = subFrameIndx;

            subFrameCfg->datapathStaticCfg.cfarCfg.cfg.thresholdScale =
                MmwDemo_convertDopplerCfarToThresh(subFrameCfg->datapathStaticCfg.cfarCfg.cfg.thresholdScale); // staticCfg->numVirtualAntennas);
            memcpy(&staticCfg->cfarCfg.cfg, &subFrameCfg->datapathStaticCfg.cfarCfg.cfg, sizeof(DPU_DopplerProc_CfarCfg));
            memcpy(&staticCfg->compressionCfg, &subFrameCfg->datapathStaticCfg.compressionCfg, sizeof(DPU_RangeProcHWA_CompressionCfg));
            memcpy(&staticCfg->localMaxCfg, &subFrameCfg->datapathStaticCfg.localMaxCfg, sizeof(DPU_DopplerProc_LocalMaxCfg));
            memcpy(&staticCfg->intfStatsdBCfg, &subFrameCfg->datapathStaticCfg.intfStatsdBCfg, sizeof(DPU_RangeProcHWADDMA_intfStatsdBCfg));
            memcpy(&staticCfg->aoaFovCfg, &subFrameCfg->datapathStaticCfg.aoaFovCfg, sizeof(DPC_ObjectDetection_FovAoaCfg));

            staticCfg->rangeCfarCfg.subFrameNum = subFrameIndx;
            subFrameCfg->datapathStaticCfg.rangeCfarCfg.cfg.thresholdScale =
                MmwDemo_convertRangeCfarToThresh(subFrameCfg->datapathStaticCfg.rangeCfarCfg.cfg.thresholdScale, staticCfg->numBandsTotal);
            memcpy(&staticCfg->rangeCfarCfg.cfg, &subFrameCfg->datapathStaticCfg.rangeCfarCfg.cfg, sizeof(DPU_CFARProc_CfarCfg));
#if defined(SUBSYS_MSS) /* if running on MSS */
            /* send pre-start config */
            errCode = MmwDemo_DPM_ioctl_blocking(gMmw_MCB.objDetDpmHandle,
                                                 DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG,
                                                 &objDetPreStartCfg,
                                                 sizeof(DPC_ObjectDetection_PreStartCfg));
            if (errCode < 0)
            {
                DebugP_log("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG [Error:%d]\n", errCode);
                goto exit;
            }
#endif
#if (defined(SOC_AWR2X44P) && defined(SUBSYS_DSS)) // || (defined(SOC_AWR2944LC) && defined(SUBSYS_MSS))

            /* send pre-start config */
            errCode = MmwDemo_DPM_ioctl_blocking(gMmw_MCB.dataPathObj.objDetDpmHandle,
                                                 DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG,
                                                 &objDetPreStartCfg,
                                                 sizeof(DPC_ObjectDetection_PreStartCfg));
            if (errCode < 0)
            {
                DebugP_log("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG [Error:%d]\n", errCode);
                goto exit;
            }

            // /* send elevation estimation config */
            // errCode = MmwDemo_DPM_ioctl_blocking (gMmw_MCB.dataPathObj.objDetDpmHandle,
            //                     DPC_OBJDET_IOCTL__STATIC_PRE_START_ELEVEST_CFG,
            //                     &gElevEstCfg, sizeof (DPC_ObjectDetection_ElevEstCfg));

            // if (errCode < 0)
            // {
            //     DebugP_log ("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_ELEVEST_CFG [Error:%d]\n", errCode);
            //     goto exit;
            // }
#else

            (void)objDetCommonCfg;
#endif
        }
    }
exit:
    return errCode;
}


/**
 *  @b Description
 *  @n
 *      MMW demo helper Function to configure sensor. User need to fill gMmw_MCB.cfg.ctrlCfg and
 *      add profiles/chirp to mmWave before calling this function
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t MmwDemo_configSensor(void)
{
    int32_t errCode = 0;

    if (MmwDemo_configLink() < 0)
    {
        DebugP_assert(0);
    }
    errCode = MmwDemo_dataPathConfig();

    return errCode;
}

/**
 *  @b Description
 *  @n
 *      The function is used to Start data path to handle chirps from front end.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dataPathStart(void)
{
    DebugP_logInfo("App: Issuing DPM_start\n");
#ifdef LVDS_STREAM
    /* Configure HW LVDS stream for the first sub-frame that will start upon
     * start of frame */
    if (gMmw_MCB.subFrameCfg[0].lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
    {
        MmwDemo_configLVDSHwData(0);
    }
#endif
}

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to start sensor.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t MmwDemo_startSensor(void)
{
    // int32_t     errCode;
    // MMWave_CalibrationCfg   calibrationCfg;

    /*****************************************************************************
     * Data path :: start data path first - this will pend for DPC to ack
     *****************************************************************************/
    MmwDemo_dataPathStart();

    /*****************************************************************************
     * RF :: now start the RF and the real time ticking
     *****************************************************************************/
    /* Populate the calibration configuration: */
    gMmw_MCB.cfg.calibrationCfg.dfeDataOutputMode                          = gMmw_MCB.cfg.ctrlCfg.dfeDataOutputMode;
    gMmw_MCB.cfg.calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
    gMmw_MCB.cfg.calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = false;
    gMmw_MCB.cfg.calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 0;
    gMmw_MCB.cfg.calibrationCfg.u.chirpCalibrationCfg.reportEn             = 1;

    DebugP_logInfo("App: MMWave_start Issued\n");

    DebugP_log("Starting Sensor (issuing MMWave_start)\n");

    MmwDemo_startLink();

    // gMmw_MCB.sensorStartCount++;
    return 0;
}

#endif
#endif /* !MMWAVELINK_ON_MSS */

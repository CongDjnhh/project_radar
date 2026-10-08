/*
 *   @file  link_test_monitoring.c
 *
 *   @brief
 *      Implements mmwavelink tests for monitoring and calibration APIs
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2019 Texas Instruments, Inc.
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

#include <ti/common/syscommon.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/edma.h>
#include <drivers/hw_include/hw_types.h>

/* mmWave SDK Include Files: */
#include <ti/control/mmwavelink/mmwavelink.h>
#include <drivers/crc.h>
#include <ti/control/mmwavelink/include/rl_driver.h>
#include <ti/utils/testlogger/logger.h>
/* ADCBUF Driver: */
#if defined(SOC_AWR294X) || defined(SOC_AWR2X44P)
#include <drivers/adcbuf.h>
#endif
#include "mmw_link_mon.h"
#include "chirp_design_HECR.h"

/**************************************************************************
 *************************** Global Variables *****************************
 **************************************************************************/
uint16_t monAeCnt [MMWAVELINK_TEST_MAX_NUM_MON] = { 0 };
uint16_t monFailRepCheck [MMWAVELINK_TEST_MAX_NUM_MON] = { 0 };
uint32_t anaMonEn = 0;
monitoringDataAe_t gMonDataAeStrct = { 0 };

/**************************************************************************
 ************************** mmWave Link Functions *************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Initialize monitoring report counters.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwaveLink_monitorReportCountInit (void)
{
    int32_t i;

    /* Reset Monitoring report counts & error report counts */
    for(i = 0; i < MMWAVELINK_TEST_MAX_NUM_MON; i++)
    {
        monFailRepCheck[i] = 0;
        monAeCnt[i] = 0;
    }
}

/**
 *  @b Description
 *  @n
 *      The function Sets the consolidated configuration of all analog monitoring excluding
 *      CQ & CP monitoring.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfAnaMonConfig (uint8_t platformType, bool useInputMask, rlUInt32_t inputMask)
{
    int32_t  retVal;
    rlMonAnaEnables_t data;

    if(useInputMask)
    {
        data.enMask1 = inputMask;
    }
    else
    {
        /* Each bit(0-31) represents different analog monitor enable configuration */
        data.enMask1 = (1 << 0)   | \
            (1 << 1)   | \
            (0 << 2)   | \
            (1 << 3)   | \
            (1 << 4)   | \
            (1 << 5)   | \
            (1 << 6)   | \
            (1 << 7)   | \
            (1 << 8)   | \
            (1 << 9)   | \
            (1 << 10)  | \
            (1 << 11)  | \
            (1 << 12)  | \
            (1 << 13)  | \
            (1 << 14)  | \
            (0 << 15)  | \
            (1 << 16)  | \
            (1 << 17)  | \
            (1 << 18)  | \
            (1 << 19)  | \
            (1 << 20)  | \
            (1 << 21)  | \
            (1 << 22)  | \
            (1 << 23)  | \
            (0 << 24)  | \
            (0 << 25)  | \
            (0 << 26)  | \
            (0 << 27)  | \
            (0 << 28)  | \
            (1 << 29)  | \
            (1 << 30)  | \
            (1 << 31);
    }

    anaMonEn = data.enMask1;

    /* Analog monitors enable */
    retVal = rlRfAnaMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,&data);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to temperature monitoring.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfTempMonConfig (void)
{
    int32_t         retVal;

    rlTempMonConf_t tempMonCfg =
    {
        .reportMode = MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK,
        .reserved0 = 0,
        .anaTempThreshMin = -10,
        .anaTempThreshMax = 60,
        .digTempThreshMin = -10,
        .digTempThreshMax = 60,
        .tempDiffThresh = 20,
        .reserved1 = 0,
        .reserved2 = 0,
    };

    /* Set Temperature monitor configuration */
    retVal = rlRfTempMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlTempMonConf_t*)&tempMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets RX gain and phase monitoring config to device.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfRxGainPhaMonConfig (void)
{
    int32_t         retVal;

    rlRxGainPhaseMonConf_t rxGainPhaseMonCfg =
    {
        .profileIndx = 0,
        .rfFreqBitMask = HIGHEST_CENTER_LOWEST_RF_FRQ_IN_PROFILES_SWEEP_BW,
        .reportMode = MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK,
        .txSel = 1,
        .rxGainAbsThresh = 100,
        .rxGainMismatchErrThresh = 100,
        .rxGainFlatnessErrThresh = 100,
        .rxGainPhaseMismatchErrThresh = (40 * (1U << 16))/360U,
        .rxGainMismatchOffsetVal[0][0] = 0x1A,
        .rxGainMismatchOffsetVal[0][1] = 0x1A,
        .rxGainMismatchOffsetVal[0][2] = 0x1A,
        .rxGainMismatchOffsetVal[0][3] = 0x1A,
        .rxGainMismatchOffsetVal[1][0] = 0x1A,
        .rxGainMismatchOffsetVal[1][1] = 0x1A,
        .rxGainMismatchOffsetVal[1][2] = 0x1A,
        .rxGainMismatchOffsetVal[1][3] = 0x1A,
        .rxGainMismatchOffsetVal[2][0] = 0x1A,
        .rxGainMismatchOffsetVal[2][1] = 0x1A,
        .rxGainMismatchOffsetVal[2][2] = 0x1A,
        .rxGainMismatchOffsetVal[2][3] = 0x1A,
        .rxGainPhaseMismatchOffsetVal[0][0] = 0x1A,
        .rxGainPhaseMismatchOffsetVal[0][1] = 0x1A,
        .rxGainPhaseMismatchOffsetVal[0][2] = 0x1A,
        .rxGainPhaseMismatchOffsetVal[0][3] = 0x1A,
        .rxGainPhaseMismatchOffsetVal[1][0] = 0x1A,
        .rxGainPhaseMismatchOffsetVal[1][1] = 0x1A,
        .rxGainPhaseMismatchOffsetVal[1][2] = 0x1A,
        .rxGainPhaseMismatchOffsetVal[1][3] = 0x1A,
        .rxGainPhaseMismatchOffsetVal[2][0] = 0x1A,
        .rxGainPhaseMismatchOffsetVal[2][1] = 0x1A,
        .rxGainPhaseMismatchOffsetVal[2][2] = 0x1A,
        .rxGainPhaseMismatchOffsetVal[2][3] = 0x1A,
        .rf3FreqDitherLimits = 0,
        .reserved0 = 0
    };

    /* Set RX Gain Phase monitor configuration */
    retVal = rlRfRxGainPhMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlRxGainPhaseMonConf_t*)&rxGainPhaseMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to RX IF filter attenuation monitoring.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfRxIfStageMonConfig (void)
{
    int32_t         retVal;

    rlRxIfStageMonConf_t monRxIfStageCfg =
    {
        .profileIndx = 0,
        .reportMode = MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK,
        .reserved0 = 0,
        .reserved1 = 0,
        .hpfCutoffErrThresh = 60,
        .lpfCutoffBandEdgeDroopThresh = 128,
        .lpfCutoffStopBandAttenThresh = 128,
        .ifaGainErrThresh = 100,
        .reserved2 = 0,
    };

    /* Set RX IF Stage monitor configuration */
    retVal = rlRfRxIfStageMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlRxIfStageMonConf_t*)&monRxIfStageCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to TX power monitoring.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfTxPowMonConfig (void)
{
    int32_t         retVal;

    rlTxnPowMonConf_t allTxPowMonCfg =
    {
        .profileIndx = 0x0,
        .rfFreqBitMask = HIGHEST_CENTER_LOWEST_RF_FRQ_IN_PROFILES_SWEEP_BW,
        .txSelMask = 0xF,
        .reportMode = MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK,
        .txPowAbsErrThresh = 30,
        .txPowFlatnessErrThresh = 30,
        .txPowOffsetValRF1 = 0x0,
        .txPowOffsetValRF2 = 0x0,
        .txPowOffsetValRF3 = 0x0,

    };

    /* Set TX Power monitor configuration */
    retVal = rlRfTxnPowrMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlTxnPowMonConf_t*)&allTxPowMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to TX ball break detection.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfTxBallbreakMonConfig (void)
{
    int32_t         retVal;

    rlTxnBallbreakMonConf_t allTxBallbreakMonCfg =
    {
    .reportMode = MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK,
    .txSelMask = 0xF,
    .reserved1 = 0,
    .reserved2 = 0,
    .monStartFreqConst = PROFILE_START_FREQ_VAL,
    .txPowBackOff = 0,
    .txReflCoeffMagThresh = -60,
    };

    /* Set TX Ballbreak monitor configuration */
    retVal = rlRfTxnBallbreakMonConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlTxnBallbreakMonConf_t*)&allTxBallbreakMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to TX gain and phase mismatch monitoring.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfTxGainPhaseMismatchMonConfig (void)
{
    int32_t         retVal;

    rlTxGainPhaseMismatchMonConf_t txGainPhMisMatchMonCfg;

    /* Set TX Gain Phase monitor configuration */
    retVal = rlRfTxGainPhaseMismatchMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlTxGainPhaseMismatchMonConf_t*)&txGainPhMisMatchMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to TX gain and phase mismatch monitoring.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfAdvTxGainPhaseMismatchMonConfig (void)
{
    int32_t         retVal;

    rlAdvTxGainPhaseMismatchMonConf_t AdvtxGainPhMisMatchMonCfg =
    {
        .profileIndx = 0x0,
        .rfFreqBitMask = HIGHEST_CENTER_LOWEST_RF_FRQ_IN_PROFILES_SWEEP_BW,
        .txEn = 0x1,
        .rxEn = 0xF,
        .reportMode = MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK,
        .monChirpSlope  = 0x1,
        .txGainMismatchThresh = 30,
        .txPhaseMismatchThresh = (25 * (1 << 16))/360 ,
        .txGainMismatchOffsetVal =  {{0,11,-75,-75},{-12,0,-88,-88},{-29,-23,-100,-100}},
        .txPhaseMismatchOffsetVal = {{0,1820,1274,1274},{50050,51688,52780},{34398,35490,39494}},
        .reserved1 = 0x0,
        .reserved0 = 0x0,
    };

    /* Set TX Gain Phase monitor configuration */
    retVal = rlRfAdvTxGainPhaseMismatchMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlAdvTxGainPhaseMismatchMonConf_t*)&AdvtxGainPhMisMatchMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to synthesizer frequency.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfSynthFreqMonConfig (void)
{
    int32_t         retVal;

    rlSynthFreqMonConf_t synthFreqMonCfg =
    {
        .profileIndx = 0,
        .reportMode = MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK,
        .freqErrThresh = 4000,
        .monStartTime = 10,
        .monitorMode = 0,
        .vcoMonEn = 0,
        .profileBitMaskLiveMode = 0,
        .reserved2 = 0
    };
    
    /* Set Synth Freq monitor configuration */
    retVal = rlRfSynthFreqMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlSynthFreqMonConf_t*)&synthFreqMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to synthesizer frequency non live monitor.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfSynthFreqNonLiveMonConfig ()
{
    int32_t         retVal;

    rlSynthFreqMonConf_t synthFreqNonLiveMonCfg[2U] =
    {
        {
            .profileIndx = 0,
            .reportMode = MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK,
            .freqErrThresh = 4000,
            .monStartTime = 10,
            .monitorMode = 1,
            .vcoMonEn = 3,
            .profileBitMaskLiveMode = 0,
            .reserved2 = 0
        },
        {
            .profileIndx = 0,
            .reportMode = MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK,
            .freqErrThresh = 4000,
            .monStartTime = 10,
            .monitorMode = 2,
            .vcoMonEn = 3,
            .profileBitMaskLiveMode = 0,
            .reserved2 = 0
        }
    };

    /* Set Synth Freq monitor configuration */
    retVal = rlRfSynthFreqMonConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlSynthFreqMonConf_t *)&synthFreqNonLiveMonCfg[0U]);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }

    /* Set Synth Freq monitor configuration */
    retVal = rlRfSynthFreqMonConfig(RL_DEVICE_MAP_INTERNAL_BSS, ( rlSynthFreqMonConf_t *)&synthFreqNonLiveMonCfg[1U]);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to TX Internal Analog Signals monitoring.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfTxIntAnaSignalMonConfig (void)
{
    int32_t         retVal;

    rlTxnIntAnaSignalsMonConf_t allTxanaSigMonCfg =
    {
        .profileIndx = 0,
        .reportMode = 2,
        .reserved1 = 0,
        .reserved2 = 0,
        .txSelMask = 0xF,
        .txPhShiftDacMonThresh = 5,
    };

    /* Set TX Internal analog signal monitor configuration */
    retVal = rlRfTxnIntAnaSignalsMonConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlTxnIntAnaSignalsMonConf_t*)&allTxanaSigMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to RX Internal Analog Signals monitoring.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfRxIntAnaSignalMonConfig (void)
{
    int32_t         retVal;
    rlRxIntAnaSignalsMonConf_t rxIntAnaSgnlMonCfg = {0,2,0,0};

    /* Set RX Internal analog signal monitor configuration */
    retVal = rlRfRxIntAnaSignalsMonConfig(RL_DEVICE_MAP_INTERNAL_BSS, &rxIntAnaSgnlMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to Power Management, Clock generation and LO distribution.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfPmClkLoIntAnaSignalsMonConfig (void)
{
    int32_t         retVal;

    rlPmClkLoIntAnaSignalsMonConf_t pmClkLoIntAnaSigMonCfg =
    {
        .profileIndx = 0,
        .reportMode = MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK,
        .reserved0 = 0,
        .reserved1 = 0,
    };

    /* Set PMCLKLO monitor configuration */
    retVal = rlRfPmClkLoIntAnaSignalsMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlPmClkLoIntAnaSignalsMonConf_t*)&pmClkLoIntAnaSigMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to GPADC Internal Analog Signals monitoring.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfGpadcIntAnaSignalsMonConfig (void)
{
    int32_t         retVal;

    rlGpadcIntAnaSignalsMonConf_t gpadcIntAnaSigMonCfg =
    {
        .reportMode = MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK,
        .reserved0 = 0,
        .reserved1 = 0,
        .reserved2 = 0,
    };

    /* Set GPADC internal analog signal monitor configuration */
    retVal = rlRfGpadcIntAnaSignalsMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlGpadcIntAnaSignalsMonConf_t*) &gpadcIntAnaSigMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to APLL and Synthesizer's control voltage signals monitoring.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfPllContrlVoltMonConfig (void)
{
    int32_t         retVal;

    rlPllContrVoltMonConf_t pllConVoltMonCfg =
    {
        .reportMode = MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK,
        .reserved0 = 0,
        .signalEnables = 7,
        .reserved1 = 0,
    };

    /* Set PLL control voltage monitor configuration */
    retVal = rlRfPllContrlVoltMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlPllContrVoltMonConf_t*)&pllConVoltMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to the DCC based clock frequency monitoring.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfDualClkCompMonConfig (void)
{
    int32_t         retVal;

    rlDualClkCompMonConf_t dualClkCompMonCfg =
    {
        .reportMode = MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK,
        .reserved0 = 0,
        .dccPairEnables = 63,
        .reserved1 = 0,
    };

    /* Set DCC monitor configuration */
    retVal = rlRfDualClkCompMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlDualClkCompMonConf_t*)&dualClkCompMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function Sets information related to  RX saturation detector monitoring.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfRxIfSatMonConfig (void)
{
    int32_t         retVal;

    rlRxSatMonConf_t rxSatMonCfg =
    {
        .profileIndx   = 0U,
        .satMonSel     = 3U,
        .reserved0     = 0U,
        .primarySliceDuration = 5,
        .numSlices     = CQ_NUM_SLICES,
        .rxChannelMask = 0U,
        .reserved1     = 0U,
        .reserved2     = 0U,
        .reserved3     = 0U,
        .reserved4     = 0U
    };

    /* Set RX IF Stage monitor configuration */
    retVal = rlRfRxIfSatMonConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlRxSatMonConf_t*)&rxSatMonCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to Configure digital monitors
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t mmwaveLink_digMonEnableConfig(bool useInputMask, rlUInt32_t inputMask)
{
    int32_t  retVal;
    rlMonDigEnables_t data = { 0 };

    if(useInputMask)
    {
        data.enMask = inputMask;
    }
    else
    {
        /* Each bit(0-31) represents different digital monitor enable configuration */
        data.enMask = (0 << 0)   | \
                      (1 << 1)   | \
                      (0 << 2)   | \
                      (1 << 3)   | \
                      (0 << 4)   | \
                      (0 << 5)   | \
                      (1 << 6)   | \
                      (1 << 7)   | \
                      (1 << 8)   | \
                      (1 << 9)   | \
                      (1 << 10)  | \
                      (1 << 11)  | \
                      (0 << 12)  | \
                      (0 << 13)  | \
                      (0 << 14)  | \
                      (0 << 15)  | \
                      (1 << 16)  | \
                      (1 << 17)  | \
                      (1 << 18)  | \
                      (1 << 19)  | \
                      (1 << 20)  | \
                      (0 << 21)  | \
                      (0 << 22)  | \
                      (0 << 23)  | \
                      (1 << 24)  | \
                      (1 << 25)  | \
                      (1 << 26)  | \
                      (0 << 27)  | \
                      (0 << 28)  | \
                      (0 << 29)  | \
                      (0 << 30)  | \
                      (0 << 31);
    }

    data.testMode = 0;
    /* Digital monitoring configuration */
    retVal = rlRfDigMonEnableConfig(RL_DEVICE_MAP_INTERNAL_BSS,&data);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }

    return 0;
};

/**
 *  @b Description
 *  @n
 *      The function is used to Configure digital monitors
 *      for periodicity.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_rlRfDigMonPeriodicConfig (void)
{
    int32_t         retVal;
    rlDigMonPeriodicConf_t data = {0x0};
    data.reportMode = 0;
    data.periodicEnableMask = 0xD;

    /* Set Digital periodic monitor configuration */
    retVal = rlRfDigMonPeriodicConfig(RL_DEVICE_MAP_INTERNAL_BSS, &data);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set Tx phase shifter DAC monitor
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfTXPhShifterDacMonConfig(void)
{
    int32_t retVal;
    rlMonTxPhShifterDacConf_t psDacData = {0};
    psDacData.profileId = 0;
    psDacData.reportMode = 2;
    psDacData.txSelMask = 0xF;
    psDacData.txPsDacMonThresh[0] = 5;
    psDacData.txPsDacMonThresh[1] = 5;
    psDacData.txPsDacMonThresh[2] = 5;
    psDacData.txPsDacMonThresh[3] = 5;
    retVal = rlRfMontxPhShiftDacConfig(RL_DEVICE_MAP_INTERNAL_BSS, &psDacData);

    if(retVal != 0)
    {
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      RF Monitoring Enable/Configuration function. Enables all monitoring in
 *      Radar SS
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_monitoring(uint8_t platformType)
{

    /* Initialize monitoring counters */
    MmwaveLink_monitorReportCountInit();

    /* Set mmWave digital monitor enable to the BSS */
    if (mmwaveLink_digMonEnableConfig(false, 0) < 0)
    {
        return -1;
    }
    /* Set mmWave digital monitor periodic config to the BSS */
    if (MmwaveLink_rlRfDigMonPeriodicConfig() < 0)
    {
        return -1;
    }

    /* Set mmWave temperature monitor config to the BSS */
    if (MmwaveLink_setRfTempMonConfig() < 0)
    {
        return -1;
    }

    /* Set mmWave RX gain and phase monitoring config to the BSS */
    if (MmwaveLink_setRfRxGainPhaMonConfig() < 0)
    {
        return -1;
    }

    /* Set mmWave RX If stage monitoring config to the BSS */
    if (MmwaveLink_setRfRxIfStageMonConfig() < 0)
    {
        return -1;
    }

    /* Set mmWave TX Power monitoring config to the BSS */
    if (MmwaveLink_setRfTxPowMonConfig() < 0)
    {
        return -1;
    }

    /* Set mmWave TX ball break monitoring config to the BSS */
    if (MmwaveLink_setRfTxBallbreakMonConfig() < 0)
    {
        return -1;
    }

    /* Set mmWave TX Gain Phase mismatch monitoring config to the BSS */ //not working
    if (MmwaveLink_setRfAdvTxGainPhaseMismatchMonConfig() < 0)
    {
        return -1;
    }
    
    /* Set mmWave Synth Frequency monitoring config to the BSS */
    if (MmwaveLink_setRfSynthFreqMonConfig() < 0)
    {
        return -1;
    }    

    /* Set mmWave TX internal analog signal config to the BSS */
    if (MmwaveLink_setRfTxIntAnaSignalMonConfig() < 0)
    {
        return -1;
    }

    /* Set mmWave RXinternal analog signal config to the BSS */
    if (MmwaveLink_setRfRxIntAnaSignalMonConfig() < 0)
    {
        return -1;
    }

    /* Set mmWave PMCLKLO internal analog signal monitoring config to the BSS */
    if (MmwaveLink_setRfPmClkLoIntAnaSignalsMonConfig() < 0)
    {
        return -1;
    }

    /* Set mmWave GPADC internal analog signal monitoring config to the BSS */
    if (MmwaveLink_setRfGpadcIntAnaSignalsMonConfig() < 0)
    {
        return -1;
    }

    /* Set mmWave Pll control volt monitoring config to the BSS */
    if (MmwaveLink_setRfPllContrlVoltMonConfig() < 0)
    {
        return -1;
    }

    /* Set mmWave Dual Clock monitoring config to the BSS */
    if (MmwaveLink_setRfDualClkCompMonConfig() < 0)
    {
        return -1;
    }

    /* Set mmWave Analog monitor config to the BSS */
    if (MmwaveLink_setRfAnaMonConfig(platformType, false, 0) < 0)
    {
        return -1;
    }

    return 0;

}

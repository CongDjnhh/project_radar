/**
 *   @file  chirp_design_HECR.h
 *
 *   @brief
 *      Defines the chirp configuration used in the application
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

#ifndef CHIRP_DESIGN_HECR_H_
#define CHIRP_DESIGN_HECR_H_

/** Basic configuration option definitions **/
/* ADC Configuration */
#define LP_ADC_MODE_REGULAR   (0U)
#define LP_ADC_MODE_LOW_POWER (1U)

#define ADC_BITS_12 (0U)
#define ADC_BITS_14 (1U)
#define ADC_BITS_16 (2U)

#define ADC_FORMAT_REAL                 (0U)
#define ADC_FORMAT_COMPLEX              (1U)
#define ADC_FORMAT_CPMLEX_WITH_IMG_BAND (2U)

#define ADCBUF_FORMAT_COMPLEX (0U)
#define ADCBUF_FORMAT_REAL    (1U)

#define ADC_I_FIRST (0U)
#define ADC_Q_FIRST (1U)

#define ADC_INTERLEAVED_MODE     (0U)
#define ADC_NON_INTERLEAVED_MODE (1U)

/* Tx Channel Configuration */
#define TX_CHANNEL_1_ENABLE       (1U << 0U)
#define TX_CHANNEL_2_ENABLE       (1U << 1U)
#define TX_CHANNEL_3_ENABLE       (1U << 2U)
#define TX_CHANNEL_4_ENABLE       (1U << 3U)
#define TX_CHANNEL_1_2_ENABLE     (TX_CHANNEL_1_ENABLE | TX_CHANNEL_2_ENABLE)
#define TX_CHANNEL_2_3_ENABLE     (TX_CHANNEL_2_ENABLE | TX_CHANNEL_3_ENABLE)
#define TX_CHANNEL_1_3_ENABLE     (TX_CHANNEL_1_ENABLE | TX_CHANNEL_3_ENABLE)
#define TX_CHANNEL_1_2_3_ENABLE   (TX_CHANNEL_1_ENABLE | TX_CHANNEL_2_ENABLE | TX_CHANNEL_3_ENABLE)
#define TX_CHANNEL_1_2_3_4_ENABLE (TX_CHANNEL_1_ENABLE | TX_CHANNEL_2_ENABLE | TX_CHANNEL_3_ENABLE | TX_CHANNEL_4_ENABLE)

/* Rx Channel Configuration */
#define RX_CHANNEL_1_ENABLE       (1U << 0U)
#define RX_CHANNEL_2_ENABLE       (1U << 1U)
#define RX_CHANNEL_3_ENABLE       (1U << 2U)
#define RX_CHANNEL_4_ENABLE       (1U << 3U)
#define RX_CHANNEL_1_2_ENABLE     (RX_CHANNEL_1_ENABLE | RX_CHANNEL_2_ENABLE)
#define RX_CHANNEL_1_3_ENABLE     (RX_CHANNEL_1_ENABLE | RX_CHANNEL_3_ENABLE)
#define RX_CHANNEL_1_4_ENABLE     (RX_CHANNEL_1_ENABLE | RX_CHANNEL_4_ENABLE)
#define RX_CHANNEL_2_3_ENABLE     (RX_CHANNEL_2_ENABLE | RX_CHANNEL_3_ENABLE)
#define RX_CHANNEL_2_4_ENABLE     (RX_CHANNEL_2_ENABLE | RX_CHANNEL_4_ENABLE)
#define RX_CHANNEL_3_4_ENABLE     (RX_CHANNEL_3_ENABLE | RX_CHANNEL_4_ENABLE)
#define RX_CHANNEL_1_2_3_ENABLE   (RX_CHANNEL_1_ENABLE | RX_CHANNEL_2_ENABLE | RX_CHANNEL_3_ENABLE)
#define RX_CHANNEL_2_3_4_ENABLE   (RX_CHANNEL_2_ENABLE | RX_CHANNEL_3_ENABLE | RX_CHANNEL_4_ENABLE)
#define RX_CHANNEL_1_3_4_ENABLE   (RX_CHANNEL_1_ENABLE | RX_CHANNEL_3_ENABLE | RX_CHANNEL_4_ENABLE)
#define RX_CHANNEL_1_2_3_4_ENABLE (RX_CHANNEL_1_ENABLE | RX_CHANNEL_2_ENABLE | RX_CHANNEL_3_ENABLE | RX_CHANNEL_4_ENABLE)

/*** channel configuration */
#define HECR_RX_CHAN_EN RX_CHANNEL_1_2_3_4_ENABLE
#define HECR_TX_CHAN_EN TX_CHANNEL_1_2_3_4_ENABLE

/*** ADC Configuration */
#define HECR_ADC_NUM_BIT    ADC_BITS_16 /* 16 bit*/
#define HECR_ADC_OUT_FORMAT ADC_FORMAT_REAL /* Complex 1x */

/*** ADCBUF config*/
#define HECR_ADCBUF_FORMAT   ADCBUF_FORMAT_REAL
#define HECR_IQ_SWAP_SEL     ADC_I_FIRST
#define HECR_CHAN_INTERLEAVE ADC_INTERLEAVED_MODE
#define HECR_CHIRP_THRESHOLD (1U)

/*** Low-power and LDO Bypass Configuration
 * CAUTION: change these parameter carefully
 */
#define HECR_LDO_BYPASS_EN       (3U)
#define HECR_SUPPY_IR_DROP       (1U)
#define HECR_IO_SUPPLY_INDICATOR (0U)
#define HECR_LOW_PWR_ADC         LP_ADC_MODE_REGULAR

/*** CRC Type, Async-event control */
#define HECR_ASYNC_EVT_DIR  0x5U
#define HECR_ASYNC_EVT_CTRL 0x0U
#define HECR_BSS_WDT_EN     0x0U

#define HECR_MSS_LOGGER_EN 0x0U
#define HECR_BSS_LOGGER_EN 0x0U

/**** You can choose to enable or disable advance chirp based on the macro below ****/
// #define HECR_ENABLE_ADV_CHIRP
// #define HECR_ENABLE_ADV_FRAME
#define HECR_ENABLE_LEGACY_FRAME

#ifdef HECR_ENABLE_ADV_CHIRP
/*** Misc. Configuration:
 *      Phase Shift
 *      Advanced chirp config */
#define HECR_PER_CHIRP_PHASE_SHIFT_EN 1U
#define HECR_ADV_CHIRP_CFG_EN         0U
#define HECR_ADV_CHIRP_ERR_CHK_DIS    0U
#define HECR_CAL_MON_TIME_ERR_CHK_DIS 0U

#define HECR_ADV_CHIRP_CHIRPS_PER_CYCLE ((HECR_FRAME_CHIRP_END_IDX - HECR_FRAME_CHIRP_START_IDX + 1) * HECR_FRAME_CHIRP_NUM_LOOP)

/* Chirp IDle Time Dither */
#define HECR_ADV_CHIRP_IDLE_TIME_DITH (1) /* 1 usec */

/* LUT Buffer size for Advanced chirp
   Max size = 12KB (12*1024) */
#define HECR_ADV_CHIRP_DUMMY_LUT_ADDR_OFFSET   3 * 1024
#define HECR_ADV_CHIRP_LUT_ADVCHIRP_TABLE_SIZE 4 * 1024
#endif

/** Frequency Limit Configuration */
#define HECR_FREQ_LOW_LIMIT  760U
#define HECR_FREQ_HIGH_LIMIT 810U

/* DATAPATH: CSI2 configuration */
#define HECR_LANE_CLK_CFG 1U /* DDR Clock */
#define HECR_DATA_RATE    1U /* 600Mbps */

#define HECR_DATAPATH_INT_SEL DATA_PATH_CSI2
#define HECR_DATAPATH_FMTPKT0 DATA_PATH_FMT0_ADC_DATA_ONLY
#define HECR_DATAPATH_FMTPKT1 DATA_PATH_FMT1_SUPRESS

#define HECR_HSI_CLK          9U /* Only 600 Mbps supported */
#define HECR_LANE_EN          LVDS_ALL_LANE_EN
#define HECR_LANE_POS_POL_SEL 0x35421U
/*  (((rlUInt8_t)1U << 0U | (rlUInt8_t)2U<< 4U | \
     (rlUInt8_t)4U<< 8U | (rlUInt8_t)5U << 12U | \
     (rlUInt8_t)3U << 16U) |((rlUInt8_t)0U << 3U | \
     (rlUInt8_t)0U<< 7U | (rlUInt8_t)0U<< 11U | \
     (rlUInt8_t)0U << 15U | (rlUInt8_t)0U << 19U)) */
#define HECR_LINE_START_END_DIS 0U

/*** Factory Calibration Configuration */
#define FACTORY_CALIB_SAVE_ENABLE    0U
#define FACTORY_CALIB_RESTORE_ENABLE 0U
#define FACTORY_CALIB_FLASH_OFFSET   0x1F0000U

/* Antenna Calibration Parameters*/
#define ANTENNA_CALIB_ENABLE 0U

/* Monitoring Settings */
#define ENABLE_MONITORS                                   (1U)
#define MON_REPORT_MODE_PERIODIC_WITHOUT_THRESHOLD_CHECK  (0U)
#define MON_REPORT_MODE_AT_FAILURE_ONLY                   (1U)
#define MON_REPORT_MODE_PERIODIC_WITH_THRESHOLD_CHECK     (2U)
#define LOWEST_RF_FRQ_IN_PROFILES_SWEEP_BW                (0U)
#define CENTER_RF_FRQ_IN_PROFILES_SWEEP_BW                (1U)
#define HIGHEST_RF_FRQ_IN_PROFILES_SWEEP_BW               (2U)
#define HIGHEST_CENTER_LOWEST_RF_FRQ_IN_PROFILES_SWEEP_BW (7U)
#define PROFILE_START_FREQ_VAL                            (1435384036U)
#define CQ_NUM_SLICES                                     63U

/*** Run Time Calibration Configuration [rlRunTimeCalibConf_t] */
#define HECR_RUN_TIME_CALIBRATION_EN 1U
/*   *            0:[Reserved]  \n
 *            1:[Reserved]  \n
 *            2:[Reserved]  \n
 *            3:[Reserved]  \n
 *            4: Enable LODIST calibration  \n
 *            5: [Reserved]  \n
 *            6: [Reserved]  \n
 *            7: [Reserved]  \n
 *            8: Enable PD_CALIBRATION_EN \n
 *            9: Enable TX Power calibration \n
 *            10:Enable RX gain calibration  \n
 *            11:[Reserved]  \n
 *            12:[Reserved]  \n
 *            31:13:[Reserved] \n
 */
#define HECR_ONE_TIME_CALIBRATION_MASK 0x711U
#define HECR_PERIODIC_CALIBRATION_MASK HECR_ONE_TIME_CALIBRATION_MASK /* change to different mask if needed */
/* Enable:1, disable:0 RunTime Periodic Calibration */
#define HECR_RUN_TIME_PERIODIC_CALIB 1U
/* 0: disable runTime, >0 is no. of frame unit */
#if RUN_TIME_PERIODIC_CALIB
#define HECR_RUN_TIME_CALIB_PEIRODICITY 2U
#else
#define HECR_RUN_TIME_CALIB_PEIRODICITY 0U
#endif
#define HECR_RUN_TIME_CALIB_REPORT_EN 1U

#define HECR_CALIB_MON_TIME_UNIT 5U

/* 0: Autonomous monitoring, 1: API based Monitoring trigger */
#define HECR_MONITORING_MODE 0

/*************************************************************************************************************************
 *                                        Common Configurations                                                          *
 *************************************************************************************************************************/
#define HECR_NUM_PROFILES   (1U)
#define HECR_NUM_DDM_CHIRPS (6U)

/*************************************************************************************************************************
 *                                        Profile Configurations                                                         *
 *************************************************************************************************************************/

/* PROFILE - 0 */

#ifndef HECR_ENABLE_ADV_CHIRP
#define HECR_PROFILE_0_PROFILE_ID            (0U)
#define HECR_PROFILE_0_VCO_SELECT            (0x0U)
#define HECR_PROFILE_0_START_FREQ_GHZ        (77U)
#define HECR_PROFILE_0_IDLE_TIME_VAL         (3.5f)
#define HECR_PROFILE_0_ADC_START_TIME_VAL    (3.5f)
#define HECR_PROFILE_0_RAMP_END_TIME_VAL     (17.33f)
#define HECR_PROFILE_0_TXOUT_POWER_BACKOFF   (0U)
#define HECR_PROFILE_0_TXPHASESHIFTER_VAL    (0U)
#define HECR_PROFILE_0_FREQ_SLOPE_MHZ_PER_US (8.883f)
#define HECR_PROFILE_0_TX_START_TIME_VAL     (0U)
/* Number of samples MUST be same across all the profile configurations */
#define HECR_PROFILE_0_ADC_SAMPLE_VAL (384U)
/*! @brief  ADC Output rate */
#define HECR_PROFILE_0_DIGOUT_SAMPLERATE_VAL (30000U)
/*! @brief  HPF 1 corner frequency. */
#define HECR_PROFILE_0_HPFCORNER_FREQ1_VAL RL_RX_HPF1_175_KHz
/*! @brief  HPF 2 corner frequency. */
#define HECR_PROFILE_0_HPFCORNER_FREQ2_VAL RL_RX_HPF2_350_KHz
/*! @brief  Rx gain is kept at the maximum . */
#define HECR_PROFILE_0_RX_GAIN_VAL   (42U)
#define HECR_PROFILE_0_PF_CAL_UPDATE (0U)

#else
#define HECR_PROFILE_0_PROFILE_ID            (0U)
#define HECR_PROFILE_0_VCO_SELECT            (0U)
#define HECR_PROFILE_0_START_FREQ_GHZ        (77U)
#define HECR_PROFILE_0_IDLE_TIME_VAL         (10.0f)
#define HECR_PROFILE_0_ADC_START_TIME_VAL    (3.5f)
#define HECR_PROFILE_0_RAMP_END_TIME_VAL     (17.33)
#define HECR_PROFILE_0_TXOUT_POWER_BACKOFF   (0U)
#define HECR_PROFILE_0_TXPHASESHIFTER_VAL    (0U)
#define HECR_PROFILE_0_FREQ_SLOPE_MHZ_PER_US (8.883f)
#define HECR_PROFILE_0_TX_START_TIME_VAL     (0U)
/* Number of samples MUST be same across all the profile configurations */
#define HECR_PROFILE_0_ADC_SAMPLE_VAL        (384U)
/*! @brief  ADC Output rate */
#define HECR_PROFILE_0_DIGOUT_SAMPLERATE_VAL (30000U)
/*! @brief  HPF 1 corner frequency. */
#define HECR_PROFILE_0_HPFCORNER_FREQ1_VAL   RL_RX_HPF1_175_KHz
/*! @brief  HPF 2 corner frequency. */
#define HECR_PROFILE_0_HPFCORNER_FREQ2_VAL   RL_RX_HPF2_350_KHz
/*! @brief  Rx gain is kept at the maximum . */
#define HECR_PROFILE_0_RX_GAIN_VAL           (170U) // 36 dB
#define HECR_PROFILE_0_PF_CAL_UPDATE         (0x0U)

#endif

/* PROFILE - 1 */
#define HECR_PROFILE_1_PROFILE_ID            (1U)
#define HECR_PROFILE_1_VCO_SELECT            (0x0U)
#define HECR_PROFILE_1_START_FREQ_GHZ        (77U)
#define HECR_PROFILE_1_IDLE_TIME_VAL         (3.5f) /* 4 usec  */
#define HECR_PROFILE_1_ADC_START_TIME_VAL    (3.5f) /* 7 usec  */
#define HECR_PROFILE_1_RAMP_END_TIME_VAL     (17.33) /* 23 usec */
#define HECR_PROFILE_1_TXOUT_POWER_BACKOFF   (0U)
#define HECR_PROFILE_1_TXPHASESHIFTER_VAL    (0U)
#define HECR_PROFILE_1_FREQ_SLOPE_MHZ_PER_US (8.883f)
#define HECR_PROFILE_1_TX_START_TIME_VAL     (0U)
/* Number of samples MUST be same across all the profile configurations */
#define HECR_PROFILE_1_ADC_SAMPLE_VAL (192U)
/*! @brief  ADC Output rate */
#define HECR_PROFILE_1_DIGOUT_SAMPLERATE_VAL (30000U)
/*! @brief  HPF 1 corner frequency. */
#define HECR_PROFILE_1_HPFCORNER_FREQ1_VAL RL_RX_HPF1_175_KHz
/*! @brief  HPF 2 corner frequency. */
#define HECR_PROFILE_1_HPFCORNER_FREQ2_VAL RL_RX_HPF2_350_KHz
/*! @brief  Rx gain is kept at the maximum . */
#define HECR_PROFILE_1_RX_GAIN_VAL   (42U)
#define HECR_PROFILE_1_PF_CAL_UPDATE (0x0U)

/* PROFILE - 2 */
#define HECR_PROFILE_2_PROFILE_ID            (2U)
#define HECR_PROFILE_2_VCO_SELECT            (0x0U)
#define HECR_PROFILE_2_START_FREQ_GHZ        (77U)
#define HECR_PROFILE_2_IDLE_TIME_VAL         (3.5f) /* 4 usec  */
#define HECR_PROFILE_2_ADC_START_TIME_VAL    (3.5f) /* 7 usec  */
#define HECR_PROFILE_2_RAMP_END_TIME_VAL     (17.33) /* 23 usec */
#define HECR_PROFILE_2_TXOUT_POWER_BACKOFF   (0U)
#define HECR_PROFILE_2_TXPHASESHIFTER_VAL    (0U)
#define HECR_PROFILE_2_FREQ_SLOPE_MHZ_PER_US (8.883f)
#define HECR_PROFILE_2_TX_START_TIME_VAL     (0U)
/* Number of samples MUST be same across all the profile configurations */
#define HECR_PROFILE_2_ADC_SAMPLE_VAL (192U)
/*! @brief  ADC Output rate */
#define HECR_PROFILE_2_DIGOUT_SAMPLERATE_VAL (30000U)
/*! @brief  HPF 1 corner frequency. */
#define HECR_PROFILE_2_HPFCORNER_FREQ1_VAL RL_RX_HPF1_175_KHz
/*! @brief  HPF 2 corner frequency. */
#define HECR_PROFILE_2_HPFCORNER_FREQ2_VAL RL_RX_HPF2_350_KHz
/*! @brief  Rx gain is kept at the maximum . */
#define HECR_PROFILE_2_RX_GAIN_VAL   (42U)
#define HECR_PROFILE_2_PF_CAL_UPDATE (0x0U)

/* PROFILE - 3 */
#define HECR_PROFILE_3_PROFILE_ID            (3U)
#define HECR_PROFILE_3_VCO_SELECT            (0x0U)
#define HECR_PROFILE_3_START_FREQ_GHZ        (77U)
#define HECR_PROFILE_3_IDLE_TIME_VAL         (3.5f) /* 4 usec  */
#define HECR_PROFILE_3_ADC_START_TIME_VAL    (3.5f) /* 7 usec  */
#define HECR_PROFILE_3_RAMP_END_TIME_VAL     (17.33) /* 23 usec */
#define HECR_PROFILE_3_TXOUT_POWER_BACKOFF   (0U)
#define HECR_PROFILE_3_TXPHASESHIFTER_VAL    (0U)
#define HECR_PROFILE_3_FREQ_SLOPE_MHZ_PER_US (8.883f)
#define HECR_PROFILE_3_TX_START_TIME_VAL     (0U)
/* Number of samples MUST be same across all the profile configurations */
#define HECR_PROFILE_3_ADC_SAMPLE_VAL (192U)
/*! @brief  ADC Output rate */
#define HECR_PROFILE_3_DIGOUT_SAMPLERATE_VAL (30000U)
/*! @brief  HPF 1 corner frequency. */
#define HECR_PROFILE_3_HPFCORNER_FREQ1_VAL RL_RX_HPF1_175_KHz
/*! @brief  HPF 2 corner frequency. */
#define HECR_PROFILE_3_HPFCORNER_FREQ2_VAL RL_RX_HPF2_350_KHz
/*! @brief  Rx gain is kept at the maximum . */
#define HECR_PROFILE_3_RX_GAIN_VAL   (42U)
#define HECR_PROFILE_3_PF_CAL_UPDATE (0x0U)


#define HECR_PROFILE_LAMBDA_MILLIMETER (MMWDEMO_SPEED_OF_LIGHT_IN_METERS_PER_USEC / HECR_PROFILE_START_FREQ_GHZ)

/*************************************************************************************************************************
 *                                          Chirp Configurations                                                         *
 *************************************************************************************************************************/

/*! @brief Define 128 chirps,  the first 64 will have an idle time of
 * 3us, and the remaining 64 will have an idle time of 14.8us
 * (11.8us extra 'idle time') */
#define HECR_CHIRP_0_PROFILE_ID         (HECR_PROFILE_0_PROFILE_ID)
#define HECR_CHIRP_0_START_INDEX        (0U)
#define HECR_CHIRP_0_END_INDEX          (5U)
#define HECR_CHIRP_0_START_FREQ_VAL     (0U)
#define HECR_CHIRP_0_FREQ_SLOPE_VAL     (0U)
#define HECR_CHIRP_0_IDLE_TIME_VAL      (0U)
#define HECR_CHIRP_0_ADC_START_TIME_VAL (0U)
#define HECR_CHIRP_0_TX_CHANNEL         (TX_CHANNEL_1_2_3_4_ENABLE)

/*************************************************************************************************************************
 *                                          Frame Configurations                                                         *
 *************************************************************************************************************************/

/* Frame configuration */
#define HECR_FRAME_CHIRP_START_IDX      HECR_CHIRP_0_START_INDEX
#define HECR_FRAME_CHIRP_END_IDX        HECR_CHIRP_0_END_INDEX
#define HECR_FRAME_CHIRP_NUM_LOOP       (128U)
#define HECR_FRAME_NUM_FRAMES           (0U)
#define HECR_FRAME_NUM_SAMPLES          HECR_PROFILE_0_ADC_SAMPLE_VAL
#define HECR_FRAME_FRAME_PERIODICITY    (50U)
#define HECR_FRAME_FRAME_TRIGGER_SELECT (1U)
#define HECR_FRAME_FRAME_TRIGGER_DELAY  (0U)

/*********** Other configurations for processing chain **********/
/* GUI Monitor parameters */
#define HECR_GUI_DETECT_OBJ         3
#define HECR_GUI_LOG_MAG_RANGE      0
#define HECR_GUI_NOISE_PROFILE      0
#define HECR_GUI_RANGE_AZI_HEAT_MAP 0
#define HECR_GUI_RANGE_DOP_HEAR_MAP 0
#define HECR_GUI_STATS_INFO         0
#define HECR_GUI_RANSAC_FILTER_MSK  1
#define HECR_GUI_TRACKERS           1

#define MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR (100.0)

/*! @brief For the ADCBufData.dataProperty.adcBits field
 */
#define ADCBUF_DATA_PROPERTY_ADCBITS_16BIT (2)

/* CFAR configuration parameters */
#define HECR_CFAR_DOPPLER_PROC_DIR        1
#define HECR_CFAR_DOPPLER_AVG_MODE        3
#define HECR_CFAR_DOPPLER_WIN_LEN         16
#define HECR_CFAR_DOPPLER_GUARD_LEN       0
#define HECR_CFAR_DOPPLER_NOISE_DIV_SHIFT 0
#define HECR_CFAR_DOPPLER_CYCLIC_MODE     1

/* threshold is a float value from 0-100dB. It needs to be later converted to linear scale (conversion can only be done
     when the number of virtual antennas is known) before passing it to CFAR DPU.
     For now, the threshold will be coded in a 16bit integer in the following way:
     suppose threshold is a float represented as XYZ.ABC it will be saved as a 16bit integer XYZAB that is, 2 decimal cases are saved.*/
#define HECR_CFAR_DOPPLER_THRESHOLD       (12.0 * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR)
#define HECR_CFAR_DOPPLER_PEAK_GRP_EN     0
#define HECR_CFAR_DOPPLER_OS_K_VAL        24
#define HECR_CFAR_DOPPLER_OS_EDGE_KSCL_EN 0
#define HECR_CFAR_DOPPLER_IS_ENABLED      1

#define HECR_CFAR_RANGE_PROC_DIR        0
#define HECR_CFAR_RANGE_AVG_MODE        3
#define HECR_CFAR_RANGE_WIN_LEN         16
#define HECR_CFAR_RANGE_GUARD_LEN       0
#define HECR_CFAR_RANGE_NOISE_DIV_SHIFT 0
#define HECR_CFAR_RANGE_CYCLIC_MODE     1
/* threshold is a float value from 0-100dB. It needs to be later converted to linear scale (conversion can only be done
     when the number of virtual antennas is known) before passing it to CFAR DPU.
     For now, the threshold will be coded in a 16bit integer in the following way:
     suppose threshold is a float represented as XYZ.ABC it will be saved as a 16bit integer XYZAB that is, 2 decimal cases are saved.*/
#define HECR_CFAR_RANGE_THRESHOLD       (15.0 * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR)
#define HECR_CFAR_RANGE_PEAK_GRP_EN     0
#define HECR_CFAR_RANGE_OS_K_VAL        7
#define HECR_CFAR_RANGE_OS_EDGE_KSCL_EN 0
#define HECR_CFAR_RANGE_IS_ENABLED      1

/* Compression Config parameters */
#define HECR_CMPR_CFG_ENABLED      1
#define HECR_CMPR_CFG_METHOD       0
#define HECR_CMPR_CFG_RATIO        0.5
#define HECR_CMPR_CFG_BINS_PER_BLK 8

/* Interference mitigation parameters */
#define HECR_INTF_MITG_MAG_SNR_DB      15
#define HECR_INTF_MITG_MAG_DIFF_SNR_DB 18

/* local maxima config parameters */
#define HECR_LOCAL_MAX_AZIM_THRES    6
#define HECR_LOCAL_MAX_DOPPLER_THRES 40

/* Antenna calibration parameters, length of [NUM_RX (cascade:8) * NUM_TX (cascade:6) * 2]   */
// it is defined in the mss_main.c file, edit there

/* Measure RX Channel Bias and Rx Channel Phase config parameters */
#define HECR_MESUR_RANG_BIAS_EN            0
#define HECR_MESUR_RANG_BIAS_SRCH_WIN_SIZE 3.0
#define HECR_MESUR_RANG_BIAS_TARGET_DIS    0.5

/* RANSAC Parameters */
#define HECR_RANSAC_ENABLE      1
#define HECR_RANSAC_INTERATIONS 200
#define HECR_RANSAC_THRESHOLD   1.0

/* Scenery params for gTrack */
#define HECR_SCENE_SENSOR_POS_X          0.0
#define HECR_SCENE_SENSOR_POS_Y          0.0
#define HECR_SCENE_SENSOR_POS_Z          1.0
#define HECR_SCENE_SENSOR_ORIN_AZIM_TILT 0.0
#define HECR_SCENE_SENSOR_ORIN_ELEV_TILT 0.0
#define HECR_SCENE_NUM_BOUND_BOX         1
#define HECR_SCENE_BOUND_BOX_X1          -14.0
#define HECR_SCENE_BOUND_BOX_X2          14.0
#define HECR_SCENE_BOUND_BOX_Y1          1.0
#define HECR_SCENE_BOUND_BOX_Y2          231.0
#define HECR_SCENE_BOUND_BOX_Z1          -5.0
#define HECR_SCENE_BOUND_BOX_Z2          5.0
#define HECR_SCENE_NUM_STATIC_BOX        0

/* app gating param for gTrack */
#define HECR_APP_GATING_GAIN      8.0
#define HECR_APP_GATING_LIM_ARR_0 10.0
#define HECR_APP_GATING_LIM_ARR_1 8.0
#define HECR_APP_GATING_LIM_ARR_2 4.0
#define HECR_APP_GATING_LIM_ARR_3 40.0

/* App State params for gTrack */
#define HECR_APP_STATE_DET2_ACT_THRE     3
#define HECR_APP_STATE_DET2_FREE_THRE    2
#define HECR_APP_STATE_ACT2_FREE_THRE    10
#define HECR_APP_STATE_STATIC2_FREE_THRE 100
#define HECR_APP_STATE_EXIT2_FREE_THRE   10
#define HECR_APP_STATE_SLEEP2_FREE_THRE  0

/* App alloc params for gTrack */
#define HECR_APP_ALLOC_SNR_THRE      20.0
#define HECR_APP_ALLOC_SNR_THRE_OBSC 20.0
#define HECR_APP_ALLOC_VEL_THRE      1.0
#define HECR_APP_ALLOC_PTS_THRE      6
#define HECR_APP_ALLOC_MX_DIS_THRE   4.0
#define HECR_APP_ALLOC_MX_VEL_THRE   20.0

/* gTrack config params */
#define HECR_GTRACK_ENABLE       1
#define HECR_GTRACK_MAX_NUM_PT   800
#define HECR_GTRACK_MAX_NUM_TRCK 30
#define HECR_GTRACK_INIT_RAD_VEL 0.0
#define HECR_GTRACK_MAX_RAD_VEL  40.8781
#define HECR_GTRACK_RAD_VEL_RESO 0.1065
#define HECR_GTRACK_MAX_ACC_0    1.0
#define HECR_GTRACK_MAX_ACC_1    10.0
#define HECR_GTRACK_MAX_ACC_2    0.0
#define HECR_GTRACK_DELTA_T      0.050 /* This value should be same as the frame periodicity */

/* AOA FOV configuration */
#define HECR_FOVCFG_MIN_AZI_DEG -90
#define HECR_FOVCFG_MAX_AZI_DEG 90
#define HECR_FOVCFG_MIN_ELV_DEG -90
#define HECR_FOVCFG_MAX_ELV_DEG 90

/* Calibration store/restore option */ // TODO : Is this mandatory?
#define HECR_CALIB_SAVE_EN      0
#define HECR_CALIB_RESTORE_EN   0
#define HECR_CALIB_FLASH_OFFSET 0

/* LVDS stream configuration */
#define HECR_LVDS_IS_HEADER_ENABLED 0
#define HECR_LVDS_DATA_FMT          MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_ADC
#define HECR_LVDS_IS_SW_ENABLED     0

#ifdef HECR_ENABLE_ADV_CHIRP
#define HECR_DFE_DATA_OUTPUT_MODE MMWave_DFEDataOutputMode_ADVANCED_CHIRP_LEGACY_FRAME
#elif defined(HECR_ENABLE_ADV_FRAME)
#define HECR_DFE_DATA_OUTPUT_MODE MMWave_DFEDataOutputMode_ADVANCED_FRAME
#else
#define HECR_DFE_DATA_OUTPUT_MODE MMWave_DFEDataOutputMode_FRAME
#endif

#define HECR_ANT_GEOMETRY_X_SPACING_BY_LAMBDA (0.5f)
#define HECR_ANT_GEOMETRY_Z_SPACING_BY_LAMBDA (0.5f)

#define HECR_FAST_RANGEPROC_IS_ENABLED                        0
#define HECR_FAST_RANGEPROC_INTF_MITIG_MAG_THRES_MIN_LIM      300
#define HECR_FAST_RANGEPROC_INTF_MITIG_MAG_DIFF_THRES_MIN_LIM 300

#endif

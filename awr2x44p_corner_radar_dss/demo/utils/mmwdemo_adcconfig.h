/**
 *   @file  mmwdemo_adcconfig.h
 *
 *   @brief
 *      API header for adc open/config functions
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
#ifndef MMWDEMO_ADCCONFIG_H
#define MMWDEMO_ADCCONFIG_H

#if defined(SOC_AWR294X) || defined(SOC_AWR2X44P)
#include <drivers/adcbuf.h>
#endif

/** @defgroup MMWDEMO_ADCCONFIG_EXTERNAL       Mmwdemo ADC Config External
 */

/**
@defgroup MMWDEMO_ADCCONFIG_EXTERNAL_FUNCTION            ADC Config External Functions
@ingroup MMWDEMO_ADCCONFIG_EXTERNAL
@brief
*   The section has a list of all internal API which are not exposed to the external
*   applications.
*/
/**
@defgroup MMWDEMO_ADCCONFIG_EXTERNAL_DATA_STRUCTURE      ADC Config External Data Structures
@ingroup MMWDEMO_ADCCONFIG_EXTERNAL
@brief
*   The section has a list of all external data structures which are used internally
*   by the ADC Config module.
*/


#ifdef __cplusplus
extern "C"
{
#endif

#ifdef SOC_AM273X
    /**
     * @brief
     *  ADC Buffer data format Parameters
     *
     * @details
     *  The structure is used to define ADC Buffer data format.
     *
     *  \ingroup ADCBUF_DRIVER_EXTERNAL_DATA_STRUCTURE
     *
     */
    typedef struct ADCBuf_dataFormat_t
    {
        /* ADC out Format
             0 - Complex Data mode
             1 - Real data mode
          */
        uint8_t adcOutFormat;

        /* Sample interleave - IQswap
             0 - I is stored in LSB, Q is stored in MSB
             1 - Q is stored in LSB, I is stored in MSB
          */
        uint8_t sampleInterleave;

        /* channel interleave
             0 - interleaved
             1 - non-interleaved
          */
        uint8_t channelInterleave;
    } ADCBuf_dataFormat;

    /**
     * @brief
     *  ADC Buffer RX channel configuration Parameters
     *
     * @details
     *  The structure is used to define ADC Buffer RX Channel configuration.
     *
     *  \ingroup ADCBUF_DRIVER_EXTERNAL_DATA_STRUCTURE
     *
     */
    typedef struct ADCBuf_RxChanConf_t
    {
        /* RX channel id - 0~3 */
        uint8_t channel;

        /* Address offset for the channel in Ping/Pong buffer
          Used only in non-interleaved mode, it must be 16 bytes aligned.
         */
        uint16_t offset;
    } ADCBuf_RxChanConf;
#endif

    /**
     * @brief
     *  ADCBUF configuration (meant for CLI configuration)
     *
     * @details
     *  The structure is used to hold all the relevant configuration
     *  which is used to configure ADCBUF.
     *
     *  \ingroup MMWDEMO_ADCCONFIG_EXTERNAL_DATA_STRUCTURE
     */
    typedef struct MmwDemo_ADCBufCfg_t
    {
        /*! ADCBUF out format:
            0-Complex,
            1-Real */
        uint8_t adcFmt;

        /*! ADCBUF IQ swap selection:
            0-I in LSB, Q in MSB,
            1-Q in LSB, I in MSB */
        uint8_t iqSwapSel;

        /*! ADCBUF channel interleave configuration:
            0-interleaved(not supported on XWR16xx),
            1- non-interleaved */
        uint8_t chInterleave;

        /**
         * @brief   Chirp Threshold configuration used for ADCBUF buffer
         */
        uint8_t chirpThreshold;
    } MmwDemo_ADCBufCfg;

#if defined(SOC_AWR294X) || defined(SOC_AWR2X44P)
    extern ADCBuf_Handle MmwDemo_ADCBufOpen();
#endif

    extern int32_t MmwDemo_ADCBufConfig(
#if defined(SOC_AWR294X) || defined(SOC_AWR2X44P)
        ADCBuf_Handle adcBufHandle,
#endif
        uint16_t           rxChannelEn,
        uint8_t            chirpThreshold,
        uint32_t           chanDataSize,
        MmwDemo_ADCBufCfg *adcBufCfg,
        uint16_t          *rxChanOffset);

#ifdef __cplusplus
}
#endif

#endif /* MMWDEMO_ADCCONFIG_H */

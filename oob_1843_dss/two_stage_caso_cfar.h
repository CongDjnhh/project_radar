/**
 * @file two_stage_caso_cfar.h
 *
 * @brief Native Two-Stage CASO-CFAR Peak Detector on C674x DSP over Detection Matrix.
 *        Stage 1: Range Domain CASO-CFAR across full chirp span (0.15m - 8.64m, 256 bins).
 *        Stage 2: Doppler Domain CASO-CFAR per valid range cell without starving static reflectors.
 */

#ifndef TWO_STAGE_CASO_CFAR_H
#define TWO_STAGE_CASO_CFAR_H

#include <stdint.h>
#include <stddef.h>
#include <ti/datapath/dpif/dpif_detmatrix.h>
#include <ti/datapath/dpif/dpif_pointcloud.h>
#include <ti/datapath/dpc/dpu/cfarcaproc/cfarcaprochwa.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MMWDEMO_CASO_MAX_RANGE_BINS       (256U)
#define MMWDEMO_CASO_MAX_DOPPLER_BINS     (32U)
#define MMWDEMO_CASO_MAX_CANDIDATES       (48U)
#define MMWDEMO_CASO_MAX_POOL_CANDIDATES  (128U)

#define MMWDEMO_CASO_RANGE_WIN_LEN_DEF    (8U)
#define MMWDEMO_CASO_RANGE_GUARD_LEN_DEF  (4U)
#define MMWDEMO_CASO_DOPP_WIN_LEN_DEF     (4U)
#define MMWDEMO_CASO_DOPP_GUARD_LEN_DEF   (2U)

/* Default thresholds in Q8 LSBs (256 LSBs ~ 6.02 dB) */
#define MMWDEMO_CASO_RANGE_THRESHOLD_Q8_DEF (341U)  /* ~8.0 dB */
#define MMWDEMO_CASO_DOPP_THRESHOLD_Q8_DEF  (256U)  /* ~6.0 dB */

#pragma DATA_SECTION(gCasoRangeProfile, ".l3ram")
#pragma DATA_ALIGN(gCasoRangeProfile, 8)
static far uint16_t gCasoRangeProfile[MMWDEMO_CASO_MAX_RANGE_BINS];

#pragma DATA_SECTION(gCasoRangePeakDop, ".l3ram")
#pragma DATA_ALIGN(gCasoRangePeakDop, 8)
static far uint16_t gCasoRangePeakDop[MMWDEMO_CASO_MAX_RANGE_BINS];

#pragma DATA_SECTION(gCasoRangeNoise, ".l3ram")
#pragma DATA_ALIGN(gCasoRangeNoise, 8)
static far uint16_t gCasoRangeNoise[MMWDEMO_CASO_MAX_RANGE_BINS];

#pragma DATA_SECTION(gCasoRangeDetected, ".l3ram")
#pragma DATA_ALIGN(gCasoRangeDetected, 8)
static far uint8_t gCasoRangeDetected[MMWDEMO_CASO_MAX_RANGE_BINS];

/**
 * @brief Native Two-Stage CASO-CFAR detector executing on C674x DSP.
 *
 * @param[in]  detMatrix             Detection matrix from DopplerProcHWA (numRangeBins x numDopplerBins)
 * @param[in]  numRangeBins          Number of range bins (e.g. 256)
 * @param[in]  numDopplerBins        Number of Doppler bins (e.g. 32)
 * @param[in]  rangeStepMeters       Range step per bin (e.g. 0.0421875 m)
 * @param[in]  minRangeMeters        Minimum range bound (e.g. 0.15 m)
 * @param[in]  maxRangeMeters        Maximum range bound (e.g. 8.64 m)
 * @param[in]  cfarCfgRange          Range CFAR configuration from CLI (optional, can be NULL)
 * @param[in]  cfarCfgDoppler        Doppler CFAR configuration from CLI (optional, can be NULL)
 * @param[out] outCfarList           Output array for detected range-Doppler candidates
 * @param[in]  maxOutCfar            Maximum capacity of outCfarList
 * @param[out] numCfarDetectedPoints Number of candidates detected
 *
 * @return 0 on success, negative error code on failure.
 */
static inline int32_t MmwDemo_TwoStageCasoCfar_process(
    const DPIF_DetMatrix *detMatrix,
    uint16_t numRangeBins,
    uint16_t numDopplerBins,
    float rangeStepMeters,
    float minRangeMeters,
    float maxRangeMeters,
    const DPU_CFARCAProc_CfarCfg *cfarCfgRange,
    const DPU_CFARCAProc_CfarCfg *cfarCfgDoppler,
    DPIF_CFARDetList *outCfarList,
    uint32_t maxOutCfar,
    uint32_t *numCfarDetectedPoints)
{
    const uint16_t *detMat;
    uint16_t rStart, rEnd;
    uint16_t r, d;
    uint16_t rangeWinLen, rangeGuardLen;
    uint16_t doppWinLen, doppGuardLen;
    uint16_t rangeThresholdQ8, doppThresholdQ8;
    uint32_t candidateCount = 0U;
    uint32_t poolLimit;

    if ((detMatrix == NULL) || (detMatrix->data == NULL) || (outCfarList == NULL) ||
        (numCfarDetectedPoints == NULL) || (numRangeBins == 0U) || (numDopplerBins == 0U) ||
        (rangeStepMeters <= 0.0f) || (numRangeBins > MMWDEMO_CASO_MAX_RANGE_BINS) ||
        (numDopplerBins > MMWDEMO_CASO_MAX_DOPPLER_BINS))
    {
        return -1;
    }

    detMat = (const uint16_t *)detMatrix->data;
    poolLimit = (maxOutCfar < MMWDEMO_CASO_MAX_POOL_CANDIDATES) ? maxOutCfar : MMWDEMO_CASO_MAX_POOL_CANDIDATES;

    /* Clamp range search bounds: 0.15 m - 8.64 m, spanning all physical bins.
     * Note: rEnd is clamped to numRangeBins - 2U so local peak check cutVal < gCasoRangeProfile[r + 1U]
     * never accesses past the end of gCasoRangeProfile (index numRangeBins). */
    rStart = (uint16_t)(minRangeMeters / rangeStepMeters);
    if (rStart < 1U)
    {
        rStart = 1U;
    }
    rEnd = (uint16_t)((maxRangeMeters / rangeStepMeters) + 0.5f);
    if (rEnd >= (numRangeBins - 1U))
    {
        rEnd = (numRangeBins >= 2U) ? (numRangeBins - 2U) : 0U;
    }
    if (rStart > rEnd)
    {
        *numCfarDetectedPoints = 0U;
        return 0;
    }

    /* Range CFAR Parameters */
    rangeWinLen   = (cfarCfgRange && cfarCfgRange->winLen > 0U)   ? cfarCfgRange->winLen   : MMWDEMO_CASO_RANGE_WIN_LEN_DEF;
    rangeGuardLen = (cfarCfgRange && cfarCfgRange->guardLen > 0U) ? cfarCfgRange->guardLen : MMWDEMO_CASO_RANGE_GUARD_LEN_DEF;
    if (cfarCfgRange && cfarCfgRange->thresholdScale > 0U)
    {
        /* thresholdScale is encoded in 0.01 dB steps: threshold_dB = scale / 100.0f
         * Q8 LSBs = (threshold_dB * 256) / 6.0206f = (scale * 256) / 602 */
        rangeThresholdQ8 = (uint16_t)(((uint32_t)cfarCfgRange->thresholdScale * 256U) / 602U);
        if (rangeThresholdQ8 < 200U) { rangeThresholdQ8 = 200U; } /* min ~4.7 dB */
        if (rangeThresholdQ8 > 600U) { rangeThresholdQ8 = 600U; } /* max ~14.1 dB */
    }
    else
    {
        rangeThresholdQ8 = MMWDEMO_CASO_RANGE_THRESHOLD_Q8_DEF;
    }

    /* Doppler CFAR Parameters */
    doppWinLen   = (cfarCfgDoppler && cfarCfgDoppler->winLen > 0U)   ? cfarCfgDoppler->winLen   : MMWDEMO_CASO_DOPP_WIN_LEN_DEF;
    doppGuardLen = (cfarCfgDoppler && cfarCfgDoppler->guardLen > 0U) ? cfarCfgDoppler->guardLen : MMWDEMO_CASO_DOPP_GUARD_LEN_DEF;
    if (cfarCfgDoppler && cfarCfgDoppler->thresholdScale > 0U)
    {
        doppThresholdQ8 = (uint16_t)(((uint32_t)cfarCfgDoppler->thresholdScale * 256U) / 602U);
        if (doppThresholdQ8 < 180U) { doppThresholdQ8 = 180U; } /* min ~4.2 dB */
        if (doppThresholdQ8 > 500U) { doppThresholdQ8 = 500U; } /* max ~11.8 dB */
    }
    else
    {
        doppThresholdQ8 = MMWDEMO_CASO_DOPP_THRESHOLD_Q8_DEF;
    }

    /* ------------------------------------------------------------------------
     * Step 1: Scan detMatrix rows to build 1D Range Profile & Doppler Peak/Noise
     * ------------------------------------------------------------------------ */
    for (r = 0U; r < numRangeBins; r++)
    {
        const uint16_t *detRow = &detMat[(uint32_t)r * numDopplerBins];
        uint32_t sumVal = 0U;
        uint16_t maxVal = 0U;
        uint16_t maxDop = 0U;

        for (d = 0U; d < numDopplerBins; d++)
        {
            uint16_t val = detRow[d];
            sumVal += (uint32_t)val;
            if (val > maxVal)
            {
                maxVal = val;
                maxDop = d;
            }
        }
        gCasoRangeProfile[r] = maxVal;
        gCasoRangePeakDop[r] = maxDop;
        gCasoRangeNoise[r]   = (uint16_t)(sumVal / numDopplerBins);
        gCasoRangeDetected[r] = 0U;
    }

    /* ------------------------------------------------------------------------
     * Step 2: Stage 1 (Range Domain CASO-CFAR)
     * Scan across all 256 range bins to detect physical obstacle reflection ranges.
     * ------------------------------------------------------------------------ */
    for (r = rStart; r <= rEnd; r++)
    {
        uint16_t cutVal = gCasoRangeProfile[r];
        int32_t leftStart, leftEnd, rightStart, rightEnd;
        int32_t k;
        uint32_t sumL = 0U, countL = 0U;
        uint32_t sumR = 0U, countR = 0U;
        uint16_t avgL, avgR, casoNoise;

        /* Range Domain Local Peak Check (strict local maximum) */
        if ((cutVal < gCasoRangeProfile[r - 1U]) || (cutVal < gCasoRangeProfile[r + 1U]))
        {
            continue;
        }
        if ((cutVal == gCasoRangeProfile[r - 1U]) && (cutVal == gCasoRangeProfile[r + 1U]))
        {
            continue;
        }

        /* Left reference window */
        leftStart = (int32_t)r - (int32_t)rangeGuardLen - (int32_t)rangeWinLen;
        leftEnd   = (int32_t)r - (int32_t)rangeGuardLen - 1;
        if (leftStart < 0) { leftStart = 0; }
        if (leftEnd >= 0)
        {
            for (k = leftStart; k <= leftEnd; k++)
            {
                sumL += (uint32_t)gCasoRangeProfile[k];
                countL++;
            }
        }

        /* Right reference window */
        rightStart = (int32_t)r + (int32_t)rangeGuardLen + 1;
        rightEnd   = (int32_t)r + (int32_t)rangeGuardLen + (int32_t)rangeWinLen;
        if (rightEnd >= (int32_t)numRangeBins) { rightEnd = (int32_t)numRangeBins - 1; }
        if (rightStart < (int32_t)numRangeBins)
        {
            for (k = rightStart; k <= rightEnd; k++)
            {
                sumR += (uint32_t)gCasoRangeProfile[k];
                countR++;
            }
        }

        /* CASO: Cell Average Smallest Of */
        if ((countL > 0U) && (countR > 0U))
        {
            avgL = (uint16_t)(sumL / countL);
            avgR = (uint16_t)(sumR / countR);
            casoNoise = (avgL < avgR) ? avgL : avgR;
        }
        else if (countL > 0U)
        {
            casoNoise = (uint16_t)(sumL / countL);
        }
        else if (countR > 0U)
        {
            casoNoise = (uint16_t)(sumR / countR);
        }
        else
        {
            casoNoise = gCasoRangeNoise[r];
        }

        uint16_t minFloorOffset = (rangeThresholdQ8 < 256U) ? (rangeThresholdQ8 / 2U) : 200U;

        /* Range detection condition:
         * 1. Exceeds local CASO noise by range threshold
         * 2. Exceeds Doppler noise floor by at least minFloorOffset */
        if ((cutVal >= (casoNoise + rangeThresholdQ8)) &&
            (cutVal >= (gCasoRangeNoise[r] + minFloorOffset)))
        {
            gCasoRangeDetected[r] = 1U;
        }
    }

    /* ------------------------------------------------------------------------
     * Step 3: Stage 2 (Doppler Domain CASO-CFAR per Valid Range Cell)
     * For each detected range, resolve velocity peaks without starving static reflectors.
     * ------------------------------------------------------------------------ */
    for (r = rStart; (r <= rEnd) && (candidateCount < poolLimit); r++)
    {
        const uint16_t *detRow;
        uint16_t dopPeaksFound = 0U;

        if (gCasoRangeDetected[r] == 0U)
        {
            continue;
        }

        detRow = &detMat[(uint32_t)r * numDopplerBins];

        for (d = 0U; d < numDopplerBins; d++)
        {
            uint16_t val = detRow[d];
            uint16_t prevD = (d == 0U) ? (numDopplerBins - 1U) : (d - 1U);
            uint16_t nextD = (d == (numDopplerBins - 1U)) ? 0U : (d + 1U);
            uint32_t sumL = 0U;
            uint32_t sumR = 0U;
            uint16_t k, avgL, avgR, casoDoppNoise;

            /* Circular local peak check */
            if ((val < detRow[prevD]) || (val < detRow[nextD]))
            {
                continue;
            }
            if ((val == detRow[prevD]) && (val == detRow[nextD]))
            {
                continue;
            }

            /* Circular left reference window */
            for (k = 0U; k < doppWinLen; k++)
            {
                uint16_t idxL = (d + numDopplerBins - doppGuardLen - 1U - k) % numDopplerBins;
                sumL += (uint32_t)detRow[idxL];
            }

            /* Circular right reference window */
            for (k = 0U; k < doppWinLen; k++)
            {
                uint16_t idxR = (d + doppGuardLen + 1U + k) % numDopplerBins;
                sumR += (uint32_t)detRow[idxR];
            }

            avgL = (uint16_t)(sumL / doppWinLen);
            avgR = (uint16_t)(sumR / doppWinLen);
            casoDoppNoise = (avgL < avgR) ? avgL : avgR;

            if (val >= (casoDoppNoise + doppThresholdQ8))
            {
                /* Valid velocity peak found */
                uint16_t snr = (uint16_t)(((uint32_t)(val - casoDoppNoise) * 60U) / 256U);
                int16_t noise = (int16_t)(((uint32_t)casoDoppNoise * 60U) / 256U);

                outCfarList[candidateCount].rangeIdx   = r;
                outCfarList[candidateCount].dopplerIdx = d;
                outCfarList[candidateCount].snr        = (int16_t)snr;
                outCfarList[candidateCount].noise      = noise;
                candidateCount++;
                dopPeaksFound++;

                if (candidateCount >= poolLimit)
                {
                    break;
                }
            }
        }

        /* --------------------------------------------------------------------
         * Fail-Safe for Static / Dominant Peak Reflectors:
         * If Stage 1 confirmed an obstacle range, but no circular Doppler peak
         * passed the Doppler threshold (e.g. 0-Doppler static target), we take
         * the peak Doppler bin (which is d = 0 for static obstacles) so that
         * static reflectors are NEVER starved.
         * -------------------------------------------------------------------- */
        if ((dopPeaksFound == 0U) && (candidateCount < poolLimit))
        {
            uint16_t peakDop = gCasoRangePeakDop[r];
            uint16_t peakVal = gCasoRangeProfile[r];
            uint16_t noiseFloor = gCasoRangeNoise[r];
            uint16_t diff = (peakVal > noiseFloor) ? (peakVal - noiseFloor) : 0U;
            uint16_t snr = (uint16_t)(((uint32_t)diff * 60U) / 256U);
            int16_t noise = (int16_t)(((uint32_t)noiseFloor * 60U) / 256U);

            outCfarList[candidateCount].rangeIdx   = r;
            outCfarList[candidateCount].dopplerIdx = peakDop;
            outCfarList[candidateCount].snr        = (int16_t)snr;
            outCfarList[candidateCount].noise      = noise;
            candidateCount++;
        }
    }

    /* ------------------------------------------------------------------------
     * Step 4: Cap Candidates to Maintain DSP Execution Time < 4 ms
     * Sort candidates descending by SNR to retain highest-confidence detections.
     * ------------------------------------------------------------------------ */
    if (candidateCount > 1U)
    {
        uint32_t i, j;
        for (i = 0U; i < candidateCount - 1U; i++)
        {
            uint32_t maxIdx = i;
            for (j = i + 1U; j < candidateCount; j++)
            {
                if (outCfarList[j].snr > outCfarList[maxIdx].snr)
                {
                    maxIdx = j;
                }
            }
            if (maxIdx != i)
            {
                DPIF_CFARDetList tmp = outCfarList[i];
                outCfarList[i] = outCfarList[maxIdx];
                outCfarList[maxIdx] = tmp;
            }
        }
    }

    /* Cap candidate count to maintain DSP execution time < 4 ms */
    if (candidateCount > MMWDEMO_CASO_MAX_CANDIDATES)
    {
        candidateCount = MMWDEMO_CASO_MAX_CANDIDATES;
    }

    *numCfarDetectedPoints = candidateCount;
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* TWO_STAGE_CASO_CFAR_H */

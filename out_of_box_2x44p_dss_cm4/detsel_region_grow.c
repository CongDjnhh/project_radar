/**
 *   @file  detsel_region_grow.c
 *
 *   @brief
 *      Pure C99 Azimuth-Guided Hysteresis Region Growing Stage-2 selector
 *      for AWR2944P DDMA radar processing.
 */

#include <stddef.h>
#include <math.h>
#include "detsel_region_grow.h"

/* Exact log2(10) constant used by TI SDK MmwDemo_convertDopplerCfarToThresh */
#define DETSEL_CONST_LOG2_10         (3.3219)
#define DETSEL_CFAR_ENCODING_FACTOR  (100.0)
#define DETSEL_Q11_SCALE             (2048.0)

int16_t DetSel_dbToQ11(float dbVal)
{
    uint16_t codedCfarVal;
    float    quantDb;
    uint32_t linVal;

    if (!(dbVal > 0.0f))
    {
        return 0;
    }
    if (dbVal > 100.0f)
    {
        dbVal = 100.0f;
    }

    /* Match CLI encoding (uint16_t)(db * 100.0) + MmwDemo_convertDopplerCfarToThresh */
    codedCfarVal = (uint16_t)((double)dbVal * DETSEL_CFAR_ENCODING_FACTOR);
    quantDb      = (float)((double)codedCfarVal / DETSEL_CFAR_ENCODING_FACTOR);
    linVal       = (uint32_t)(((double)quantDb / 20.0) * DETSEL_CONST_LOG2_10 * DETSEL_Q11_SCALE);

    if (linVal > 32767UL)
    {
        linVal = 32767UL;
    }
    return (int16_t)linVal;
}

int32_t DetSel_validateCfg(const DetSel_Cfg *cfg)
{
    if (cfg == NULL)
    {
        return DETSEL_ERR_NULL_PTR;
    }
    if (cfg->mode > DETSEL_MODE_REGION_GROW)
    {
        return DETSEL_ERR_INVALID_CFG;
    }
    if ((cfg->baselineThrDb < 0.0f) || (cfg->baselineThrDb > 100.0f) || !isfinite(cfg->baselineThrDb))
    {
        return DETSEL_ERR_INVALID_CFG;
    }
    if ((cfg->mode != DETSEL_MODE_TI_BASELINE) && (cfg->baselineThrDb <= 0.0f))
    {
        return DETSEL_ERR_INVALID_CFG;
    }
    if (cfg->mode == DETSEL_MODE_REGION_GROW)
    {
        if (!isfinite(cfg->growThrDb) || (cfg->growThrDb <= 0.0f) ||
            (cfg->growThrDb > cfg->baselineThrDb) ||
            !isfinite(cfg->sidelobeGuardDb) || (cfg->sidelobeGuardDb < 0.0f) || (cfg->sidelobeGuardDb > 100.0f) ||
            !isfinite(cfg->growRangeM) || (cfg->growRangeM <= 0.0f) ||
            !isfinite(cfg->growDopplerMps) || (cfg->growDopplerMps <= 0.0f) ||
            !isfinite(cfg->growAzimBeams) || (cfg->growAzimBeams <= 0.0f) ||
            (cfg->maxGrowHops == 0U))
        {
            return DETSEL_ERR_INVALID_CFG;
        }
    }
    return DETSEL_OK;
}

int32_t DetSel_deriveBins(
    const DetSel_Physics *phys,
    const DetSel_Cfg     *cfg,
    DetSel_DerivedBins   *outBins)
{
    int32_t status;
    float   rBinsF;
    float   dBinsF;
    float   aBinsF;

    if ((phys == NULL) || (cfg == NULL) || (outBins == NULL))
    {
        return DETSEL_ERR_NULL_PTR;
    }
    if (!isfinite(phys->rangeStep) || (phys->rangeStep <= 0.0f) ||
        !isfinite(phys->dopplerStep) || (phys->dopplerStep <= 0.0f) ||
        (phys->numRangeBins < 2U) || (phys->numRangeBins > DETSEL_MAX_RANGE_BINS) ||
        (phys->numDopplerBins < 2U) ||
        (phys->numAzimFFTBins < 2U) ||
        (phys->numVirtualAntAzim == 0U) || (phys->numVirtualAntAzim > phys->numAzimFFTBins))
    {
        return DETSEL_ERR_INVALID_PHYSICS;
    }

    status = DetSel_validateCfg(cfg);
    if (status != DETSEL_OK)
    {
        return status;
    }

    outBins->baseThrQ11 = DetSel_dbToQ11(cfg->baselineThrDb);
    if (cfg->mode != DETSEL_MODE_REGION_GROW)
    {
        outBins->rWin             = 1U;
        outBins->dWin             = 1U;
        outBins->aWin             = 1U;
        outBins->growThrQ11       = outBins->baseThrQ11;
        outBins->sidelobeQ11      = 0U;
        outBins->zeroDopMidThrQ11 = outBins->baseThrQ11;
        return DETSEL_OK;
    }

    rBinsF = ceilf(cfg->growRangeM / phys->rangeStep);
    dBinsF = ceilf(cfg->growDopplerMps / phys->dopplerStep);
    aBinsF = ceilf(cfg->growAzimBeams * ((float)phys->numAzimFFTBins / (float)phys->numVirtualAntAzim));

    outBins->rWin = (rBinsF >= 1.0f) ?
        ((rBinsF > (float)phys->numRangeBins) ? phys->numRangeBins : (uint16_t)rBinsF) : 1U;
    outBins->dWin = (dBinsF >= 1.0f) ?
        ((dBinsF > (float)(phys->numDopplerBins / 2U)) ? (uint16_t)(phys->numDopplerBins / 2U) : (uint16_t)dBinsF) : 1U;
    outBins->aWin = (aBinsF >= 1.0f) ?
        ((aBinsF > (float)(phys->numAzimFFTBins / 2U)) ? (uint16_t)(phys->numAzimFFTBins / 2U) : (uint16_t)aBinsF) : 1U;

    outBins->growThrQ11       = DetSel_dbToQ11(cfg->growThrDb);
    outBins->sidelobeQ11      = (uint16_t)DetSel_dbToQ11(cfg->sidelobeGuardDb);
    outBins->zeroDopMidThrQ11 = (int16_t)(((int32_t)outBins->growThrQ11 + (int32_t)outBins->baseThrQ11) / 2);

    return DETSEL_OK;
}

int32_t DetSel_regionGrow(
    const DetSel_Physics *phys,
    const DetSel_Cfg     *cfg,
    uint16_t              numCand,
    DetSel_ScratchBuf    *scratch,
    DetSel_OutStats      *outStats)
{
    DetSel_DerivedBins bins;
    int32_t            status;
    uint16_t           i;
    uint16_t           r;
    uint16_t           curIdx;
    uint16_t           qHead;
    uint16_t           qTail;
    uint16_t           numSeeds = 0U;
    uint16_t           numGrown = 0U;

    if ((phys == NULL) || (cfg == NULL) || (scratch == NULL) || (outStats == NULL))
    {
        return DETSEL_ERR_NULL_PTR;
    }
    if (numCand > DETSEL_MAX_CANDIDATES)
    {
        return DETSEL_ERR_TOO_MANY_CAND;
    }

    outStats->numSeeds    = 0U;
    outStats->numGrown    = 0U;
    outStats->numSelected = 0U;
    outStats->truncated   = 0U;
    outStats->reserved    = 0U;

    status = DetSel_deriveBins(phys, cfg, &bins);
    if (status != DETSEL_OK)
    {
        return status;
    }

    if (numCand == 0U)
    {
        return DETSEL_OK;
    }

    /* Step 1: Verify monotonic non-decreasing rangeIdx & build O(N + N_r) range-gate LUT */
    for (i = 0U; i < numCand; i++)
    {
        if (scratch->cand[i].rangeIdx >= phys->numRangeBins)
        {
            return DETSEL_ERR_RANGE_OOB;
        }
        if ((i > 0U) && (scratch->cand[i].rangeIdx < scratch->cand[i - 1U].rangeIdx))
        {
            return DETSEL_ERR_NON_MONOTONIC;
        }
    }

    curIdx = 0U;
    for (r = 0U; r < phys->numRangeBins; r++)
    {
        scratch->gateStart[r] = curIdx;
        while ((curIdx < numCand) && (scratch->cand[curIdx].rangeIdx == r))
        {
            curIdx++;
        }
    }
    scratch->gateStart[phys->numRangeBins] = numCand;

    /* Step 2: Initialize BFS queue from exact TI Baseline seeds */
    qHead = 0U;
    qTail = 0U;
    for (i = 0U; i < numCand; i++)
    {
        uint8_t t = scratch->cand[i].initTag;
        if ((t & DETSEL_TAG_BASELINE) != 0U)
        {
            t |= DETSEL_TAG_SEED;
            scratch->hop[i]       = 0U;
            scratch->rootPeak[i]  = scratch->cand[i].peakQ11;
            scratch->rootRange[i] = scratch->cand[i].rangeIdx;
            scratch->bfsQueue[qTail++] = i;
            numSeeds++;
        }
        else
        {
            scratch->hop[i]       = 0xFFFFU;
            scratch->rootPeak[i]  = 0U;
            scratch->rootRange[i] = 0U;
        }
        scratch->tag[i] = t;
    }

    /* Step 3: 3D (Range, ActualDoppler, AzimIdx) BFS hysteresis expansion */
    while (qHead < qTail)
    {
        uint16_t u    = scratch->bfsQueue[qHead++];
        uint16_t r_u  = scratch->cand[u].rangeIdx;
        uint16_t d_u  = (scratch->cand[u].dopIdxActual >= phys->numDopplerBins) ?
                        (uint16_t)(scratch->cand[u].dopIdxActual % phys->numDopplerBins) :
                        scratch->cand[u].dopIdxActual;
        uint16_t a_u  = (uint16_t)(scratch->cand[u].azimIdx % phys->numAzimFFTBins);
        uint16_t rMin;
        uint16_t rMax;

        if (scratch->hop[u] >= cfg->maxGrowHops)
        {
            continue;
        }

        /* Skip range bin 0 consistent with DPU_DopplerProcHWA_extractObjectList */
        rMin = (r_u > bins.rWin) ? (uint16_t)(r_u - bins.rWin) : 1U;
        if (rMin == 0U)
        {
            rMin = 1U;
        }
        rMax = (((uint32_t)r_u + (uint32_t)bins.rWin) < (uint32_t)phys->numRangeBins) ?
               (uint16_t)(r_u + bins.rWin) : (uint16_t)(phys->numRangeBins - 1U);

        for (r = rMin; r <= rMax; r++)
        {
            uint16_t vStart = scratch->gateStart[r];
            uint16_t vEnd   = scratch->gateStart[r + 1U];
            uint16_t drRoot = (r >= scratch->rootRange[u]) ?
                              (uint16_t)(r - scratch->rootRange[u]) :
                              (uint16_t)(scratch->rootRange[u] - r);
            uint16_t v;

            for (v = vStart; v < vEnd; v++)
            {
                uint16_t d_v;
                uint16_t diffD;
                uint16_t circD;
                uint16_t a_v;
                uint16_t diffA;
                uint16_t circA;

                if ((scratch->tag[v] & (DETSEL_TAG_SEED | DETSEL_TAG_GROWN)) != 0U)
                {
                    continue;
                }

                /* 1. Hysteresis SNR margin check */
                if (scratch->cand[v].marginQ11 < bins.growThrQ11)
                {
                    continue;
                }

                /* 2. Circular Actual-Doppler continuity (prevents DDMA sub-band alias bridges) */
                d_v   = (scratch->cand[v].dopIdxActual >= phys->numDopplerBins) ?
                        (uint16_t)(scratch->cand[v].dopIdxActual % phys->numDopplerBins) :
                        scratch->cand[v].dopIdxActual;
                diffD = (d_v >= d_u) ? (uint16_t)(d_v - d_u) : (uint16_t)(d_u - d_v);
                circD = ((uint16_t)(phys->numDopplerBins - diffD) < diffD) ?
                        (uint16_t)(phys->numDopplerBins - diffD) : diffD;
                if (circD > bins.dWin)
                {
                    continue;
                }

                /* 3. Circular Azimuth-FFT continuity (leverages 12-vAnt aperture) */
                a_v   = (uint16_t)(scratch->cand[v].azimIdx % phys->numAzimFFTBins);
                diffA = (a_v >= a_u) ? (uint16_t)(a_v - a_u) : (uint16_t)(a_u - a_v);
                circA = ((uint16_t)(phys->numAzimFFTBins - diffA) < diffA) ?
                        (uint16_t)(phys->numAzimFFTBins - diffA) : diffA;
                if (circA > bins.aWin)
                {
                    continue;
                }

                /* 4. Range sinc sidelobe guard for |r_v - rootRange| >= 2 */
                if ((drRoot >= 2U) &&
                    (((uint32_t)scratch->cand[v].peakQ11 + (uint32_t)bins.sidelobeQ11) <
                     (uint32_t)scratch->rootPeak[u]))
                {
                    continue;
                }

                /* 5. Zero-Doppler floor clutter bleed guard */
                if ((d_v == 0U) && (d_u == 0U) && (drRoot > 1U) &&
                    (scratch->cand[v].marginQ11 < bins.zeroDopMidThrQ11))
                {
                    continue;
                }

                scratch->tag[v]       |= DETSEL_TAG_GROWN;
                scratch->hop[v]        = (uint16_t)(scratch->hop[u] + 1U);
                scratch->rootPeak[v]   = scratch->rootPeak[u];
                scratch->rootRange[v]  = scratch->rootRange[u];
                scratch->bfsQueue[qTail++] = v;
                numGrown++;
            }
        }
    }

    outStats->numSeeds    = numSeeds;
    outStats->numGrown    = numGrown;
    outStats->numSelected = (uint16_t)(numSeeds + numGrown);
    return DETSEL_OK;
}

uint16_t DetSel_selectIndices(
    const DetSel_Cand *cand,
    const uint8_t     *tag,
    uint16_t           numCand,
    uint8_t            primaryMask,
    uint8_t            secondaryMask,
    uint16_t           maxOut,
    uint16_t          *bfsScratchIdx,
    uint16_t          *selIdx,
    uint8_t           *outTruncated)
{
    uint8_t  anyMask = (uint8_t)(primaryMask | secondaryMask);
    uint16_t totalEligible = 0U;
    uint16_t outCount = 0U;
    uint16_t secCount = 0U;
    uint16_t i;
    uint16_t j;

    if (outTruncated != NULL)
    {
        *outTruncated = 0U;
    }
    if ((cand == NULL) || (tag == NULL) || (selIdx == NULL) || (maxOut == 0U))
    {
        return 0U;
    }

    for (i = 0U; i < numCand; i++)
    {
        if ((tag[i] & anyMask) != 0U)
        {
            totalEligible++;
        }
    }

    /* Fast path (99.9% of frames): all eligible fit within maxOut (793) */
    if (totalEligible <= maxOut)
    {
        for (i = 0U; i < numCand; i++)
        {
            if ((tag[i] & anyMask) != 0U)
            {
                selIdx[outCount++] = i;
            }
        }
        return outCount;
    }

    /* Overflow path: preserve 100% of primaryMask (Baseline/SEED) first */
    if (outTruncated != NULL)
    {
        *outTruncated = 1U;
    }
    for (i = 0U; i < numCand; i++)
    {
        if ((tag[i] & primaryMask) != 0U)
        {
            if (outCount < maxOut)
            {
                selIdx[outCount++] = i;
            }
        }
        else if (((tag[i] & secondaryMask) != 0U) && (bfsScratchIdx != NULL))
        {
            bfsScratchIdx[secCount++] = i;
        }
    }

    /* Fill remaining slots with highest-marginQ11 secondary candidates (tie-break: lower index) */
    if ((outCount < maxOut) && (bfsScratchIdx != NULL) && (secCount > 0U))
    {
        uint16_t need = (uint16_t)(maxOut - outCount);
        if (need > secCount)
        {
            need = secCount;
        }
        /* Partial selection sort for top `need` secondary candidates */
        for (i = 0U; i < need; i++)
        {
            uint16_t bestPos = i;
            int16_t  bestM   = cand[bfsScratchIdx[i]].marginQ11;
            uint16_t bestIdx = bfsScratchIdx[i];

            for (j = (uint16_t)(i + 1U); j < secCount; j++)
            {
                uint16_t cIdx = bfsScratchIdx[j];
                int16_t  cM   = cand[cIdx].marginQ11;
                if ((cM > bestM) || ((cM == bestM) && (cIdx < bestIdx)))
                {
                    bestPos = j;
                    bestM   = cM;
                    bestIdx = cIdx;
                }
            }
            if (bestPos != i)
            {
                uint16_t tmp           = bfsScratchIdx[i];
                bfsScratchIdx[i]       = bfsScratchIdx[bestPos];
                bfsScratchIdx[bestPos] = tmp;
            }
            selIdx[outCount++] = bfsScratchIdx[i];
        }

        /* Restore ascending candidate index order via insertion sort on selIdx[0..outCount-1] */
        for (i = 1U; i < outCount; i++)
        {
            uint16_t key = selIdx[i];
            j = i;
            while ((j > 0U) && (selIdx[j - 1U] > key))
            {
                selIdx[j] = selIdx[j - 1U];
                j--;
            }
            selIdx[j] = key;
        }
    }

    return outCount;
}

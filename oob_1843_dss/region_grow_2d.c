/**
 * @file region_grow_2d.c
 * @brief Hardware-agnostic 2D Anisotropic Range-Doppler Region Growing Core
 *        implementation.
 */
#include "region_grow_2d.h"

#if defined(__TI_COMPILER_VERSION__) && defined(SUBSYS_DSS) && defined(SOC_XWR18XX)
#pragma CODE_SECTION(Alg_RegionGrow2D_noiseAnchor, ".surfaceCode");
#pragma CODE_SECTION(Alg_RegionGrow2D_growFromSeed2D, ".surfaceCode");
#pragma CODE_SECTION(Alg_RegionGrow2D_condenseDopplerRidges, ".surfaceCode");
#endif

static uint16_t Alg_RegionGrow2D_snrDb10FromQ8(uint16_t energyQ8, uint16_t anchorQ8)
{
    if (energyQ8 > anchorQ8)
    {
        return (uint16_t)(((uint32_t)(energyQ8 - anchorQ8) * 15U + 32U) >> 6U);
    }
    return 1U;
}

static uint16_t Alg_RegionGrow2D_farWindowMedian(
    const uint16_t *detMat,
    uint16_t numDopplerBins,
    uint16_t dopIdx,
    uint16_t rFirst,
    uint16_t farCnt)
{
    uint16_t targetLow = (uint16_t)((farCnt - 1U) >> 1U);
    uint16_t targetHigh = (uint16_t)(farCnt >> 1U);
    uint16_t vLow = 0xFFFFU;
    uint16_t vHigh = 0xFFFFU;
    uint16_t i;
    uint16_t j;

    for (i = 0U; i < farCnt; i++)
    {
        uint16_t vi = detMat[(uint32_t)(rFirst + i) * numDopplerBins + dopIdx];
        uint16_t rank = 0U;

        for (j = 0U; j < farCnt; j++)
        {
            uint16_t vj = detMat[(uint32_t)(rFirst + j) * numDopplerBins + dopIdx];
            if ((vj < vi) || ((vj == vi) && (j < i)))
            {
                rank++;
            }
        }
        if (rank == targetLow)
        {
            vLow = vi;
        }
        if (rank == targetHigh)
        {
            vHigh = vi;
        }
    }
    return (uint16_t)(((uint32_t)vLow + (uint32_t)vHigh) >> 1U);
}

uint16_t Alg_RegionGrow2D_noiseAnchor(
    const uint16_t *detMat,
    uint16_t numDopplerBins,
    uint16_t minRange,
    uint16_t maxRange,
    uint16_t rSeed,
    uint16_t dopIdx,
    const uint16_t params[ALG_RG2D_NUM_PARAMS])
{
    uint16_t farCnt = 0U;
    uint16_t farFirst = 0U;
    uint16_t nearCnt = 0U;
    uint32_t nearSum = 0U;
    uint16_t nFar = 0xFFFFU;
    uint16_t nNear = 0xFFFFU;
    uint16_t farInner = params[ALG_RG2D_PARAM_FAR_INNER];
    uint16_t farOuter = params[ALG_RG2D_PARAM_FAR_OUTER];
    uint16_t nearInner = params[ALG_RG2D_PARAM_NEAR_INNER];
    uint16_t nearOuter = params[ALG_RG2D_PARAM_NEAR_OUTER];
    uint16_t k;

    if (farOuter >= farInner)
    {
        uint16_t maxSpan = (uint16_t)(farOuter - farInner + 1U);
        if (maxSpan > 16U)
        {
            maxSpan = 16U;
        }
        for (k = 0U; k < maxSpan; k++)
        {
            uint16_t off = (uint16_t)(farInner + k);
            if (rSeed >= (uint16_t)(minRange + off))
            {
                farFirst = (uint16_t)(rSeed - off);
                farCnt++;
            }
        }
        if (farCnt > 0U)
        {
            nFar = Alg_RegionGrow2D_farWindowMedian(
                detMat, numDopplerBins, dopIdx, farFirst, farCnt);
        }
    }

    if (nearOuter >= nearInner)
    {
        uint16_t nearSpan = (uint16_t)(nearOuter - nearInner + 1U);
        for (k = 0U; k < nearSpan; k++)
        {
            uint16_t off = (uint16_t)(nearInner + k);
            if (rSeed >= (uint16_t)(minRange + off))
            {
                uint16_t r = (uint16_t)(rSeed - off);
                nearSum += detMat[(uint32_t)r * numDopplerBins + dopIdx];
                nearCnt++;
            }
        }
    }
    if (nearCnt > 0U)
    {
        nNear = (uint16_t)(nearSum / nearCnt);
    }

    if ((farCnt > 0U) && (nearCnt > 0U))
    {
        return (nFar < nNear) ? nFar : nNear;
    }
    if (nearCnt > 0U)
    {
        return nNear;
    }
    if (farCnt > 0U)
    {
        return nFar;
    }

    if (farOuter >= farInner)
    {
        uint16_t maxSpan = (uint16_t)(farOuter - farInner + 1U);
        if (maxSpan > 16U)
        {
            maxSpan = 16U;
        }
        if (((uint32_t)rSeed + farInner) <= (uint32_t)maxRange)
        {
            farFirst = (uint16_t)(rSeed + farInner);
            for (k = 0U; k < maxSpan; k++)
            {
                uint16_t off = (uint16_t)(farInner + k);
                if (((uint32_t)rSeed + off) <= (uint32_t)maxRange)
                {
                    farCnt++;
                }
            }
            if (farCnt > 0U)
            {
                return Alg_RegionGrow2D_farWindowMedian(
                    detMat, numDopplerBins, dopIdx, farFirst, farCnt);
            }
        }
    }

    return detMat[(uint32_t)rSeed * numDopplerBins + dopIdx];
}

uint16_t Alg_RegionGrow2D_growFromSeed2D(
    const uint16_t *detMat,
    uint16_t numDopplerBins,
    uint16_t minRange,
    uint16_t maxRange,
    uint16_t rSeed,
    uint16_t dopIdx,
    uint8_t staticClutterRemovalEnabled,
    const uint16_t params[ALG_RG2D_NUM_PARAMS],
    Alg_RegionGrow2D_Workspace *ws)
{
    uint16_t rStart;
    uint16_t seedVal;
    uint16_t anchor;
    uint32_t exitThr;
    uint32_t enterThr;
    uint16_t effMaxSpan;
    uint16_t clutterCap;
    uint16_t rHardMax;
    uint16_t k;
    uint16_t r;
    uint16_t rCurr;
    uint16_t rEnd;
    uint16_t peakVal;
    uint16_t valleyVal;
    uint16_t valleyIdx;
    uint16_t clusterPeakVal;
    uint16_t clusterPeakRange;
    uint8_t inValley = 0U;
    uint8_t splitAtValley = 0U;
    uint32_t dBit;

    if ((detMat == NULL) || (params == NULL) || (ws == NULL) ||
        (ws->grownMask == NULL) || (ws->seedMask == NULL) || (ws->cellSnr == NULL) ||
        (dopIdx >= numDopplerBins) || (dopIdx >= ALG_RG2D_MAX_BITMAP_DOPPLER_BINS) ||
        (rSeed < minRange) || (rSeed > maxRange))
    {
        return rSeed;
    }

    rStart = rSeed;
    seedVal = detMat[(uint32_t)rSeed * numDopplerBins + dopIdx];
    dBit = 1UL << dopIdx;

    /* Operator 1: Leftward Edge Backtracking with Sidelobe Guard */
    for (k = 1U; k <= params[ALG_RG2D_PARAM_MAX_BACKTRACK]; k++)
    {
        uint16_t rPrev;
        uint16_t ePrev;
        uint16_t localAnchor;

        if (rSeed < (uint16_t)(minRange + k))
        {
            break;
        }
        rPrev = (uint16_t)(rSeed - k);
        ePrev = detMat[(uint32_t)rPrev * numDopplerBins + dopIdx];

        if ((k >= 2U) &&
            (params[ALG_RG2D_PARAM_SIDELOBE_GUARD_Q8] > 0U) &&
            (((uint32_t)ePrev + params[ALG_RG2D_PARAM_SIDELOBE_GUARD_Q8]) < (uint32_t)seedVal))
        {
            break;
        }

        localAnchor = Alg_RegionGrow2D_noiseAnchor(
            detMat, numDopplerBins, minRange, maxRange, rPrev, dopIdx, params);
        if ((uint32_t)ePrev >= ((uint32_t)localAnchor + params[ALG_RG2D_PARAM_ENTER_Q8]))
        {
            rStart = rPrev;
        }
        else
        {
            break;
        }
    }

    /* Operator 2: Frozen Noise Baseline & Zero-Doppler Clutter Guard */
    anchor = Alg_RegionGrow2D_noiseAnchor(
        detMat, numDopplerBins, minRange, maxRange, rStart, dopIdx, params);
    exitThr = (uint32_t)anchor + params[ALG_RG2D_PARAM_EXIT_Q8];
    enterThr = (uint32_t)anchor + params[ALG_RG2D_PARAM_ENTER_Q8];

    if ((uint32_t)detMat[(uint32_t)rStart * numDopplerBins + dopIdx] < exitThr)
    {
        ws->grownMask[rSeed] |= dBit;
        return rSeed;
    }

    effMaxSpan = params[ALG_RG2D_PARAM_MAX_RANGE_SPAN];
    clutterCap = (uint16_t)(params[ALG_RG2D_PARAM_MAX_BACKTRACK] + 1U);
    if (clutterCap < 2U)
    {
        clutterCap = 2U;
    }
    rHardMax = (((uint32_t)rStart + effMaxSpan) <= (uint32_t)maxRange)
        ? (uint16_t)(rStart + effMaxSpan) : maxRange;

    /* Operator 3: Primary Range Marching with Topographic Valley Split & Sidelobe Guard */
    peakVal = detMat[(uint32_t)rStart * numDopplerBins + dopIdx];
    valleyVal = peakVal;
    valleyIdx = rStart;
    clusterPeakVal = (seedVal > peakVal) ? seedVal : peakVal;
    clusterPeakRange = (seedVal > peakVal) ? rSeed : rStart;

    for (rCurr = (uint16_t)(rStart + 1U); rCurr <= rHardMax; rCurr++)
    {
        uint16_t e = detMat[(uint32_t)rCurr * numDopplerBins + dopIdx];
        uint16_t distFromPeak;

        if ((uint32_t)e < exitThr)
        {
            break;
        }
        if ((dopIdx == 0U) &&
            (staticClutterRemovalEnabled == 0U) &&
            ((uint16_t)(rCurr - rStart) > clutterCap) &&
            ((uint32_t)e < enterThr))
        {
            break;
        }

        if (e > clusterPeakVal)
        {
            clusterPeakVal = e;
            clusterPeakRange = rCurr;
        }

        distFromPeak = (rCurr >= clusterPeakRange)
            ? (uint16_t)(rCurr - clusterPeakRange)
            : (uint16_t)(clusterPeakRange - rCurr);
        if ((distFromPeak >= 2U) &&
            (params[ALG_RG2D_PARAM_SIDELOBE_GUARD_Q8] > 0U) &&
            (((uint32_t)e + params[ALG_RG2D_PARAM_SIDELOBE_GUARD_Q8]) < (uint32_t)clusterPeakVal))
        {
            break;
        }

        if (inValley == 0U)
        {
            if (e >= peakVal)
            {
                peakVal = e;
            }
            else if ((uint16_t)(peakVal - e) >= params[ALG_RG2D_PARAM_PROM_Q8])
            {
                inValley = 1U;
                valleyVal = e;
                valleyIdx = rCurr;
            }
        }
        else
        {
            if (e < valleyVal)
            {
                valleyVal = e;
                valleyIdx = rCurr;
            }
            else if ((uint16_t)(e - valleyVal) >= params[ALG_RG2D_PARAM_RISE_Q8])
            {
                splitAtValley = 1U;
                break;
            }
        }
    }

    if ((splitAtValley != 0U) ||
        ((inValley != 0U) &&
         (peakVal > detMat[(uint32_t)(rCurr - 1U) * numDopplerBins + dopIdx]) &&
         ((uint16_t)(peakVal - detMat[(uint32_t)(rCurr - 1U) * numDopplerBins + dopIdx]) >= params[ALG_RG2D_PARAM_PROM_Q8])))
    {
        rEnd = (uint16_t)(valleyIdx - 1U);
        if ((splitAtValley != 0U) && (rCurr <= maxRange))
        {
            ws->seedMask[rCurr] |= dBit;
        }
    }
    else
    {
        rEnd = (uint16_t)(rCurr - 1U);
    }

    if (rEnd < rStart)
    {
        rEnd = rStart;
    }
    if ((splitAtValley == 0U) && (rEnd < rSeed))
    {
        rEnd = rSeed;
    }

    for (r = rStart; r <= rEnd; r++)
    {
        uint16_t e = detMat[(uint32_t)r * numDopplerBins + dopIdx];
        uint16_t snr = Alg_RegionGrow2D_snrDb10FromQ8(e, anchor);
        uint32_t snrIdx = (uint32_t)r * ws->maxDopplerStride + dopIdx;
        ws->grownMask[r] |= dBit;
        if (snr > ws->cellSnr[snrIdx])
        {
            ws->cellSnr[snrIdx] = snr;
        }
    }

    /* Operator 4: Anisotropic 2D BFS Expansion across Doppler & Sloped Range-Doppler Ridges */
    if ((params[ALG_RG2D_PARAM_MAX_DOPPLER_SPAN] > 0U) &&
        (ws->bfsQueue != NULL) &&
        (ws->queueCapacity > 0U))
    {
        uint16_t qHead;
        uint16_t qTail = 0U;

        for (r = rStart; r <= rEnd; r++)
        {
            if (qTail < ws->queueCapacity)
            {
                ws->bfsQueue[qTail].rangeIdx = r;
                ws->bfsQueue[qTail].dopplerIdx = dopIdx;
                qTail++;
            }
        }

        for (qHead = 0U; qHead < ws->queueCapacity; qHead++)
        {
            Alg_RegionGrow2D_QueueNode curr;
            uint16_t eCurr;
            uint16_t step;

            if (qHead >= qTail)
            {
                break;
            }
            curr = ws->bfsQueue[qHead];
            eCurr = detMat[(uint32_t)curr.rangeIdx * numDopplerBins + curr.dopplerIdx];

            /* A. Circular Doppler Neighbors */
            for (step = 0U; step < 2U; step++)
            {
                uint16_t dNbr = (step == 0U)
                    ? (uint16_t)((curr.dopplerIdx + 1U) % numDopplerBins)
                    : (uint16_t)((curr.dopplerIdx + numDopplerBins - 1U) % numDopplerBins);
                uint16_t dDist = (dNbr >= dopIdx) ? (uint16_t)(dNbr - dopIdx) : (uint16_t)(dopIdx - dNbr);
                uint16_t eDop;
                uint16_t dopAnchor;

                if ((uint16_t)(numDopplerBins - dDist) < dDist)
                {
                    dDist = (uint16_t)(numDopplerBins - dDist);
                }
                if ((dDist > params[ALG_RG2D_PARAM_MAX_DOPPLER_SPAN]) ||
                    (dNbr >= ALG_RG2D_MAX_BITMAP_DOPPLER_BINS))
                {
                    continue;
                }
                if ((ws->grownMask[curr.rangeIdx] & (1UL << dNbr)) != 0U)
                {
                    continue;
                }

                eDop = detMat[(uint32_t)curr.rangeIdx * numDopplerBins + dNbr];
                dopAnchor = Alg_RegionGrow2D_noiseAnchor(
                    detMat, numDopplerBins, minRange, maxRange, curr.rangeIdx, dNbr, params);

                if (((uint32_t)eDop >= ((uint32_t)dopAnchor + params[ALG_RG2D_PARAM_EXIT_Q8])) &&
                    (((uint32_t)eDop + params[ALG_RG2D_PARAM_PROM_Q8]) >= (uint32_t)eCurr) &&
                    ((params[ALG_RG2D_PARAM_SIDELOBE_GUARD_Q8] == 0U) ||
                     (((uint32_t)eDop + params[ALG_RG2D_PARAM_SIDELOBE_GUARD_Q8]) >= (uint32_t)clusterPeakVal)))
                {
                    uint16_t snr = Alg_RegionGrow2D_snrDb10FromQ8(eDop, dopAnchor);
                    uint32_t snrIdx = (uint32_t)curr.rangeIdx * ws->maxDopplerStride + dNbr;
                    ws->grownMask[curr.rangeIdx] |= (1UL << dNbr);
                    if (snr > ws->cellSnr[snrIdx])
                    {
                        ws->cellSnr[snrIdx] = snr;
                    }
                    if (qTail < ws->queueCapacity)
                    {
                        ws->bfsQueue[qTail].rangeIdx = curr.rangeIdx;
                        ws->bfsQueue[qTail].dopplerIdx = dNbr;
                        qTail++;
                    }
                }
            }

            /* B. Sloped Range Continuation on Off-Seed Doppler Rows */
            if (curr.dopplerIdx != dopIdx)
            {
                for (step = 0U; step < 2U; step++)
                {
                    uint16_t rNbr;
                    uint16_t eRng;
                    uint16_t rngAnchor;
                    uint16_t distFromPeak;

                    if (step == 0U)
                    {
                        if (curr.rangeIdx >= rHardMax)
                        {
                            continue;
                        }
                        rNbr = (uint16_t)(curr.rangeIdx + 1U);
                    }
                    else
                    {
                        if (curr.rangeIdx <= rStart)
                        {
                            continue;
                        }
                        rNbr = (uint16_t)(curr.rangeIdx - 1U);
                    }

                    if ((ws->grownMask[rNbr] & (1UL << curr.dopplerIdx)) != 0U)
                    {
                        continue;
                    }

                    eRng = detMat[(uint32_t)rNbr * numDopplerBins + curr.dopplerIdx];
                    rngAnchor = Alg_RegionGrow2D_noiseAnchor(
                        detMat, numDopplerBins, minRange, maxRange, rNbr, curr.dopplerIdx, params);
                    distFromPeak = (rNbr >= clusterPeakRange)
                        ? (uint16_t)(rNbr - clusterPeakRange)
                        : (uint16_t)(clusterPeakRange - rNbr);

                    if (((uint32_t)eRng >= ((uint32_t)rngAnchor + params[ALG_RG2D_PARAM_ENTER_Q8])) &&
                        (((uint32_t)eRng + params[ALG_RG2D_PARAM_PROM_Q8]) >= (uint32_t)eCurr) &&
                        ((params[ALG_RG2D_PARAM_SIDELOBE_GUARD_Q8] == 0U) ||
                         (distFromPeak < 2U) ||
                         (((uint32_t)eRng + params[ALG_RG2D_PARAM_SIDELOBE_GUARD_Q8]) >= (uint32_t)clusterPeakVal)))
                    {
                        uint16_t snr = Alg_RegionGrow2D_snrDb10FromQ8(eRng, rngAnchor);
                        uint32_t snrIdx = (uint32_t)rNbr * ws->maxDopplerStride + curr.dopplerIdx;
                        ws->grownMask[rNbr] |= (1UL << curr.dopplerIdx);
                        if (snr > ws->cellSnr[snrIdx])
                        {
                            ws->cellSnr[snrIdx] = snr;
                        }
                        if (qTail < ws->queueCapacity)
                        {
                            ws->bfsQueue[qTail].rangeIdx = rNbr;
                            ws->bfsQueue[qTail].dopplerIdx = curr.dopplerIdx;
                            qTail++;
                        }
                    }
                }
            }
        }
    }

    return rEnd;
}

void Alg_RegionGrow2D_condenseDopplerRidges(
    const uint16_t *detMat,
    uint16_t numDopplerBins,
    uint16_t minRange,
    uint16_t maxRange,
    const uint16_t params[ALG_RG2D_NUM_PARAMS],
    Alg_RegionGrow2D_Workspace *ws)
{
    uint16_t dopLim;
    uint16_t r;

    if ((detMat == NULL) || (params == NULL) || (ws == NULL) ||
        (ws->grownMask == NULL) || (ws->seedMask == NULL) || (ws->cellSnr == NULL) ||
        (numDopplerBins == 0U) || (params[ALG_RG2D_PARAM_MAX_DOPPLER_SPAN] == 0U))
    {
        return;
    }

    dopLim = (numDopplerBins < ALG_RG2D_MAX_BITMAP_DOPPLER_BINS)
        ? numDopplerBins : ALG_RG2D_MAX_BITMAP_DOPPLER_BINS;

    for (r = minRange; r <= maxRange; r++)
    {
        uint32_t mask = ws->grownMask[r];
        uint32_t seedRow = ws->seedMask[r];
        uint32_t condensed = 0U;
        uint16_t d;

        if (mask == 0U)
        {
            continue;
        }

        for (d = 0U; d < dopLim; d++)
        {
            uint32_t dBit = 1UL << d;
            uint16_t dPrev;
            uint16_t dNext;
            uint16_t e;
            uint8_t hasPrev;
            uint8_t hasNext;
            uint8_t domPrev = 0U;
            uint8_t domNext = 0U;

            if ((mask & dBit) == 0U)
            {
                continue;
            }

            dPrev = (d == 0U) ? (uint16_t)(numDopplerBins - 1U) : (uint16_t)(d - 1U);
            dNext = ((uint16_t)(d + 1U) >= numDopplerBins) ? 0U : (uint16_t)(d + 1U);
            e = detMat[(uint32_t)r * numDopplerBins + d];

            hasPrev = ((dPrev < dopLim) && ((mask & (1UL << dPrev)) != 0U)) ? 1U : 0U;
            hasNext = ((dNext < dopLim) && ((mask & (1UL << dNext)) != 0U)) ? 1U : 0U;

            if (hasPrev != 0U)
            {
                uint16_t ePrev = detMat[(uint32_t)r * numDopplerBins + dPrev];
                if ((ePrev > e) ||
                    ((ePrev == e) &&
                     ((seedRow & dBit) == 0U) &&
                     (((seedRow & (1UL << dPrev)) != 0U) || (dPrev < d))))
                {
                    domPrev = 1U;
                }
            }

            if (hasNext != 0U)
            {
                uint16_t eNext = detMat[(uint32_t)r * numDopplerBins + dNext];
                if ((eNext > e) ||
                    ((eNext == e) &&
                     ((seedRow & dBit) == 0U) &&
                     (((seedRow & (1UL << dNext)) != 0U) || (dNext < d))))
                {
                    domNext = 1U;
                }
            }

            if ((domPrev == 0U) && (domNext == 0U))
            {
                uint16_t maxSkirtSnr = 0U;
                condensed |= dBit;

                if (hasPrev != 0U)
                {
                    uint16_t prevSnr = ws->cellSnr[(uint32_t)r * ws->maxDopplerStride + dPrev];
                    if (prevSnr > maxSkirtSnr)
                    {
                        maxSkirtSnr = prevSnr;
                    }
                }
                if (hasNext != 0U)
                {
                    uint16_t nextSnr = ws->cellSnr[(uint32_t)r * ws->maxDopplerStride + dNext];
                    if (nextSnr > maxSkirtSnr)
                    {
                        maxSkirtSnr = nextSnr;
                    }
                }
                if (maxSkirtSnr > 0U)
                {
                    uint16_t boost = (uint16_t)(maxSkirtSnr >> 2U);
                    if (boost > 15U)
                    {
                        boost = 15U;
                    }
                    if (boost == 0U)
                    {
                        boost = 1U;
                    }
                    ws->cellSnr[(uint32_t)r * ws->maxDopplerStride + d] =
                        (uint16_t)(ws->cellSnr[(uint32_t)r * ws->maxDopplerStride + d] + boost);
                }
            }
        }

        ws->grownMask[r] = condensed;
    }
}

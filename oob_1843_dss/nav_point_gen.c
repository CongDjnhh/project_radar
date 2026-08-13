/**
 * @file nav_point_gen.c
 *
 * @brief Per-frame surface and discrete-object point generation.
 *
 * The source estimator is shared with the TLV11 debug path. This module adds
 * navigation semantics: nearest-surface reduction and rejection of extended
 * azimuth runs before points are offered to GTRACK.
 */

#include <stdint.h>
#include <string.h>
#include <math.h>

#include <ti/utils/cycleprofiler/cycle_profiler.h>

#include "nav_point_gen.h"

#define MMWDEMO_NAV_NUM_ANGLE_CELLS \
    (MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS * \
     MMWDEMO_NAV_NUM_ELEVATION_SECTORS)
#define MMWDEMO_NAV_SELECTION_BYTES \
    ((MMWDEMO_NAV_NUM_ANGLE_CELLS + 7U) / 8U)
#define MMWDEMO_NAV_CANDIDATE_BUFFER_SIZE \
    (sizeof(MmwDemo_DenseElevation_OutputHeader) + \
     MMWDEMO_DENSE_ELEV_MAX_OUTPUT_SAMPLES * \
     sizeof(MmwDemo_DenseElevation_OutputSample))

/* L3 is effectively full in the xWR18xx OOB image. These temporary arrays are
 * active only during inter-frame navigation processing, so place them in the
 * otherwise unused 16-KiB L1D SRAM instead of consuming persistent L3. */
#pragma DATA_SECTION(gNavNearestCandidate, ".navScratch")
#pragma DATA_ALIGN(gNavNearestCandidate, 8)
static int16_t gNavNearestCandidate[MMWDEMO_NAV_NUM_ANGLE_CELLS];

#pragma DATA_SECTION(gNavCellSelected, ".navScratch")
#pragma DATA_ALIGN(gNavCellSelected, 8)
static uint8_t gNavCellSelected[MMWDEMO_NAV_SELECTION_BYTES];

#pragma DATA_SECTION(gNavDiscreteEligible, ".navScratch")
#pragma DATA_ALIGN(gNavDiscreteEligible, 8)
static uint8_t gNavDiscreteEligible[MMWDEMO_NAV_SELECTION_BYTES];

static uint8_t MmwDemo_NavPointGen_isSelected(
    const uint8_t *selection,
    uint16_t index)
{
    return (uint8_t)((selection[index >> 3U] >> (index & 7U)) & 1U);
}

static void MmwDemo_NavPointGen_setSelected(
    uint8_t *selection,
    uint16_t index)
{
    selection[index >> 3U] |= (uint8_t)(1U << (index & 7U));
}

static uint16_t MmwDemo_NavPointGen_abs16(int16_t value)
{
    int32_t expanded = value;
    if (expanded < 0)
    {
        expanded = -expanded;
    }
    return (uint16_t)expanded;
}

static uint16_t MmwDemo_NavPointGen_cellIndex(
    const MmwDemo_DenseElevation_OutputSample *sample)
{
    int16_t az = (int16_t)sample->azimuthIdx +
                 (int16_t)(MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS / 2U);
    int32_t sourceV = sample->sourceVQ15;
    int32_t scaled;
    int16_t el;

    /* Group by the continuous source direction cosine, not by the legacy
     * 16-bin display index. Sixty-four sectors preserve the useful precision
     * inside narrow elevation FOVs while sourceVQ15 remains authoritative. */
    if (sourceV >= 0)
    {
        scaled = (sourceV *
                  (int32_t)(MMWDEMO_NAV_NUM_ELEVATION_SECTORS / 2U) +
                  16383) / 32767;
    }
    else
    {
        scaled = -(((-sourceV) *
                    (int32_t)(MMWDEMO_NAV_NUM_ELEVATION_SECTORS / 2U) +
                    16383) / 32767);
    }
    el = (int16_t)(scaled +
         (int32_t)(MMWDEMO_NAV_NUM_ELEVATION_SECTORS / 2U));

    if (az < 0)
    {
        az = 0;
    }
    else if (az >= (int16_t)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS)
    {
        az = (int16_t)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS - 1;
    }
    if (el < 0)
    {
        el = 0;
    }
    else if (el >= (int16_t)MMWDEMO_NAV_NUM_ELEVATION_SECTORS)
    {
        el = (int16_t)MMWDEMO_NAV_NUM_ELEVATION_SECTORS - 1;
    }

    return (uint16_t)((uint16_t)az *
                      MMWDEMO_NAV_NUM_ELEVATION_SECTORS +
                      (uint16_t)el);
}

static uint8_t MmwDemo_NavPointGen_sector(
    const MmwDemo_DenseElevation_OutputSample *sample)
{
    int16_t az = (int16_t)sample->azimuthIdx +
                 (int16_t)(MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS / 2U);
    uint16_t binsPerSector =
        MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS /
        MMWDEMO_NAV_NUM_AZIMUTH_SECTORS;

    if (az < 0)
    {
        az = 0;
    }
    else if (az >= (int16_t)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS)
    {
        az = (int16_t)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS - 1;
    }
    return (uint8_t)((uint16_t)az / binsPerSector);
}

static void MmwDemo_NavPointGen_copyPoint(
    MmwDemo_NavPoint *dst,
    const MmwDemo_DenseElevation_OutputSample *src)
{
    dst->rangeIdx = src->rangeIdx;
    dst->azimuthIdx = src->azimuthIdx;
    dst->elevationIdx = src->elevationIdx;
    dst->dopplerIdx = src->dopplerIdx;
    dst->snrDb10 = src->snrDb10;
    dst->sourceVQ15 = src->sourceVQ15;
}

static int16_t MmwDemo_NavPointGen_findContinuousNeighbor(
    int16_t az,
    int16_t centerElevationSector,
    const MmwDemo_DenseElevation_OutputSample *previous,
    const MmwDemo_DenseElevation_OutputSample *samples,
    const MmwDemo_NavPointGen_Cfg *cfg)
{
    uint16_t scaledElevationTolerance =
        ((uint16_t)cfg->surfaceElevationNeighborBins *
         MMWDEMO_NAV_NUM_ELEVATION_SECTORS +
         MMWDEMO_DENSE_ELEV_NUM_ELEVATION_BINS - 1U) /
        MMWDEMO_DENSE_ELEV_NUM_ELEVATION_BINS;
    int16_t offset;
    int16_t bestIndex = -1;
    uint32_t bestDelta = 0xFFFFFFFFU;

    if ((az < 0) ||
        (az >= (int16_t)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS))
    {
        return -1;
    }
    for (offset = -(int16_t)scaledElevationTolerance;
         offset <= (int16_t)scaledElevationTolerance;
         offset++)
    {
        int16_t elevationSector = centerElevationSector + offset;
        uint16_t neighborCell;
        int16_t neighborIndex;
        int32_t signedDelta;
        uint32_t rangeDelta;

        if ((elevationSector < 0) ||
            (elevationSector >=
             (int16_t)MMWDEMO_NAV_NUM_ELEVATION_SECTORS))
        {
            continue;
        }
        neighborCell = (uint16_t)((uint16_t)az *
                       MMWDEMO_NAV_NUM_ELEVATION_SECTORS +
                       (uint16_t)elevationSector);
        neighborIndex = gNavNearestCandidate[neighborCell];
        if (neighborIndex < 0)
        {
            continue;
        }
        signedDelta = (int32_t)samples[neighborIndex].rangeIdx -
                      (int32_t)previous->rangeIdx;
        rangeDelta = (signedDelta < 0) ?
                     (uint32_t)(-signedDelta) : (uint32_t)signedDelta;
        if ((rangeDelta <= (uint32_t)cfg->surfaceRangeToleranceBins) &&
            ((bestIndex < 0) || (rangeDelta < bestDelta) ||
             ((rangeDelta == bestDelta) &&
              (samples[neighborIndex].snrDb10 > samples[bestIndex].snrDb10))))
        {
            bestIndex = neighborIndex;
            bestDelta = rangeDelta;
        }
    }
    return bestIndex;
}

/** Returns nonzero only when the center belongs to a genuinely contiguous
 * azimuth run. Each branch follows its previous range/elevation sector so a
 * slanted wall is accepted while isolated points separated by gaps are not. */
static uint8_t MmwDemo_NavPointGen_isExtendedSurface(
    uint16_t cell,
    const MmwDemo_DenseElevation_OutputSample *samples,
    const MmwDemo_NavPointGen_Cfg *cfg)
{
    int16_t centerAz = (int16_t)(cell /
                       MMWDEMO_NAV_NUM_ELEVATION_SECTORS);
    int16_t centerElevation = (int16_t)(cell %
                              MMWDEMO_NAV_NUM_ELEVATION_SECTORS);
    int16_t centerIndex = gNavNearestCandidate[cell];
    uint16_t contiguousCount = 1U;
    int16_t direction;

    if ((centerIndex < 0) || (cfg->surfaceMinRunLength <= 1U))
    {
        return 0U;
    }

    for (direction = -1; direction <= 1; direction += 2)
    {
        const MmwDemo_DenseElevation_OutputSample *previous =
            &samples[centerIndex];
        int16_t previousElevation = centerElevation;
        uint16_t step;

        for (step = 1U; step < cfg->surfaceMinRunLength; step++)
        {
            int16_t neighborIndex =
                MmwDemo_NavPointGen_findContinuousNeighbor(
                    centerAz + direction * (int16_t)step,
                    previousElevation, previous, samples, cfg);
            if (neighborIndex < 0)
            {
                break;
            }
            previous = &samples[neighborIndex];
            previousElevation = (int16_t)(
                MmwDemo_NavPointGen_cellIndex(previous) %
                MMWDEMO_NAV_NUM_ELEVATION_SECTORS);
            contiguousCount++;
            if (contiguousCount >= cfg->surfaceMinRunLength)
            {
                return 1U;
            }
        }
    }
    return 0U;
}

static uint8_t MmwDemo_NavPointGen_isHigherPriority(
    const MmwDemo_DenseElevation_OutputSample *candidate,
    const MmwDemo_DenseElevation_OutputSample *current)
{
    if (current == NULL)
    {
        return 1U;
    }
    if (candidate->rangeIdx != current->rangeIdx)
    {
        return (candidate->rangeIdx < current->rangeIdx) ? 1U : 0U;
    }
    if (candidate->snrDb10 != current->snrDb10)
    {
        return (candidate->snrDb10 > current->snrDb10) ? 1U : 0U;
    }
    return (MmwDemo_NavPointGen_abs16(candidate->dopplerIdx) >
            MmwDemo_NavPointGen_abs16(current->dopplerIdx)) ? 1U : 0U;
}

static int16_t MmwDemo_NavPointGen_findBestUnselected(
    const MmwDemo_DenseElevation_OutputSample *samples,
    int16_t requiredSector,
    const uint8_t *eligibility)
{
    uint16_t cell;
    int16_t bestCell = -1;
    const MmwDemo_DenseElevation_OutputSample *best = NULL;

    for (cell = 0U; cell < MMWDEMO_NAV_NUM_ANGLE_CELLS; cell++)
    {
        int16_t sampleIndex = gNavNearestCandidate[cell];
        if ((sampleIndex < 0) ||
            (MmwDemo_NavPointGen_isSelected(gNavCellSelected, cell) != 0U) ||
            ((eligibility != NULL) &&
             (MmwDemo_NavPointGen_isSelected(eligibility, cell) == 0U)) ||
            ((requiredSector >= 0) &&
             ((int16_t)MmwDemo_NavPointGen_sector(&samples[sampleIndex]) !=
              requiredSector)))
        {
            continue;
        }
        if (MmwDemo_NavPointGen_isHigherPriority(
                &samples[sampleIndex], best) != 0U)
        {
            best = &samples[sampleIndex];
            bestCell = (int16_t)cell;
        }
    }
    return bestCell;
}

static uint16_t MmwDemo_NavPointGen_emitPrioritized(
    const MmwDemo_DenseElevation_OutputSample *samples,
    const uint8_t *eligibility,
    uint16_t maxOutput,
    MmwDemo_NavPoint *output)
{
    uint16_t count = 0U;
    uint8_t sector;

    memset(gNavCellSelected, 0, sizeof(gNavCellSelected));

    /* Enforce angular coverage first. Within every sector, retain the nearest
     * hazards and break equal-range ties by SNR instead of array index. */
    for (sector = 0U;
         (sector < MMWDEMO_NAV_NUM_AZIMUTH_SECTORS) &&
         (count < maxOutput);
         sector++)
    {
        uint16_t quota = maxOutput / MMWDEMO_NAV_NUM_AZIMUTH_SECTORS;
        uint16_t emittedInSector = 0U;
        if (sector < (maxOutput % MMWDEMO_NAV_NUM_AZIMUTH_SECTORS))
        {
            quota++;
        }

        while ((emittedInSector < quota) && (count < maxOutput))
        {
            int16_t bestCell = MmwDemo_NavPointGen_findBestUnselected(
                samples, (int16_t)sector, eligibility);
            int16_t candidateIndex;
            if (bestCell < 0)
            {
                break;
            }
            candidateIndex = gNavNearestCandidate[bestCell];
            MmwDemo_NavPointGen_copyPoint(&output[count],
                                           &samples[candidateIndex]);
            MmwDemo_NavPointGen_setSelected(gNavCellSelected,
                                             (uint16_t)bestCell);
            count++;
            emittedInSector++;
        }
    }

    /* Reuse quota left by sparse sectors, still ordered globally by safety
     * priority rather than signed angle/index. */
    while (count < maxOutput)
    {
        int16_t bestCell = MmwDemo_NavPointGen_findBestUnselected(
            samples, -1, eligibility);
        int16_t candidateIndex;
        if (bestCell < 0)
        {
            break;
        }
        candidateIndex = gNavNearestCandidate[bestCell];
        MmwDemo_NavPointGen_copyPoint(&output[count],
                                       &samples[candidateIndex]);
        MmwDemo_NavPointGen_setSelected(gNavCellSelected,
                                         (uint16_t)bestCell);
        count++;
    }
    return count;
}

static uint16_t MmwDemo_NavPointGen_emitSurface(
    const MmwDemo_DenseElevation_OutputSample *samples,
    uint16_t maxOutput,
    MmwDemo_NavPoint *output,
    uint16_t *availableCount)
{
    uint16_t cell;
    uint16_t total = 0U;

    for (cell = 0U; cell < MMWDEMO_NAV_NUM_ANGLE_CELLS; cell++)
    {
        if (gNavNearestCandidate[cell] >= 0)
        {
            total++;
        }
    }
    *availableCount = total;
    return MmwDemo_NavPointGen_emitPrioritized(
        samples, NULL, maxOutput, output);
}

static int32_t MmwDemo_NavPointGen_copyDebug(
    const MmwDemo_DenseElevation_OutputHeader *candidateHeader,
    uint8_t *debugBuffer,
    uint32_t debugBufferSize,
    uint16_t debugMaxSamples,
    uint32_t *debugSize)
{
    MmwDemo_DenseElevation_OutputHeader *debugHeader;
    uint16_t copyCount = candidateHeader->numSamples;
    uint32_t capacity;

    *debugSize = 0U;
    if ((debugBuffer == NULL) ||
        (debugBufferSize < sizeof(MmwDemo_DenseElevation_OutputHeader)))
    {
        return MMWDEMO_NAV_EOUTPUT_CAPACITY;
    }
    capacity = (debugBufferSize - sizeof(*candidateHeader)) /
               sizeof(MmwDemo_DenseElevation_OutputSample);
    if (copyCount > debugMaxSamples)
    {
        copyCount = debugMaxSamples;
    }
    if (copyCount > capacity)
    {
        copyCount = (uint16_t)capacity;
    }

    memcpy(debugBuffer, candidateHeader, sizeof(*candidateHeader));
    debugHeader = (MmwDemo_DenseElevation_OutputHeader *)debugBuffer;
    debugHeader->numSamples = copyCount;
    if (copyCount < candidateHeader->numSamples)
    {
        debugHeader->flags |= MMWDEMO_DENSE_FLAG_OUTPUT_LIMITED;
    }
    if (copyCount != 0U)
    {
        memcpy(debugBuffer + sizeof(*candidateHeader),
               (const uint8_t *)candidateHeader + candidateHeader->headerSize,
               (uint32_t)copyCount *
               sizeof(MmwDemo_DenseElevation_OutputSample));
    }
    *debugSize = sizeof(*candidateHeader) +
                 (uint32_t)copyCount *
                 sizeof(MmwDemo_DenseElevation_OutputSample);
    return 0;
}

int32_t MmwDemo_NavPointGen_process(
    const MmwDemo_NavPointGen_Cfg *cfg,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
    uint32_t cycleBudget,
    uint8_t debugEnabled,
    uint8_t *debugBuffer,
    uint32_t debugBufferSize,
    uint16_t debugMaxSamples,
    uint32_t *debugSize,
    uint8_t *navBuffer,
    uint32_t navBufferSize,
    uint32_t *navSize)
{
    MmwDemo_DenseElevation_OutputHeader *candidateHeader;
    MmwDemo_DenseElevation_OutputSample *samples;
    MmwDemo_NavFrameHeader *navHeader;
    MmwDemo_NavPoint *surfaceOutput;
    MmwDemo_NavPoint *discreteOutput;
    uint32_t candidateSize = 0U;
    uint32_t startCycles;
    uint32_t requiredCapacity;
    uint16_t sampleIndex;
    uint16_t cell;
    uint16_t surfaceAvailable = 0U;
    uint16_t discreteAvailable = 0U;
    uint16_t maxSurface;
    uint16_t maxDiscrete;
    uint16_t flags;
    int32_t retVal;

    if (debugSize != NULL)
    {
        *debugSize = 0U;
    }
    if (navSize != NULL)
    {
        *navSize = 0U;
    }
    if ((cfg == NULL) || (staticCfg == NULL) || (navBuffer == NULL) ||
        (debugBuffer == NULL) ||
        (debugBufferSize < MMWDEMO_NAV_CANDIDATE_BUFFER_SIZE) ||
        (navSize == NULL) || (debugSize == NULL) ||
        (navBufferSize < sizeof(MmwDemo_NavFrameHeader)) ||
        (cfg->maxSurfacePoints == 0U) ||
        (cfg->uartMaxSurfacePoints == 0U) ||
        (cfg->maxDiscretePoints == 0U) ||
        (cfg->maxSurfacePoints > MMWDEMO_NAV_MAX_SURFACE_POINTS) ||
        (cfg->uartMaxSurfacePoints > MMWDEMO_NAV_MAX_SURFACE_POINTS) ||
        (cfg->maxDiscretePoints > MMWDEMO_NAV_MAX_DISCRETE_POINTS) ||
        !(cfg->minMovingSpeedMps >= 0.0f) ||
        !(staticCfg->rangeStepMeters > 0.0f) ||
        !(staticCfg->dopplerStepMps > 0.0f))
    {
        return MMWDEMO_NAV_EINVAL;
    }

    maxSurface = (cfg->uartMaxSurfacePoints < cfg->maxSurfacePoints) ?
                 cfg->uartMaxSurfacePoints : cfg->maxSurfacePoints;
    maxDiscrete = cfg->maxDiscretePoints;
    requiredCapacity = sizeof(MmwDemo_NavFrameHeader) +
        ((uint32_t)maxSurface + (uint32_t)maxDiscrete) *
        sizeof(MmwDemo_NavPoint);
    if (requiredCapacity > navBufferSize)
    {
        return MMWDEMO_NAV_EOUTPUT_CAPACITY;
    }

    startCycles = Cycleprofiler_getTimeStamp();
    retVal = MmwDemo_DenseElevation_process(
        &cfg->sourceCfg,
        &staticCfg->sourceStaticCfg,
        cycleBudget,
        debugBuffer,
        debugBufferSize,
        MMWDEMO_DENSE_ELEV_MAX_OUTPUT_SAMPLES,
        &candidateSize);
    if (retVal != 0)
    {
        return retVal;
    }

    /* The dense/debug buffer is scratch until navigation has copied the
     * selected points. A separately enabled TLV11 pass may safely overwrite
     * it afterwards, avoiding a duplicate 15-KiB L3 candidate buffer. */
    candidateHeader =
        (MmwDemo_DenseElevation_OutputHeader *)debugBuffer;
    if ((candidateSize < sizeof(*candidateHeader)) ||
        (candidateHeader->magic != MMWDEMO_DENSE_ELEVATION_MAGIC) ||
        (candidateHeader->version !=
         MMWDEMO_DENSE_ELEVATION_PROTOCOL_VERSION) ||
        (candidateHeader->headerSize != sizeof(*candidateHeader)) ||
        (candidateHeader->sampleSize !=
         sizeof(MmwDemo_DenseElevation_OutputSample)) ||
        (candidateHeader->numSamples >
         MMWDEMO_DENSE_ELEV_MAX_OUTPUT_SAMPLES) ||
        (candidateSize != sizeof(*candidateHeader) +
         (uint32_t)candidateHeader->numSamples *
         sizeof(MmwDemo_DenseElevation_OutputSample)))
    {
        return MMWDEMO_NAV_ECANDIDATE_PAYLOAD;
    }

    samples = (MmwDemo_DenseElevation_OutputSample *)(
        debugBuffer + candidateHeader->headerSize);
    for (cell = 0U; cell < MMWDEMO_NAV_NUM_ANGLE_CELLS; cell++)
    {
        gNavNearestCandidate[cell] = -1;
    }
    memset(gNavDiscreteEligible, 0, sizeof(gNavDiscreteEligible));

    /* One nearest return per azimuth/elevation ray. At equal range, retain
     * the higher-SNR source. */
    for (sampleIndex = 0U;
         sampleIndex < candidateHeader->numSamples;
         sampleIndex++)
    {
        int16_t currentIndex;
        cell = MmwDemo_NavPointGen_cellIndex(&samples[sampleIndex]);
        currentIndex = gNavNearestCandidate[cell];
        if ((currentIndex < 0) ||
            (samples[sampleIndex].rangeIdx < samples[currentIndex].rangeIdx) ||
            ((samples[sampleIndex].rangeIdx ==
              samples[currentIndex].rangeIdx) &&
             (samples[sampleIndex].snrDb10 >
              samples[currentIndex].snrDb10)))
        {
            gNavNearestCandidate[cell] = (int16_t)sampleIndex;
        }
    }

    /* Select tracker input only from nearest rays. Extended surfaces are
     * deliberately excluded even if platform motion gives them Doppler. */
    flags = MMWDEMO_NAV_FLAG_FRAME_COMPLETE |
            MMWDEMO_NAV_FLAG_NO_EGO_COMPENSATION |
            MMWDEMO_NAV_FLAG_ELEVATION_FILTERED;
    if ((candidateHeader->flags &
         MMWDEMO_DENSE_FLAG_TIME_BUDGET_EXCEEDED) != 0U)
    {
        flags |= MMWDEMO_NAV_FLAG_TIME_BUDGET_EXCEEDED;
    }
    if (maxSurface < cfg->maxSurfacePoints)
    {
        flags |= MMWDEMO_NAV_FLAG_UART_LIMITED;
    }
    for (cell = 0U; cell < MMWDEMO_NAV_NUM_ANGLE_CELLS; cell++)
    {
        int16_t nearestIndex = gNavNearestCandidate[cell];
        if (nearestIndex >= 0)
        {
            float speed = (float)MmwDemo_NavPointGen_abs16(
                              samples[nearestIndex].dopplerIdx) *
                          staticCfg->dopplerStepMps;
            if (speed >= cfg->minMovingSpeedMps)
            {
                if (MmwDemo_NavPointGen_isExtendedSurface(
                        cell, samples, cfg) != 0U)
                {
                    flags |= MMWDEMO_NAV_FLAG_EXTENDED_REJECTED;
                }
                else
                {
                    MmwDemo_NavPointGen_setSelected(
                        gNavDiscreteEligible, cell);
                    discreteAvailable++;
                }
            }
        }
    }

    memset(navBuffer, 0, sizeof(MmwDemo_NavFrameHeader));
    navHeader = (MmwDemo_NavFrameHeader *)navBuffer;
    surfaceOutput = (MmwDemo_NavPoint *)(navBuffer + sizeof(*navHeader));
    navHeader->numSurfacePoints = MmwDemo_NavPointGen_emitSurface(
        samples, maxSurface, surfaceOutput, &surfaceAvailable);
    discreteOutput = surfaceOutput + navHeader->numSurfacePoints;
    navHeader->numDiscretePoints = MmwDemo_NavPointGen_emitPrioritized(
        samples, gNavDiscreteEligible, maxDiscrete, discreteOutput);

    if (surfaceAvailable > navHeader->numSurfacePoints)
    {
        flags |= MMWDEMO_NAV_FLAG_SURFACE_LIMITED;
    }
    if (discreteAvailable > navHeader->numDiscretePoints)
    {
        flags |= MMWDEMO_NAV_FLAG_DISCRETE_LIMITED;
    }
    if (debugEnabled != 0U)
    {
        retVal = MmwDemo_NavPointGen_copyDebug(
            candidateHeader, debugBuffer, debugBufferSize,
            debugMaxSamples, debugSize);
        if (retVal != 0)
        {
            return retVal;
        }
        flags |= MMWDEMO_NAV_FLAG_DEBUG_COPIED;
    }

    navHeader->magic = MMWDEMO_NAV_MAGIC;
    navHeader->version = MMWDEMO_NAV_PROTOCOL_VERSION;
    navHeader->headerSize = sizeof(*navHeader);
    navHeader->pointSize = sizeof(MmwDemo_NavPoint);
    navHeader->flags = flags;
    navHeader->subFrameNumber = candidateHeader->subFrameNumber;
    navHeader->numAzimuthBins = candidateHeader->numAzimuthBins;
    navHeader->numElevationBins = candidateHeader->numElevationBins;
    navHeader->numRangeBins = candidateHeader->numRangeBins;
    navHeader->numDopplerBins = candidateHeader->numDopplerBins;
    navHeader->rangeStart = candidateHeader->rangeStart;
    navHeader->rangeEnd = candidateHeader->rangeEnd;
    navHeader->protocolStatus = candidateHeader->protocolStatus;
    navHeader->radarFrameNumber = candidateHeader->radarFrameNumber;
    navHeader->rangeStepMeters = staticCfg->rangeStepMeters;
    navHeader->dopplerStepMps = staticCfg->dopplerStepMps;
    navHeader->framePeriodSec = staticCfg->framePeriodSec;
    navHeader->elevationFovDeg10 =
        (uint32_t)(uint16_t)candidateHeader->minElevationDeg10 |
        ((uint32_t)(uint16_t)candidateHeader->maxElevationDeg10 << 16U);
    navHeader->rangeBiasMeters = staticCfg->rangeBiasMeters;
    navHeader->cycleBudget = cycleBudget;
    navHeader->processingCycles =
        Cycleprofiler_getTimeStamp() - startCycles;
    if ((cycleBudget != 0U) &&
        (navHeader->processingCycles > cycleBudget))
    {
        flags |= MMWDEMO_NAV_FLAG_TIME_BUDGET_EXCEEDED;
    }
    navHeader->flags = flags;

    *navSize = sizeof(*navHeader) +
        ((uint32_t)navHeader->numSurfacePoints +
         (uint32_t)navHeader->numDiscretePoints) *
        sizeof(MmwDemo_NavPoint);
    return 0;
}

void MmwDemo_NavPointGen_reset(void)
{
    memset(gNavNearestCandidate, 0xFF, sizeof(gNavNearestCandidate));
    memset(gNavCellSelected, 0, sizeof(gNavCellSelected));
    memset(gNavDiscreteEligible, 0, sizeof(gNavDiscreteEligible));
}

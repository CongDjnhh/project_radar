/**
 * @file nav_point_gen.c
 *
 * @brief Per-frame surface point generation and nearest-surface reduction.
 *
 * The dense source estimator provides candidate points for nearest-surface
 * angular ray reduction and prioritized point selection.
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
typedef struct MmwDemo_NavScratch_t
{
    int16_t nearestCandidate[MMWDEMO_NAV_NUM_ANGLE_CELLS];
} MmwDemo_NavScratch;

typedef char MmwDemo_NavScratchSizeCheck[
    (sizeof(MmwDemo_NavScratch) <=
     (MMWDEMO_NAV_NUM_ANGLE_CELLS * sizeof(int16_t))) ? 1 : -1];

#pragma DATA_SECTION(gNavScratch, ".navScratch")
#pragma DATA_ALIGN(gNavScratch, 8)
static MmwDemo_NavScratch gNavScratch;

#pragma DATA_SECTION(gNavCellSelected, ".navScratch")
#pragma DATA_ALIGN(gNavCellSelected, 8)
static uint8_t gNavCellSelected[MMWDEMO_NAV_SELECTION_BYTES];

#define gNavNearestCandidate (gNavScratch.nearestCandidate)

/* Cycle profiling for emitSurface (includes emitPrioritized + findBestUnselected).
 * Read via CCS watch to assess O(N*M) cost per review point #5. */
volatile uint32_t gNavEmitSurfaceCycles[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
volatile uint32_t gNavEmitSurfaceMaxCycles[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];

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
    DPIF_PointCloudCartesian *dstPoint,
    DPIF_PointCloudSideInfo *dstSideInfo,
    const MmwDemo_DenseElevation_OutputSample *src,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg)
{
    float dist = (float)src->rangeIdx * staticCfg->rangeStepMeters + staticCfg->rangeBiasMeters;
    if (dist < 0.0f)
    {
        dist = 0.0f;
    }

    float u = (float)src->azimuthUQ15 / 32767.0f;
    if (u > 1.0f) u = 1.0f;
    if (u < -1.0f) u = -1.0f;

    float v = (float)src->sourceVQ15 / 32767.0f;
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;

    float x = dist * u;
    float z = dist * v;
    float ySq = (dist * dist) - (x * x) - (z * z);
    float y = (ySq > 0.0f) ? sqrtf(ySq) : 0.0f;

    if (dstPoint != NULL)
    {
        dstPoint->x = x;
        dstPoint->y = y;
        dstPoint->z = z;
        dstPoint->velocity = (float)src->dopplerIdx * staticCfg->dopplerStepMps;
    }

    if (dstSideInfo != NULL)
    {
        float width_u = (float)src->azimuthWidthUQ15 / 32767.0f;
        float width = dist * width_u;
        if (width < 0.05f)
        {
            width = 0.05f;
        }

        if (src->powerDb10 != (int16_t)-32768)
        {
            dstSideInfo->snr = src->powerDb10;
        }
        else
        {
            dstSideInfo->snr = (int16_t)src->snrDb10;
        }
        /* Preserve physical obstacle width in mm in sideInfo->noise */
        dstSideInfo->noise = (int16_t)(width * 1000.0f);
    }
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
    uint16_t sampleCount,
    int16_t requiredSector)
{
    uint16_t sampleIndex;
    int16_t bestCell = -1;
    const MmwDemo_DenseElevation_OutputSample *best = NULL;

    /* Iterate only occupied source samples. The nearest-candidate equality
     * removes duplicates without rescanning all 4096 angle cells per output. */
    for (sampleIndex = 0U; sampleIndex < sampleCount; sampleIndex++)
    {
        uint16_t cell = MmwDemo_NavPointGen_cellIndex(&samples[sampleIndex]);
        if ((gNavNearestCandidate[cell] != (int16_t)sampleIndex) ||
            (MmwDemo_NavPointGen_isSelected(gNavCellSelected, cell) != 0U) ||
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
    uint16_t sampleCount,
    uint16_t maxOutput,
    DPIF_PointCloudCartesian *outPoints,
    DPIF_PointCloudSideInfo *outSideInfo,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg)
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
            int16_t bestCell;
            int16_t candidateIndex;
            bestCell = MmwDemo_NavPointGen_findBestUnselected(
                samples, sampleCount, (int16_t)sector);
            if (bestCell < 0)
            {
                break;
            }
            candidateIndex = gNavNearestCandidate[bestCell];
            MmwDemo_NavPointGen_copyPoint(
                &outPoints[count],
                (outSideInfo != NULL) ? &outSideInfo[count] : NULL,
                &samples[candidateIndex], staticCfg);
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
        int16_t bestCell;
        int16_t candidateIndex;
        bestCell = MmwDemo_NavPointGen_findBestUnselected(
            samples, sampleCount, -1);
        if (bestCell < 0)
        {
            break;
        }
        candidateIndex = gNavNearestCandidate[bestCell];
        MmwDemo_NavPointGen_copyPoint(
            &outPoints[count],
            (outSideInfo != NULL) ? &outSideInfo[count] : NULL,
            &samples[candidateIndex], staticCfg);
        MmwDemo_NavPointGen_setSelected(gNavCellSelected,
                                         (uint16_t)bestCell);
        count++;
    }
    return count;
}

static uint16_t MmwDemo_NavPointGen_emitSurface(
    const MmwDemo_DenseElevation_OutputSample *samples,
    uint16_t sampleCount,
    uint16_t maxOutput,
    DPIF_PointCloudCartesian *outPoints,
    DPIF_PointCloudSideInfo *outSideInfo,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
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
        samples, sampleCount, maxOutput, outPoints, outSideInfo, staticCfg);
}

static int32_t MmwDemo_NavPointGen_buildNearestMap(
    const MmwDemo_DenseElevation_OutputSample *samples,
    uint16_t sampleCount)
{
    uint16_t cell;
    uint16_t sampleIndex;

    for (cell = 0U; cell < MMWDEMO_NAV_NUM_ANGLE_CELLS; cell++)
    {
        gNavNearestCandidate[cell] = -1;
    }
    for (sampleIndex = 0U; sampleIndex < sampleCount; sampleIndex++)
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
    return 0;
}

int32_t MmwDemo_NavPointGen_process(
    const MmwDemo_NavPointGen_Cfg *cfg,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
    uint32_t deadlineStartCycles,
    uint32_t deadlineOffsetCycles,
    uint32_t cycleBudget,
    uint8_t *candidateBuffer,
    uint32_t candidateBufferSize,
    DPIF_PointCloudCartesian *outPoints,
    DPIF_PointCloudSideInfo *outSideInfo,
    uint16_t maxOutputPoints,
    uint32_t *numPointsOut)
{
    MmwDemo_DenseElevation_OutputHeader *candidateHeader;
    MmwDemo_DenseElevation_OutputSample *samples;
    uint32_t candidateSize = 0U;
    uint32_t emitSurfaceStartCycles;
    uint16_t surfaceAvailable = 0U;
    uint16_t maxSurface;
    int32_t retVal;

    if (numPointsOut != NULL)
    {
        *numPointsOut = 0U;
    }
    if ((cfg == NULL) || (staticCfg == NULL) || (outPoints == NULL) ||
        (candidateBuffer == NULL) ||
        (candidateBufferSize < MMWDEMO_NAV_CANDIDATE_BUFFER_SIZE) ||
        (numPointsOut == NULL) ||
        (maxOutputPoints == 0U) ||
        (cfg->maxSurfacePoints == 0U) ||
        (!isfinite(staticCfg->rangeStepMeters)) ||
        !(staticCfg->rangeStepMeters > 0.0f) ||
        (!isfinite(staticCfg->dopplerStepMps)) ||
        (!isfinite(staticCfg->framePeriodSec)) ||
        (!isfinite(staticCfg->rangeBiasMeters)) ||
        (staticCfg->sourceStaticCfg.subFrameNum >=
         MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES) ||
        !(staticCfg->dopplerStepMps > 0.0f))
    {
        return MMWDEMO_NAV_EINVAL;
    }

    maxSurface = (cfg->maxSurfacePoints < maxOutputPoints) ?
                 cfg->maxSurfacePoints : maxOutputPoints;
    if ((cfg->uartMaxSurfacePoints != 0U) &&
        (cfg->uartMaxSurfacePoints < maxSurface))
    {
        maxSurface = cfg->uartMaxSurfacePoints;
    }

    retVal = MmwDemo_DenseElevation_process(
        &cfg->sourceCfg,
        &staticCfg->sourceStaticCfg,
        deadlineStartCycles,
        deadlineOffsetCycles,
        cycleBudget,
        candidateBuffer,
        candidateBufferSize,
        MMWDEMO_DENSE_ELEV_MAX_OUTPUT_SAMPLES,
        &candidateSize);
    if (retVal != 0)
    {
        return retVal;
    }

    /* The dense candidate buffer is scratch until navigation has copied the
     * selected points into the frame output. */
    candidateHeader =
        (MmwDemo_DenseElevation_OutputHeader *)candidateBuffer;
    if ((candidateSize < sizeof(*candidateHeader)) ||
        (candidateHeader->magic != MMWDEMO_DENSE_ELEVATION_MAGIC) ||
        (candidateHeader->version !=
         MMWDEMO_DENSE_ELEVATION_PROTOCOL_VERSION) ||
        (candidateHeader->headerSize != sizeof(*candidateHeader)) ||
         (candidateHeader->sampleSize !=
          sizeof(MmwDemo_DenseElevation_OutputSample)) ||
        (candidateHeader->protocolStatus != 0U) ||
        ((candidateHeader->flags & MMWDEMO_DENSE_REQUIRED_FLAGS) !=
         MMWDEMO_DENSE_REQUIRED_FLAGS) ||
        (candidateHeader->numSamples >
         MMWDEMO_DENSE_ELEV_MAX_OUTPUT_SAMPLES) ||
        (candidateSize != sizeof(*candidateHeader) +
         (uint32_t)candidateHeader->numSamples *
         sizeof(MmwDemo_DenseElevation_OutputSample)))
    {
        return MMWDEMO_NAV_ECANDIDATE_PAYLOAD;
    }

    samples = (MmwDemo_DenseElevation_OutputSample *)(
        candidateBuffer + candidateHeader->headerSize);
    /* One nearest return per azimuth/elevation ray. At equal range, retain
     * the higher-SNR source. */
    retVal = MmwDemo_NavPointGen_buildNearestMap(
        samples, candidateHeader->numSamples);
    if (retVal != 0)
    {
        return retVal;
    }

    emitSurfaceStartCycles = Cycleprofiler_getTimeStamp();
    *numPointsOut = (uint32_t)MmwDemo_NavPointGen_emitSurface(
        samples, candidateHeader->numSamples, maxSurface,
        outPoints, outSideInfo, staticCfg,
        &surfaceAvailable);

    {
        uint32_t emitElapsed = Cycleprofiler_getTimeStamp() -
                               emitSurfaceStartCycles;
        uint16_t sfIdx = candidateHeader->subFrameNumber;
        gNavEmitSurfaceCycles[sfIdx] = emitElapsed;
        if (emitElapsed > gNavEmitSurfaceMaxCycles[sfIdx])
        {
            gNavEmitSurfaceMaxCycles[sfIdx] = emitElapsed;
        }
    }

    return 0;
}

void MmwDemo_NavPointGen_reset(void)
{
    memset(&gNavScratch, 0, sizeof(gNavScratch));
    memset(gNavCellSelected, 0, sizeof(gNavCellSelected));
}

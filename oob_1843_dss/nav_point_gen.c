/**
 * @file nav_point_gen.c
 *
 * @brief Per-frame surface and discrete-object point generation.
 *
 * The dense source estimator is internal scratch for nearest-surface reduction.
 * A bounded Cartesian DBSCAN then separates compact obstacle clusters from
 * wall-like surfaces before points reach GTRACK.
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
typedef union MmwDemo_NavScratch_u
{
    int16_t nearestCandidate[MMWDEMO_NAV_NUM_ANGLE_CELLS];
    struct
    {
        float x[MMWDEMO_NAV_DBSCAN_MAX_POINTS];
        float y[MMWDEMO_NAV_DBSCAN_MAX_POINTS];
        float z[MMWDEMO_NAV_DBSCAN_MAX_POINTS];
        float compensatedDoppler[MMWDEMO_NAV_DBSCAN_MAX_POINTS];
        uint16_t queue[MMWDEMO_NAV_DBSCAN_MAX_POINTS];
        uint8_t label[MMWDEMO_NAV_DBSCAN_MAX_POINTS];
    } cluster;
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

#define MMWDEMO_NAV_DBSCAN_UNVISITED (0U)
#define MMWDEMO_NAV_DBSCAN_WALL      (253U)
#define MMWDEMO_NAV_DBSCAN_NOISE     (254U)

volatile uint32_t gNavDbscanProcessCount = 0U;
volatile uint32_t gNavDbscanLimitedCount = 0U;
volatile uint32_t gNavDbscanWallClusterCount = 0U;
volatile uint32_t gNavEgoValidCount = 0U;
volatile uint32_t gNavEgoLowConfidenceCount = 0U;

/* Cycle profiling for emitSurface (includes emitPrioritized + findBestUnselected).
 * Read via CCS watch to assess O(N*M) cost per review point #5. */
volatile uint32_t gNavEmitSurfaceCycles[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
volatile uint32_t gNavEmitSurfaceMaxCycles[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];

/* External ego freshness is tracked per subframe against the radar frame
 * number. Counting process calls would expire too quickly with advanced-frame
 * configurations and would fail to expire across skipped navigation frames. */
static uint32_t
    gNavLastEgoSequence[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
static uint32_t
    gNavExternalEgoUpdateFrame[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
static uint8_t
    gNavExternalEgoHasFrame[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];

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
    const MmwDemo_DenseElevation_OutputSample *src,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg)
{
    float dist = (float)src->rangeIdx * staticCfg->rangeStepMeters + staticCfg->rangeBiasMeters;
    
    float u = (float)src->azimuthUQ15 / 32767.0f;
    if (u > 1.0f) u = 1.0f;
    if (u < -1.0f) u = -1.0f;
    float az = asinf(u) * (180.0f / 3.1415926535f);

    float v = (float)src->sourceVQ15 / 32767.0f;
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    float elev = asinf(v) * (180.0f / 3.1415926535f);
    
    float width_u = (float)src->azimuthWidthUQ15 / 32767.0f;
    float width = dist * width_u;
    if (width < 0.05f) width = 0.05f;

    float power;
    if (src->powerDb10 != (int16_t)-32768)
    {
        power = (float)src->powerDb10 / 10.0f;
    }
    else
    {
        power = (float)src->snrDb10 / 10.0f;
    }

    dst->distance_m = dist;
    dst->width_m = width;
    dst->azimuth_deg = az;
    dst->elevation_deg = elev;
    dst->power_db = power;
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

static uint8_t MmwDemo_NavPointGen_deadlineExpired(
    uint32_t deadlineStartCycles,
    uint32_t deadlineOffsetCycles)
{
    return ((deadlineOffsetCycles != 0U) &&
            ((Cycleprofiler_getTimeStamp() - deadlineStartCycles) >=
             deadlineOffsetCycles))
               ? 1U : 0U;
}

static int16_t MmwDemo_NavPointGen_findBestUnselected(
    const MmwDemo_DenseElevation_OutputSample *samples,
    uint16_t sampleCount,
    int16_t requiredSector,
    const uint8_t *eligibility)
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
    uint16_t sampleCount,
    const uint8_t *eligibility,
    uint16_t maxOutput,
    MmwDemo_NavPoint *output,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
    uint32_t deadlineStartCycles,
    uint32_t deadlineOffsetCycles,
    uint8_t *timeLimited)
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
            if (MmwDemo_NavPointGen_deadlineExpired(
                    deadlineStartCycles, deadlineOffsetCycles) != 0U)
            {
                *timeLimited = 1U;
                return count;
            }
            bestCell = MmwDemo_NavPointGen_findBestUnselected(
                samples, sampleCount, (int16_t)sector, eligibility);
            if (bestCell < 0)
            {
                break;
            }
            candidateIndex = gNavNearestCandidate[bestCell];
            MmwDemo_NavPointGen_copyPoint(&output[count],
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
        if (MmwDemo_NavPointGen_deadlineExpired(
                deadlineStartCycles, deadlineOffsetCycles) != 0U)
        {
            *timeLimited = 1U;
            return count;
        }
        bestCell = MmwDemo_NavPointGen_findBestUnselected(
            samples, sampleCount, -1, eligibility);
        if (bestCell < 0)
        {
            break;
        }
        candidateIndex = gNavNearestCandidate[bestCell];
        MmwDemo_NavPointGen_copyPoint(&output[count],
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
    MmwDemo_NavPoint *output,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
    uint16_t *availableCount,
    uint32_t deadlineStartCycles,
    uint32_t deadlineOffsetCycles,
    uint8_t *timeLimited)
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
        samples, sampleCount, NULL, maxOutput, output, staticCfg,
        deadlineStartCycles, deadlineOffsetCycles, timeLimited);
}

static int32_t MmwDemo_NavPointGen_buildNearestMap(
    const MmwDemo_DenseElevation_OutputSample *samples,
    uint16_t sampleCount,
    uint32_t deadlineStartCycles,
    uint32_t deadlineOffsetCycles)
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
        if (((sampleIndex & 63U) == 63U) &&
            (MmwDemo_NavPointGen_deadlineExpired(
                deadlineStartCycles, deadlineOffsetCycles) != 0U))
        {
            return MMWDEMO_NAV_ETIME_BUDGET;
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
    uint8_t *navBuffer,
    uint32_t navBufferSize,
    uint32_t *navSize)
{
    MmwDemo_DenseElevation_OutputHeader *candidateHeader;
    MmwDemo_DenseElevation_OutputSample *samples;
    MmwDemo_NavFrameHeader *navHeader;
    MmwDemo_NavPoint *surfaceOutput;
    uint32_t candidateSize = 0U;
    uint32_t processingStartCycles;
    uint32_t emitSurfaceStartCycles;
    uint16_t surfaceAvailable = 0U;
    uint16_t maxSurface;
    uint16_t flags;
    uint8_t timeLimited = 0U;
    int32_t retVal;

    if (navSize != NULL)
    {
        *navSize = 0U;
    }
    if ((cfg == NULL) || (staticCfg == NULL) || (navBuffer == NULL) ||
        (candidateBuffer == NULL) ||
        (candidateBufferSize < MMWDEMO_NAV_CANDIDATE_BUFFER_SIZE) ||
        (navSize == NULL) ||
        (navBufferSize < sizeof(MmwDemo_NavFrameHeader)) ||
        (cfg->maxSurfacePoints == 0U) ||
        (cfg->uartMaxSurfacePoints == 0U) ||
        (cfg->maxSurfacePoints > MMWDEMO_NAV_MAX_SURFACE_POINTS) ||
        (cfg->uartMaxSurfacePoints > MMWDEMO_NAV_MAX_SURFACE_POINTS) ||
        (cfg->surfaceRangeToleranceBins > 32U) ||
        (cfg->surfaceElevationNeighborBins > 8U) ||
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

    maxSurface = (cfg->uartMaxSurfacePoints < cfg->maxSurfacePoints) ?
                 cfg->uartMaxSurfacePoints : cfg->maxSurfacePoints;
    if (sizeof(MmwDemo_NavFrameHeader) + (uint32_t)maxSurface * sizeof(MmwDemo_NavPoint) > navBufferSize)
    {
        return MMWDEMO_NAV_EOUTPUT_CAPACITY;
    }

    processingStartCycles = Cycleprofiler_getTimeStamp();
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
        samples, candidateHeader->numSamples,
        deadlineStartCycles, deadlineOffsetCycles);
    if (retVal != 0)
    {
        return retVal;
    }

    /* Surface publication and object clustering are separate products. */
    flags = MMWDEMO_NAV_FLAG_FRAME_COMPLETE |
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
    memset(navBuffer, 0, sizeof(MmwDemo_NavFrameHeader));
    navHeader = (MmwDemo_NavFrameHeader *)navBuffer;
    surfaceOutput = (MmwDemo_NavPoint *)(navBuffer + sizeof(*navHeader));

    emitSurfaceStartCycles = Cycleprofiler_getTimeStamp();
    navHeader->numSurfacePoints = MmwDemo_NavPointGen_emitSurface(
        samples, candidateHeader->numSamples, maxSurface, surfaceOutput, staticCfg,
        &surfaceAvailable, deadlineStartCycles, deadlineOffsetCycles,
        &timeLimited);
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
    if (timeLimited != 0U)
    {
        return MMWDEMO_NAV_ETIME_BUDGET;
    }
    navHeader->numDiscretePoints = 0U;

    if (surfaceAvailable > navHeader->numSurfacePoints)
    {
        flags |= MMWDEMO_NAV_FLAG_SURFACE_LIMITED;
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
        Cycleprofiler_getTimeStamp() - processingStartCycles;

    *navSize = sizeof(*navHeader) +
        (uint32_t)navHeader->numSurfacePoints * sizeof(MmwDemo_NavPoint);
    return 0;
}

void MmwDemo_NavPointGen_reset(void)
{
    memset(&gNavScratch, 0, sizeof(gNavScratch));
    memset(gNavCellSelected, 0, sizeof(gNavCellSelected));
    memset(gNavLastEgoSequence, 0, sizeof(gNavLastEgoSequence));
    memset(gNavExternalEgoUpdateFrame, 0,
           sizeof(gNavExternalEgoUpdateFrame));
    memset(gNavExternalEgoHasFrame, 0,
           sizeof(gNavExternalEgoHasFrame));
}

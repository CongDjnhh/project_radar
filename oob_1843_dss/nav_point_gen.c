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

/* ========================================================================
 * Motion-compensated two-frame confirmation — replaces the decay/hit/count
 * persistence grid. The old filter admitted every sample because the hit
 * bonus (+NAV_HIST_HIT_BONUS) was applied before the threshold test inside
 * the same frame, so nothing was ever rejected.
 *
 * A sample is published only when its (azimuth, range-group) cell — or the
 * cell the target is predicted to have occupied one frame ago — was also
 * detected in the previous frame:
 *   - single-frame spikes (RFI, multipath flicker) are suppressed;
 *   - persistent surfaces at every range are retained (multi-layer);
 *   - closing targets stay confirmed because the lookup follows the
 *     Doppler-predicted range walk (positive Doppler = approaching, so the
 *     previous range was larger).
 * Cost per frame: O(samples + grid clear) byte operations. A newly detected
 * surface is withheld for one frame; platform yaw likewise costs one
 * re-confirmation frame (azimuth cells shift with the platform).
 * ====================================================================== */
#define NAV_HIST_NUM_AZ    MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS  /* 64 */
#define NAV_HIST_NUM_RNG   (64U)   /* range bins grouped 4 per cell    */
#define NAV_HIST_MAP_SIZE  (NAV_HIST_NUM_AZ * NAV_HIST_NUM_RNG) /* 4096 */
#define NAV_HIST_RANGE_GROUP_SHIFT (2U)
#define NAV_HIST_AZ_SPREAD (1U)    /* hits also mark neighbour azimuths */
#define NAV_MOTION_MAX_RANGE_BINS  (16) /* clamp of Doppler prediction   */

/* Persistent across frames → must reside in L3 (not L1D scratch). */
#pragma DATA_SECTION(gNavHistoryMap, ".l3ram")
#pragma DATA_ALIGN(gNavHistoryMap, 8)
static far uint8_t gNavHistoryMap[NAV_HIST_MAP_SIZE];  /* 4,096 bytes */

/* gNavCellSelected is still needed by emitPrioritized for sector balancing.
 * It is scratch (re-zeroed every frame) so L1D placement is correct. */
#pragma DATA_SECTION(gNavCellSelected, ".navScratch")
#pragma DATA_ALIGN(gNavCellSelected, 8)
static uint8_t gNavCellSelected[MMWDEMO_NAV_SELECTION_BYTES];

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
    MmwDemo_CustomObstacle *dstObstacle,
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

    if (dstObstacle != NULL)
    {
        dstObstacle->x = x;
        dstObstacle->y = y;
        dstObstacle->z = z;
        dstObstacle->velocity = (float)src->dopplerIdx * staticCfg->dopplerStepMps;

        float width_u = (float)src->azimuthWidthUQ15 / 32767.0f;
        float width = dist * width_u;
        if (width < 0.05f)
        {
            width = 0.05f;
        }

        if (src->powerDb10 != (int16_t)-32768)
        {
            dstObstacle->snr = src->powerDb10;
        }
        else
        {
            dstObstacle->snr = (int16_t)src->snrDb10;
        }
        /* Preserve physical obstacle width in mm */
        dstObstacle->width = (int16_t)(width * 1000.0f);
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
    int16_t bestIdx = -1;
    const MmwDemo_DenseElevation_OutputSample *best = NULL;

    /* After temporal filtering every sample in the array is genuine.
     * We only skip samples whose angle cell was already emitted (isSelected)
     * to avoid duplicates in the final output. */
    for (sampleIndex = 0U; sampleIndex < sampleCount; sampleIndex++)
    {
        uint16_t cell = MmwDemo_NavPointGen_cellIndex(&samples[sampleIndex]);
        if ((MmwDemo_NavPointGen_isSelected(gNavCellSelected, cell) != 0U) ||
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
            bestIdx = (int16_t)sampleIndex;
        }
    }
    return bestIdx;
}

static uint16_t MmwDemo_NavPointGen_emitPrioritized(
    const MmwDemo_DenseElevation_OutputSample *samples,
    uint16_t sampleCount,
    uint16_t maxOutput,
    MmwDemo_CustomObstacle *outObstacles,
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
            int16_t bestIdx;
            bestIdx = MmwDemo_NavPointGen_findBestUnselected(
                samples, sampleCount, (int16_t)sector);
            if (bestIdx < 0)
            {
                break;
            }
            MmwDemo_NavPointGen_copyPoint(
                &outObstacles[count],
                &samples[bestIdx], staticCfg);
            MmwDemo_NavPointGen_setSelected(gNavCellSelected,
                MmwDemo_NavPointGen_cellIndex(&samples[bestIdx]));
            count++;
            emittedInSector++;
        }
    }

    /* Reuse quota left by sparse sectors, still ordered globally by safety
     * priority rather than signed angle/index. */
    while (count < maxOutput)
    {
        int16_t bestIdx;
        bestIdx = MmwDemo_NavPointGen_findBestUnselected(
            samples, sampleCount, -1);
        if (bestIdx < 0)
        {
            break;
        }
        MmwDemo_NavPointGen_copyPoint(
            &outObstacles[count],
            &samples[bestIdx], staticCfg);
        MmwDemo_NavPointGen_setSelected(gNavCellSelected,
            MmwDemo_NavPointGen_cellIndex(&samples[bestIdx]));
        count++;
    }
    return count;
}

static uint16_t MmwDemo_NavPointGen_emitSurface(
    const MmwDemo_DenseElevation_OutputSample *samples,
    uint16_t sampleCount,
    uint16_t maxOutput,
    MmwDemo_CustomObstacle *outObstacles,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
    uint16_t *availableCount)
{
    /* After temporal filtering, every sample in the array is a confirmed
     * surface.  availableCount simply reports how many survived the filter. */
    *availableCount = sampleCount;
    return MmwDemo_NavPointGen_emitPrioritized(
        samples, sampleCount, maxOutput, outObstacles, staticCfg);
}

/* ========================================================================
 * Motion-compensated two-frame confirmation pipeline
 * ====================================================================== */

static uint16_t MmwDemo_NavPointGen_histIndex(int16_t az, uint16_t rngGrp)
{
    /* Sample azimuthIdx is a signed peak bin (-32..+31); the grid row is
     * unsigned (0..63), so shift before clamping. */
    az += (int16_t)(NAV_HIST_NUM_AZ / 2U);
    if (az < 0)
    {
        az = 0;
    }
    else if (az >= (int16_t)NAV_HIST_NUM_AZ)
    {
        az = (int16_t)NAV_HIST_NUM_AZ - 1;
    }
    if (rngGrp >= NAV_HIST_NUM_RNG)
    {
        rngGrp = NAV_HIST_NUM_RNG - 1U;
    }
    return (uint16_t)((uint16_t)az * NAV_HIST_NUM_RNG + rngGrp);
}

/**
 * Phase 1: CONFIRM — keep only samples whose (azimuth, range-group) cell,
 * or the Doppler-predicted previous position, was also detected last frame.
 * Compacts the array in place and returns the survivor count.
 */
static uint16_t MmwDemo_NavPointGen_historyFilter(
    MmwDemo_DenseElevation_OutputSample *samples,
    uint16_t sampleCount,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg)
{
    float binsPerFrame;
    uint16_t writeIdx = 0U;
    uint16_t s;

    if (staticCfg->framePeriodSec <= 0.0f)
    {
        /* Frame timing not yet published by MSS: fail open instead of
         * suppressing every sample on the first frames after boot. */
        return sampleCount;
    }
    binsPerFrame = staticCfg->dopplerStepMps * staticCfg->framePeriodSec /
                   staticCfg->rangeStepMeters;

    for (s = 0U; s < sampleCount; s++)
    {
        int16_t az = samples[s].azimuthIdx;
        int32_t rangeBin = (int32_t)samples[s].rangeIdx;
        float raw = (float)samples[s].dopplerIdx * binsPerFrame;
        int32_t delta;
        int32_t prevBin;
        int32_t bin;
        uint8_t hit;

        delta = (raw >= 0.0f) ? (int32_t)(raw + 0.5f)
                              : -(int32_t)(-raw + 0.5f);
        if (delta > NAV_MOTION_MAX_RANGE_BINS)
        {
            delta = NAV_MOTION_MAX_RANGE_BINS;
        }
        if (delta < -NAV_MOTION_MAX_RANGE_BINS)
        {
            delta = -NAV_MOTION_MAX_RANGE_BINS;
        }
        prevBin = rangeBin + delta;

        hit = gNavHistoryMap[MmwDemo_NavPointGen_histIndex(
            az, (uint16_t)rangeBin >> NAV_HIST_RANGE_GROUP_SHIFT)];
        if (hit == 0U)
        {
            /* +/-1 bin around the predicted previous position absorbs the
             * Doppler-quantisation error and group-boundary rounding. */
            for (bin = prevBin - 1; (bin <= prevBin + 1) && (hit == 0U); bin++)
            {
                if (bin < 0)
                {
                    continue;
                }
                hit = gNavHistoryMap[MmwDemo_NavPointGen_histIndex(
                    az, (uint16_t)bin >> NAV_HIST_RANGE_GROUP_SHIFT)];
            }
        }

        if (hit != 0U)
        {
            if (writeIdx != s)
            {
                samples[writeIdx] = samples[s];
            }
            writeIdx++;
        }
    }
    return writeIdx;
}

/**
 * Phase 2: RECORD — publish this frame's detections (before suppression) as
 * next frame's history. Hits mark the azimuth neighbours too, so a peak that
 * jitters one azimuth bin between frames still confirms. Recording every
 * detection keeps a newly appeared surface confirmable on its second frame.
 */
static void MmwDemo_NavPointGen_historyWrite(
    const MmwDemo_DenseElevation_OutputSample *samples,
    uint16_t sampleCount)
{
    uint16_t s;
    int16_t a;

    memset(gNavHistoryMap, 0, sizeof(gNavHistoryMap));
    for (s = 0U; s < sampleCount; s++)
    {
        int16_t az = samples[s].azimuthIdx;
        uint16_t rngGrp =
            samples[s].rangeIdx >> NAV_HIST_RANGE_GROUP_SHIFT;

        for (a = az - NAV_HIST_AZ_SPREAD; a <= az + NAV_HIST_AZ_SPREAD; a++)
        {
            gNavHistoryMap[MmwDemo_NavPointGen_histIndex(a, rngGrp)] = 1U;
        }
    }
}

int32_t MmwDemo_NavPointGen_process(
    const MmwDemo_NavPointGen_Cfg *cfg,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
    uint8_t *candidateBuffer,
    uint32_t candidateBufferSize,
    MmwDemo_CustomObstacle *outObstacles,
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
    if ((cfg == NULL) || (staticCfg == NULL) || (outObstacles == NULL) ||
        (candidateBuffer == NULL) ||
        (candidateBufferSize < MMWDEMO_NAV_CANDIDATE_BUFFER_SIZE) ||
        (numPointsOut == NULL) ||
        (maxOutputPoints == 0U) ||
        (cfg->maxSurfacePoints == 0U) ||
        (!isfinite(staticCfg->rangeStepMeters)) ||
        !(staticCfg->rangeStepMeters > 0.0f) ||
        (!isfinite(staticCfg->dopplerStepMps)) ||
        (!isfinite(staticCfg->framePeriodSec)) ||
        (staticCfg->framePeriodSec < 0.0f) ||
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
    /* Motion-compensated two-frame confirmation: confirm against last
     * frame's history, then record this frame's detections so the next
     * frame can confirm them. Replaces buildNearestMap(): multi-layer
     * detection is kept while single-frame transient noise is rejected. */
    {
        uint16_t detectedCount = candidateHeader->numSamples;
        candidateHeader->numSamples = MmwDemo_NavPointGen_historyFilter(
            samples, detectedCount, staticCfg);
        MmwDemo_NavPointGen_historyWrite(samples, detectedCount);
    }

    emitSurfaceStartCycles = Cycleprofiler_getTimeStamp();
    *numPointsOut = (uint32_t)MmwDemo_NavPointGen_emitSurface(
        samples, candidateHeader->numSamples, maxSurface,
        outObstacles, staticCfg,
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
    memset(gNavHistoryMap, 0, sizeof(gNavHistoryMap));
    memset(gNavCellSelected, 0, sizeof(gNavCellSelected));
}

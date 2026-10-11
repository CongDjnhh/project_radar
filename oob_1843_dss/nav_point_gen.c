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

#define MMWDEMO_NAV_CLUSTER_MAX_BOXES   (64U)
#define MMWDEMO_NAV_CLUSTER_MAX_SAMPLES (256U)

typedef struct MmwDemo_NavClusterAcc_t {
    uint16_t minRange;
    uint16_t maxRange;
    float    minU;
    float    maxU;
    float    minV;
    float    maxV;
    int16_t  peakDoppler;
    int16_t  peakSnr;
    int16_t  peakPower;
    uint16_t sampleCount;
} MmwDemo_NavClusterAcc;

#pragma DATA_SECTION(gNavClusterParent, ".navScratch")
#pragma DATA_ALIGN(gNavClusterParent, 8)
static int16_t gNavClusterParent[MMWDEMO_NAV_CLUSTER_MAX_SAMPLES];

#pragma DATA_SECTION(gNavRootToBox, ".navScratch")
#pragma DATA_ALIGN(gNavRootToBox, 8)
static int16_t gNavRootToBox[MMWDEMO_NAV_CLUSTER_MAX_SAMPLES];

#pragma DATA_SECTION(gNavClusterAcc, ".navScratch")
#pragma DATA_ALIGN(gNavClusterAcc, 8)
static MmwDemo_NavClusterAcc gNavClusterAcc[MMWDEMO_NAV_CLUSTER_MAX_BOXES];

/* Cycle profiling for emitSurface. Read via CCS watch to assess execution cost. */
volatile uint32_t gNavEmitSurfaceCycles[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
volatile uint32_t gNavEmitSurfaceMaxCycles[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];

static int16_t MmwDemo_NavCluster_find(int16_t *parent, int16_t i)
{
    int16_t root = i;
    while (root != parent[root])
    {
        root = parent[root];
    }
    while (i != root)
    {
        int16_t next = parent[i];
        parent[i] = root;
        i = next;
    }
    return root;
}

static uint16_t MmwDemo_NavPointGen_emitObstacleBoxes(
    const MmwDemo_DenseElevation_OutputSample *samples,
    uint16_t sampleCount,
    uint16_t maxOutput,
    ObstacleBox *outObstacles,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg)
{
    uint16_t N = (sampleCount > MMWDEMO_NAV_CLUSTER_MAX_SAMPLES) ?
                 MMWDEMO_NAV_CLUSTER_MAX_SAMPLES : sampleCount;
    uint16_t i, j, b;
    uint16_t numBoxes = 0U;
    uint16_t outCount = 0U;
    uint16_t limit;

    if ((N == 0U) || (maxOutput == 0U) || (outObstacles == NULL))
    {
        return 0U;
    }

    /* Step 1: Initialize Union-Find sets */
    for (i = 0U; i < N; i++)
    {
        gNavClusterParent[i] = (int16_t)i;
        gNavRootToBox[i] = -1;
    }

    /* Step 2: Spatial and kinematic connectivity clustering */
    for (i = 0U; i < N; i++)
    {
        int32_t r_i = (int32_t)samples[i].rangeIdx;
        int32_t d_i = (int32_t)samples[i].dopplerIdx;
        float u_i = (float)samples[i].azimuthUQ15 / 32767.0f;
        float hw_u_i = 0.5f * ((float)samples[i].azimuthWidthUQ15 / 32767.0f);
        float u_min_i = u_i - hw_u_i;
        float u_max_i = u_i + hw_u_i;

        float v_i = (float)samples[i].sourceVQ15 / 32767.0f;
        float hw_v_i = 0.5f * ((float)samples[i].elevationWidthUQ15 / 32767.0f);
        float v_min_i = v_i - hw_v_i;
        float v_max_i = v_i + hw_v_i;

        for (j = i + 1U; j < N; j++)
        {
            int32_t r_diff = (int32_t)samples[j].rangeIdx - r_i;
            if (r_diff < 0) { r_diff = -r_diff; }
            if (r_diff > 2)
            {
                continue; /* Range gap too large */
            }

            int32_t d_diff = (int32_t)samples[j].dopplerIdx - d_i;
            if (d_diff < 0) { d_diff = -d_diff; }
            if (d_diff > 3)
            {
                continue; /* Doppler velocity difference too large */
            }

            float u_j = (float)samples[j].azimuthUQ15 / 32767.0f;
            float hw_u_j = 0.5f * ((float)samples[j].azimuthWidthUQ15 / 32767.0f);
            float u_min_j = u_j - hw_u_j;
            float u_max_j = u_j + hw_u_j;

            /* Check azimuth overlap / adjacency within ~2 bins (0.06 in u-space) */
            if ((u_min_i > (u_max_j + 0.06f)) || (u_min_j > (u_max_i + 0.06f)))
            {
                continue;
            }

            float v_j = (float)samples[j].sourceVQ15 / 32767.0f;
            float hw_v_j = 0.5f * ((float)samples[j].elevationWidthUQ15 / 32767.0f);
            float v_min_j = v_j - hw_v_j;
            float v_max_j = v_j + hw_v_j;

            /* Check elevation overlap / adjacency within tolerance (0.15 in v-space) */
            if ((v_min_i > (v_max_j + 0.15f)) || (v_min_j > (v_max_i + 0.15f)))
            {
                continue;
            }

            /* Connected: merge sets */
            {
                int16_t root_i = MmwDemo_NavCluster_find(gNavClusterParent, (int16_t)i);
                int16_t root_j = MmwDemo_NavCluster_find(gNavClusterParent, (int16_t)j);
                if (root_i != root_j)
                {
                    gNavClusterParent[root_i] = root_j;
                }
            }
        }
    }

    /* Step 3: Aggregate clusters into bounding box descriptors */
    for (i = 0U; i < N; i++)
    {
        int16_t root = MmwDemo_NavCluster_find(gNavClusterParent, (int16_t)i);
        int16_t boxIdx = gNavRootToBox[root];

        float u = (float)samples[i].azimuthUQ15 / 32767.0f;
        float hw_u = 0.5f * ((float)samples[i].azimuthWidthUQ15 / 32767.0f);
        float u_min = u - hw_u;
        float u_max = u + hw_u;

        float v = (float)samples[i].sourceVQ15 / 32767.0f;
        float hw_v = 0.5f * ((float)samples[i].elevationWidthUQ15 / 32767.0f);
        float v_min = v - hw_v;
        float v_max = v + hw_v;

        if (boxIdx < 0)
        {
            if (numBoxes >= MMWDEMO_NAV_CLUSTER_MAX_BOXES)
            {
                continue;
            }
            boxIdx = (int16_t)numBoxes;
            gNavRootToBox[root] = boxIdx;
            numBoxes++;

            gNavClusterAcc[boxIdx].minRange = samples[i].rangeIdx;
            gNavClusterAcc[boxIdx].maxRange = samples[i].rangeIdx;
            gNavClusterAcc[boxIdx].minU = u_min;
            gNavClusterAcc[boxIdx].maxU = u_max;
            gNavClusterAcc[boxIdx].minV = v_min;
            gNavClusterAcc[boxIdx].maxV = v_max;
            gNavClusterAcc[boxIdx].peakDoppler = samples[i].dopplerIdx;
            gNavClusterAcc[boxIdx].peakSnr = (int16_t)samples[i].snrDb10;
            gNavClusterAcc[boxIdx].peakPower = samples[i].powerDb10;
            gNavClusterAcc[boxIdx].sampleCount = 1U;
        }
        else
        {
            if (samples[i].rangeIdx < gNavClusterAcc[boxIdx].minRange)
            {
                gNavClusterAcc[boxIdx].minRange = samples[i].rangeIdx;
            }
            if (samples[i].rangeIdx > gNavClusterAcc[boxIdx].maxRange)
            {
                gNavClusterAcc[boxIdx].maxRange = samples[i].rangeIdx;
            }
            if (u_min < gNavClusterAcc[boxIdx].minU)
            {
                gNavClusterAcc[boxIdx].minU = u_min;
            }
            if (u_max > gNavClusterAcc[boxIdx].maxU)
            {
                gNavClusterAcc[boxIdx].maxU = u_max;
            }
            if (v_min < gNavClusterAcc[boxIdx].minV)
            {
                gNavClusterAcc[boxIdx].minV = v_min;
            }
            if (v_max > gNavClusterAcc[boxIdx].maxV)
            {
                gNavClusterAcc[boxIdx].maxV = v_max;
            }
            if ((int16_t)samples[i].snrDb10 > gNavClusterAcc[boxIdx].peakSnr)
            {
                gNavClusterAcc[boxIdx].peakSnr = (int16_t)samples[i].snrDb10;
                gNavClusterAcc[boxIdx].peakDoppler = samples[i].dopplerIdx;
            }
            if (samples[i].powerDb10 > gNavClusterAcc[boxIdx].peakPower)
            {
                gNavClusterAcc[boxIdx].peakPower = samples[i].powerDb10;
            }
            gNavClusterAcc[boxIdx].sampleCount++;
        }
    }

    /* Step 4: Compute physical 3D box coordinates, dimensions and SNR */
    limit = (numBoxes < maxOutput) ? numBoxes : maxOutput;
    for (b = 0U; b < limit; b++)
    {
        float rangeSpan = (float)(gNavClusterAcc[b].maxRange - gNavClusterAcc[b].minRange + 1U);
        float depth_y = rangeSpan * staticCfg->rangeStepMeters;
        if (depth_y < staticCfg->rangeStepMeters)
        {
            depth_y = staticCfg->rangeStepMeters;
        }

        float rCenter = 0.5f * (float)(gNavClusterAcc[b].minRange + gNavClusterAcc[b].maxRange);
        float dist = rCenter * staticCfg->rangeStepMeters + staticCfg->rangeBiasMeters;
        if (dist < 0.0f)
        {
            dist = 0.0f;
        }

        float minU = gNavClusterAcc[b].minU;
        float maxU = gNavClusterAcc[b].maxU;
        if (minU < -1.0f) { minU = -1.0f; }
        if (maxU >  1.0f) { maxU =  1.0f; }
        if (minU > maxU)  { float tmp = minU; minU = maxU; maxU = tmp; }
        float uCenter = 0.5f * (minU + maxU);
        float width_u = maxU - minU;
        float width_x = dist * width_u;
        if (width_x < 0.10f)
        {
            width_x = 0.10f;
        }

        float minV = gNavClusterAcc[b].minV;
        float maxV = gNavClusterAcc[b].maxV;
        if (minV < -1.0f) { minV = -1.0f; }
        if (maxV >  1.0f) { maxV =  1.0f; }
        if (minV > maxV)  { float tmp = minV; minV = maxV; maxV = tmp; }
        float vCenter = 0.5f * (minV + maxV);
        float height_v = maxV - minV;
        float height_z = dist * height_v;
        if (height_z < 0.10f)
        {
            height_z = 0.10f;
        }

        float x = dist * uCenter;
        float z = dist * vCenter;
        float ySq = (dist * dist) - (x * x) - (z * z);
        float y = (ySq > 0.0f) ? sqrtf(ySq) : 0.0f;

        outObstacles[b].x = x;
        outObstacles[b].y = y;
        outObstacles[b].z = z;
        outObstacles[b].width_x = width_x;
        outObstacles[b].depth_y = depth_y;
        outObstacles[b].height_z = height_z;
        outObstacles[b].velocity = (float)gNavClusterAcc[b].peakDoppler * staticCfg->dopplerStepMps;
        outObstacles[b].snr = (gNavClusterAcc[b].peakPower != (int16_t)-32768) ?
                              gNavClusterAcc[b].peakPower : gNavClusterAcc[b].peakSnr;
        outObstacles[b].boxId = (int16_t)(b + 1U);
    }
    outCount = limit;

    /* Step 5: Sort obstacles by forward distance (y): closest hazards first */
    for (i = 1U; i < outCount; i++)
    {
        ObstacleBox key = outObstacles[i];
        int16_t k = (int16_t)i - 1;
        while ((k >= 0) && (outObstacles[k].y > key.y))
        {
            outObstacles[k + 1] = outObstacles[k];
            k--;
        }
        outObstacles[k + 1] = key;
    }

    /* Assign canonical sequential box IDs after sorting */
    for (i = 0U; i < outCount; i++)
    {
        outObstacles[i].boxId = (int16_t)(i + 1U);
    }

    return outCount;
}

static uint16_t MmwDemo_NavPointGen_emitSurface(
    const MmwDemo_DenseElevation_OutputSample *samples,
    uint16_t sampleCount,
    uint16_t maxOutput,
    ObstacleBox *outObstacles,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
    uint16_t *availableCount)
{
    /* After temporal filtering, every sample in the array is a confirmed
     * surface.  availableCount simply reports how many survived the filter. */
    *availableCount = sampleCount;
    return MmwDemo_NavPointGen_emitObstacleBoxes(
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
}

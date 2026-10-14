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

extern volatile uint32_t gNavDbgDenseSamples;
extern volatile uint32_t gNavDbgFilteredSamples;

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
#define MMWDEMO_NAV_PERSIST_MAX_OBSTACLES (64U)
#define MMWDEMO_NAV_PERSIST_DEFAULT_TTL   (3U)

typedef struct MmwDemo_NavPersistObstacle_t {
    ObstacleBox box;
    uint8_t     ttl;        /*!< Remaining lifetime in frames */
    uint8_t     valid;      /*!< 1 = active, 0 = inactive */
} MmwDemo_NavPersistObstacle;

/* Persistent across frames -> must reside in L3 RAM */
#pragma DATA_SECTION(gNavPersistList, ".l3ram")
#pragma DATA_ALIGN(gNavPersistList, 8)
static far MmwDemo_NavPersistObstacle gNavPersistList[MMWDEMO_NAV_PERSIST_MAX_OBSTACLES];

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

#pragma DATA_SECTION(gNavRawBoxes, ".navScratch")
#pragma DATA_ALIGN(gNavRawBoxes, 8)
static ObstacleBox gNavRawBoxes[MMWDEMO_NAV_CLUSTER_MAX_BOXES];

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

static uint16_t MmwDemo_NavPointGen_updatePersistence(
    const ObstacleBox *currBoxes,
    uint16_t numCurr,
    ObstacleBox *outBoxes,
    uint16_t maxOutput,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg)
{
    uint16_t p, c;
    uint8_t matchedCurr[MMWDEMO_NAV_CLUSTER_MAX_BOXES];
    uint8_t matchedPersist[MMWDEMO_NAV_PERSIST_MAX_OBSTACLES];
    uint16_t outCount = 0U;

    memset(matchedCurr, 0, sizeof(matchedCurr));
    memset(matchedPersist, 0, sizeof(matchedPersist));

    /* Match current detections against active persistent obstacles */
    for (c = 0U; c < numCurr; c++)
    {
        float bestDist = 1e9f;
        int16_t bestP = -1;

        for (p = 0U; p < MMWDEMO_NAV_PERSIST_MAX_OBSTACLES; p++)
        {
            float dy, dx, dz, totalDist, maxDy;

            if (gNavPersistList[p].valid == 0U)
            {
                continue;
            }
            if (matchedPersist[p] != 0U)
            {
                continue; /* 1-to-1 association: avoid multiple current boxes stealing the same track */
            }

            dy = currBoxes[c].y - gNavPersistList[p].box.y;
            dx = currBoxes[c].x - gNavPersistList[p].box.x;
            dz = currBoxes[c].z - gNavPersistList[p].box.z;
            if (dy < 0.0f) { dy = -dy; }
            if (dx < 0.0f) { dx = -dx; }
            if (dz < 0.0f) { dz = -dz; }

            /* Motion-compensated matching gate (tightened base gate prevents merging closely spaced obstacles) */
            maxDy = 0.35f + fabsf(gNavPersistList[p].box.velocity) * staticCfg->framePeriodSec;
            if ((dy <= maxDy) && (dx <= 0.45f) && (dz <= 0.60f))
            {
                totalDist = dy + dx + dz;
                if (totalDist < bestDist)
                {
                    bestDist = totalDist;
                    bestP = (int16_t)p;
                }
            }
        }

        if (bestP >= 0)
        {
            /* Matched: update persistent obstacle with latest measurement */
            gNavPersistList[bestP].box = currBoxes[c];
            gNavPersistList[bestP].ttl = MMWDEMO_NAV_PERSIST_DEFAULT_TTL;
            matchedCurr[c] = 1U;
            matchedPersist[bestP] = 1U;
        }
    }

    /* Fail-open: add all new/unmatched current boxes immediately */
    for (c = 0U; c < numCurr; c++)
    {
        if (matchedCurr[c] == 0U)
        {
            for (p = 0U; p < MMWDEMO_NAV_PERSIST_MAX_OBSTACLES; p++)
            {
                if (gNavPersistList[p].valid == 0U)
                {
                    gNavPersistList[p].box = currBoxes[c];
                    gNavPersistList[p].ttl = MMWDEMO_NAV_PERSIST_DEFAULT_TTL;
                    gNavPersistList[p].valid = 1U;
                    matchedPersist[p] = 1U;
                    break;
                }
            }
        }
    }

    /* Age out unmatched persistent obstacles (intermittent dropout handling) */
    for (p = 0U; p < MMWDEMO_NAV_PERSIST_MAX_OBSTACLES; p++)
    {
        if (gNavPersistList[p].valid == 0U)
        {
            continue;
        }

        if (matchedPersist[p] == 0U)
        {
            /* Dropped in this frame: decrement TTL */
            if (gNavPersistList[p].ttl > 0U)
            {
                gNavPersistList[p].ttl--;
            }
            if (gNavPersistList[p].ttl == 0U)
            {
                gNavPersistList[p].valid = 0U;
            }
            else
            {
                /* Persist with damped motion extrapolation (velocity damping prevents overshoot under acceleration) */
                gNavPersistList[p].box.y += gNavPersistList[p].box.velocity * staticCfg->framePeriodSec;
                gNavPersistList[p].box.velocity *= 0.85f;
                if ((gNavPersistList[p].box.y < 0.15f) || (gNavPersistList[p].box.y > 8.64f))
                {
                    gNavPersistList[p].valid = 0U;
                }
            }
        }
    }

    /* Emit active persistent obstacles to outBoxes */
    for (p = 0U; (p < MMWDEMO_NAV_PERSIST_MAX_OBSTACLES) && (outCount < maxOutput); p++)
    {
        if (gNavPersistList[p].valid != 0U)
        {
            outBoxes[outCount] = gNavPersistList[p].box;
            outCount++;
        }
    }

    /* Sort by forward distance (y): closest hazards first */
    if (outCount > 1U)
    {
        uint16_t i;
        for (i = 1U; i < outCount; i++)
        {
            ObstacleBox key = outBoxes[i];
            int16_t k = (int16_t)i - 1;
            while ((k >= 0) && (outBoxes[k].y > key.y))
            {
                outBoxes[k + 1] = outBoxes[k];
                k--;
            }
            outBoxes[k + 1] = key;
        }
    }

    /* Assign canonical sequential box IDs after sorting */
    {
        uint16_t i;
        for (i = 0U; i < outCount; i++)
        {
            outBoxes[i].boxId = (int16_t)(i + 1U);
        }
    }

    return outCount;
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
    uint16_t limit = 0U;

    if ((maxOutput == 0U) || (outObstacles == NULL))
    {
        return 0U;
    }

    if ((N > 0U) && (samples != NULL))
    {
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
            }
        }

        /* Step 4: Compute physical 3D box coordinates, dimensions and SNR */
        limit = (numBoxes < MMWDEMO_NAV_CLUSTER_MAX_BOXES) ? numBoxes : MMWDEMO_NAV_CLUSTER_MAX_BOXES;
        for (b = 0U; b < limit; b++)
        {
            float rangeSpan = (float)(gNavClusterAcc[b].maxRange - gNavClusterAcc[b].minRange + 1U);
            float depth_y = rangeSpan * staticCfg->rangeStepMeters;
            float rCenter;
            float dist;
            float minU, maxU, uCenter, width_u, width_x;
            float minV, maxV, vCenter, height_v, height_z;
            float uvNormSq, x, z, ySq, y;

            if (depth_y < staticCfg->rangeStepMeters)
            {
                depth_y = staticCfg->rangeStepMeters;
            }
            if (depth_y > 2.50f)
            {
                depth_y = 2.50f;
            }

            rCenter = 0.5f * (float)(gNavClusterAcc[b].minRange + gNavClusterAcc[b].maxRange);
            dist = rCenter * staticCfg->rangeStepMeters + staticCfg->rangeBiasMeters;
            if (dist < 0.0f)
            {
                dist = 0.0f;
            }

            minU = gNavClusterAcc[b].minU;
            maxU = gNavClusterAcc[b].maxU;
            if (minU < -1.0f) { minU = -1.0f; }
            if (maxU >  1.0f) { maxU =  1.0f; }
            if (minU > maxU)  { float tmp = minU; minU = maxU; maxU = tmp; }
            uCenter = 0.5f * (minU + maxU);
            width_u = maxU - minU;
            width_x = dist * width_u;
            if (width_x < 0.10f)
            {
                width_x = 0.10f;
            }
            if (width_x > 3.00f)
            {
                width_x = 3.00f;
            }

            minV = gNavClusterAcc[b].minV;
            maxV = gNavClusterAcc[b].maxV;
            if (minV < -1.0f) { minV = -1.0f; }
            if (maxV >  1.0f) { maxV =  1.0f; }
            if (minV > maxV)  { float tmp = minV; minV = maxV; maxV = tmp; }
            vCenter = 0.5f * (minV + maxV);
            height_v = maxV - minV;
            height_z = dist * height_v;
            if (height_z < 0.10f)
            {
                height_z = 0.10f;
            }
            if (height_z > 3.00f)
            {
                height_z = 3.00f;
            }

            /* Projection onto unit forward hemisphere to prevent ySq < 0 at wide angles */
            uvNormSq = (uCenter * uCenter) + (vCenter * vCenter);
            if (uvNormSq > 0.99f)
            {
                float scale = sqrtf(0.99f / uvNormSq);
                uCenter *= scale;
                vCenter *= scale;
            }

            x = dist * uCenter;
            z = dist * vCenter;
            ySq = (dist * dist) - (x * x) - (z * z);
            y = (ySq > 0.0f) ? sqrtf(ySq) : 0.0f;

            gNavRawBoxes[b].x = x;
            gNavRawBoxes[b].y = y;
            gNavRawBoxes[b].z = z;
            gNavRawBoxes[b].width_x = width_x;
            gNavRawBoxes[b].depth_y = depth_y;
            gNavRawBoxes[b].height_z = height_z;
            gNavRawBoxes[b].velocity = (float)gNavClusterAcc[b].peakDoppler * staticCfg->dopplerStepMps;
            /* Prioritize true CFAR SNR (in 0.1 dB units); fallback to power if SNR not valid */
            gNavRawBoxes[b].snr = (gNavClusterAcc[b].peakSnr > 0) ?
                                  gNavClusterAcc[b].peakSnr : gNavClusterAcc[b].peakPower;
            gNavRawBoxes[b].boxId = (int16_t)(b + 1U);
        }
    }

    /* Update multi-frame persistence: fail-open for new obstacles, preserve confirmed ones */
    return MmwDemo_NavPointGen_updatePersistence(gNavRawBoxes, limit, outObstacles, maxOutput, staticCfg);
}

static uint16_t MmwDemo_NavPointGen_emitSurface(
    const MmwDemo_DenseElevation_OutputSample *samples,
    uint16_t sampleCount,
    uint16_t maxOutput,
    ObstacleBox *outObstacles,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
    uint16_t *availableCount)
{
    *availableCount = sampleCount;
    return MmwDemo_NavPointGen_emitObstacleBoxes(
        samples, sampleCount, maxOutput, outObstacles, staticCfg);
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

    gNavDbgDenseSamples = (uint32_t)candidateHeader->numSamples;

    emitSurfaceStartCycles = Cycleprofiler_getTimeStamp();
    *numPointsOut = (uint32_t)MmwDemo_NavPointGen_emitSurface(
        samples, candidateHeader->numSamples, maxSurface,
        outObstacles, staticCfg,
        &surfaceAvailable);

    gNavDbgFilteredSamples = *numPointsOut;

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
    memset(gNavPersistList, 0, sizeof(gNavPersistList));
}

/**
 * @file surface_gen.c
 *
 * @brief 3D Adaptive DBSCAN clustering and conservative XYZ bounding box generation.
 *        Segments point cloud candidates into Dynamic and Static partitions,
 *        applies distance-adaptive DBSCAN clustering eps(R) = min(1.0, 0.25 + 0.08*R),
 *        and emits tight axis-aligned bounding boxes (AABB) without aperture inflation.
 *        Maintains exact 32-byte ObstacleBox wire ABI (TLV 20 Dynamic, TLV 22 Static).
 */
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include "surface_gen.h"

#define SURFACE_SAMPLES                 MMWDEMO_DENSE_ELEV_MAX_OUTPUT_SAMPLES
#define SURFACE_HISTORY                 64U
/* Note: Temporal hold duration is dynamically computed as an integer multiple of frame periods */
#define SURFACE_MAX_ROW_SAMPLES         MMWDEMO_DENSE_ELEV_MAX_RANGE_SAMPLES
#define SURFACE_BUFFER_SIZE             (sizeof(MmwDemo_DenseElevation_OutputHeader) + \
                                         SURFACE_SAMPLES * sizeof(MmwDemo_DenseElevation_OutputSample))

#define DBSCAN_MAX_POINTS_PER_GROUP     128U
#define DBSCAN_MIN_PTS                  2U

typedef struct DbscanPoint_t {
    float x;
    float y;
    float z;
    float r;
    float velocity;
    uint16_t snr;
} DbscanPoint;

typedef struct DbscanCluster_t {
    float minX, maxX;
    float minY, maxY;
    float minZ, maxZ;
    float sumVel;
    uint16_t maxSnr;
    uint16_t numPoints;
} DbscanCluster;

typedef struct SurfaceHistory_t {
    ObstacleBox box;
    uint32_t ageUs;
    uint8_t valid;
} SurfaceHistory;

#pragma DATA_SECTION(gDbscanDynPoints, ".l3ram")
#pragma DATA_ALIGN(gDbscanDynPoints, 8)
static far DbscanPoint gDbscanDynPoints[DBSCAN_MAX_POINTS_PER_GROUP];

#pragma DATA_SECTION(gDbscanStatPoints, ".l3ram")
#pragma DATA_ALIGN(gDbscanStatPoints, 8)
static far DbscanPoint gDbscanStatPoints[DBSCAN_MAX_POINTS_PER_GROUP];

#pragma DATA_SECTION(gDbscanClusters, ".l3ram")
#pragma DATA_ALIGN(gDbscanClusters, 8)
static far DbscanCluster gDbscanClusters[MMWDEMO_SURFACE_MAX_BOXES];

#pragma DATA_SECTION(gDbscanLabels, ".l3ram")
static far int16_t gDbscanLabels[DBSCAN_MAX_POINTS_PER_GROUP];

#pragma DATA_SECTION(gDbscanQueue, ".l3ram")
static far uint8_t gDbscanQueue[DBSCAN_MAX_POINTS_PER_GROUP];

#pragma DATA_SECTION(gSurfaceHistory, ".l3ram")
static far SurfaceHistory gSurfaceHistory[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES][SURFACE_HISTORY];

extern MmwDemo_HSRAM gHSRAM;
static uint8_t gSurfaceMotion[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
volatile uint32_t gSurfaceInvalidGeometryCount[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
volatile uint32_t gNavEmitSurfaceCycles[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
volatile uint32_t gNavEmitSurfaceMaxCycles[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
extern volatile uint32_t gNavDbgDenseSamples;
extern volatile uint32_t gNavDbgFilteredSamples;
static int16_t gSurfaceDopplerGuardBins = 1;

/* Compile-time ABI guard */
typedef char SurfaceObstacleBoxWireCheck[(sizeof(ObstacleBox) == 32U) ? 1 : -1];

static int16_t Surface_calcDopplerGuardBins(const MmwDemo_SurfaceGen_StaticCfg *cfg)
{
    float vRes = cfg->dopplerStepMps;
    float vUser = cfg->minMovingSpeedMps;

    if (vRes > 0.0f) {
        if (vUser > 0.0f) {
            int32_t bins = (int32_t)((vUser / vRes) + 0.5f);
            return (bins < 1) ? 1 : (int16_t)bins;
        }
    }
    return 1;
}

void MmwDemo_SurfaceGen_resetSubFrame(uint16_t sf)
{
    if (sf < MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES) {
        memset(gSurfaceHistory[sf], 0, sizeof(gSurfaceHistory[sf]));
        gSurfaceMotion[sf] = 0xFFU;
    }
}

void MmwDemo_SurfaceGen_reset(void)
{
    uint16_t sf;
    gSurfaceDopplerGuardBins = 1;
    for (sf = 0U; sf < MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES; sf++)
    {
        MmwDemo_SurfaceGen_resetSubFrame(sf);
    }
}

#pragma CODE_SECTION(Surface_history, ".surfaceCode")
static uint16_t Surface_history(ObstacleBox *out, uint16_t current, uint16_t cap,
                                const MmwDemo_SurfaceGen_StaticCfg *cfg)
{
    uint16_t sf = cfg->sourceStaticCfg.subFrameNum;
    SurfaceHistory *h = gSurfaceHistory[sf];
    uint8_t matched[SURFACE_HISTORY], matchedCurr[MMWDEMO_SURFACE_MAX_BOXES];
    uint16_t c, p, count = current, held = 0U;
    float dt = (cfg->framePeriodSec > 0.0f) ? cfg->framePeriodSec : ((float)cfg->elapsedUsec * 1e-6f);
    float rStep = cfg->rangeStepMeters;
    float azimBeam = (cfg->physics != NULL && cfg->physics->azimuthBeamwidthRad > 0.0f) ?
        cfg->physics->azimuthBeamwidthRad : ((cfg->numVirtualAntAzim > 0U) ? (2.0f / (float)cfg->numVirtualAntAzim) : 0.25f);
    float elevBeam = (cfg->physics != NULL && cfg->physics->elevationBeamwidthRad > 0.0f) ?
        cfg->physics->elevationBeamwidthRad : 2.0f;
    uint32_t holdUs = (cfg->framePeriodSec > 0.0f) ?
        (uint32_t)(ceilf(0.10f / cfg->framePeriodSec) * cfg->framePeriodSec * 1e6f + 0.5f) :
        (uint32_t)(cfg->elapsedUsec * 2U);
    if (holdUs < 100000UL) { holdUs = 100000UL; }

    memset(matched, 0, sizeof(matched));
    memset(matchedCurr, 0, sizeof(matchedCurr));
    if (gSurfaceMotion[sf] != cfg->motionMode) {
        MmwDemo_SurfaceGen_resetSubFrame(sf);
        gSurfaceMotion[sf] = cfg->motionMode;
    }
    for (p = 0U; p < SURFACE_HISTORY; p++) {
        if (h[p].valid) {
            if (cfg->elapsedUsec > holdUs ||
                h[p].ageUs > holdUs - cfg->elapsedUsec) {
                h[p].valid = 0U;
            } else {
                h[p].ageUs += cfg->elapsedUsec;
            }
        }
    }
    for (c = 0U; c < current; c++) {
        int16_t best = -1;
        float bestDist = 1e9f;
        for (p = 0U; p < SURFACE_HISTORY; p++) {
            float dx, dy, dz, dist;
            float gate_x, gate_y, gate_z;
            if (!h[p].valid || matched[p]) continue;
            dx = fabsf(out[c].x - h[p].box.x);
            dy = fabsf(out[c].y - h[p].box.y);
            dz = fabsf(out[c].z - h[p].box.z);
            dist = dx + dy + dz;

            gate_y = fabsf(h[p].box.velocity) * dt + 2.0f * rStep;
            gate_x = h[p].box.y * azimBeam + 2.0f * rStep;
            gate_z = h[p].box.y * elevBeam + 2.0f * rStep;
            if (gate_x < 2.0f * rStep) gate_x = 2.0f * rStep;
            if (gate_y < 2.0f * rStep) gate_y = 2.0f * rStep;
            if (gate_z < 2.0f * rStep) gate_z = 2.0f * rStep;

            if (dx <= gate_x && dy <= gate_y && dz <= gate_z && dist < bestDist) {
                bestDist = dist;
                best = (int16_t)p;
            }
        }
        if (best >= 0) {
            h[best].box = out[c];
            h[best].ageUs = 0U;
            matched[best] = 1U;
            matchedCurr[c] = 1U;
        }
    }
    if (cfg->motionMode == MMWDEMO_SURFACE_MOTION_FIXED) {
        for (p = 0U; p < SURFACE_HISTORY; p++) {
            if (h[p].valid && !matched[p]) {
                if (count < cap) {
                    out[count++] = h[p].box;
                    held++;
                } else {
                    cfg->status->flags |= MMWDEMO_SURFACE_BOX_LIMIT;
                }
            }
        }
    }
    for (c = 0U; c < current; c++) {
        int16_t slot = -1;
        if (matchedCurr[c]) continue;
        for (p = 0U; p < SURFACE_HISTORY; p++) {
            if (matched[p]) continue;
            if (!h[p].valid) {
                slot = (int16_t)p;
                break;
            }
            if (slot < 0 || h[p].ageUs > h[slot].ageUs) {
                slot = (int16_t)p;
            }
        }
        if (slot >= 0) {
            h[slot].box = out[c];
            h[slot].ageUs = 0U;
            h[slot].valid = 1U;
            matched[slot] = 1U;
        } else {
            cfg->status->flags |= MMWDEMO_SURFACE_HISTORY_LIMIT;
        }
    }
    cfg->status->heldCount = held;
    if (held) cfg->status->flags |= MMWDEMO_SURFACE_HOLD_ACTIVE;
    return count;
}

#pragma CODE_SECTION(Surface_runDbscan, ".surfaceCode")
static uint16_t Surface_runDbscan(
    const DbscanPoint *points,
    uint16_t numPoints,
    DbscanCluster *clusters,
    uint16_t maxClusters,
    const MmwDemo_SurfaceGen_StaticCfg *cfg)
{
    uint16_t numClusters = 0U;
    uint16_t i, j, k;
    float baseTolM = (cfg->clusterRangeTolM > 0.0f) ? cfg->clusterRangeTolM : (1.5f * cfg->rangeStepMeters);
    float kAzim = (cfg->physics != NULL && cfg->physics->azimuthBeamwidthRad > 0.0f) ?
        cfg->physics->azimuthBeamwidthRad :
        ((cfg->numVirtualAntAzim > 0U) ? (2.0f / (float)cfg->numVirtualAntAzim) : 0.25f);
    float baseTolSq = baseTolM * baseTolM;

    if ((numPoints == 0U) || (maxClusters == 0U))
    {
        return 0U;
    }

    for (i = 0U; i < numPoints; i++)
    {
        gDbscanLabels[i] = -1; /* -1 = UNVISITED */
    }

    for (i = 0U; i < numPoints; i++)
    {
        uint16_t neighborCount = 0U;
        uint8_t qHead = 0U;
        uint8_t qTail = 0U;
        float r_i = points[i].r;

        if (gDbscanLabels[i] != -1)
        {
            continue;
        }

        /* Find neighbors of point i with range-adaptive beam divergence: epsSq(R) = baseTolSq + (kAzim*R)^2 */
        for (j = 0U; j < numPoints; j++)
        {
            float rAvg = 0.5f * (r_i + points[j].r);
            float xSpread = kAzim * rAvg;
            float epsSq = baseTolSq + xSpread * xSpread;
            float dx, dy, dz, distSq;

            dx = points[i].x - points[j].x;
            dy = points[i].y - points[j].y;
            dz = points[i].z - points[j].z;
            distSq = dx * dx + dy * dy + dz * dz;

            if (distSq <= epsSq)
            {
                neighborCount++;
                if ((j != i) && (qTail < DBSCAN_MAX_POINTS_PER_GROUP))
                {
                    gDbscanQueue[qTail++] = (uint8_t)j;
                }
            }
        }

        if (neighborCount < DBSCAN_MIN_PTS)
        {
            gDbscanLabels[i] = -2; /* -2 = NOISE */
            continue;
        }

        /* Core point found: start new cluster */
        if (numClusters >= maxClusters)
        {
            break;
        }

        gDbscanLabels[i] = (int16_t)numClusters;

        /* BFS expansion of current cluster */
        while (qHead < qTail)
        {
            uint8_t curr = gDbscanQueue[qHead++];
            int16_t currLabel = gDbscanLabels[curr];
            uint16_t currNeighborCount = 0U;

            if (currLabel == -2)
            {
                /* Convert border noise point to current cluster */
                gDbscanLabels[curr] = (int16_t)numClusters;
            }
            else if (currLabel == -1)
            {
                /* Unvisited point: assign to cluster */
                gDbscanLabels[curr] = (int16_t)numClusters;

                /* Check if curr is also a core point */
                for (k = 0U; k < numPoints; k++)
                {
                    float rAvgK = 0.5f * (points[curr].r + points[k].r);
                    float xSpreadK = kAzim * rAvgK;
                    float epsSqK = baseTolSq + xSpreadK * xSpreadK;
                    float dxK, dyK, dzK, distSqK;

                    dxK = points[curr].x - points[k].x;
                    dyK = points[curr].y - points[k].y;
                    dzK = points[curr].z - points[k].z;
                    distSqK = dxK * dxK + dyK * dyK + dzK * dzK;

                    if (distSqK <= epsSqK)
                    {
                        currNeighborCount++;
                    }
                }

                if (currNeighborCount >= DBSCAN_MIN_PTS)
                {
                    /* Core point: add unvisited/noise neighbors to queue */
                    for (k = 0U; k < numPoints; k++)
                    {
                        if (gDbscanLabels[k] <= -1)
                        {
                            float rAvgK = 0.5f * (points[curr].r + points[k].r);
                            float xSpreadK = kAzim * rAvgK;
                            float epsSqK = baseTolSq + xSpreadK * xSpreadK;
                            float dxK, dyK, dzK, distSqK;

                            dxK = points[curr].x - points[k].x;
                            dyK = points[curr].y - points[k].y;
                            dzK = points[curr].z - points[k].z;
                            distSqK = dxK * dxK + dyK * dyK + dzK * dzK;

                            if (distSqK <= epsSqK)
                            {
                                uint8_t inQueue = 0U;
                                uint8_t q;
                                for (q = qHead; q < qTail; q++)
                                {
                                    if (gDbscanQueue[q] == (uint8_t)k)
                                    {
                                        inQueue = 1U;
                                        break;
                                    }
                                }
                                if ((inQueue == 0U) && (qTail < DBSCAN_MAX_POINTS_PER_GROUP))
                                {
                                    gDbscanQueue[qTail++] = (uint8_t)k;
                                }
                            }
                        }
                    }
                }
            }
        }

        numClusters++;
    }

    /* Initialize cluster bounding boxes */
    for (i = 0U; i < numClusters; i++)
    {
        clusters[i].minX = FLT_MAX;
        clusters[i].maxX = -FLT_MAX;
        clusters[i].minY = FLT_MAX;
        clusters[i].maxY = -FLT_MAX;
        clusters[i].minZ = FLT_MAX;
        clusters[i].maxZ = -FLT_MAX;
        clusters[i].sumVel = 0.0f;
        clusters[i].maxSnr = 0U;
        clusters[i].numPoints = 0U;
    }

    /* Aggregate point coordinates into cluster tight AABB */
    for (i = 0U; i < numPoints; i++)
    {
        int16_t cId = gDbscanLabels[i];
        if ((cId >= 0) && ((uint16_t)cId < numClusters))
        {
            DbscanCluster *c = &clusters[cId];
            if (points[i].x < c->minX) c->minX = points[i].x;
            if (points[i].x > c->maxX) c->maxX = points[i].x;
            if (points[i].y < c->minY) c->minY = points[i].y;
            if (points[i].y > c->maxY) c->maxY = points[i].y;
            if (points[i].z < c->minZ) c->minZ = points[i].z;
            if (points[i].z > c->maxZ) c->maxZ = points[i].z;
            c->sumVel += points[i].velocity;
            if (points[i].snr > c->maxSnr) c->maxSnr = points[i].snr;
            c->numPoints++;
        }
    }

    return numClusters;
}

#pragma CODE_SECTION(Surface_clusterToBox, ".surfaceCode")
static void Surface_clusterToBox(
    const DbscanCluster *c,
    const MmwDemo_SurfaceGen_StaticCfg *cfg,
    uint8_t isDynamic,
    uint16_t boxIdNum,
    ObstacleBox *out)
{
    float cx = 0.5f * (c->minX + c->maxX);
    float cy = 0.5f * (c->minY + c->maxY);
    float cz = 0.5f * (c->minZ + c->maxZ);
    float wx = c->maxX - c->minX;
    float dy = c->maxY - c->minY;
    float hz = c->maxZ - c->minZ;
    float avgVel = (c->numPoints > 0U) ? (c->sumVel / (float)c->numPoints) : 0.0f;
    uint8_t type = MMWDEMO_OBSTACLE_CLASS_UNKNOWN;

    /* Numerical floor to avoid degenerate zero-volume bounding box in visualizer */
    if (wx < 0.05f) wx = 0.05f;
    if (dy < cfg->rangeStepMeters) dy = cfg->rangeStepMeters;
    if (hz < 0.05f) hz = 0.05f;

    out->x = cx;
    out->y = cy;
    out->z = cz;
    out->width_x = wx;
    out->depth_y = dy;
    out->height_z = hz;
    out->velocity = avgVel;
    out->snr = (int16_t)c->maxSnr;

    if (isDynamic != 0U)
    {
        if ((wx <= 1.0f) && (dy <= 1.0f) && (hz <= 0.6f) && (cz >= 0.4f))
        {
            type = MMWDEMO_OBSTACLE_CLASS_DRONE;
        }
        else if ((wx <= 0.9f) && (dy <= 0.9f) && (hz >= 1.0f) && (cz <= 0.5f))
        {
            type = MMWDEMO_OBSTACLE_CLASS_PEDESTRIAN;
        }
        else if (((wx >= 1.1f) || (dy >= 1.5f)) && (c->maxSnr >= 200U))
        {
            type = MMWDEMO_OBSTACLE_CLASS_VEHICLE;
        }
        else
        {
            type = MMWDEMO_OBSTACLE_CLASS_MOVING;
        }
        out->boxId = (int16_t)(MMWDEMO_PACK_OBSTACLE_BOX_ID(type, boxIdNum) | 0x8000);
    }
    else
    {
        type = MMWDEMO_OBSTACLE_CLASS_UNKNOWN;
        out->boxId = MMWDEMO_PACK_OBSTACLE_BOX_ID(type, boxIdNum);
    }
}

#pragma CODE_SECTION(MmwDemo_SurfaceGen_cluster, ".surfaceCode")
int32_t MmwDemo_SurfaceGen_cluster(
    const MmwDemo_DenseElevation_OutputSample *samples, uint16_t n,
    const MmwDemo_SurfaceGen_StaticCfg *cfg,
    ObstacleBox *out, uint16_t cap, uint32_t *count)
{
    uint16_t i, c;
    uint16_t numDyn = 0U;
    uint16_t numStat = 0U;
    uint16_t numDynClusters = 0U;
    uint16_t numStatClusters = 0U;
    uint16_t outIdx = 0U;
    uint16_t numDynamicBoxes;
    uint16_t numStaticBoxes;
    float vRes, vThresh;

    if (count != NULL) *count = 0U;
    if ((cfg == NULL) || (out == NULL) || (count == NULL) || (cfg->status == NULL) ||
        (n > SURFACE_SAMPLES) || ((n != 0U) && (samples == NULL)) || (cap == 0U) ||
        (cfg->sourceStaticCfg.subFrameNum >= MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES) ||
        (cfg->sourceStaticCfg.numRangeBins == 0U) || (cfg->sourceStaticCfg.numDopplerBins == 0U) ||
        (cfg->motionMode > MMWDEMO_SURFACE_MOTION_FIXED) ||
        ((cfg->motionMode == MMWDEMO_SURFACE_MOTION_FIXED) && (cfg->elapsedUsec == 0U)) ||
        !isfinite(cfg->rangeStepMeters) || !(cfg->rangeStepMeters > 0.0f) ||
        !isfinite(cfg->rangeBiasMeters) || !isfinite(cfg->dopplerStepMps) ||
        !(cfg->dopplerStepMps > 0.0f) || !isfinite(cfg->minRangeMeters) ||
        !isfinite(cfg->maxRangeMeters) || (cfg->minRangeMeters < 0.0f) ||
        !(cfg->maxRangeMeters > cfg->minRangeMeters))
    {
        return MMWDEMO_NAV_EINVAL;
    }

    gSurfaceDopplerGuardBins = Surface_calcDopplerGuardBins(cfg);
    if (cap > MMWDEMO_SURFACE_MAX_BOXES)
    {
        cap = MMWDEMO_SURFACE_MAX_BOXES;
    }

    vRes = cfg->dopplerStepMps;
    vThresh = (float)gSurfaceDopplerGuardBins * vRes;

    /* Extract discrete 3D points from samples without aperture beamwidth elongation */
    for (i = 0U; i < n; i++)
    {
        const MmwDemo_DenseElevation_OutputSample *s = &samples[i];
        float r = (float)s->rangeIdx * cfg->rangeStepMeters - cfg->rangeBiasMeters;
        float u = (float)s->azimuthUQ15 / 32767.0f;
        float v = (float)s->sourceVQ15 / 32767.0f;
        float uvSq = u * u + v * v;
        float cosSq, vel;
        DbscanPoint pt;

        if ((r < cfg->minRangeMeters) || (r > cfg->maxRangeMeters) || (r <= 0.0f))
        {
            continue;
        }
        if (uvSq > 1.0f)
        {
            cfg->status->flags |= MMWDEMO_SURFACE_INVALID_GEOMETRY;
            gSurfaceInvalidGeometryCount[cfg->sourceStaticCfg.subFrameNum]++;
            continue;
        }

        cosSq = 1.0f - uvSq;
        if (cosSq < 0.0f)
        {
            cosSq = 0.0f;
        }

        pt.x = r * u;
        pt.y = r * sqrtf(cosSq);
        pt.z = r * v;
        pt.r = r;
        pt.velocity = (float)s->dopplerIdx * vRes;
        pt.snr = s->snrDb10;
        cfg->status->denseClustered++;

        vel = pt.velocity;
        if (fabsf(vel) > vThresh)
        {
            /* Dynamic partition */
            if (numDyn < DBSCAN_MAX_POINTS_PER_GROUP)
            {
                gDbscanDynPoints[numDyn++] = pt;
            }
            else
            {
                uint16_t minIdx = 0U;
                uint16_t m;
                for (m = 1U; m < DBSCAN_MAX_POINTS_PER_GROUP; m++)
                {
                    if (gDbscanDynPoints[m].snr < gDbscanDynPoints[minIdx].snr)
                    {
                        minIdx = m;
                    }
                }
                if (pt.snr > gDbscanDynPoints[minIdx].snr)
                {
                    gDbscanDynPoints[minIdx] = pt;
                }
            }
        }
        else
        {
            /* Static partition */
            if (numStat < DBSCAN_MAX_POINTS_PER_GROUP)
            {
                gDbscanStatPoints[numStat++] = pt;
            }
            else
            {
                uint16_t minIdx = 0U;
                uint16_t m;
                for (m = 1U; m < DBSCAN_MAX_POINTS_PER_GROUP; m++)
                {
                    if (gDbscanStatPoints[m].snr < gDbscanStatPoints[minIdx].snr)
                    {
                        minIdx = m;
                    }
                }
                if (pt.snr > gDbscanStatPoints[minIdx].snr)
                {
                    gDbscanStatPoints[minIdx] = pt;
                }
            }
        }
    }

    /* 1. Run 3D Adaptive DBSCAN on Dynamic points */
    numDynClusters = Surface_runDbscan(
        gDbscanDynPoints, numDyn, gDbscanClusters, cap, cfg);

    for (c = 0U; (c < numDynClusters) && (outIdx < cap); c++)
    {
        if (gDbscanClusters[c].numPoints >= DBSCAN_MIN_PTS)
        {
            Surface_clusterToBox(&gDbscanClusters[c], cfg, 1U, outIdx + 1U, &out[outIdx]);
            outIdx++;
        }
    }
    numDynamicBoxes = outIdx;

    /* 2. Run 3D Adaptive DBSCAN on Static points */
    if (outIdx < cap)
    {
        numStatClusters = Surface_runDbscan(
            gDbscanStatPoints, numStat, gDbscanClusters, cap - outIdx, cfg);

        for (c = 0U; (c < numStatClusters) && (outIdx < cap); c++)
        {
            if (gDbscanClusters[c].numPoints >= DBSCAN_MIN_PTS)
            {
                Surface_clusterToBox(&gDbscanClusters[c], cfg, 0U, outIdx + 1U, &out[outIdx]);
                outIdx++;
            }
        }
    }
    /* Apply temporal tracking if configured for fixed motion */
    if (cfg->motionMode == MMWDEMO_SURFACE_MOTION_FIXED)
    {
        outIdx = Surface_history(out, outIdx, cap, cfg);
    }
    numStaticBoxes = outIdx - numDynamicBoxes;

    /* Sort dynamic boxes by distance y ascending */
    for (i = 1U; i < numDynamicBoxes; i++)
    {
        ObstacleBox key = out[i];
        uint16_t at = i;
        while ((at > 0U) && ((out[at - 1U].y > key.y) ||
               ((out[at - 1U].y == key.y) && (out[at - 1U].x > key.x))))
        {
            out[at] = out[at - 1U];
            at--;
        }
        out[at] = key;
    }

    /* Sort static boxes by distance y ascending */
    for (i = numDynamicBoxes + 1U; i < outIdx; i++)
    {
        ObstacleBox key = out[i];
        uint16_t at = i;
        while ((at > numDynamicBoxes) && ((out[at - 1U].y > key.y) ||
               ((out[at - 1U].y == key.y) && (out[at - 1U].x > key.x))))
        {
            out[at] = out[at - 1U];
            at--;
        }
        out[at] = key;
    }

    /* Update HSRAM split counts and status telemetry */
    gHSRAM.numDynamicObstacles = numDynamicBoxes;
    gHSRAM.numStaticObstacles = numStaticBoxes;
    cfg->status->clusterCount = (uint16_t)(numDynClusters + numStatClusters);
    cfg->status->boxCount = outIdx;
    *count = (uint32_t)outIdx;

    return 0;
}

#pragma CODE_SECTION(MmwDemo_SurfaceGen_process, ".surfaceCode")
int32_t MmwDemo_SurfaceGen_process(
    const MmwDemo_SurfaceGen_Cfg *cfg, const MmwDemo_SurfaceGen_StaticCfg *staticCfg,
    uint8_t *buffer, uint32_t size, MmwDemo_CustomObstacle *out,
    uint16_t maxOutput, uint32_t *count)
{
    MmwDemo_DenseElevation_OutputHeader *header;
    MmwDemo_DenseElevation_OutputSample *samples;
    MmwDemo_DenseElevation_OutputSample rangeSamples[SURFACE_MAX_ROW_SAMPLES];
    uint32_t start, elapsed, totalProduced = 0U;
    uint16_t cap, sf, minRange = 0U, maxRange = 0U, processedCfar = 0U, processedRanges = 0U;
    uint16_t r, k, bufCount = 0U;
    uint8_t cfarClamped = 0U, denseLimited = 0U;
    int32_t ret;

    if (count != NULL) *count = 0U;
    if ((cfg == NULL) || (staticCfg == NULL) || (staticCfg->status == NULL) || (buffer == NULL) ||
        (size < SURFACE_BUFFER_SIZE) || (out == NULL) || (count == NULL) || (maxOutput == 0U) ||
        (cfg->maxSurfacePoints == 0U) ||
        (staticCfg->sourceStaticCfg.subFrameNum >= MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES) ||
        (staticCfg->sourceStaticCfg.numRangeBins == 0U) || (staticCfg->sourceStaticCfg.numDopplerBins == 0U) ||
        (staticCfg->motionMode > MMWDEMO_SURFACE_MOTION_FIXED) ||
        ((staticCfg->motionMode == MMWDEMO_SURFACE_MOTION_FIXED) && (staticCfg->elapsedUsec == 0U)) ||
        !isfinite(staticCfg->rangeStepMeters) || !(staticCfg->rangeStepMeters > 0.0f) ||
        !isfinite(staticCfg->rangeBiasMeters) || !isfinite(staticCfg->dopplerStepMps) ||
        !(staticCfg->dopplerStepMps > 0.0f) || !isfinite(staticCfg->minRangeMeters) ||
        !isfinite(staticCfg->maxRangeMeters) || (staticCfg->minRangeMeters < 0.0f) ||
        !(staticCfg->maxRangeMeters > staticCfg->minRangeMeters))
    {
        return MMWDEMO_NAV_EINVAL;
    }

    sf = staticCfg->sourceStaticCfg.subFrameNum;
    gSurfaceDopplerGuardBins = Surface_calcDopplerGuardBins(staticCfg);
    start = Cycleprofiler_getTimeStamp();

    ret = MmwDemo_DenseElevation_prepareFrame(&cfg->sourceCfg, &staticCfg->sourceStaticCfg,
        &minRange, &maxRange, &processedCfar, &cfarClamped);
    if (ret != 0)
    {
        staticCfg->status->flags |= MMWDEMO_SURFACE_PROCESS_ERROR;
        return ret;
    }
    if (cfarClamped != 0U)
    {
        staticCfg->status->flags |= MMWDEMO_SURFACE_CFAR_LIMIT;
    }

    header = (MmwDemo_DenseElevation_OutputHeader *)buffer;
    samples = (MmwDemo_DenseElevation_OutputSample *)(buffer + sizeof(*header));

    for (r = minRange; r <= maxRange; r++)
    {
        uint16_t nRange = 0U;
        uint8_t rangeLim = 0U;
        if (MmwDemo_DenseElevation_processRange(r, &cfg->sourceCfg, &staticCfg->sourceStaticCfg,
                rangeSamples, SURFACE_MAX_ROW_SAMPLES, &nRange, &rangeLim) != 0U)
        {
            processedRanges++;
            if (rangeLim != 0U)
            {
                denseLimited = 1U;
            }
            if (nRange > 0U)
            {
                totalProduced += nRange;
                for (k = 0U; (k < nRange) && (bufCount < SURFACE_SAMPLES); k++)
                {
                    samples[bufCount++] = rangeSamples[k];
                }
            }
        }
    }

    /* Populate TLV20 dense point cloud header with complete spherical FOV metadata */
    memset(header, 0, sizeof(*header));
    header->magic = MMWDEMO_DENSE_ELEVATION_MAGIC;
    header->version = MMWDEMO_DENSE_ELEVATION_PROTOCOL_VERSION;
    header->headerSize = sizeof(*header);
    header->sampleSize = sizeof(*samples);
    header->numSamples = bufCount;
    header->flags = MMWDEMO_DENSE_REQUIRED_FLAGS |
        (((denseLimited != 0U) || (totalProduced > SURFACE_SAMPLES)) ? MMWDEMO_DENSE_FLAG_OUTPUT_LIMITED : 0U) |
        (staticCfg->sourceStaticCfg.staticClutterRemovalEnabled ? MMWDEMO_DENSE_FLAG_STATIC_CLUTTER_APPLIED : 0U);
    header->subFrameNumber = sf;
    header->rangeStart = minRange;
    header->rangeEnd = maxRange + 1U;
    header->numRangeBins = staticCfg->sourceStaticCfg.numRangeBins;
    header->numDopplerBins = staticCfg->sourceStaticCfg.numDopplerBins;
    header->numAzimuthBins = MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
    header->numElevationBins = MMWDEMO_DENSE_ELEV_NUM_ELEVATION_BINS;
    header->minAzimuthDeg10 = (int16_t)(cfg->sourceCfg.minAzimuthDeg * 10.0f);
    header->maxAzimuthDeg10 = (int16_t)(cfg->sourceCfg.maxAzimuthDeg * 10.0f);
    header->minElevationDeg10 = (int16_t)(cfg->sourceCfg.minElevationDeg * 10.0f);
    header->maxElevationDeg10 = (int16_t)(cfg->sourceCfg.maxElevationDeg * 10.0f);
    header->radarFrameNumber = staticCfg->sourceStaticCfg.frameNumber;
    header->reservedPad = (uint32_t)processedCfar | ((uint32_t)processedRanges << 16U);

    staticCfg->status->cfarProcessed = processedCfar;
    staticCfg->status->rangeCount = processedRanges;
    staticCfg->status->denseProduced = (totalProduced > 65535U) ? 65535U : (uint16_t)totalProduced;
    if (denseLimited != 0U)
    {
        staticCfg->status->flags |= MMWDEMO_SURFACE_DENSE_LIMIT;
    }

    cap = (cfg->maxSurfacePoints < maxOutput) ? cfg->maxSurfacePoints : maxOutput;
    if ((cfg->uartMaxSurfacePoints != 0U) && (cfg->uartMaxSurfacePoints < cap))
    {
        cap = cfg->uartMaxSurfacePoints;
        staticCfg->status->flags |= MMWDEMO_SURFACE_UART_LIMIT;
    }
    if (cap > MMWDEMO_SURFACE_MAX_BOXES)
    {
        cap = MMWDEMO_SURFACE_MAX_BOXES;
    }

    gNavDbgDenseSamples = staticCfg->status->denseProduced;

    /* Execute 3D Adaptive DBSCAN clustering directly on discrete candidate point cloud */
    ret = MmwDemo_SurfaceGen_cluster(samples, bufCount, staticCfg, out, cap, count);

    elapsed = Cycleprofiler_getTimeStamp() - start;
    header->processingCycles = elapsed;
    gNavEmitSurfaceCycles[sf] = elapsed;
    if (elapsed > gNavEmitSurfaceMaxCycles[sf])
    {
        gNavEmitSurfaceMaxCycles[sf] = elapsed;
    }
    gNavDbgFilteredSamples = *count;

    return ret;
}

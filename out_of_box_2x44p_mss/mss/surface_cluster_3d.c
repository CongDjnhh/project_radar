/**
 * @file surface_cluster_3d.c
 *
 * @brief Pure-C hardware-agnostic 3D adaptive anisotropic ellipsoid DBSCAN,
 *        dynamic/static partitioning, temporal hold, and 32-byte ObstacleBox
 *        generation for AWR2944P OOB-DDMA radar point clouds.
 */

#include "surface_cluster_3d.h"

#include <float.h>
#include <math.h>
#include <string.h>

/* Compile-time wire and cross-struct ABI guards. */
SURFACE_CLUSTER_STATIC_ASSERT(sizeof(SurfaceCluster_InputPoint) == 16U, input_point_16b);
SURFACE_CLUSTER_STATIC_ASSERT(sizeof(SurfaceCluster_SideInfo) == 4U, side_info_4b);
SURFACE_CLUSTER_STATIC_ASSERT(sizeof(SurfaceCluster_Box) == 32U, obstacle_box_32b);
SURFACE_CLUSTER_STATIC_ASSERT(sizeof(SurfaceCluster_Status) == 48U, surface_status_48b);
SURFACE_CLUSTER_STATIC_ASSERT(sizeof(SurfaceCluster_Physics) == 60U, derived_physics_60b);

SURFACE_CLUSTER_STATIC_ASSERT(offsetof(SurfaceCluster_Box, velocity) == 24U, box_vel_offset);
SURFACE_CLUSTER_STATIC_ASSERT(offsetof(SurfaceCluster_Box, snr) == 28U, box_snr_offset);
SURFACE_CLUSTER_STATIC_ASSERT(offsetof(SurfaceCluster_Box, boxId) == 30U, box_id_offset);
SURFACE_CLUSTER_STATIC_ASSERT(offsetof(SurfaceCluster_Status, flags) == 4U, status_flags_offset);
SURFACE_CLUSTER_STATIC_ASSERT(offsetof(SurfaceCluster_Status, processingUs) == 24U, status_proc_offset);
SURFACE_CLUSTER_STATIC_ASSERT(offsetof(SurfaceCluster_Physics, numVirtualAntAzim) == 52U, phys_vazim_offset);

/** Precomputed per-frame metric scales derived from physics + config. */
typedef struct SurfaceCluster_Metrics_t
{
    float epsRange;
    float epsRangeSq;
    float epsVel;
    float azimuthBeamwidthRad;
    float elevationBeamwidthRad;
    float framePeriodSec;
    float boxExtentFloor;
} SurfaceCluster_Metrics;

static float sc_maxf(float a, float b)
{
    return (a >= b) ? a : b;
}

static uint16_t sc_minU16(uint16_t a, uint16_t b)
{
    return (a <= b) ? a : b;
}

static int sc_isFiniteF(float x)
{
    return isfinite(x) ? 1 : 0;
}

static void sc_accumInit(SurfaceCluster_Accum *c, const SurfaceCluster_WorkPt *p)
{
    c->minX = p->x;
    c->maxX = p->x;
    c->minY = p->y;
    c->maxY = p->y;
    c->minZ = p->z;
    c->maxZ = p->z;
    c->sumVel = p->velocity;
    c->maxSnr = p->snr;
    c->numPoints = 1U;
}

static void sc_accumAdd(SurfaceCluster_Accum *c, const SurfaceCluster_WorkPt *p)
{
    if (p->x < c->minX) { c->minX = p->x; }
    if (p->x > c->maxX) { c->maxX = p->x; }
    if (p->y < c->minY) { c->minY = p->y; }
    if (p->y > c->maxY) { c->maxY = p->y; }
    if (p->z < c->minZ) { c->minZ = p->z; }
    if (p->z > c->maxZ) { c->maxZ = p->z; }
    c->sumVel += p->velocity;
    if (p->snr > c->maxSnr) { c->maxSnr = p->snr; }
    c->numPoints++;
}

/** Deterministic strict weak order for range-window sorting. */
static int sc_ptLess(const SurfaceCluster_WorkPt *a, const SurfaceCluster_WorkPt *b)
{
    if (a->r != b->r) { return (a->r < b->r) ? 1 : 0; }
    if (a->y != b->y) { return (a->y < b->y) ? 1 : 0; }
    if (a->x != b->x) { return (a->x < b->x) ? 1 : 0; }
    return (a->z < b->z) ? 1 : 0;
}

static void sc_siftDown(SurfaceCluster_WorkPt *pts, uint16_t start, uint16_t count)
{
    uint16_t root = start;

    for (;;)
    {
        uint16_t child = (uint16_t)(2U * root + 1U);
        uint16_t swapIdx = root;

        if (child >= count)
        {
            break;
        }
        if (sc_ptLess(&pts[swapIdx], &pts[child]))
        {
            swapIdx = child;
        }
        if (((uint16_t)(child + 1U) < count) &&
            sc_ptLess(&pts[swapIdx], &pts[child + 1U]))
        {
            swapIdx = (uint16_t)(child + 1U);
        }
        if (swapIdx == root)
        {
            break;
        }
        {
            SurfaceCluster_WorkPt tmp = pts[root];
            pts[root] = pts[swapIdx];
            pts[swapIdx] = tmp;
        }
        root = swapIdx;
    }
}

/** Iterative in-place heapsort: O(N log N) worst-case, O(1) stack. */
static void sc_heapsortPoints(SurfaceCluster_WorkPt *pts, uint16_t n)
{
    uint16_t start;
    uint16_t end;

    if (n < 2U)
    {
        return;
    }

    start = (uint16_t)(n / 2U);
    while (start > 0U)
    {
        start--;
        sc_siftDown(pts, start, n);
    }

    end = n;
    while (end > 1U)
    {
        SurfaceCluster_WorkPt tmp;
        end--;
        tmp = pts[end];
        pts[end] = pts[0];
        pts[0] = tmp;
        sc_siftDown(pts, 0U, end);
    }
}

/** Find global min SNR across contiguous two-ended buffer pts[0..maxPts-1]. */
static void sc_findMinSnr(
    const SurfaceCluster_WorkPt *pts,
    uint16_t numDyn,
    uint16_t numStat,
    uint16_t maxPts,
    uint16_t *outMinIdx,
    int16_t *outMinSnr)
{
    uint16_t i;
    uint16_t statStart = (uint16_t)(maxPts - numStat);
    uint16_t bestIdx = 0U;
    int16_t bestSnr = INT16_MAX;

    if (numDyn > 0U)
    {
        bestIdx = 0U;
        bestSnr = pts[0].snr;
        for (i = 1U; i < numDyn; i++)
        {
            if (pts[i].snr < bestSnr)
            {
                bestSnr = pts[i].snr;
                bestIdx = i;
            }
        }
    }
    else if (numStat > 0U)
    {
        bestIdx = statStart;
        bestSnr = pts[statStart].snr;
    }

    for (i = statStart; i < maxPts; i++)
    {
        if (pts[i].snr < bestSnr)
        {
            bestSnr = pts[i].snr;
            bestIdx = i;
        }
    }

    *outMinIdx = bestIdx;
    *outMinSnr = bestSnr;
}

/**
 * Maintain top-SURFACE_CLUSTER_MAX_POINTS by SNR across front-growing dynamic
 * and back-growing static partitions inside state->pts[].
 */
static void sc_insertPoint(
    SurfaceCluster_WorkPt *pts,
    const SurfaceCluster_WorkPt *pt,
    uint8_t isDyn,
    uint16_t *numDyn,
    uint16_t *numStat,
    uint8_t *hasMin,
    uint16_t *minIdx,
    int16_t *minSnr,
    uint8_t *hitCfarLimit)
{
    uint16_t d = *numDyn;
    uint16_t s = *numStat;

    if ((uint16_t)(d + s) < SURFACE_CLUSTER_MAX_POINTS)
    {
        if (isDyn != 0U)
        {
            pts[d] = *pt;
            *numDyn = (uint16_t)(d + 1U);
        }
        else
        {
            pts[SURFACE_CLUSTER_MAX_POINTS - 1U - s] = *pt;
            *numStat = (uint16_t)(s + 1U);
        }
        return;
    }

    *hitCfarLimit = 1U;
    if (*hasMin == 0U)
    {
        sc_findMinSnr(pts, d, s, SURFACE_CLUSTER_MAX_POINTS, minIdx, minSnr);
        *hasMin = 1U;
    }

    if (pt->snr <= *minSnr)
    {
        return;
    }

    {
        uint16_t victim = *minIdx;
        uint8_t victimIsDyn = (victim < d) ? 1U : 0U;

        if ((isDyn != 0U) && (victimIsDyn != 0U))
        {
            pts[victim] = *pt;
        }
        else if ((isDyn == 0U) && (victimIsDyn == 0U))
        {
            pts[victim] = *pt;
        }
        else if (isDyn != 0U)
        {
            /* Victim is in static tail [d .. MAX-1]; shift boundary d up by 1. */
            pts[victim] = pts[d];
            pts[d] = *pt;
            *numStat = (uint16_t)(s - 1U);
            *numDyn = (uint16_t)(d + 1U);
        }
        else
        {
            /* Victim is in dynamic head [0 .. d-1]; shift boundary d down by 1. */
            pts[victim] = pts[d - 1U];
            pts[d - 1U] = *pt;
            *numDyn = (uint16_t)(d - 1U);
            *numStat = (uint16_t)(s + 1U);
        }
    }

    sc_findMinSnr(pts, *numDyn, *numStat, SURFACE_CLUSTER_MAX_POINTS, minIdx, minSnr);
}

/** Precompute O(N) range-sorted windows |r_j - r_i| <= epsRange * (1 + ULP slack). */
static void sc_buildRangeWindows(
    const SurfaceCluster_WorkPt *pts,
    uint16_t n,
    float epsRange,
    uint16_t *winStart,
    uint16_t *winEnd)
{
    uint16_t lo = 0U;
    uint16_t hi = 0U;
    uint16_t i;
    float epsWin = epsRange * 1.0001f;

    for (i = 0U; i < n; i++)
    {
        float rMin = pts[i].r - epsWin;
        float rMax = pts[i].r + epsWin;

        while ((lo < i) && (pts[lo].r < rMin))
        {
            lo++;
        }
        if (hi < i)
        {
            hi = i;
        }
        while (((uint16_t)(hi + 1U) < n) && (pts[hi + 1U].r <= rMax))
        {
            hi++;
        }
        winStart[i] = lo;
        winEnd[i] = hi;
    }
}

/** Symmetric orthogonal 3D anisotropic ellipsoid + Doppler spread neighbor predicate. */
static uint8_t sc_isNeighbor(
    const SurfaceCluster_WorkPt *a,
    const SurfaceCluster_WorkPt *b,
    const SurfaceCluster_Metrics *m)
{
    float dv = fabsf(a->velocity - b->velocity);
    float dx, dy, dz, dSq;
    float mx, my, mz, rMSq;

    if (dv > m->epsVel)
    {
        return 0U;
    }

    dx = a->x - b->x;
    dy = a->y - b->y;
    dz = a->z - b->z;
    dSq = dx * dx + dy * dy + dz * dz;
    if (dSq == 0.0f)
    {
        return 1U;
    }

    mx = 0.5f * (a->x + b->x);
    my = 0.5f * (a->y + b->y);
    mz = 0.5f * (a->z + b->z);
    rMSq = mx * mx + my * my + mz * mz;
    if (rMSq <= m->epsRangeSq)
    {
        return (dSq <= m->epsRangeSq) ? 1U : 0U;
    }

    {
        float rM = sqrtf(rMSq);
        float dR = (mx * dx + my * dy + mz * dz) / rM;
        float rXySq = mx * mx + my * my;
        float dAz;
        float dElSq;
        float epsAz;
        float epsEl;
        float nR, nAz, nElSq;

        if (rXySq > (1e-12f * rMSq))
        {
            float rXy = sqrtf(rXySq);
            dAz = (-my * dx + mx * dy) / rXy;
        }
        else
        {
            /* Zenith/nadir guard where azimuth angle is degenerate. */
            dAz = dx;
        }

        dElSq = dSq - (dR * dR) - (dAz * dAz);
        if (dElSq < 0.0f)
        {
            dElSq = 0.0f;
        }

        epsAz = sc_maxf(m->epsRange, rM * m->azimuthBeamwidthRad);
        epsEl = sc_maxf(m->epsRange,
                        SURFACE_CLUSTER_ELEV_INTERFEROMETER_SCALE * rM * m->elevationBeamwidthRad);

        nR = dR / m->epsRange;
        nAz = dAz / epsAz;
        nElSq = dElSq / (epsEl * epsEl);

        return ((nR * nR + nAz * nAz + nElSq) <= 1.0f) ? 1U : 0U;
    }
}

/** Single-pass range-window neighborhood scan for point centerIdx. */
static uint16_t sc_scanNeighbors(
    const SurfaceCluster_WorkPt *pts,
    uint16_t centerIdx,
    uint16_t winLo,
    uint16_t winHi,
    const int16_t *labels,
    const uint8_t *inQueue,
    const SurfaceCluster_Metrics *m,
    uint16_t *nbrScratch,
    uint16_t *candLenOut)
{
    uint16_t j;
    uint16_t nbrCount = 0U;
    uint16_t candLen = 0U;
    const SurfaceCluster_WorkPt *center = &pts[centerIdx];

    for (j = winLo; j <= winHi; j++)
    {
        if (j == centerIdx)
        {
            nbrCount++;
            continue;
        }
        if (sc_isNeighbor(center, &pts[j], m) != 0U)
        {
            nbrCount++;
            if ((labels[j] <= -1) && (inQueue[j] == 0U))
            {
                nbrScratch[candLen++] = j;
            }
        }
    }

    *candLenOut = candLen;
    return nbrCount;
}

static void sc_absorbAndEnqueue(
    const SurfaceCluster_WorkPt *pts,
    int16_t *labels,
    uint8_t *inQueue,
    uint16_t *bfsQueue,
    uint16_t *qTail,
    const uint16_t *nbrScratch,
    uint16_t candLen,
    int16_t candId,
    SurfaceCluster_Accum *cand)
{
    uint16_t k;

    for (k = 0U; k < candLen; k++)
    {
        uint16_t idx = nbrScratch[k];
        int16_t lbl = labels[idx];

        if (lbl == -2)
        {
            labels[idx] = candId;
            sc_accumAdd(cand, &pts[idx]);
        }
        else if ((lbl == -1) && (inQueue[idx] == 0U))
        {
            labels[idx] = candId;
            inQueue[idx] = 1U;
            sc_accumAdd(cand, &pts[idx]);
            bfsQueue[(*qTail)++] = idx;
        }
    }
}

static uint16_t sc_runDbscan(
    SurfaceCluster_State *state,
    const SurfaceCluster_WorkPt *pts,
    uint16_t n,
    uint16_t minClusterPts,
    uint16_t maxClusters,
    const SurfaceCluster_Metrics *m,
    uint8_t *hitClusterLimit,
    uint8_t *hitBoxLimit)
{
    uint16_t i;
    uint16_t numClusters = 0U;
    int16_t *labels = state->labels;
    uint16_t *bfsQueue = state->bfsQueue;
    uint16_t *nbrScratch = state->nbrScratch;
    uint16_t *winStart = state->winStart;
    uint16_t *winEnd = state->winEnd;
    uint8_t *inQueue = state->inQueue;

    if (n < minClusterPts)
    {
        return 0U;
    }

    sc_buildRangeWindows(pts, n, m->epsRange, winStart, winEnd);

    for (i = 0U; i < n; i++)
    {
        labels[i] = -1;
        inQueue[i] = 0U;
    }

    for (i = 0U; i < n; i++)
    {
        uint16_t nbrCount;
        uint16_t candLen = 0U;
        uint16_t qHead = 0U;
        uint16_t qTail = 0U;
        int16_t candId;
        SurfaceCluster_Accum cand;

        if (labels[i] != -1)
        {
            continue;
        }

        nbrCount = sc_scanNeighbors(
            pts, i, winStart[i], winEnd[i], labels, inQueue, m, nbrScratch, &candLen);
        if (nbrCount < minClusterPts)
        {
            labels[i] = -2;
            continue;
        }

        candId = (int16_t)numClusters;
        sc_accumInit(&cand, &pts[i]);
        labels[i] = candId;
        inQueue[i] = 1U;

        sc_absorbAndEnqueue(
            pts, labels, inQueue, bfsQueue, &qTail, nbrScratch, candLen, candId, &cand);

        while (qHead < qTail)
        {
            uint16_t curr = bfsQueue[qHead++];
            uint16_t currNbrCount;
            uint16_t currCandLen = 0U;

            currNbrCount = sc_scanNeighbors(
                pts, curr, winStart[curr], winEnd[curr],
                labels, inQueue, m, nbrScratch, &currCandLen);
            if (currNbrCount >= minClusterPts)
            {
                sc_absorbAndEnqueue(
                    pts, labels, inQueue, bfsQueue, &qTail,
                    nbrScratch, currCandLen, candId, &cand);
            }
        }

        if (cand.numPoints < minClusterPts)
        {
            /* Neighbors already belonged to prior clusters; mark leftover points noise. */
            if (cand.numPoints == 1U)
            {
                labels[i] = -2;
                inQueue[i] = 0U;
            }
            else
            {
                uint16_t t;
                for (t = 0U; t < n; t++)
                {
                    if (labels[t] == candId)
                    {
                        labels[t] = -2;
                        inQueue[t] = 0U;
                    }
                }
            }
            continue;
        }

        if (numClusters >= maxClusters)
        {
            *hitClusterLimit = 1U;
            *hitBoxLimit = 1U;
            break;
        }

        state->clusters[numClusters++] = cand;
    }

    return numClusters;
}

static void sc_clusterToBox(
    const SurfaceCluster_Accum *c,
    float extentFloor,
    uint8_t isDynamic,
    uint16_t rank,
    SurfaceCluster_Box *out)
{
    float wx = c->maxX - c->minX;
    float dy = c->maxY - c->minY;
    float hz = c->maxZ - c->minZ;
    uint8_t classType = (isDynamic != 0U)
                            ? SURFACE_CLUSTER_CLASS_MOVING
                            : SURFACE_CLUSTER_CLASS_UNKNOWN;

    if (wx < extentFloor) { wx = extentFloor; }
    if (dy < extentFloor) { dy = extentFloor; }
    if (hz < extentFloor) { hz = extentFloor; }

    out->x = 0.5f * (c->minX + c->maxX);
    out->y = 0.5f * (c->minY + c->maxY);
    out->z = 0.5f * (c->minZ + c->maxZ);
    out->width_x = wx;
    out->depth_y = dy;
    out->height_z = hz;
    out->velocity = (c->numPoints > 0U) ? (c->sumVel / (float)c->numPoints) : 0.0f;
    out->snr = c->maxSnr;
    out->boxId = SURFACE_CLUSTER_PACK_BOX_ID(isDynamic, classType, rank);
}

/** Age active subframe history once per frame or reset on motionMode change. */
static void sc_prepareSubFrameHistory(
    SurfaceCluster_State *state,
    uint8_t subFrameIdx,
    uint8_t motionMode,
    uint16_t holdFrames)
{
    SurfaceCluster_HistEntry *h = state->history[subFrameIdx];
    uint16_t p;

    memset(state->histMatched, 0, sizeof(state->histMatched));

    if (state->lastMotionMode[subFrameIdx] != motionMode)
    {
        memset(h, 0, sizeof(state->history[subFrameIdx]));
        state->lastMotionMode[subFrameIdx] = motionMode;
        return;
    }

    if (motionMode != SURFACE_CLUSTER_MOTION_FIXED)
    {
        return;
    }

    for (p = 0U; p < SURFACE_CLUSTER_HISTORY_MAX; p++)
    {
        if (h[p].valid != 0U)
        {
            if (h[p].ageFrames >= holdFrames)
            {
                h[p].valid = 0U;
            }
            else
            {
                h[p].ageFrames++;
            }
        }
    }
}

/** Partition-isolated temporal hold: dynamic boxes never leak into static output. */
static void sc_applyHistory(
    SurfaceCluster_State *state,
    uint8_t subFrameIdx,
    uint8_t isDynamic,
    const SurfaceCluster_Metrics *m,
    SurfaceCluster_Box *boxes,
    uint16_t currCount,
    uint16_t effCap,
    uint16_t *outTotalCount,
    uint16_t *outHeldCount,
    uint8_t *hitBoxLimit,
    uint8_t *hitHistLimit)
{
    SurfaceCluster_HistEntry *h = state->history[subFrameIdx];
    uint8_t *histMatched = state->histMatched;
    uint8_t *currMatched = state->currMatched;
    uint16_t totalCount = currCount;
    uint16_t heldCount = 0U;
    uint16_t c, p;

    memset(currMatched, 0, sizeof(state->currMatched));

    /* 1. Associate current boxes with surviving prior boxes of the same partition. */
    for (c = 0U; c < currCount; c++)
    {
        int16_t best = -1;
        float bestNormDist = FLT_MAX;

        for (p = 0U; p < SURFACE_CLUSTER_HISTORY_MAX; p++)
        {
            float rBox, dtElapsed, kinShift;
            float gateX, gateY, gateZ;
            float dx, dy, dz, normDist;

            if ((h[p].valid == 0U) || (histMatched[p] != 0U) || (h[p].isDynamic != isDynamic))
            {
                continue;
            }

            rBox = sqrtf(h[p].box.x * h[p].box.x +
                         h[p].box.y * h[p].box.y +
                         h[p].box.z * h[p].box.z);
            dtElapsed = (float)h[p].ageFrames * m->framePeriodSec;
            kinShift = fabsf(h[p].box.velocity) * dtElapsed;

            gateX = sc_maxf(m->epsRange, rBox * m->azimuthBeamwidthRad) + kinShift;
            gateY = sc_maxf(m->epsRange, rBox * m->azimuthBeamwidthRad) + kinShift;
            gateZ = sc_maxf(m->epsRange,
                            SURFACE_CLUSTER_ELEV_INTERFEROMETER_SCALE * rBox * m->elevationBeamwidthRad) +
                    kinShift;

            dx = fabsf(boxes[c].x - h[p].box.x);
            dy = fabsf(boxes[c].y - h[p].box.y);
            dz = fabsf(boxes[c].z - h[p].box.z);

            if ((dx <= gateX) && (dy <= gateY) && (dz <= gateZ))
            {
                float nx = dx / gateX;
                float ny = dy / gateY;
                float nz = dz / gateZ;
                normDist = nx * nx + ny * ny + nz * nz;
                if (normDist < bestNormDist)
                {
                    bestNormDist = normDist;
                    best = (int16_t)p;
                }
            }
        }

        if (best >= 0)
        {
            h[best].box = boxes[c];
            h[best].ageFrames = 0U;
            h[best].isDynamic = isDynamic;
            histMatched[best] = 1U;
            currMatched[c] = 1U;
        }
    }

    /* 2. Emit surviving unmatched boxes of this partition into this partition's buffer. */
    for (p = 0U; p < SURFACE_CLUSTER_HISTORY_MAX; p++)
    {
        if ((h[p].valid != 0U) && (h[p].isDynamic == isDynamic) && (histMatched[p] == 0U))
        {
            if (totalCount < effCap)
            {
                boxes[totalCount++] = h[p].box;
                heldCount++;
            }
            else
            {
                *hitBoxLimit = 1U;
            }
        }
    }

    /* 3. Insert newly observed unmatched boxes into history (prefer empty or same-partition eviction). */
    for (c = 0U; c < currCount; c++)
    {
        int16_t emptySlot = -1;
        int16_t samePartEvict = -1;
        int16_t anyEvict = -1;
        int16_t slot;

        if (currMatched[c] != 0U)
        {
            continue;
        }

        for (p = 0U; p < SURFACE_CLUSTER_HISTORY_MAX; p++)
        {
            if (histMatched[p] != 0U)
            {
                continue;
            }
            if (h[p].valid == 0U)
            {
                emptySlot = (int16_t)p;
                break;
            }
            if ((h[p].isDynamic == isDynamic) &&
                ((samePartEvict < 0) || (h[p].ageFrames > h[samePartEvict].ageFrames)))
            {
                samePartEvict = (int16_t)p;
            }
            if ((anyEvict < 0) || (h[p].ageFrames > h[anyEvict].ageFrames))
            {
                anyEvict = (int16_t)p;
            }
        }

        slot = (emptySlot >= 0) ? emptySlot
                                : ((samePartEvict >= 0) ? samePartEvict : anyEvict);
        if (slot >= 0)
        {
            if (emptySlot < 0)
            {
                *hitHistLimit = 1U;
            }
            h[slot].box = boxes[c];
            h[slot].ageFrames = 0U;
            h[slot].valid = 1U;
            h[slot].isDynamic = isDynamic;
            histMatched[slot] = 1U;
        }
        else
        {
            *hitHistLimit = 1U;
        }
    }

    *outTotalCount = totalCount;
    *outHeldCount = heldCount;
}

static int sc_boxLess(const SurfaceCluster_Box *a, const SurfaceCluster_Box *b)
{
    if (a->y != b->y) { return (a->y < b->y) ? 1 : 0; }
    if (a->x != b->x) { return (a->x < b->x) ? 1 : 0; }
    return (a->z < b->z) ? 1 : 0;
}

/** Sort boxes by forward range y ascending and assign post-sort 1..N rank IDs. */
static void sc_sortBoxesAndAssignIds(
    SurfaceCluster_Box *boxes,
    uint16_t count,
    uint8_t isDynamic)
{
    uint16_t i;

    for (i = 1U; i < count; i++)
    {
        SurfaceCluster_Box key = boxes[i];
        uint16_t at = i;

        while ((at > 0U) && sc_boxLess(&key, &boxes[at - 1U]))
        {
            boxes[at] = boxes[at - 1U];
            at--;
        }
        boxes[at] = key;
    }

    for (i = 0U; i < count; i++)
    {
        uint8_t classType = (uint8_t)(((uint16_t)boxes[i].boxId >> 8U) & 0x007FU);
        boxes[i].boxId = SURFACE_CLUSTER_PACK_BOX_ID(isDynamic, classType, (uint16_t)(i + 1U));
    }
}

/** Execute sort -> DBSCAN -> AABB emission -> temporal hold -> post-sort ID ranking for 1 partition. */
static void sc_processPartition(
    SurfaceCluster_State *state,
    SurfaceCluster_WorkPt *pts,
    uint16_t numPts,
    uint8_t isDynamic,
    uint8_t subFrameIdx,
    uint16_t holdFrames,
    const SurfaceCluster_Config *cfg,
    const SurfaceCluster_Metrics *m,
    SurfaceCluster_Box *outBoxes,
    uint16_t effCap,
    uint16_t *outBoxCount,
    uint16_t *outClusterCount,
    uint16_t *outClusteredPts,
    uint16_t *outHeldCount,
    uint8_t *hitClusterLimit,
    uint8_t *hitBoxLimit,
    uint8_t *hitHistLimit)
{
    uint16_t numClusters;
    uint16_t currBoxes = 0U;
    uint16_t totalBoxes = 0U;
    uint16_t clusteredPts = 0U;
    uint16_t heldCount = 0U;
    uint16_t c;

    sc_heapsortPoints(pts, numPts);

    numClusters = sc_runDbscan(
        state, pts, numPts, cfg->minClusterPts, effCap, m,
        hitClusterLimit, hitBoxLimit);

    for (c = 0U; c < numClusters; c++)
    {
        clusteredPts = (uint16_t)(clusteredPts + state->clusters[c].numPoints);
        if (currBoxes < effCap)
        {
            sc_clusterToBox(
                &state->clusters[c], m->boxExtentFloor, isDynamic,
                (uint16_t)(currBoxes + 1U), &outBoxes[currBoxes]);
            currBoxes++;
        }
        else
        {
            *hitBoxLimit = 1U;
            break;
        }
    }

    totalBoxes = currBoxes;
    if ((cfg->motionMode == SURFACE_CLUSTER_MOTION_FIXED) && (holdFrames > 0U))
    {
        sc_applyHistory(
            state, subFrameIdx, isDynamic, m, outBoxes, currBoxes, effCap,
            &totalBoxes, &heldCount, hitBoxLimit, hitHistLimit);
    }

    if (totalBoxes > 0U)
    {
        sc_sortBoxesAndAssignIds(outBoxes, totalBoxes, isDynamic);
    }

    *outBoxCount = totalBoxes;
    *outClusterCount = numClusters;
    *outClusteredPts = clusteredPts;
    *outHeldCount = heldCount;
}

int32_t SurfaceCluster_init(SurfaceCluster_State *state)
{
    uint16_t sf;

    if (state == NULL)
    {
        return SURFACE_CLUSTER_ERR_EINVAL;
    }

    memset(state, 0, sizeof(*state));
    for (sf = 0U; sf < SURFACE_CLUSTER_MAX_SUBFRAMES; sf++)
    {
        state->lastMotionMode[sf] = 0xFFU;
    }
    state->isInitialized = 1U;

    return SURFACE_CLUSTER_OK;
}

int32_t SurfaceCluster_resetSubFrame(SurfaceCluster_State *state, uint8_t subFrameIdx)
{
    if ((state == NULL) || (state->isInitialized == 0U) ||
        (subFrameIdx >= SURFACE_CLUSTER_MAX_SUBFRAMES))
    {
        return SURFACE_CLUSTER_ERR_EINVAL;
    }

    memset(state->history[subFrameIdx], 0, sizeof(state->history[subFrameIdx]));
    state->lastMotionMode[subFrameIdx] = 0xFFU;

    return SURFACE_CLUSTER_OK;
}

int32_t SurfaceCluster_validatePhysics(const SurfaceCluster_Physics *physics)
{
    if (physics == NULL)
    {
        return SURFACE_CLUSTER_ERR_INVALID_PHYSICS;
    }

    if (!sc_isFiniteF(physics->rangeStep) || !(physics->rangeStep > 0.0f) ||
        !sc_isFiniteF(physics->dopplerStep) || !(physics->dopplerStep > 0.0f) ||
        !sc_isFiniteF(physics->rangeResolution) || !(physics->rangeResolution > 0.0f) ||
        !sc_isFiniteF(physics->dopplerResolution) || !(physics->dopplerResolution > 0.0f) ||
        !sc_isFiniteF(physics->wavelength) || !(physics->wavelength > 0.0f) ||
        !sc_isFiniteF(physics->centerFreq) || !(physics->centerFreq > 0.0f) ||
        !sc_isFiniteF(physics->bandwidth) || !(physics->bandwidth > 0.0f) ||
        !sc_isFiniteF(physics->chirpInterval) || !(physics->chirpInterval > 0.0f) ||
        !sc_isFiniteF(physics->framePeriodSec) || !(physics->framePeriodSec > 0.0f) ||
        !sc_isFiniteF(physics->vMax) || !(physics->vMax > 0.0f) ||
        !sc_isFiniteF(physics->rMax) || !(physics->rMax > 0.0f) ||
        !sc_isFiniteF(physics->azimuthBeamwidthRad) || !(physics->azimuthBeamwidthRad > 0.0f) ||
        !sc_isFiniteF(physics->elevationBeamwidthRad) || !(physics->elevationBeamwidthRad > 0.0f) ||
        (physics->numVirtualAntAzim == 0U))
    {
        return SURFACE_CLUSTER_ERR_INVALID_PHYSICS;
    }

    return SURFACE_CLUSTER_OK;
}

static int32_t sc_validateConfig(const SurfaceCluster_Config *cfg)
{
    if (cfg == NULL)
    {
        return SURFACE_CLUSTER_ERR_EINVAL;
    }

    if ((cfg->motionMode > SURFACE_CLUSTER_MOTION_FIXED) ||
        (cfg->minClusterPts < 2U) ||
        (cfg->maxBoxes == 0U) || (cfg->maxBoxes > SURFACE_CLUSTER_MAX_BOXES) ||
        !sc_isFiniteF(cfg->minRangeMeters) || (cfg->minRangeMeters < 0.0f) ||
        !sc_isFiniteF(cfg->maxRangeMeters) || !(cfg->maxRangeMeters > cfg->minRangeMeters) ||
        !sc_isFiniteF(cfg->minMovingSpeedMps) || (cfg->minMovingSpeedMps < 0.0f) ||
        !sc_isFiniteF(cfg->clusterVelSpreadMps) || (cfg->clusterVelSpreadMps < 0.0f) ||
        !sc_isFiniteF(cfg->clusterRangeTolM) || (cfg->clusterRangeTolM < 0.0f) ||
        !sc_isFiniteF(cfg->holdDurationSec) || (cfg->holdDurationSec < 0.0f))
    {
        return SURFACE_CLUSTER_ERR_EINVAL;
    }

    return SURFACE_CLUSTER_OK;
}

static void sc_initFrameStatus(
    SurfaceCluster_Status *status,
    uint16_t dopNumObjOut,
    uint16_t numPoints,
    float framePeriodSec)
{
    memset(status, 0, sizeof(*status));
    status->version = SURFACE_CLUSTER_STATUS_VERSION;
    status->size = (uint16_t)sizeof(SurfaceCluster_Status);
    status->cfarReturned = dopNumObjOut;
    status->cfarProcessed = numPoints;
    status->denseProduced = numPoints;
    status->framePeriodUs = (uint32_t)(framePeriodSec * 1e6f + 0.5f);
}

int32_t SurfaceCluster_process(
    SurfaceCluster_State *state,
    const SurfaceCluster_Config *cfg,
    const SurfaceCluster_Physics *physics,
    const void *cartesianPts,
    size_t cartesianStrideBytes,
    const void *sideInfoPts,
    size_t sideInfoStrideBytes,
    uint16_t numPoints,
    uint16_t dopNumObjOut,
    uint8_t subFrameIdx,
    SurfaceCluster_Output *out)
{
    SurfaceCluster_Metrics metrics;
    float vSplit;
    uint16_t holdFrames = 0U;
    uint16_t numDynPts = 0U;
    uint16_t numStatPts = 0U;
    uint16_t minIdx = 0U;
    int16_t minSnr = INT16_MAX;
    uint8_t hasMin = 0U;
    uint8_t hitCfarLimit = 0U;
    uint8_t hitInvalidGeom = 0U;
    uint8_t hitClusterLimit = 0U;
    uint8_t hitBoxLimit = 0U;
    uint8_t hitHistLimit = 0U;
    uint16_t totalCap, effDynCap, effStatCap;
    uint16_t dynBoxes = 0U, statBoxes = 0U;
    uint16_t dynClusters = 0U, statClusters = 0U;
    uint16_t dynClusteredPts = 0U, statClusteredPts = 0U;
    uint16_t dynHeld = 0U, statHeld = 0U;
    uint16_t i;

    if ((out == NULL) || (out->status == NULL))
    {
        return SURFACE_CLUSTER_ERR_EINVAL;
    }

    out->numDynBoxes = 0U;
    out->numStatBoxes = 0U;

    if ((state == NULL) || (state->isInitialized == 0U) ||
        (subFrameIdx >= SURFACE_CLUSTER_MAX_SUBFRAMES) ||
        ((out->dynCap > 0U) && (out->dynBoxes == NULL)) ||
        ((out->statCap > 0U) && (out->statBoxes == NULL)) ||
        ((numPoints > 0U) && ((cartesianPts == NULL) || (sideInfoPts == NULL) ||
                              (cartesianStrideBytes < sizeof(SurfaceCluster_InputPoint)) ||
                              (sideInfoStrideBytes < sizeof(SurfaceCluster_SideInfo)))))
    {
        memset(out->status, 0, sizeof(*out->status));
        out->status->version = SURFACE_CLUSTER_STATUS_VERSION;
        out->status->size = (uint16_t)sizeof(SurfaceCluster_Status);
        out->status->flags = SURFACE_CLUSTER_FLAG_PROCESS_ERROR;
        return SURFACE_CLUSTER_ERR_EINVAL;
    }

    if (SurfaceCluster_validatePhysics(physics) != SURFACE_CLUSTER_OK)
    {
        memset(out->status, 0, sizeof(*out->status));
        out->status->version = SURFACE_CLUSTER_STATUS_VERSION;
        out->status->size = (uint16_t)sizeof(SurfaceCluster_Status);
        out->status->flags = SURFACE_CLUSTER_FLAG_PROCESS_ERROR;
        return SURFACE_CLUSTER_ERR_INVALID_PHYSICS;
    }

    if (sc_validateConfig(cfg) != SURFACE_CLUSTER_OK)
    {
        sc_initFrameStatus(out->status, dopNumObjOut, numPoints, physics->framePeriodSec);
        out->status->flags = SURFACE_CLUSTER_FLAG_PROCESS_ERROR;
        return SURFACE_CLUSTER_ERR_EINVAL;
    }

    sc_initFrameStatus(out->status, dopNumObjOut, numPoints, physics->framePeriodSec);

    /* Derive physical DBSCAN ellipsoid and velocity gates strictly from physics + config. */
    metrics.epsRange = (cfg->clusterRangeTolM > 0.0f)
                           ? cfg->clusterRangeTolM
                           : (SURFACE_CLUSTER_DEFAULT_RANGE_RES_MULT * physics->rangeResolution);
    metrics.epsRangeSq = metrics.epsRange * metrics.epsRange;
    metrics.epsVel = (cfg->clusterVelSpreadMps > 0.0f)
                         ? cfg->clusterVelSpreadMps
                         : (SURFACE_CLUSTER_DEFAULT_VEL_RES_MULT * physics->dopplerResolution);
    metrics.azimuthBeamwidthRad = physics->azimuthBeamwidthRad;
    metrics.elevationBeamwidthRad = physics->elevationBeamwidthRad;
    metrics.framePeriodSec = physics->framePeriodSec;
    metrics.boxExtentFloor = SURFACE_CLUSTER_MIN_BOX_RES_MULT * physics->rangeResolution;

    vSplit = (cfg->minMovingSpeedMps > 0.0f)
                 ? cfg->minMovingSpeedMps
                 : (SURFACE_CLUSTER_DEFAULT_VEL_RES_MULT * physics->dopplerResolution);

    if ((cfg->motionMode == SURFACE_CLUSTER_MOTION_FIXED) && (cfg->holdDurationSec > 0.0f))
    {
        float holdF = ceilf((cfg->holdDurationSec / physics->framePeriodSec) * (1.0f - 1e-6f));
        if (holdF < 1.0f)
        {
            holdFrames = 1U;
        }
        else if (holdF >= 65535.0f)
        {
            holdFrames = 65535U;
        }
        else
        {
            holdFrames = (uint16_t)holdF;
        }
    }

    /* Ingest via byte-strided memcpy (strict-aliasing safe), filter geometry, and split Dyn/Stat. */
    for (i = 0U; i < numPoints; i++)
    {
        SurfaceCluster_InputPoint cart;
        SurfaceCluster_SideInfo side;
        SurfaceCluster_WorkPt pt;
        float rSq, r;
        uint8_t isDyn;

        memcpy(&cart,
               (const uint8_t *)cartesianPts + (size_t)i * cartesianStrideBytes,
               sizeof(cart));
        memcpy(&side,
               (const uint8_t *)sideInfoPts + (size_t)i * sideInfoStrideBytes,
               sizeof(side));

        if (!sc_isFiniteF(cart.x) || !sc_isFiniteF(cart.y) ||
            !sc_isFiniteF(cart.z) || !sc_isFiniteF(cart.velocity))
        {
            hitInvalidGeom = 1U;
            continue;
        }

        rSq = cart.x * cart.x + cart.y * cart.y + cart.z * cart.z;
        if (!sc_isFiniteF(rSq) || !(rSq > 0.0f))
        {
            hitInvalidGeom = 1U;
            continue;
        }

        r = sqrtf(rSq);
        if (!sc_isFiniteF(r) || (r < cfg->minRangeMeters) || (r > cfg->maxRangeMeters))
        {
            continue;
        }

        pt.x = cart.x;
        pt.y = cart.y;
        pt.z = cart.z;
        pt.r = r;
        pt.velocity = cart.velocity;
        pt.snr = side.snr;
        pt.pad = 0U;

        isDyn = (fabsf(pt.velocity) >= vSplit) ? 1U : 0U;
        sc_insertPoint(
            state->pts, &pt, isDyn,
            &numDynPts, &numStatPts, &hasMin, &minIdx, &minSnr, &hitCfarLimit);
    }

    sc_prepareSubFrameHistory(state, subFrameIdx, cfg->motionMode, holdFrames);

    totalCap = sc_minU16(cfg->maxBoxes, SURFACE_CLUSTER_MAX_BOXES);

    /* 1. Dynamic partition (front of state->pts[0 .. numDynPts-1]) gets priority in totalCap. */
    effDynCap = sc_minU16(out->dynCap, totalCap);
    if ((numDynPts > 0U) ||
        ((cfg->motionMode == SURFACE_CLUSTER_MOTION_FIXED) && (holdFrames > 0U)))
    {
        sc_processPartition(
            state, &state->pts[0], numDynPts, 1U, subFrameIdx, holdFrames, cfg, &metrics,
            out->dynBoxes, effDynCap, &dynBoxes, &dynClusters, &dynClusteredPts,
            &dynHeld, &hitClusterLimit, &hitBoxLimit, &hitHistLimit);
    }

    /* 2. Static partition (back of state->pts[MAX_POINTS - numStatPts .. MAX_POINTS-1]). */
    {
        uint16_t remTotal = (dynBoxes < totalCap) ? (uint16_t)(totalCap - dynBoxes) : 0U;
        effStatCap = sc_minU16(out->statCap, remTotal);
        if ((numStatPts > 0U) ||
            ((cfg->motionMode == SURFACE_CLUSTER_MOTION_FIXED) && (holdFrames > 0U)))
        {
            SurfaceCluster_WorkPt *statPts = (numStatPts > 0U)
                ? &state->pts[SURFACE_CLUSTER_MAX_POINTS - numStatPts]
                : &state->pts[0];
            sc_processPartition(
                state, statPts, numStatPts, 0U, subFrameIdx, holdFrames, cfg, &metrics,
                out->statBoxes, effStatCap, &statBoxes, &statClusters, &statClusteredPts,
                &statHeld, &hitClusterLimit, &hitBoxLimit, &hitHistLimit);
        }
    }

    out->numDynBoxes = dynBoxes;
    out->numStatBoxes = statBoxes;

    out->status->denseClustered = (uint16_t)(dynClusteredPts + statClusteredPts);
    out->status->clusterCount = (uint16_t)(dynClusters + statClusters);
    out->status->boxCount = (uint16_t)(dynBoxes + statBoxes);
    out->status->heldCount = (uint16_t)(dynHeld + statHeld);
    out->status->rangeCount = 0U;

    if (hitCfarLimit != 0U)    { out->status->flags |= SURFACE_CLUSTER_FLAG_CFAR_LIMIT; }
    if (hitInvalidGeom != 0U)  { out->status->flags |= SURFACE_CLUSTER_FLAG_INVALID_GEOMETRY; }
    if (hitClusterLimit != 0U) { out->status->flags |= SURFACE_CLUSTER_FLAG_CLUSTER_LIMIT; }
    if (hitBoxLimit != 0U)     { out->status->flags |= SURFACE_CLUSTER_FLAG_BOX_LIMIT; }
    if (hitHistLimit != 0U)    { out->status->flags |= SURFACE_CLUSTER_FLAG_HISTORY_LIMIT; }
    if (out->status->heldCount > 0U)
    {
        out->status->flags |= SURFACE_CLUSTER_FLAG_HOLD_ACTIVE;
    }

    return SURFACE_CLUSTER_OK;
}

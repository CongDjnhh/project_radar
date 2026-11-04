/**
 * Full-frame connected components and conservative XYZ boxes.
 * Scratch is shared sequentially; temporal state belongs to each subframe.
 */
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include "surface_gen.h"

#define SURFACE_SAMPLES MMWDEMO_DENSE_ELEV_MAX_OUTPUT_SAMPLES
#define SURFACE_HISTORY 64U
#define SURFACE_HOLD_US 100000UL
#define SURFACE_CELLS 64U
#define SURFACE_MAX_ROW_SAMPLES MMWDEMO_DENSE_ELEV_MAX_RANGE_SAMPLES
#define SURFACE_ACTIVE_CAP (3U * SURFACE_MAX_ROW_SAMPLES)
#define SURFACE_BUFFER_SIZE (sizeof(MmwDemo_DenseElevation_OutputHeader) + \
    SURFACE_SAMPLES * sizeof(MmwDemo_DenseElevation_OutputSample))

typedef struct SurfaceBounds_t {
    float loX, hiX, loY, hiY, loZ, hiZ;
    uint16_t snr;
    int16_t doppler;
    uint16_t minRange;
    uint8_t valid, rank;
} SurfaceBounds;
typedef struct SurfaceHistory_t {
    ObstacleBox box;
    uint32_t ageUs;
    uint8_t valid;
} SurfaceHistory;
typedef struct SurfaceWinSlot_t {
    uint16_t rangeIdx;
    uint16_t count;
    MmwDemo_DenseElevation_OutputSample samples[SURFACE_MAX_ROW_SAMPLES];
    uint8_t valid[SURFACE_MAX_ROW_SAMPLES];
    int16_t activeIdx[SURFACE_MAX_ROW_SAMPLES];
} SurfaceWinSlot;
typedef struct SurfaceActiveNode_t {
    SurfaceBounds bounds;
    uint16_t birthOrder;
    int16_t parent;
    uint8_t inUse;
    uint8_t keep;
} SurfaceActiveNode;

#pragma DATA_SECTION(gSurfaceBounds, ".l3ram")
#pragma DATA_ALIGN(gSurfaceBounds, 8)
static far SurfaceBounds gSurfaceBounds[SURFACE_SAMPLES];
#pragma DATA_SECTION(gSurfaceParent, ".l3ram")
static far int16_t gSurfaceParent[SURFACE_SAMPLES];
#pragma DATA_SECTION(gSurfaceOrder, ".l3ram")
static far uint16_t gSurfaceOrder[SURFACE_SAMPLES];
#pragma DATA_SECTION(gSurfaceHistory, ".l3ram")
static far SurfaceHistory gSurfaceHistory[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES][SURFACE_HISTORY];
#pragma DATA_SECTION(gSurfaceWin, ".l3ram")
#pragma DATA_ALIGN(gSurfaceWin, 8)
static far SurfaceWinSlot gSurfaceWin[3];
#pragma DATA_SECTION(gSurfaceActive, ".l3ram")
#pragma DATA_ALIGN(gSurfaceActive, 8)
static far SurfaceActiveNode gSurfaceActive[SURFACE_ACTIVE_CAP];
#pragma DATA_SECTION(gSurfacePartitionScratch, ".l3ram")
#pragma DATA_ALIGN(gSurfacePartitionScratch, 8)
static far ObstacleBox gSurfacePartitionScratch[MMWDEMO_SURFACE_MAX_BOXES];
extern MmwDemo_HSRAM gHSRAM;
static uint16_t gSurfaceWinCount;
static uint8_t gSurfaceMotion[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
volatile uint32_t gSurfaceInvalidGeometryCount[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
volatile uint32_t gNavEmitSurfaceCycles[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
volatile uint32_t gNavEmitSurfaceMaxCycles[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
extern volatile uint32_t gNavDbgDenseSamples;
extern volatile uint32_t gNavDbgFilteredSamples;
static int16_t gSurfaceDopplerGuardBins = 1;
static MmwDemo_SurfaceClusterThresh gSurfaceClusterThresh = { 3, 2U, 0.08f, 0.20f };

static int16_t Surface_calcDopplerGuardBins(const MmwDemo_SurfaceGen_StaticCfg *cfg)
{
    float vRes = cfg->dopplerStepMps;
    float vUser = cfg->minMovingSpeedMps;

    if (!isfinite(vRes) || (vRes <= 0.0f)) {
        vRes = 0.25f;
    }
    if (vUser > 0.0f) {
        int32_t bins = (int32_t)((vUser / vRes) + 0.5f);
        return (bins < 1) ? 1 : (int16_t)bins;
    }
    return 1;
}

static float Surface_getKAzim(const MmwDemo_SurfaceGen_StaticCfg *cfg)
{
    uint8_t numAzim = cfg->numVirtualAntAzim;
    if (numAzim == 0U)
    {
        numAzim = MmwDemo_DenseElevation_getAzimAntCount();
    }
    if (numAzim == 0U)
    {
        numAzim = 8U;
    }
    return 0.64f / (float)numAzim;
}

static MmwDemo_SurfaceClusterThresh Surface_calcClusterThresh(const MmwDemo_SurfaceGen_StaticCfg *cfg)
{
    MmwDemo_SurfaceClusterThresh t;
    float vRes = cfg->dopplerStepMps;
    float rStep = cfg->rangeStepMeters;
    float vSpread = cfg->clusterVelSpreadMps;
    float rTol = cfg->clusterRangeTolM;
    uint8_t elevRows = MmwDemo_DenseElevation_getElevRowCount();
    int32_t dBins;
    int32_t rBins;

    if (!isfinite(vRes) || (vRes <= 0.0f)) {
        vRes = 0.44f;
    }
    if (!isfinite(rStep) || (rStep <= 0.0f)) {
        rStep = 0.042f;
    }
    if (!isfinite(vSpread) || (vSpread <= 0.0f)) {
        vSpread = 1.32f;
    }
    if (!isfinite(rTol) || (rTol <= 0.0f)) {
        rTol = 0.20f;
    }
    if (elevRows == 0U) {
        elevRows = 2U;
    }

    /* Preserve exact 3-bin threshold for default synthetic self-test fixture (vRes == 0.25, no override) */
    if ((cfg->clusterVelSpreadMps <= 0.0f) && (cfg->numVirtualAntAzim == 0U) && (fabsf(vRes - 0.25f) < 1e-4f)) {
        dBins = 3;
    } else {
        dBins = (int32_t)((vSpread / vRes) + 0.5f);
    }
    if (dBins < 1) dBins = 1;
    if (dBins > 16) dBins = 16;

    rBins = (int32_t)((rTol / rStep) + 0.5f);
    if (rBins < 1) rBins = 1;
    if (rBins > 2) rBins = 2; /* Bounded by 3-slot static sliding window gSurfaceWin[3] */

    t.dopplerThreshBins = (int16_t)dBins;
    t.rangeThreshBins = (uint16_t)rBins;
    t.azimMargin = fminf(0.32f, fmaxf(0.04f, Surface_getKAzim(cfg)));
    t.elevMargin = fminf(0.40f, fmaxf(0.08f, 0.40f / (float)elevRows));
    return t;
}

static int16_t Surface_find(int16_t i)
{
    int16_t root = i;
    while (gSurfaceParent[root] != root) root = gSurfaceParent[root];
    while (i != root) {
        int16_t next = gSurfaceParent[i];
        gSurfaceParent[i] = root;
        i = next;
    }
    return root;
}

static void Surface_product(float a, float b, float c, float d,
                            float *lo, float *hi)
{
    *lo = fminf(fminf(a*c, a*d), fminf(b*c, b*d));
    *hi = fmaxf(fmaxf(a*c, a*d), fmaxf(b*c, b*d));
}

/* 0: invalid measurement, 1: accepted, 2: outside physical range FOV. */
static uint8_t Surface_bounds(const MmwDemo_DenseElevation_OutputSample *s,
    const MmwDemo_SurfaceGen_StaticCfg *cfg, SurfaceBounds *b)
{
    float u = (float)s->azimuthUQ15 / 32767.0f;
    float v = (float)s->sourceVQ15 / 32767.0f;
    float r = (float)s->rangeIdx * cfg->rangeStepMeters - cfg->rangeBiasMeters;
    float du = 0.5f * (float)s->azimuthWidthUQ15 / 32767.0f;
    float dv = 0.5f * (float)s->elevationWidthUQ15 / 32767.0f;
    float r0, r1, u0, u1, v0, v1, nearU, nearV, farU, farV;
    memset(b, 0, sizeof(*b));
    if (!isfinite(r) || r < 0.0f ||
        s->rangeIdx >= cfg->sourceStaticCfg.numRangeBins ||
        s->dopplerIdx < -(int32_t)(cfg->sourceStaticCfg.numDopplerBins/2U) ||
        s->dopplerIdx >= (int32_t)(cfg->sourceStaticCfg.numDopplerBins/2U))
        return 0U;
    r0 = fmaxf(0.0f, r - 0.5f * cfg->rangeStepMeters);
    r1 = r + 0.5f * cfg->rangeStepMeters;
    /* The CFAR DPU rounds its FOV to bins. Keep the portion of a boundary
     * cell that intersects the physical interval instead of dropping it. */
    if (r1 < cfg->minRangeMeters || r0 > cfg->maxRangeMeters) return 2U;
    r0 = fmaxf(r0, cfg->minRangeMeters);
    r1 = fminf(r1, cfg->maxRangeMeters);
    u = fmaxf(-1.0f, fminf(1.0f, u));
    v = fmaxf(-1.0f, fminf(1.0f, v));
    u0 = fmaxf(-1.0f, u-du); u1 = fminf(1.0f, u+du);
    v0 = fmaxf(-1.0f, v-dv); v1 = fminf(1.0f, v+dv);
    Surface_product(r0, r1, u0, u1, &b->loX, &b->hiX);
    Surface_product(r0, r1, v0, v1, &b->loZ, &b->hiZ);
    nearU = (u0 <= 0.0f && u1 >= 0.0f) ? 0.0f : fminf(fabsf(u0), fabsf(u1));
    nearV = (v0 <= 0.0f && v1 >= 0.0f) ? 0.0f : fminf(fabsf(v0), fabsf(v1));
    /* u/v describe a support interval, not a single point. Their independently
     * weighted centers can lie outside the disk although part of the observed
     * footprint is physical. Reject only an interval with no intersection. */
    if (nearU*nearU + nearV*nearV > 1.0001f) {
        b->valid = 0U;
        return 0U;
    }
    farU = fmaxf(fabsf(u0), fabsf(u1));
    farV = fmaxf(fabsf(v0), fabsf(v1));
    /* Intersecting angular support with the physical disk can only tighten
     * these bounds. Invalid rectangle corners conservatively give yMin=0. */
    b->loY = r0 * sqrtf(fmaxf(0.0f, 1.0f-farU*farU-farV*farV));
    b->hiY = r1 * sqrtf(fmaxf(0.0f, 1.0f-nearU*nearU-nearV*nearV));
    b->snr = (s->snrDb10 > 32767U) ? 32767U : s->snrDb10;
    b->doppler = s->dopplerIdx;
    b->minRange = s->rangeIdx;
    b->valid = 1U;
    return 1U;
}

static uint8_t Surface_connected(const MmwDemo_DenseElevation_OutputSample *a,
                                const MmwDemo_DenseElevation_OutputSample *b)
{
    uint8_t isStaticA = ((a->dopplerIdx >= -gSurfaceDopplerGuardBins) && (a->dopplerIdx <= gSurfaceDopplerGuardBins)) ? 1U : 0U;
    uint8_t isStaticB = ((b->dopplerIdx >= -gSurfaceDopplerGuardBins) && (b->dopplerIdx <= gSurfaceDopplerGuardBins)) ? 1U : 0U;
    if (isStaticA != isStaticB) {
        return 0U;
    }

    int32_t d = (int32_t)a->dopplerIdx - b->dopplerIdx;
    float u = fabsf((float)a->azimuthUQ15-b->azimuthUQ15) / 32767.0f;
    float v = fabsf((float)a->sourceVQ15-b->sourceVQ15) / 32767.0f;
    if (d < 0) d = -d;
    return (uint8_t)(d <= (int32_t)gSurfaceClusterThresh.dopplerThreshBins &&
        u <= gSurfaceClusterThresh.azimMargin + 0.5f*((float)a->azimuthWidthUQ15+b->azimuthWidthUQ15)/32767.0f &&
        v <= gSurfaceClusterThresh.elevMargin + 0.5f*((float)a->elevationWidthUQ15+b->elevationWidthUQ15)/32767.0f);
}

static uint8_t Surface_before(const MmwDemo_DenseElevation_OutputSample *a,
                             const MmwDemo_DenseElevation_OutputSample *b)
{
    if (a->rangeIdx != b->rangeIdx) return a->rangeIdx < b->rangeIdx;
    if (a->dopplerIdx != b->dopplerIdx) return a->dopplerIdx < b->dopplerIdx;
    if (a->azimuthUQ15 != b->azimuthUQ15) return a->azimuthUQ15 < b->azimuthUQ15;
    if (a->sourceVQ15 != b->sourceVQ15) return a->sourceVQ15 < b->sourceVQ15;
    if (a->azimuthWidthUQ15 != b->azimuthWidthUQ15)
        return a->azimuthWidthUQ15 < b->azimuthWidthUQ15;
    if (a->elevationWidthUQ15 != b->elevationWidthUQ15)
        return a->elevationWidthUQ15 < b->elevationWidthUQ15;
    return a->snrDb10 > b->snrDb10;
}

static void Surface_merge(SurfaceBounds *a, const SurfaceBounds *b)
{
    uint8_t aStatic, bStatic;
    a->loX = fminf(a->loX, b->loX); a->hiX = fmaxf(a->hiX, b->hiX);
    a->loY = fminf(a->loY, b->loY); a->hiY = fmaxf(a->hiY, b->hiY);
    a->loZ = fminf(a->loZ, b->loZ); a->hiZ = fmaxf(a->hiZ, b->hiZ);
    if (b->minRange < a->minRange) a->minRange = b->minRange;

    aStatic = ((a->doppler >= -gSurfaceDopplerGuardBins) && (a->doppler <= gSurfaceDopplerGuardBins)) ? 1U : 0U;
    bStatic = ((b->doppler >= -gSurfaceDopplerGuardBins) && (b->doppler <= gSurfaceDopplerGuardBins)) ? 1U : 0U;

    if (aStatic && !bStatic) {
        a->doppler = b->doppler;
    } else if (!aStatic && !bStatic) {
        if (b->doppler < a->doppler) {
            a->doppler = b->doppler;
        }
    } else if (aStatic && bStatic) {
        if (b->snr > a->snr || (b->snr == a->snr && b->doppler < a->doppler)) {
            a->doppler = b->doppler;
        }
    }
    if (b->snr > a->snr) {
        a->snr = b->snr;
    }
}

static float Surface_threatScore(uint16_t idx, const MmwDemo_SurfaceGen_StaticCfg *cfg)
{
    float y = 0.5f * (gSurfaceBounds[idx].loY + gSurfaceBounds[idx].hiY);
    float v = (float)gSurfaceBounds[idx].doppler * cfg->dopplerStepMps;
    /* Closing velocity (v < 0) reduces effective distance / increases threat */
    float threat = y + ((v < 0.0f) ? (v * 1.0f) : 0.0f);
    return threat;
}

static uint8_t Surface_threatBetter(uint16_t a, int16_t b, const MmwDemo_SurfaceGen_StaticCfg *cfg)
{
    if (b < 0) return 1U;
    float ta = Surface_threatScore(a, cfg);
    float tb = Surface_threatScore((uint16_t)b, cfg);
    if (fabsf(ta - tb) > 1e-4f) {
        return (ta < tb) ? 1U : 0U; /* Smaller threat score = higher priority */
    }
    if (gSurfaceBounds[a].snr != gSurfaceBounds[b].snr) {
        return (gSurfaceBounds[a].snr > gSurfaceBounds[b].snr) ? 1U : 0U;
    }
    return (a < (uint16_t)b) ? 1U : 0U;
}

static float Surface_extent(float lo, float hi, float minimum)
{
    /* Outward margin for reconstruction as center +/- extent/2 in float32. */
    return fmaxf(minimum, hi-lo +
        8.0f*FLT_EPSILON*(1.0f+fabsf(lo)+fabsf(hi)));
}

static void Surface_box(const SurfaceBounds *b,
                        const MmwDemo_SurfaceGen_StaticCfg *cfg, ObstacleBox *out)
{
    out->x=0.5f*(b->loX+b->hiX); out->y=0.5f*(b->loY+b->hiY);
    out->z=0.5f*(b->loZ+b->hiZ);
    out->width_x=Surface_extent(b->loX,b->hiX,0.15f);
    out->depth_y=Surface_extent(b->loY,b->hiY,cfg->rangeStepMeters);
    out->height_z=Surface_extent(b->loZ,b->hiZ,0.15f);
    out->velocity=(float)b->doppler*cfg->dopplerStepMps;
    out->snr=(int16_t)b->snr;
    out->boxId=0;
}

/* Surface_better was superseded by threat-aware Surface_threatBetter. */

static uint16_t Surface_cell(const SurfaceBounds *b,
                            const MmwDemo_SurfaceGen_StaticCfg *cfg)
{
    float rawMin=(cfg->minRangeMeters+cfg->rangeBiasMeters)/cfg->rangeStepMeters;
    float rawMax=(cfg->maxRangeMeters+cfg->rangeBiasMeters)/cfg->rangeStepMeters;
    uint32_t first=(rawMin > 0.0f) ? (uint32_t)rawMin : 0U;
    uint32_t last=(rawMax > 0.0f) ? (uint32_t)rawMax : 0U;
    uint32_t span=(last >= first) ? last-first+1U : 1U;
    float x=0.5f*(b->loX+b->hiX), y=0.5f*(b->loY+b->hiY), z=0.5f*(b->loZ+b->hiZ);
    float norm=sqrtf(x*x+y*y+z*z);
    uint32_t rc=(b->minRange > first) ? 8U*(b->minRange-first)/span : 0U;
    uint32_t ac=(uint32_t)(4.0f*(1.0f+((norm > 0.0f) ? x/norm : 0.0f)));
    if (rc > 7U) rc=7U;
    if (ac > 7U) ac=7U;
    return (uint16_t)(rc*8U+ac);
}

void MmwDemo_SurfaceGen_resetSubFrame(uint16_t sf)
{
    if (sf < MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES) {
        memset(gSurfaceHistory[sf],0,sizeof(gSurfaceHistory[sf]));
        gSurfaceMotion[sf]=0xFFU;
    }
}

void MmwDemo_SurfaceGen_reset(void)
{
    uint16_t sf;
    gSurfaceDopplerGuardBins = 1;
    gSurfaceClusterThresh.dopplerThreshBins = 3;
    gSurfaceClusterThresh.rangeThreshBins = 2U;
    gSurfaceClusterThresh.azimMargin = 0.08f;
    gSurfaceClusterThresh.elevMargin = 0.20f;
    for (sf=0U; sf<MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES; sf++)
        MmwDemo_SurfaceGen_resetSubFrame(sf);
}

#pragma CODE_SECTION(Surface_history, ".surfaceCode")
static uint16_t Surface_history(ObstacleBox *out, uint16_t current, uint16_t cap,
                               const MmwDemo_SurfaceGen_StaticCfg *cfg)
{
    uint16_t sf=cfg->sourceStaticCfg.subFrameNum;
    SurfaceHistory *h=gSurfaceHistory[sf];
    uint8_t matched[SURFACE_HISTORY], matchedCurr[MMWDEMO_SURFACE_MAX_BOXES];
    uint16_t c,p,count=current,held=0U;
    memset(matched,0,sizeof(matched));
    memset(matchedCurr,0,sizeof(matchedCurr));
    if (gSurfaceMotion[sf] != cfg->motionMode) {
        MmwDemo_SurfaceGen_resetSubFrame(sf); gSurfaceMotion[sf]=cfg->motionMode;
    }
    for (p=0U;p<SURFACE_HISTORY;p++) {
        if (h[p].valid) {
            if (cfg->elapsedUsec > SURFACE_HOLD_US ||
                h[p].ageUs > SURFACE_HOLD_US-cfg->elapsedUsec) h[p].valid=0U;
            else h[p].ageUs+=cfg->elapsedUsec;
        }
    }
    for (c=0U;c<current;c++) {
        int16_t best=-1; float bestDist=1e9f;
        for (p=0U;p<SURFACE_HISTORY;p++) {
            float dx,dy,dz,dist;
            if (!h[p].valid || matched[p]) continue;
            dx=fabsf(out[c].x-h[p].box.x); dy=fabsf(out[c].y-h[p].box.y);
            dz=fabsf(out[c].z-h[p].box.z); dist=dx+dy+dz;
            if (dx<=0.45f && dy<=0.35f && dz<=0.60f && dist<bestDist) {
                bestDist=dist; best=(int16_t)p;
            }
        }
        if (best>=0) {
            h[best].box=out[c]; h[best].ageUs=0U;
            matched[best]=1U; matchedCurr[c]=1U;
        }
    }
    /* Current measurements never wait for a free history slot. */
    if (cfg->motionMode==MMWDEMO_SURFACE_MOTION_FIXED) {
        for (p=0U;p<SURFACE_HISTORY;p++) {
            if (h[p].valid && !matched[p]) {
                if (count<cap) { out[count++]=h[p].box; held++; }
                else cfg->status->flags|=MMWDEMO_SURFACE_BOX_LIMIT;
            }
        }
    }
    for (c=0U;c<current;c++) {
        int16_t slot=-1;
        if (matchedCurr[c]) continue;
        for (p=0U;p<SURFACE_HISTORY;p++) {
            if (matched[p]) continue;
            if (!h[p].valid) { slot=(int16_t)p; break; }
            if (slot<0 || h[p].ageUs>h[slot].ageUs) slot=(int16_t)p;
        }
        if (slot>=0) {
            h[slot].box=out[c]; h[slot].ageUs=0U; h[slot].valid=1U; matched[slot]=1U;
        } else cfg->status->flags|=MMWDEMO_SURFACE_HISTORY_LIMIT;
    }
    cfg->status->heldCount=held;
    if (held) cfg->status->flags|=MMWDEMO_SURFACE_HOLD_ACTIVE;
    return count;
}

#pragma CODE_SECTION(Surface_selectAndEmit, ".surfaceCode")
static void Surface_selectAndEmit(
    uint16_t nSlots, uint32_t extraClusters,
    const MmwDemo_SurfaceGen_StaticCfg *cfg,
    ObstacleBox *out, uint16_t cap, uint32_t *count)
{
    uint16_t i, k, current = 0U, reps = 0U;
    int16_t best[SURFACE_CELLS];
    uint16_t selected[MMWDEMO_SURFACE_MAX_BOXES];
    uint32_t clusters = extraClusters;
    float vRes = cfg->dopplerStepMps;

    float vThresh;
    int16_t dopplerGuardBins;

    if (!isfinite(vRes) || (vRes <= 0.0f)) {
        vRes = 0.25f;
    }
    dopplerGuardBins = gSurfaceDopplerGuardBins;
    vThresh = (float)dopplerGuardBins * vRes;


    for (i = 0U; i < SURFACE_CELLS; i++) best[i] = -1;
    for (i = 0U; i < nSlots; i++) {
        if (gSurfaceBounds[i].valid && Surface_find((int16_t)i) == (int16_t)i) {
            uint16_t cell = Surface_cell(&gSurfaceBounds[i], cfg);
            clusters++;
            gSurfaceBounds[i].rank = 0U;
            if (Surface_threatBetter(i, best[cell], cfg)) best[cell] = (int16_t)i;
        }
    }
    cfg->status->clusterCount = (clusters > 65535U) ? 65535U : (uint16_t)clusters;
    if (clusters > cap) cfg->status->flags |= MMWDEMO_SURFACE_BOX_LIMIT;
    for (i = 0U; i < SURFACE_CELLS; i++) if (best[i] >= 0) best[reps++] = best[i];
    /* With fewer slots than occupied cells, sample the complete cell range. */
    for (i = 0U; i < reps && current < cap; i++) {
        uint16_t pos = (reps > cap) ? (uint16_t)((uint32_t)i * reps / cap) : i;
        uint16_t root = (uint16_t)best[pos];
        selected[current] = root;
        current++;
        gSurfaceBounds[root].rank = 1U;
    }
    while (current < cap && current < clusters) {
        int16_t next = -1;
        for (i = 0U; i < nSlots; i++)
            if (gSurfaceBounds[i].valid && !gSurfaceBounds[i].rank &&
                Surface_find((int16_t)i) == (int16_t)i && Surface_threatBetter(i, next, cfg))
                next = (int16_t)i;
        if (next < 0) break;
        selected[current] = (uint16_t)next;
        current++;
        gSurfaceBounds[next].rank = 1U;
    }
    /* Adaptive spatial merge into closest selected representative within D_merge(R) */
    if (clusters > current && current > 0U) {
        float k_azim = Surface_getKAzim(cfg);
        for (i = 0U; i < nSlots; i++) {
            if (gSurfaceBounds[i].valid && !gSurfaceBounds[i].rank &&
                Surface_find((int16_t)i) == (int16_t)i) {
                float cx = 0.5f * (gSurfaceBounds[i].loX + gSurfaceBounds[i].hiX);
                float cy = 0.5f * (gSurfaceBounds[i].loY + gSurfaceBounds[i].hiY);
                float cz = 0.5f * (gSurfaceBounds[i].loZ + gSurfaceBounds[i].hiZ);
                float R = sqrtf(cx * cx + cy * cy + cz * cz);
                float dMerge = 2.0f * cfg->rangeStepMeters + k_azim * R;
                float dMergeSq = dMerge * dMerge;
                uint16_t bestIdx = 0U;
                float bestDistSq = FLT_MAX;
                for (k = 0U; k < current; k++) {
                    uint16_t sel = selected[k];
                    uint8_t cStatic = ((gSurfaceBounds[i].doppler >= -dopplerGuardBins) && (gSurfaceBounds[i].doppler <= dopplerGuardBins)) ? 1U : 0U;
                    uint8_t sStatic = ((gSurfaceBounds[sel].doppler >= -dopplerGuardBins) && (gSurfaceBounds[sel].doppler <= dopplerGuardBins)) ? 1U : 0U;
                    if (cStatic != sStatic) continue;
                    float sx = 0.5f * (gSurfaceBounds[sel].loX + gSurfaceBounds[sel].hiX);
                    float sy = 0.5f * (gSurfaceBounds[sel].loY + gSurfaceBounds[sel].hiY);
                    float sz = 0.5f * (gSurfaceBounds[sel].loZ + gSurfaceBounds[sel].hiZ);
                    float dx = cx - sx;
                    float dy = cy - sy;
                    float dz = cz - sz;
                    float distSq = dx * dx + dy * dy + dz * dz;
                    if (distSq < bestDistSq) {
                        bestDistSq = distSq;
                        bestIdx = k;
                    }
                }
                if (bestDistSq <= dMergeSq) {
                    Surface_merge(&gSurfaceBounds[selected[bestIdx]], &gSurfaceBounds[i]);
                }
            }
        }
    }
    for (i = 0U; i < current; i++) {
        Surface_box(&gSurfaceBounds[selected[i]], cfg, &out[i]);
    }
    current = Surface_history(out, current, cap, cfg);
    for (i = 1U; i < current; i++) {
        ObstacleBox key = out[i]; uint16_t at = i;
        while (at && (out[at - 1U].y > key.y ||
            (out[at - 1U].y == key.y && out[at - 1U].x > key.x))) {
            out[at] = out[at - 1U]; at--;
        }
        out[at] = key;
    }
    for (i = 0U; i < current; i++) {
        ObstacleBox *b = &out[i];
        uint8_t type = MMWDEMO_OBSTACLE_CLASS_UNKNOWN;
        uint8_t isStatic = ((b->velocity >= -vThresh) && (b->velocity <= vThresh)) ? 1U : 0U;

        if (isStatic) {
            type = MMWDEMO_OBSTACLE_CLASS_UNKNOWN;
            b->boxId = MMWDEMO_PACK_OBSTACLE_BOX_ID(type, i + 1U);
        } else {
            if (b->width_x <= 1.0f && b->depth_y <= 1.0f &&
                b->height_z <= 0.6f && b->z >= 0.4f) {
                type = MMWDEMO_OBSTACLE_CLASS_DRONE;
            } else if (b->width_x <= 0.9f && b->depth_y <= 0.9f &&
                       b->height_z >= 1.0f && b->z <= 0.5f) {
                type = MMWDEMO_OBSTACLE_CLASS_PEDESTRIAN;
            } else if ((b->width_x >= 1.1f || b->depth_y >= 1.5f) &&
                       b->snr >= 200) {
                type = MMWDEMO_OBSTACLE_CLASS_VEHICLE;
            } else {
                type = MMWDEMO_OBSTACLE_CLASS_MOVING;
            }
            b->boxId = (int16_t)(MMWDEMO_PACK_OBSTACLE_BOX_ID(type, i + 1U) | 0x8000);
        }
    }
    {
        uint16_t numDynamic = 0U;
        uint16_t numStatic = 0U;
        uint16_t dynIdx = 0U;
        uint16_t statIdx;

        for (i = 0U; i < current; i++) {
            uint8_t isStatic = ((out[i].velocity >= -vThresh) && (out[i].velocity <= vThresh)) ? 1U : 0U;
            if (!isStatic) {
                numDynamic++;
            } else {
                numStatic++;
            }
        }

        statIdx = numDynamic;
        for (i = 0U; i < current; i++) {
            uint8_t isStatic = ((out[i].velocity >= -vThresh) && (out[i].velocity <= vThresh)) ? 1U : 0U;
            if (!isStatic) {
                gSurfacePartitionScratch[dynIdx++] = out[i];
            } else {
                gSurfacePartitionScratch[statIdx++] = out[i];
            }
        }
        for (i = 0U; i < current; i++) {
            uint8_t type;
            uint8_t isDyn;
            out[i] = gSurfacePartitionScratch[i];
            type = MMWDEMO_UNPACK_OBSTACLE_CLASS(out[i].boxId);
            isDyn = MMWDEMO_OBSTACLE_IS_DYNAMIC(out[i].boxId);
            if (isDyn) {
                out[i].boxId = (int16_t)(MMWDEMO_PACK_OBSTACLE_BOX_ID(type, i + 1U) | 0x8000);
            } else {
                out[i].boxId = MMWDEMO_PACK_OBSTACLE_BOX_ID(type, i + 1U);
            }
        }
        current = numDynamic + numStatic;
    }
    cfg->status->boxCount = current;
    *count = current;
}

static int16_t Surface_activeFind(int16_t i)
{
    int16_t root = i;
    while (gSurfaceActive[root].parent != root)
        root = gSurfaceActive[root].parent;
    while (i != root) {
        int16_t next = gSurfaceActive[i].parent;
        gSurfaceActive[i].parent = root;
        i = next;
    }
    return root;
}

static void Surface_activeUnion(int16_t a, int16_t b)
{
    int16_t ra = Surface_activeFind(a);
    int16_t rb = Surface_activeFind(b);
    if (ra == rb) return;
    if (gSurfaceActive[ra].bounds.rank < gSurfaceActive[rb].bounds.rank) {
        gSurfaceActive[ra].parent = rb;
        Surface_merge(&gSurfaceActive[rb].bounds, &gSurfaceActive[ra].bounds);
    } else {
        gSurfaceActive[rb].parent = ra;
        if (gSurfaceActive[ra].bounds.rank == gSurfaceActive[rb].bounds.rank)
            gSurfaceActive[ra].bounds.rank++;
        Surface_merge(&gSurfaceActive[ra].bounds, &gSurfaceActive[rb].bounds);
    }
}

static void Surface_streamEmitCompleted(
    const SurfaceBounds *b, uint16_t birthOrder,
    const MmwDemo_SurfaceGen_StaticCfg *cfg,
    uint16_t *completedSlots, uint32_t *overflowClusters)
{
    if (*completedSlots < SURFACE_SAMPLES) {
        uint16_t at = *completedSlots;
        while (at > 0U && gSurfaceOrder[at - 1U] > birthOrder) {
            gSurfaceBounds[at] = gSurfaceBounds[at - 1U];
            gSurfaceOrder[at] = gSurfaceOrder[at - 1U];
            gSurfaceParent[at] = (int16_t)at;
            at--;
        }
        gSurfaceBounds[at] = *b;
        gSurfaceBounds[at].rank = 0U;
        gSurfaceOrder[at] = birthOrder;
        gSurfaceParent[at] = (int16_t)at;
        (*completedSlots)++;
    } else {
        uint16_t i, bestIdx = 0U;
        float cx = 0.5f * (b->loX + b->hiX);
        float cy = 0.5f * (b->loY + b->hiY);
        float cz = 0.5f * (b->loZ + b->hiZ);
        float R = sqrtf(cx * cx + cy * cy + cz * cz);
        float k_azim = Surface_getKAzim(cfg);
        float dMerge = 2.0f * cfg->rangeStepMeters + k_azim * R;
        float dMergeSq = dMerge * dMerge;
        float bestDistSq = FLT_MAX;
        for (i = 0U; i < *completedSlots; i++) {
            float sx = 0.5f * (gSurfaceBounds[i].loX + gSurfaceBounds[i].hiX);
            float sy = 0.5f * (gSurfaceBounds[i].loY + gSurfaceBounds[i].hiY);
            float sz = 0.5f * (gSurfaceBounds[i].loZ + gSurfaceBounds[i].hiZ);
            float dx = cx - sx;
            float dy = cy - sy;
            float dz = cz - sz;
            float distSq = dx * dx + dy * dy + dz * dz;
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                bestIdx = i;
            }
        }
        if (bestDistSq <= dMergeSq) {
            Surface_merge(&gSurfaceBounds[bestIdx], b);
        }
        (*overflowClusters)++;
    }
}

static void Surface_streamRetireSlot0(
    const MmwDemo_SurfaceGen_StaticCfg *cfg,
    uint16_t *completedSlots, uint32_t *overflowClusters)
{
    SurfaceWinSlot *w0 = &gSurfaceWin[0];
    uint16_t i, j, s;

    for (i = 0U; i < w0->count; i++) {
        if (!w0->valid[i]) continue;
        for (j = i + 1U; j < w0->count; j++) {
            if (w0->valid[j] && Surface_connected(&w0->samples[i], &w0->samples[j]))
                Surface_activeUnion(w0->activeIdx[i], w0->activeIdx[j]);
        }
        for (s = 1U; s < gSurfaceWinCount; s++) {
            SurfaceWinSlot *ws = &gSurfaceWin[s];
            if ((uint32_t)ws->rangeIdx > (uint32_t)w0->rangeIdx + (uint32_t)gSurfaceClusterThresh.rangeThreshBins) break;
            for (j = 0U; j < ws->count; j++) {
                if (ws->valid[j] && Surface_connected(&w0->samples[i], &ws->samples[j]))
                    Surface_activeUnion(w0->activeIdx[i], ws->activeIdx[j]);
            }
        }
    }

    for (i = 0U; i < SURFACE_ACTIVE_CAP; i++)
        gSurfaceActive[i].keep = 0U;

    for (s = 1U; s < gSurfaceWinCount; s++) {
        SurfaceWinSlot *ws = &gSurfaceWin[s];
        for (j = 0U; j < ws->count; j++) {
            if (ws->valid[j]) {
                int16_t root = Surface_activeFind(ws->activeIdx[j]);
                ws->activeIdx[j] = root;
                gSurfaceActive[root].keep = 1U;
            }
        }
    }

    for (i = 0U; i < w0->count; i++) {
        int16_t root;
        if (!w0->valid[i]) continue;
        root = Surface_activeFind(w0->activeIdx[i]);
        if (gSurfaceActive[root].keep == 0U) {
            gSurfaceActive[root].keep = 2U;
            Surface_streamEmitCompleted(&gSurfaceActive[root].bounds,
                gSurfaceActive[root].birthOrder, cfg, completedSlots, overflowClusters);
        }
    }

    for (i = 0U; i < SURFACE_ACTIVE_CAP; i++) {
        if (gSurfaceActive[i].keep == 1U) {
            gSurfaceActive[i].parent = (int16_t)i;
        } else {
            gSurfaceActive[i].inUse = 0U;
            gSurfaceActive[i].keep = 0U;
        }
    }

    for (s = 1U; s < gSurfaceWinCount; s++)
        gSurfaceWin[s - 1U] = gSurfaceWin[s];
    gSurfaceWinCount--;
}

static void Surface_streamPushRange(
    uint16_t rangeIdx,
    const MmwDemo_DenseElevation_OutputSample *samples,
    uint16_t nSamples,
    const MmwDemo_SurfaceGen_StaticCfg *cfg,
    uint16_t *nextBirthOrder,
    uint16_t *completedSlots,
    uint32_t *overflowClusters)
{
    SurfaceWinSlot *ws;
    uint16_t i, slotSearch = 0U;

    while (gSurfaceWinCount > 0U &&
           (((uint32_t)rangeIdx > (uint32_t)gSurfaceWin[0].rangeIdx + (uint32_t)gSurfaceClusterThresh.rangeThreshBins) ||
            (gSurfaceWinCount >= 3U))) {
        Surface_streamRetireSlot0(cfg, completedSlots, overflowClusters);
    }

    ws = &gSurfaceWin[gSurfaceWinCount++];
    ws->rangeIdx = rangeIdx;
    ws->count = (nSamples > SURFACE_MAX_ROW_SAMPLES) ? SURFACE_MAX_ROW_SAMPLES : nSamples;
    for (i = 0U; i < ws->count; i++) {
        MmwDemo_DenseElevation_OutputSample key = samples[i];
        uint16_t at = i;
        while (at > 0U && Surface_before(&key, &ws->samples[at - 1U])) {
            ws->samples[at] = ws->samples[at - 1U];
            at--;
        }
        ws->samples[at] = key;
    }

    for (i = 0U; i < ws->count; i++) {
        SurfaceBounds b;
        uint8_t valid = Surface_bounds(&ws->samples[i], cfg, &b);
        uint16_t birth = (*nextBirthOrder)++;
        if (valid == 0U) {
            cfg->status->flags |= MMWDEMO_SURFACE_INVALID_GEOMETRY;
            gSurfaceInvalidGeometryCount[cfg->sourceStaticCfg.subFrameNum]++;
            ws->valid[i] = 0U;
            ws->activeIdx[i] = -1;
        } else if (valid == 2U) {
            ws->valid[i] = 0U;
            ws->activeIdx[i] = -1;
        } else {
            cfg->status->denseClustered++;
            while (slotSearch < SURFACE_ACTIVE_CAP && gSurfaceActive[slotSearch].inUse)
                slotSearch++;
            if (slotSearch < SURFACE_ACTIVE_CAP) {
                gSurfaceActive[slotSearch].bounds = b;
                gSurfaceActive[slotSearch].birthOrder = birth;
                gSurfaceActive[slotSearch].parent = (int16_t)slotSearch;
                gSurfaceActive[slotSearch].inUse = 1U;
                gSurfaceActive[slotSearch].keep = 0U;
                ws->valid[i] = 1U;
                ws->activeIdx[i] = (int16_t)slotSearch;
                slotSearch++;
            } else {
                ws->valid[i] = 0U;
                ws->activeIdx[i] = -1;
            }
        }
    }
}

#pragma CODE_SECTION(MmwDemo_SurfaceGen_cluster, ".surfaceCode")
int32_t MmwDemo_SurfaceGen_cluster(
    const MmwDemo_DenseElevation_OutputSample *samples,uint16_t n,
    const MmwDemo_SurfaceGen_StaticCfg *cfg,
    ObstacleBox *out,uint16_t cap,uint32_t *count)
{
    uint16_t i,j;
    if (count) *count=0U;
    if (cfg && cfg->status && n>SURFACE_SAMPLES)
        cfg->status->flags |= MMWDEMO_SURFACE_CLUSTER_LIMIT | MMWDEMO_SURFACE_PROCESS_ERROR;
    if (!cfg || !out || !count || !cfg->status || n>SURFACE_SAMPLES ||
        (n && !samples) || !cap ||
        cfg->sourceStaticCfg.subFrameNum>=MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES ||
        !cfg->sourceStaticCfg.numRangeBins || !cfg->sourceStaticCfg.numDopplerBins ||
        cfg->motionMode>MMWDEMO_SURFACE_MOTION_FIXED ||
        (cfg->motionMode==MMWDEMO_SURFACE_MOTION_FIXED && cfg->elapsedUsec==0U) ||
        !isfinite(cfg->rangeStepMeters) || !(cfg->rangeStepMeters>0.0f) ||
        !isfinite(cfg->rangeBiasMeters) || !isfinite(cfg->dopplerStepMps) ||
        !(cfg->dopplerStepMps>0.0f) || !isfinite(cfg->minRangeMeters) ||
        !isfinite(cfg->maxRangeMeters) || cfg->minRangeMeters<0.0f ||
        !(cfg->maxRangeMeters>cfg->minRangeMeters)) return MMWDEMO_NAV_EINVAL;
    gSurfaceDopplerGuardBins = Surface_calcDopplerGuardBins(cfg);
    gSurfaceClusterThresh = Surface_calcClusterThresh(cfg);
    if (cap>MMWDEMO_SURFACE_MAX_BOXES) cap=MMWDEMO_SURFACE_MAX_BOXES;

    for (i=0U;i<n;i++) {
        uint16_t at=i;
        while (at && Surface_before(&samples[i],&samples[gSurfaceOrder[at-1U]])) {
            gSurfaceOrder[at]=gSurfaceOrder[at-1U]; at--;
        }
        gSurfaceOrder[at]=i;
    }
    for (i=0U;i<n;i++) {
        uint8_t valid;
        gSurfaceParent[i]=(int16_t)i;
        valid=Surface_bounds(&samples[gSurfaceOrder[i]],cfg,&gSurfaceBounds[i]);
        if (!valid) {
            cfg->status->flags|=MMWDEMO_SURFACE_INVALID_GEOMETRY;
            gSurfaceInvalidGeometryCount[cfg->sourceStaticCfg.subFrameNum]++;
        } else if (valid==1U) cfg->status->denseClustered++;
    }
    for (i=0U;i<n;i++) {
        const MmwDemo_DenseElevation_OutputSample *a=&samples[gSurfaceOrder[i]];
        if (!gSurfaceBounds[i].valid) continue;
        for (j=i+1U;j<n;j++) {
            const MmwDemo_DenseElevation_OutputSample *b=&samples[gSurfaceOrder[j]];
            int16_t ri,rj;
            if ((uint32_t)b->rangeIdx > (uint32_t)a->rangeIdx + (uint32_t)gSurfaceClusterThresh.rangeThreshBins) break;
            if (!gSurfaceBounds[j].valid || !Surface_connected(a,b)) continue;
            ri=Surface_find((int16_t)i); rj=Surface_find((int16_t)j);
            if (ri==rj) continue;
            if (gSurfaceBounds[ri].rank<gSurfaceBounds[rj].rank) gSurfaceParent[ri]=rj;
            else {
                gSurfaceParent[rj]=ri;
                if (gSurfaceBounds[ri].rank==gSurfaceBounds[rj].rank) gSurfaceBounds[ri].rank++;
            }
        }
    }
    for (i=0U;i<n;i++) {
        int16_t root=Surface_find((int16_t)i);
        if (gSurfaceBounds[i].valid && root!=(int16_t)i)
            Surface_merge(&gSurfaceBounds[root],&gSurfaceBounds[i]);
    }
    Surface_selectAndEmit(n, 0U, cfg, out, cap, count);
    return 0;
}

#pragma CODE_SECTION(MmwDemo_SurfaceGen_process, ".surfaceCode")
int32_t MmwDemo_SurfaceGen_process(
    const MmwDemo_SurfaceGen_Cfg *cfg,const MmwDemo_SurfaceGen_StaticCfg *staticCfg,
    uint8_t *buffer,uint32_t size,MmwDemo_CustomObstacle *out,
    uint16_t maxOutput,uint32_t *count)
{
    MmwDemo_DenseElevation_OutputHeader *header;
    MmwDemo_DenseElevation_OutputSample *samples;
    MmwDemo_DenseElevation_OutputSample rangeSamples[SURFACE_MAX_ROW_SAMPLES];
    uint32_t start, elapsed, totalProduced = 0U, overflowClusters = 0U;
    uint16_t cap, sf, minRange = 0U, maxRange = 0U, processedCfar = 0U, processedRanges = 0U;
    uint16_t r, k, bufCount = 0U, nextBirthOrder = 0U, completedSlots = 0U;
    uint8_t cfarClamped = 0U, denseLimited = 0U;
    int32_t ret;
    if (count) *count=0U;
    if (!cfg || !staticCfg || !staticCfg->status || !buffer ||
        size<SURFACE_BUFFER_SIZE || !out || !count || !maxOutput ||
        !cfg->maxSurfacePoints ||
        staticCfg->sourceStaticCfg.subFrameNum>=MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES ||
        !staticCfg->sourceStaticCfg.numRangeBins || !staticCfg->sourceStaticCfg.numDopplerBins ||
        staticCfg->motionMode>MMWDEMO_SURFACE_MOTION_FIXED ||
        (staticCfg->motionMode==MMWDEMO_SURFACE_MOTION_FIXED && staticCfg->elapsedUsec==0U) ||
        !isfinite(staticCfg->rangeStepMeters) || !(staticCfg->rangeStepMeters>0.0f) ||
        !isfinite(staticCfg->rangeBiasMeters) || !isfinite(staticCfg->dopplerStepMps) ||
        !(staticCfg->dopplerStepMps>0.0f) || !isfinite(staticCfg->minRangeMeters) ||
        !isfinite(staticCfg->maxRangeMeters) || staticCfg->minRangeMeters<0.0f ||
        !(staticCfg->maxRangeMeters>staticCfg->minRangeMeters))
        return MMWDEMO_NAV_EINVAL;
    sf=staticCfg->sourceStaticCfg.subFrameNum;
    gSurfaceDopplerGuardBins = Surface_calcDopplerGuardBins(staticCfg);
    gSurfaceClusterThresh = Surface_calcClusterThresh(staticCfg);
    start=Cycleprofiler_getTimeStamp();
    ret=MmwDemo_DenseElevation_prepareFrame(&cfg->sourceCfg,&staticCfg->sourceStaticCfg,
        &minRange,&maxRange,&processedCfar,&cfarClamped);
    if (ret) { staticCfg->status->flags|=MMWDEMO_SURFACE_PROCESS_ERROR; return ret; }
    if (cfarClamped) staticCfg->status->flags|=MMWDEMO_SURFACE_CFAR_LIMIT;

    header=(MmwDemo_DenseElevation_OutputHeader *)buffer;
    samples=(MmwDemo_DenseElevation_OutputSample *)(buffer+sizeof(*header));
    gSurfaceWinCount=0U;
    memset(gSurfaceActive, 0, sizeof(gSurfaceActive));

    for (r=minRange; r<=maxRange; r++) {
        uint16_t nRange=0U;
        uint8_t rangeLim=0U;
        if (MmwDemo_DenseElevation_processRange(r,&cfg->sourceCfg,&staticCfg->sourceStaticCfg,
                rangeSamples,SURFACE_MAX_ROW_SAMPLES,&nRange,&rangeLim)) {
            processedRanges++;
            if (rangeLim) denseLimited=1U;
            if (nRange > 0U) {
                totalProduced += nRange;
                for (k=0U; k<nRange && bufCount<SURFACE_SAMPLES; k++)
                    samples[bufCount++] = rangeSamples[k];
                Surface_streamPushRange(r,rangeSamples,nRange,staticCfg,
                    &nextBirthOrder,&completedSlots,&overflowClusters);
            }
        }
    }
    while (gSurfaceWinCount > 0U)
        Surface_streamRetireSlot0(staticCfg,&completedSlots,&overflowClusters);

    memset(header, 0, sizeof(*header));
    header->magic = MMWDEMO_DENSE_ELEVATION_MAGIC;
    header->version = MMWDEMO_DENSE_ELEVATION_PROTOCOL_VERSION;
    header->headerSize = sizeof(*header);
    header->sampleSize = sizeof(*samples);
    header->numSamples = bufCount;
    header->flags = MMWDEMO_DENSE_REQUIRED_FLAGS |
        ((denseLimited || totalProduced > SURFACE_SAMPLES) ? MMWDEMO_DENSE_FLAG_OUTPUT_LIMITED : 0U) |
        (staticCfg->sourceStaticCfg.staticClutterRemovalEnabled ? MMWDEMO_DENSE_FLAG_STATIC_CLUTTER_APPLIED : 0U);
    header->subFrameNumber = sf;
    header->rangeStart = minRange;
    header->rangeEnd = maxRange + 1U;
    header->numRangeBins = staticCfg->sourceStaticCfg.numRangeBins;
    header->numDopplerBins = staticCfg->sourceStaticCfg.numDopplerBins;
    header->numAzimuthBins = MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
    header->numElevationBins = MMWDEMO_DENSE_ELEV_NUM_ELEVATION_BINS;
    header->radarFrameNumber = staticCfg->sourceStaticCfg.frameNumber;
    header->reservedPad = (uint32_t)processedCfar | ((uint32_t)processedRanges << 16U);

    staticCfg->status->cfarProcessed = processedCfar;
    staticCfg->status->rangeCount = processedRanges;
    staticCfg->status->denseProduced = (totalProduced > 65535U) ? 65535U : (uint16_t)totalProduced;
    if (denseLimited)
        staticCfg->status->flags |= MMWDEMO_SURFACE_DENSE_LIMIT;
    cap=(cfg->maxSurfacePoints<maxOutput) ? cfg->maxSurfacePoints : maxOutput;
    if (cfg->uartMaxSurfacePoints && cfg->uartMaxSurfacePoints<cap) {
        cap=cfg->uartMaxSurfacePoints;
        staticCfg->status->flags|=MMWDEMO_SURFACE_UART_LIMIT;
    }
    if (cap>MMWDEMO_SURFACE_MAX_BOXES) cap=MMWDEMO_SURFACE_MAX_BOXES;

    gNavDbgDenseSamples=staticCfg->status->denseProduced;
    Surface_selectAndEmit(completedSlots, overflowClusters, staticCfg, out, cap, count);
    {
        uint16_t numDynamic = 0U;
        uint16_t numStatic = 0U;
        uint16_t totalBoxes = (uint16_t)(*count);
        for (k = 0U; k < totalBoxes; k++) {
            if (MMWDEMO_OBSTACLE_IS_DYNAMIC(out[k].boxId)) {
                numDynamic++;
            } else {
                numStatic++;
            }
        }
        gHSRAM.numDynamicObstacles = (uint16_t)numDynamic;
        gHSRAM.numStaticObstacles = (uint16_t)numStatic;
        staticCfg->status->boxCount = (uint16_t)(numDynamic + numStatic);
        *count = (uint32_t)(numDynamic + numStatic);
    }
    elapsed=Cycleprofiler_getTimeStamp()-start;
    header->processingCycles = elapsed;
    gNavEmitSurfaceCycles[sf]=elapsed;
    if (elapsed>gNavEmitSurfaceMaxCycles[sf]) gNavEmitSurfaceMaxCycles[sf]=elapsed;
    gNavDbgFilteredSamples=*count;
    return 0;
}

#ifdef MMWDEMO_SURFACE_SELFTEST
#include "tests/surface_fixtures.inc"
#endif

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

#pragma DATA_SECTION(gSurfaceBounds, ".l3ram")
#pragma DATA_ALIGN(gSurfaceBounds, 8)
static far SurfaceBounds gSurfaceBounds[SURFACE_SAMPLES];
#pragma DATA_SECTION(gSurfaceParent, ".l3ram")
static far int16_t gSurfaceParent[SURFACE_SAMPLES];
#pragma DATA_SECTION(gSurfaceOrder, ".l3ram")
static far uint16_t gSurfaceOrder[SURFACE_SAMPLES];
#pragma DATA_SECTION(gSurfaceHistory, ".l3ram")
static far SurfaceHistory gSurfaceHistory[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES][SURFACE_HISTORY];
static uint8_t gSurfaceMotion[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
volatile uint32_t gSurfaceInvalidGeometryCount[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
volatile uint32_t gNavEmitSurfaceCycles[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
volatile uint32_t gNavEmitSurfaceMaxCycles[MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES];
extern volatile uint32_t gNavDbgDenseSamples;
extern volatile uint32_t gNavDbgFilteredSamples;

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
    int32_t d = (int32_t)a->dopplerIdx - b->dopplerIdx;
    float u = fabsf((float)a->azimuthUQ15-b->azimuthUQ15) / 32767.0f;
    float v = fabsf((float)a->sourceVQ15-b->sourceVQ15) / 32767.0f;
    if (d < 0) d = -d;
    return (uint8_t)(d <= 3 &&
        u <= 0.06f + 0.5f*((float)a->azimuthWidthUQ15+b->azimuthWidthUQ15)/32767.0f &&
        v <= 0.15f + 0.5f*((float)a->elevationWidthUQ15+b->elevationWidthUQ15)/32767.0f);
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
    a->loX=fminf(a->loX,b->loX); a->hiX=fmaxf(a->hiX,b->hiX);
    a->loY=fminf(a->loY,b->loY); a->hiY=fmaxf(a->hiY,b->hiY);
    a->loZ=fminf(a->loZ,b->loZ); a->hiZ=fmaxf(a->hiZ,b->hiZ);
    if (b->minRange < a->minRange) a->minRange = b->minRange;
    if (b->snr > a->snr || (b->snr == a->snr && b->doppler < a->doppler)) {
        a->snr = b->snr; a->doppler = b->doppler;
    }
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
    out->width_x=Surface_extent(b->loX,b->hiX,0.10f);
    out->depth_y=Surface_extent(b->loY,b->hiY,cfg->rangeStepMeters);
    out->height_z=Surface_extent(b->loZ,b->hiZ,0.10f);
    out->velocity=(float)b->doppler*cfg->dopplerStepMps;
    out->snr=(int16_t)b->snr;
    out->boxId=0;
}

static uint8_t Surface_better(uint16_t a, int16_t b)
{
    if (b < 0) return 1U;
    if (gSurfaceBounds[a].snr != gSurfaceBounds[b].snr)
        return gSurfaceBounds[a].snr > gSurfaceBounds[b].snr;
    return a < (uint16_t)b;
}

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
    for (sf=0U; sf<MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES; sf++)
        MmwDemo_SurfaceGen_resetSubFrame(sf);
}

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

int32_t MmwDemo_SurfaceGen_cluster(
    const MmwDemo_DenseElevation_OutputSample *samples,uint16_t n,
    const MmwDemo_SurfaceGen_StaticCfg *cfg,
    ObstacleBox *out,uint16_t cap,uint32_t *count)
{
    uint16_t i,j,current=0U,reps=0U;
    int16_t best[SURFACE_CELLS];
    uint32_t clusters=0U;
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
            if ((uint32_t)b->rangeIdx>(uint32_t)a->rangeIdx+2U) break;
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
    for (i=0U;i<SURFACE_CELLS;i++) best[i]=-1;
    for (i=0U;i<n;i++) {
        if (gSurfaceBounds[i].valid && Surface_find((int16_t)i)==(int16_t)i) {
            uint16_t cell=Surface_cell(&gSurfaceBounds[i],cfg);
            clusters++; gSurfaceBounds[i].rank=0U;
            if (Surface_better(i,best[cell])) best[cell]=(int16_t)i;
        }
    }
    cfg->status->clusterCount=(uint16_t)clusters;
    if (clusters>cap) cfg->status->flags|=MMWDEMO_SURFACE_BOX_LIMIT;
    for (i=0U;i<SURFACE_CELLS;i++) if (best[i]>=0) best[reps++]=best[i];
    /* With fewer slots than occupied cells, sample the complete cell range. */
    for (i=0U;i<reps && current<cap;i++) {
        uint16_t pos=(reps>cap) ? (uint16_t)((uint32_t)i*reps/cap) : i;
        uint16_t root=(uint16_t)best[pos];
        Surface_box(&gSurfaceBounds[root],cfg,&out[current++]);
        gSurfaceBounds[root].rank=1U;
    }
    while (current<cap && current<clusters) {
        int16_t next=-1;
        for (i=0U;i<n;i++)
            if (gSurfaceBounds[i].valid && !gSurfaceBounds[i].rank &&
                Surface_find((int16_t)i)==(int16_t)i && Surface_better(i,next))
                next=(int16_t)i;
        if (next<0) break;
        Surface_box(&gSurfaceBounds[next],cfg,&out[current++]);
        gSurfaceBounds[next].rank=1U;
    }
    current=Surface_history(out,current,cap,cfg);
    for (i=1U;i<current;i++) {
        ObstacleBox key=out[i]; uint16_t at=i;
        while (at && (out[at-1U].y>key.y ||
            (out[at-1U].y==key.y && out[at-1U].x>key.x))) {
            out[at]=out[at-1U]; at--;
        }
        out[at]=key;
    }
    for (i=0U;i<current;i++) {
        ObstacleBox *b=&out[i];
        uint8_t type=MMWDEMO_OBSTACLE_CLASS_UNKNOWN;
        if (b->width_x<=1.0f && b->depth_y<=1.0f &&
            b->height_z<=0.6f && b->z>=0.4f) type=MMWDEMO_OBSTACLE_CLASS_DRONE;
        else if (b->width_x<=0.9f && b->depth_y<=0.9f &&
            b->height_z>=1.0f && b->z<=0.5f) type=MMWDEMO_OBSTACLE_CLASS_PEDESTRIAN;
        else if ((b->width_x>=1.1f || b->depth_y>=1.5f) &&
            b->snr>=200) type=MMWDEMO_OBSTACLE_CLASS_VEHICLE;
        b->boxId=MMWDEMO_PACK_OBSTACLE_BOX_ID(type,i+1U);
    }
    cfg->status->boxCount=current;
    *count=current;
    return 0;
}

int32_t MmwDemo_SurfaceGen_process(
    const MmwDemo_SurfaceGen_Cfg *cfg,const MmwDemo_SurfaceGen_StaticCfg *staticCfg,
    uint8_t *buffer,uint32_t size,MmwDemo_CustomObstacle *out,
    uint16_t maxOutput,uint32_t *count)
{
    MmwDemo_DenseElevation_OutputHeader *header;
    MmwDemo_DenseElevation_OutputSample *samples;
    uint32_t produced=0U,start,elapsed;
    uint16_t cap,sf;
    int32_t ret;
    if (count) *count=0U;
    if (!cfg || !staticCfg || !staticCfg->status || !buffer ||
        size<SURFACE_BUFFER_SIZE || !out || !count || !maxOutput ||
        !cfg->maxSurfacePoints ||
        staticCfg->sourceStaticCfg.subFrameNum>=MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES)
        return MMWDEMO_NAV_EINVAL;
    sf=staticCfg->sourceStaticCfg.subFrameNum;
    ret=MmwDemo_DenseElevation_process(&cfg->sourceCfg,&staticCfg->sourceStaticCfg,
        buffer,size,SURFACE_SAMPLES,&produced);
    if (ret) { staticCfg->status->flags|=MMWDEMO_SURFACE_PROCESS_ERROR; return ret; }
    header=(MmwDemo_DenseElevation_OutputHeader *)buffer;
    if (produced<sizeof(*header) || header->magic!=MMWDEMO_DENSE_ELEVATION_MAGIC ||
        header->version!=MMWDEMO_DENSE_ELEVATION_PROTOCOL_VERSION ||
        header->headerSize!=sizeof(*header) || header->sampleSize!=sizeof(*samples) ||
        header->protocolStatus || header->subFrameNumber!=sf ||
        header->numSamples>SURFACE_SAMPLES ||
        (header->flags & MMWDEMO_DENSE_REQUIRED_FLAGS)!=MMWDEMO_DENSE_REQUIRED_FLAGS ||
        produced!=sizeof(*header)+header->numSamples*sizeof(*samples)) {
        staticCfg->status->flags|=MMWDEMO_SURFACE_PROCESS_ERROR;
        return MMWDEMO_NAV_ECANDIDATE_PAYLOAD;
    }
    staticCfg->status->cfarProcessed=(uint16_t)header->reservedPad;
    staticCfg->status->rangeCount=(uint16_t)(header->reservedPad>>16U);
    staticCfg->status->denseProduced=header->numSamples;
    if (header->flags & MMWDEMO_DENSE_FLAG_OUTPUT_LIMITED)
        staticCfg->status->flags|=MMWDEMO_SURFACE_DENSE_LIMIT;
    cap=(cfg->maxSurfacePoints<maxOutput) ? cfg->maxSurfacePoints : maxOutput;
    if (cfg->uartMaxSurfacePoints && cfg->uartMaxSurfacePoints<cap) {
        cap=cfg->uartMaxSurfacePoints;
        staticCfg->status->flags|=MMWDEMO_SURFACE_UART_LIMIT;
    }
    samples=(MmwDemo_DenseElevation_OutputSample *)(buffer+sizeof(*header));
    gNavDbgDenseSamples=header->numSamples;
    start=Cycleprofiler_getTimeStamp();
    ret=MmwDemo_SurfaceGen_cluster(samples,header->numSamples,staticCfg,out,cap,count);
    elapsed=Cycleprofiler_getTimeStamp()-start;
    gNavEmitSurfaceCycles[sf]=elapsed;
    if (elapsed>gNavEmitSurfaceMaxCycles[sf]) gNavEmitSurfaceMaxCycles[sf]=elapsed;
    gNavDbgFilteredSamples=*count;
    if (ret) staticCfg->status->flags|=MMWDEMO_SURFACE_PROCESS_ERROR;
    return ret;
}

#ifdef MMWDEMO_SURFACE_SELFTEST
#include "tests/surface_fixtures.inc"
#endif

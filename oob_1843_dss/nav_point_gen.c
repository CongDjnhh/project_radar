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
    const MmwDemo_DenseElevation_OutputSample *src)
{
    dst->rangeIdx = src->rangeIdx;
    dst->azimuthIdx = src->azimuthIdx;
    dst->elevationIdx = src->elevationIdx;
    dst->dopplerIdx = src->dopplerIdx;
    dst->snrDb10 = src->snrDb10;
    dst->sourceVQ15 = src->sourceVQ15;
    dst->powerDb10 = src->powerDb10;
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
                                       &samples[candidateIndex]);
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
        samples, sampleCount, NULL, maxOutput, output,
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

static float MmwDemo_NavPointGen_clampf(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

static void MmwDemo_NavPointGen_prepareCartesian(
    const MmwDemo_NavPoint *points,
    uint16_t pointCount,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg)
{
    uint16_t index;

    for (index = 0U; index < pointCount; index++)
    {
        float u = 2.0f * (float)points[index].azimuthIdx /
                  (float)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
        float v = (float)points[index].sourceVQ15 / 32767.0f;
        float horizontalSquared;
        float range = (float)points[index].rangeIdx *
                      staticCfg->rangeStepMeters -
                      staticCfg->rangeBiasMeters;

        u = MmwDemo_NavPointGen_clampf(u, -1.0f, 1.0f);
        v = MmwDemo_NavPointGen_clampf(v, -1.0f, 1.0f);
        horizontalSquared = 1.0f - u * u - v * v;
        if (horizontalSquared < 0.0f)
        {
            horizontalSquared = 0.0f;
        }
        if (range < 0.0f)
        {
            range = 0.0f;
        }
        gNavScratch.cluster.x[index] = range * u;
        gNavScratch.cluster.y[index] = range * sqrtf(horizontalSquared);
        gNavScratch.cluster.z[index] = range * v;
        gNavScratch.cluster.compensatedDoppler[index] =
            (float)points[index].dopplerIdx * staticCfg->dopplerStepMps;
    }
}

static uint8_t MmwDemo_NavPointGen_dbscanNeighbor(
    uint16_t first,
    uint16_t second,
    const MmwDemo_NavPoint *points,
    const MmwDemo_NavPointGen_Cfg *cfg,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg)
{
    float dx = gNavScratch.cluster.x[first] -
               gNavScratch.cluster.x[second];
    float dy = gNavScratch.cluster.y[first] -
               gNavScratch.cluster.y[second];
    float dz = gNavScratch.cluster.z[first] -
               gNavScratch.cluster.z[second];
    float firstRange = (float)points[first].rangeIdx *
                       staticCfg->rangeStepMeters;
    float secondRange = (float)points[second].rangeIdx *
                        staticCfg->rangeStepMeters;
    float nearRange = (firstRange < secondRange) ? firstRange : secondRange;
    float epsilon = cfg->dbscanEpsilonM +
                    cfg->dbscanRangeScale * nearRange;

    return ((dx * dx + dy * dy + dz * dz) <= epsilon * epsilon) ? 1U : 0U;
}

static uint16_t MmwDemo_NavPointGen_dbscanNeighborCount(
    uint16_t center,
    uint16_t pointCount,
    const MmwDemo_NavPoint *points,
    const MmwDemo_NavPointGen_Cfg *cfg,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
    uint32_t deadlineStartCycles,
    uint32_t deadlineOffsetCycles,
    uint32_t *distanceTests,
    uint8_t *timeLimited)
{
    uint16_t index;
    uint16_t count = 0U;

    for (index = 0U; index < pointCount; index++)
    {
        count += MmwDemo_NavPointGen_dbscanNeighbor(
            center, index, points, cfg, staticCfg);
        (*distanceTests)++;
        if (((*distanceTests & 255U) == 0U) &&
            (MmwDemo_NavPointGen_deadlineExpired(
                deadlineStartCycles, deadlineOffsetCycles) != 0U))
        {
            *timeLimited = 1U;
            break;
        }
    }
    return count;
}

static uint8_t MmwDemo_NavPointGen_runDbscan(
    MmwDemo_NavPoint *points,
    uint16_t pointCount,
    const MmwDemo_NavPointGen_Cfg *cfg,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
    uint32_t deadlineStartCycles,
    uint32_t deadlineOffsetCycles,
    uint8_t *limited,
    uint8_t *timeLimited)
{
    uint16_t index;
    uint8_t clusterCount = 0U;
    uint32_t distanceTests = 0U;

    memset(gNavScratch.cluster.label, MMWDEMO_NAV_DBSCAN_UNVISITED,
           pointCount * sizeof(gNavScratch.cluster.label[0]));
    for (index = 0U; index < pointCount; index++)
    {
        uint16_t queueRead;
        uint16_t queueWrite;
        uint16_t neighbor;

        if (gNavScratch.cluster.label[index] !=
            MMWDEMO_NAV_DBSCAN_UNVISITED)
        {
            continue;
        }
        if (MmwDemo_NavPointGen_dbscanNeighborCount(
                index, pointCount, points, cfg, staticCfg,
                deadlineStartCycles, deadlineOffsetCycles,
                &distanceTests, timeLimited) <
            cfg->dbscanMinPoints)
        {
            if (*timeLimited != 0U)
            {
                return clusterCount;
            }
            gNavScratch.cluster.label[index] = MMWDEMO_NAV_DBSCAN_NOISE;
            continue;
        }
        if (*timeLimited != 0U)
        {
            return clusterCount;
        }
        if (clusterCount >= cfg->dbscanMaxClusters)
        {
            gNavScratch.cluster.label[index] = MMWDEMO_NAV_DBSCAN_NOISE;
            *limited = 1U;
            continue;
        }

        clusterCount++;
        gNavScratch.cluster.label[index] = clusterCount;
        queueRead = 0U;
        queueWrite = 0U;
        for (neighbor = 0U; neighbor < pointCount; neighbor++)
        {
            uint8_t isNeighbor = MmwDemo_NavPointGen_dbscanNeighbor(
                index, neighbor, points, cfg, staticCfg);
            distanceTests++;
            if (((distanceTests & 255U) == 0U) &&
                (MmwDemo_NavPointGen_deadlineExpired(
                    deadlineStartCycles, deadlineOffsetCycles) != 0U))
            {
                *timeLimited = 1U;
                return clusterCount;
            }
            if ((neighbor != index) && (isNeighbor != 0U))
            {
                if (gNavScratch.cluster.label[neighbor] ==
                    MMWDEMO_NAV_DBSCAN_UNVISITED)
                {
                    gNavScratch.cluster.label[neighbor] = clusterCount;
                    gNavScratch.cluster.queue[queueWrite++] = neighbor;
                }
                else if (gNavScratch.cluster.label[neighbor] ==
                         MMWDEMO_NAV_DBSCAN_NOISE)
                {
                    gNavScratch.cluster.label[neighbor] = clusterCount;
                }
            }
        }

        while (queueRead < queueWrite)
        {
            uint16_t current = gNavScratch.cluster.queue[queueRead++];
            if (MmwDemo_NavPointGen_dbscanNeighborCount(
                    current, pointCount, points, cfg, staticCfg,
                    deadlineStartCycles, deadlineOffsetCycles,
                    &distanceTests, timeLimited) >=
                cfg->dbscanMinPoints)
            {
                if (*timeLimited != 0U)
                {
                    return clusterCount;
                }
                for (neighbor = 0U; neighbor < pointCount; neighbor++)
                {
                    uint8_t isNeighbor = MmwDemo_NavPointGen_dbscanNeighbor(
                        current, neighbor, points, cfg, staticCfg);
                    distanceTests++;
                    if (((distanceTests & 255U) == 0U) &&
                        (MmwDemo_NavPointGen_deadlineExpired(
                            deadlineStartCycles,
                            deadlineOffsetCycles) != 0U))
                    {
                        *timeLimited = 1U;
                        return clusterCount;
                    }
                    if (isNeighbor == 0U)
                    {
                        continue;
                    }
                    if (gNavScratch.cluster.label[neighbor] ==
                        MMWDEMO_NAV_DBSCAN_UNVISITED)
                    {
                        gNavScratch.cluster.label[neighbor] = clusterCount;
                        if (queueWrite < pointCount)
                        {
                            gNavScratch.cluster.queue[queueWrite++] = neighbor;
                        }
                        else
                        {
                            *limited = 1U;
                        }
                    }
                    else if (gNavScratch.cluster.label[neighbor] ==
                             MMWDEMO_NAV_DBSCAN_NOISE)
                    {
                        gNavScratch.cluster.label[neighbor] = clusterCount;
                    }
                }
            }
            else if (*timeLimited != 0U)
            {
                return clusterCount;
            }
        }
    }
    return clusterCount;
}

static uint16_t MmwDemo_NavPointGen_classifyWallClusters(
    uint16_t pointCount,
    uint8_t clusterCount,
    const MmwDemo_NavPointGen_Cfg *cfg,
    uint8_t egoValid,
    uint32_t deadlineStartCycles,
    uint32_t deadlineOffsetCycles,
    uint8_t *timeLimited)
{
    uint8_t cluster;
    uint16_t wallClusters = 0U;

    for (cluster = 1U; cluster <= clusterCount; cluster++)
    {
        uint16_t index;
        uint16_t count = 0U;
        float meanX = 0.0f;
        float meanY = 0.0f;
        float covarianceXX = 0.0f;
        float covarianceXY = 0.0f;
        float covarianceYY = 0.0f;
        float trace;
        float root;
        float majorVariance;
        float minorVariance;
        float majorLength;
        float thickness;
        float aspect;
        float meanAbsCompensatedDoppler = 0.0f;

        if (MmwDemo_NavPointGen_deadlineExpired(
                deadlineStartCycles, deadlineOffsetCycles) != 0U)
        {
            *timeLimited = 1U;
            return wallClusters;
        }

        for (index = 0U; index < pointCount; index++)
        {
            if (gNavScratch.cluster.label[index] == cluster)
            {
                meanX += gNavScratch.cluster.x[index];
                meanY += gNavScratch.cluster.y[index];
                meanAbsCompensatedDoppler +=
                    fabsf(gNavScratch.cluster.compensatedDoppler[index]);
                count++;
            }
            if (((index & 63U) == 63U) &&
                (MmwDemo_NavPointGen_deadlineExpired(
                    deadlineStartCycles, deadlineOffsetCycles) != 0U))
            {
                *timeLimited = 1U;
                return wallClusters;
            }
        }
        if (count == 0U)
        {
            continue;
        }
        meanX /= (float)count;
        meanY /= (float)count;
        meanAbsCompensatedDoppler /= (float)count;
        for (index = 0U; index < pointCount; index++)
        {
            if (gNavScratch.cluster.label[index] == cluster)
            {
                float dx = gNavScratch.cluster.x[index] - meanX;
                float dy = gNavScratch.cluster.y[index] - meanY;
                covarianceXX += dx * dx;
                covarianceXY += dx * dy;
                covarianceYY += dy * dy;
            }
            if (((index & 63U) == 63U) &&
                (MmwDemo_NavPointGen_deadlineExpired(
                    deadlineStartCycles, deadlineOffsetCycles) != 0U))
            {
                *timeLimited = 1U;
                return wallClusters;
            }
        }
        covarianceXX /= (float)count;
        covarianceXY /= (float)count;
        covarianceYY /= (float)count;
        trace = covarianceXX + covarianceYY;
        root = sqrtf((covarianceXX - covarianceYY) *
                     (covarianceXX - covarianceYY) +
                     4.0f * covarianceXY * covarianceXY);
        majorVariance = 0.5f * (trace + root);
        minorVariance = 0.5f * (trace - root);
        if (minorVariance < 0.0f)
        {
            minorVariance = 0.0f;
        }
        majorLength = 4.0f * sqrtf(majorVariance);
        thickness = 4.0f * sqrtf(minorVariance);
        aspect = sqrtf(majorVariance /
                       ((minorVariance > 0.0001f) ?
                        minorVariance : 0.0001f));

        /* Geometry alone can erase a long moving obstacle. Suppress a
         * wall-like cluster only when frame-aligned ego compensation is valid
         * and the cluster is stationary in the compensated radial domain. */
        if ((egoValid != 0U) &&
            (meanAbsCompensatedDoppler <=
             cfg->staticVelocityThresholdMps) &&
            (count >= (uint16_t)(2U * cfg->dbscanMinPoints)) &&
            (majorLength >= cfg->wallMinLengthM) &&
            (thickness <= cfg->wallMaxThicknessM) &&
            (aspect >= cfg->wallMinAspectRatio))
        {
            wallClusters++;
            for (index = 0U; index < pointCount; index++)
            {
                if (gNavScratch.cluster.label[index] == cluster)
                {
                    gNavScratch.cluster.label[index] =
                        MMWDEMO_NAV_DBSCAN_WALL;
                }
                if (((index & 63U) == 63U) &&
                    (MmwDemo_NavPointGen_deadlineExpired(
                        deadlineStartCycles, deadlineOffsetCycles) != 0U))
                {
                    *timeLimited = 1U;
                    return wallClusters;
                }
            }
        }
    }
    return wallClusters;
}

static uint8_t MmwDemo_NavPointGen_estimateSceneEgo(
    const MmwDemo_NavPoint *points,
    uint16_t pointCount,
    const MmwDemo_NavPointGen_Cfg *cfg,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
    float *velocityX,
    float *velocityY,
    float *confidence,
    uint32_t deadlineStartCycles,
    uint32_t deadlineOffsetCycles,
    uint8_t *timeLimited)
{
    uint16_t stride;
    uint16_t first;
    uint16_t second;
    uint16_t bestInliers = 0U;
    float bestResidual = 1.0e30f;
    float bestX = 0.0f;
    float bestY = 0.0f;
    float threshold = cfg->sceneResidualThresholdMps;

    if ((pointCount < cfg->sceneMinInliers) || !(threshold > 0.0f))
    {
        *confidence = 0.0f;
        return 0U;
    }
    stride = (pointCount + 15U) / 16U;
    if (stride == 0U)
    {
        stride = 1U;
    }
    for (first = 0U; first < pointCount; first += stride)
    {
        for (second = (uint16_t)(first + stride);
             second < pointCount;
             second += stride)
        {
            float rangeFirst = sqrtf(
                gNavScratch.cluster.x[first] * gNavScratch.cluster.x[first] +
                gNavScratch.cluster.y[first] * gNavScratch.cluster.y[first] +
                gNavScratch.cluster.z[first] * gNavScratch.cluster.z[first]);
            float rangeSecond = sqrtf(
                gNavScratch.cluster.x[second] * gNavScratch.cluster.x[second] +
                gNavScratch.cluster.y[second] * gNavScratch.cluster.y[second] +
                gNavScratch.cluster.z[second] * gNavScratch.cluster.z[second]);
            float firstU;
            float firstForward;
            float secondU;
            float secondForward;
            float determinant;
            float firstB;
            float secondB;
            float candidateX;
            float candidateY;
            float speedSquared;
            uint16_t index;
            uint16_t inliers = 0U;
            float residualSum = 0.0f;

            if (MmwDemo_NavPointGen_deadlineExpired(
                    deadlineStartCycles, deadlineOffsetCycles) != 0U)
            {
                *timeLimited = 1U;
                *confidence = 0.0f;
                return 0U;
            }

            if ((rangeFirst <= 0.001f) || (rangeSecond <= 0.001f))
            {
                continue;
            }
            firstU = gNavScratch.cluster.x[first] / rangeFirst;
            firstForward = gNavScratch.cluster.y[first] / rangeFirst;
            secondU = gNavScratch.cluster.x[second] / rangeSecond;
            secondForward = gNavScratch.cluster.y[second] / rangeSecond;
            determinant = firstU * secondForward -
                          secondU * firstForward;
            if (fabsf(determinant) < 0.05f)
            {
                continue;
            }
            firstB = -((float)points[first].dopplerIdx *
                       staticCfg->dopplerStepMps);
            secondB = -((float)points[second].dopplerIdx *
                        staticCfg->dopplerStepMps);
            candidateX = (firstB * secondForward -
                          secondB * firstForward) / determinant;
            candidateY = (firstU * secondB - secondU * firstB) /
                         determinant;
            speedSquared = candidateX * candidateX +
                           candidateY * candidateY;
            if ((!isfinite(candidateX)) || (!isfinite(candidateY)) ||
                (!isfinite(speedSquared)) ||
                (speedSquared > cfg->egoMaxSpeedMps * cfg->egoMaxSpeedMps))
            {
                continue;
            }
            for (index = 0U; index < pointCount; index++)
            {
                float range;
                float residual;
                if (((index & 63U) == 0U) &&
                    (MmwDemo_NavPointGen_deadlineExpired(
                        deadlineStartCycles, deadlineOffsetCycles) != 0U))
                {
                    *timeLimited = 1U;
                    *confidence = 0.0f;
                    return 0U;
                }
                range = sqrtf(
                    gNavScratch.cluster.x[index] *
                    gNavScratch.cluster.x[index] +
                    gNavScratch.cluster.y[index] *
                    gNavScratch.cluster.y[index] +
                    gNavScratch.cluster.z[index] *
                    gNavScratch.cluster.z[index]);
                if (range <= 0.001f)
                {
                    continue;
                }
                residual = (float)points[index].dopplerIdx *
                           staticCfg->dopplerStepMps +
                           (gNavScratch.cluster.x[index] / range) * candidateX +
                           (gNavScratch.cluster.y[index] / range) * candidateY;
                residual = fabsf(residual);
                if (residual <= threshold)
                {
                    inliers++;
                    residualSum += residual;
                }
                if (((index & 63U) == 63U) &&
                    (MmwDemo_NavPointGen_deadlineExpired(
                        deadlineStartCycles, deadlineOffsetCycles) != 0U))
                {
                    *timeLimited = 1U;
                    *confidence = 0.0f;
                    return 0U;
                }
            }
            if ((inliers > bestInliers) ||
                ((inliers == bestInliers) &&
                 (residualSum < bestResidual)))
            {
                bestInliers = inliers;
                bestResidual = residualSum;
                bestX = candidateX;
                bestY = candidateY;
            }
        }
    }
    if (bestInliers < cfg->sceneMinInliers)
    {
        *confidence = 0.0f;
        return 0U;
    }

    {
        float sumUU = 0.0f;
        float sumUF = 0.0f;
        float sumFF = 0.0f;
        float sumUB = 0.0f;
        float sumFB = 0.0f;
        float residualSum = 0.0f;
        uint16_t inliers = 0U;
        uint16_t index;
        float determinant;
        float speedSquared;

        for (index = 0U; index < pointCount; index++)
        {
            float range;
            float u;
            float forward;
            float measured;
            float residual;
            float b;
            if (((index & 63U) == 0U) &&
                (MmwDemo_NavPointGen_deadlineExpired(
                    deadlineStartCycles, deadlineOffsetCycles) != 0U))
            {
                *timeLimited = 1U;
                *confidence = 0.0f;
                return 0U;
            }
            range = sqrtf(
                gNavScratch.cluster.x[index] * gNavScratch.cluster.x[index] +
                gNavScratch.cluster.y[index] * gNavScratch.cluster.y[index] +
                gNavScratch.cluster.z[index] * gNavScratch.cluster.z[index]);
            if (range <= 0.001f)
            {
                continue;
            }
            u = gNavScratch.cluster.x[index] / range;
            forward = gNavScratch.cluster.y[index] / range;
            measured = (float)points[index].dopplerIdx *
                       staticCfg->dopplerStepMps;
            residual = fabsf(measured + u * bestX + forward * bestY);
            if (residual > threshold)
            {
                continue;
            }
            b = -measured;
            sumUU += u * u;
            sumUF += u * forward;
            sumFF += forward * forward;
            sumUB += u * b;
            sumFB += forward * b;
            residualSum += residual;
            inliers++;
            if (((index & 63U) == 63U) &&
                (MmwDemo_NavPointGen_deadlineExpired(
                    deadlineStartCycles, deadlineOffsetCycles) != 0U))
            {
                *timeLimited = 1U;
                *confidence = 0.0f;
                return 0U;
            }
        }
        determinant = sumUU * sumFF - sumUF * sumUF;
        if ((inliers < cfg->sceneMinInliers) ||
            (fabsf(determinant) < 0.01f))
        {
            *confidence = 0.0f;
            return 0U;
        }
        *velocityX = (sumUB * sumFF - sumFB * sumUF) / determinant;
        *velocityY = (sumUU * sumFB - sumUF * sumUB) / determinant;
        speedSquared = *velocityX * *velocityX +
                       *velocityY * *velocityY;
        if ((!isfinite(*velocityX)) || (!isfinite(*velocityY)) ||
            (!isfinite(speedSquared)) ||
            (speedSquared > cfg->egoMaxSpeedMps * cfg->egoMaxSpeedMps))
        {
            *confidence = 0.0f;
            return 0U;
        }

        /* The RANSAC model selected the refit set. Re-score the final LS
         * solution itself; confidence must never describe the pre-refit model. */
        residualSum = 0.0f;
        inliers = 0U;
        for (index = 0U; index < pointCount; index++)
        {
            float range = sqrtf(
                gNavScratch.cluster.x[index] * gNavScratch.cluster.x[index] +
                gNavScratch.cluster.y[index] * gNavScratch.cluster.y[index] +
                gNavScratch.cluster.z[index] * gNavScratch.cluster.z[index]);
            if (range > 0.001f)
            {
                float measured = (float)points[index].dopplerIdx *
                                 staticCfg->dopplerStepMps;
                float residual = fabsf(
                    measured +
                    (gNavScratch.cluster.x[index] / range) * *velocityX +
                    (gNavScratch.cluster.y[index] / range) * *velocityY);
                if (residual <= threshold)
                {
                    inliers++;
                    residualSum += residual;
                }
            }
            if (((index & 63U) == 63U) &&
                (MmwDemo_NavPointGen_deadlineExpired(
                    deadlineStartCycles, deadlineOffsetCycles) != 0U))
            {
                *timeLimited = 1U;
                *confidence = 0.0f;
                return 0U;
            }
        }
        if (inliers < cfg->sceneMinInliers)
        {
            *confidence = 0.0f;
            return 0U;
        }
        *confidence = ((float)inliers / (float)pointCount) *
            MmwDemo_NavPointGen_clampf(
                1.0f - (residualSum / ((float)inliers * threshold)),
                0.0f, 1.0f);
    }
    return (*confidence >= cfg->egoConfidenceThreshold) ? 1U : 0U;
}

static uint8_t MmwDemo_NavPointGen_compensateDoppler(
    const MmwDemo_NavPoint *points,
    uint16_t pointCount,
    const MmwDemo_NavPointGen_Cfg *cfg,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
    uint16_t *flags,
    float *appliedVelocityX,
    float *appliedVelocityY,
    float *appliedVelocityZ,
    float *appliedConfidence,
    uint32_t deadlineStartCycles,
    uint32_t deadlineOffsetCycles,
    uint8_t *timeLimited)
{
    float velocityX = 0.0f;
    float velocityY = 0.0f;
    float velocityZ = 0.0f;
    float confidence = 0.0f;
    uint8_t valid = 0U;
    uint16_t index;

    if (cfg->egoMode == MMWDEMO_NAV_EGO_MODE_EXTERNAL)
    {
        uint16_t subFrame = staticCfg->sourceStaticCfg.subFrameNum;
        uint32_t ageFrames;
        float speedSquared;
        *flags |= MMWDEMO_NAV_FLAG_EGO_EXTERNAL |
                  MMWDEMO_NAV_FLAG_TRANSLATION_ONLY;
        if ((gNavExternalEgoHasFrame[subFrame] == 0U) ||
            (cfg->egoSequence != gNavLastEgoSequence[subFrame]))
        {
            gNavLastEgoSequence[subFrame] = cfg->egoSequence;
            gNavExternalEgoUpdateFrame[subFrame] =
                staticCfg->sourceStaticCfg.frameNumber;
            gNavExternalEgoHasFrame[subFrame] = 1U;
        }
        ageFrames = staticCfg->sourceStaticCfg.frameNumber -
                    gNavExternalEgoUpdateFrame[subFrame];
        velocityX = cfg->egoVelocityXMps;
        velocityY = cfg->egoVelocityYMps;
        velocityZ = cfg->egoVelocityZMps;
        confidence = cfg->egoConfidence;
        speedSquared = velocityX * velocityX + velocityY * velocityY +
                       velocityZ * velocityZ;
        valid = ((confidence >= cfg->egoConfidenceThreshold) &&
                 (speedSquared <= cfg->egoMaxSpeedMps * cfg->egoMaxSpeedMps) &&
                 ((cfg->egoVelocityValidFrames == 0U) ||
                  (ageFrames <=
                   cfg->egoVelocityValidFrames))) ? 1U : 0U;
    }
    else if (cfg->egoMode == MMWDEMO_NAV_EGO_MODE_SCENE)
    {
        *flags |= MMWDEMO_NAV_FLAG_EGO_SCENE |
                  MMWDEMO_NAV_FLAG_TRANSLATION_ONLY;
        valid = MmwDemo_NavPointGen_estimateSceneEgo(
            points, pointCount, cfg, staticCfg,
            &velocityX, &velocityY, &confidence,
            deadlineStartCycles, deadlineOffsetCycles, timeLimited);
        if (*timeLimited != 0U)
        {
            return 0U;
        }
    }

    if (valid != 0U)
    {
        *flags |= MMWDEMO_NAV_FLAG_EGO_VALID;
        gNavEgoValidCount++;
    }
    else
    {
        *flags |= MMWDEMO_NAV_FLAG_NO_EGO_COMPENSATION;
        if (cfg->egoMode != MMWDEMO_NAV_EGO_MODE_OFF)
        {
            *flags |= MMWDEMO_NAV_FLAG_EGO_LOW_CONFIDENCE;
            gNavEgoLowConfidenceCount++;
        }
    }

    for (index = 0U; index < pointCount; index++)
    {
        float measured = (float)points[index].dopplerIdx *
                         staticCfg->dopplerStepMps;
        if (valid != 0U)
        {
            float range = sqrtf(
                gNavScratch.cluster.x[index] * gNavScratch.cluster.x[index] +
                gNavScratch.cluster.y[index] * gNavScratch.cluster.y[index] +
                gNavScratch.cluster.z[index] * gNavScratch.cluster.z[index]);
            if (range > 0.001f)
            {
                measured +=
                    (gNavScratch.cluster.x[index] * velocityX +
                     gNavScratch.cluster.y[index] * velocityY +
                     gNavScratch.cluster.z[index] * velocityZ) / range;
            }
        }
        gNavScratch.cluster.compensatedDoppler[index] = measured;
        if (((index & 63U) == 63U) &&
            (MmwDemo_NavPointGen_deadlineExpired(
                deadlineStartCycles, deadlineOffsetCycles) != 0U))
        {
            *timeLimited = 1U;
            return 0U;
        }
    }
    *appliedVelocityX = (valid != 0U) ? velocityX : 0.0f;
    *appliedVelocityY = (valid != 0U) ? velocityY : 0.0f;
    *appliedVelocityZ = (valid != 0U) ? velocityZ : 0.0f;
    *appliedConfidence = confidence;
    return valid;
}

static uint16_t MmwDemo_NavPointGen_emitClusterPoints(
    const MmwDemo_NavPoint *surfacePoints,
    uint16_t pointCount,
    uint16_t maxOutput,
    MmwDemo_NavPoint *output,
    uint16_t *availableCount)
{
    uint16_t index;
    uint16_t count = 0U;
    uint16_t available = 0U;

    for (index = 0U; index < pointCount; index++)
    {
        uint8_t label = gNavScratch.cluster.label[index];
        if ((label == MMWDEMO_NAV_DBSCAN_UNVISITED) ||
            (label == MMWDEMO_NAV_DBSCAN_NOISE) ||
            (label == MMWDEMO_NAV_DBSCAN_WALL))
        {
            continue;
        }
        available++;
        if (count < maxOutput)
        {
            /* Keep the native raw Doppler for GTRACK. MSS derives the
             * compensated float from this value and the frame-aligned ego
             * vector, avoiding compensate/quantize/uncompensate loss. */
            output[count] = surfacePoints[index];
            count++;
        }
    }
    *availableCount = available;
    return count;
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
    MmwDemo_NavPoint *discreteOutput;
    uint32_t candidateSize = 0U;
    uint32_t processingStartCycles;
    uint32_t requiredCapacity;
    uint16_t surfaceAvailable = 0U;
    uint16_t discreteAvailable = 0U;
    uint16_t maxSurface;
    uint16_t maxDiscrete;
    uint16_t clusterPointCount = 0U;
    uint16_t clusterAvailable = 0U;
    uint16_t wallClusterCount = 0U;
    uint16_t flags;
    uint8_t clusterCount = 0U;
    uint8_t dbscanLimited = 0U;
    uint8_t egoValid = 0U;
    uint8_t timeLimited = 0U;
    float appliedEgoVelocityX = 0.0f;
    float appliedEgoVelocityY = 0.0f;
    float appliedEgoVelocityZ = 0.0f;
    float appliedEgoConfidence = 0.0f;
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
        (cfg->maxDiscretePoints == 0U) ||
        (cfg->maxSurfacePoints > MMWDEMO_NAV_MAX_SURFACE_POINTS) ||
        (cfg->uartMaxSurfacePoints > MMWDEMO_NAV_MAX_SURFACE_POINTS) ||
        (cfg->maxDiscretePoints > MMWDEMO_NAV_MAX_DISCRETE_POINTS) ||
        (cfg->surfaceRangeToleranceBins > 32U) ||
        (cfg->surfaceMinRunLength == 0U) ||
        (cfg->surfaceMinRunLength > 64U) ||
        (cfg->surfaceElevationNeighborBins > 8U) ||
        (cfg->dbscanEnabled > 1U) ||
        (cfg->dbscanMinPoints < 2U) ||
        (cfg->dbscanMinPoints > 32U) ||
        (cfg->dbscanMaxPoints == 0U) ||
        (cfg->dbscanMaxPoints > MMWDEMO_NAV_DBSCAN_MAX_POINTS) ||
        (cfg->dbscanMaxClusters == 0U) ||
        (cfg->dbscanMaxClusters > MMWDEMO_NAV_DBSCAN_MAX_CLUSTERS) ||
        (cfg->egoMode > MMWDEMO_NAV_EGO_MODE_SCENE) ||
        (cfg->sceneMinInliers < 2U) ||
        (cfg->sceneMinInliers > cfg->dbscanMaxPoints) ||
        (!isfinite(cfg->dbscanEpsilonM)) ||
        !(cfg->dbscanEpsilonM > 0.0f) ||
        (!isfinite(cfg->dbscanRangeScale)) ||
        !(cfg->dbscanRangeScale >= 0.0f) ||
        (!isfinite(cfg->wallMinLengthM)) ||
        !(cfg->wallMinLengthM > 0.0f) ||
        (!isfinite(cfg->wallMaxThicknessM)) ||
        !(cfg->wallMaxThicknessM > 0.0f) ||
        (!isfinite(cfg->wallMinAspectRatio)) ||
        !(cfg->wallMinAspectRatio >= 1.0f) ||
        (!isfinite(cfg->staticVelocityThresholdMps)) ||
        !(cfg->staticVelocityThresholdMps >= 0.0f) ||
        (!isfinite(cfg->egoVelocityXMps)) ||
        (!isfinite(cfg->egoVelocityYMps)) ||
        (!isfinite(cfg->egoVelocityZMps)) ||
        (!isfinite(cfg->egoConfidence)) ||
        !(cfg->egoConfidence >= 0.0f) ||
        !(cfg->egoConfidence <= 1.0f) ||
        (!isfinite(cfg->egoConfidenceThreshold)) ||
        !(cfg->egoConfidenceThreshold >= 0.0f) ||
        !(cfg->egoConfidenceThreshold <= 1.0f) ||
        (!isfinite(cfg->egoMaxSpeedMps)) ||
        !(cfg->egoMaxSpeedMps > 0.0f) ||
        (!isfinite(cfg->sceneResidualThresholdMps)) ||
        !(cfg->sceneResidualThresholdMps > 0.0f) ||
        (!isfinite(cfg->minMovingSpeedMps)) ||
        !(cfg->minMovingSpeedMps >= 0.0f) ||
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
    maxDiscrete = cfg->maxDiscretePoints;
    requiredCapacity = sizeof(MmwDemo_NavFrameHeader) +
        ((uint32_t)(((cfg->dbscanEnabled != 0U) &&
                     (cfg->dbscanMaxPoints > maxSurface)) ?
                    cfg->dbscanMaxPoints : maxSurface) +
         (uint32_t)maxDiscrete) *
        sizeof(MmwDemo_NavPoint);
    if (requiredCapacity > navBufferSize)
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
    if (cfg->dbscanEnabled != 0U)
    {
        MmwDemo_NavPoint *temporaryDiscrete =
            (MmwDemo_NavPoint *)(navBuffer + navBufferSize -
                (uint32_t)maxDiscrete * sizeof(MmwDemo_NavPoint));

        flags |= MMWDEMO_NAV_FLAG_DBSCAN_ACTIVE;
        /* Clustering input is deliberately independent of the TLV12 UART
         * cap. Otherwise MSS bandwidth feedback would silently change TLV13
         * target detection from frame to frame. */
        clusterPointCount = MmwDemo_NavPointGen_emitSurface(
            samples, candidateHeader->numSamples, cfg->dbscanMaxPoints,
            surfaceOutput, &clusterAvailable,
            deadlineStartCycles, deadlineOffsetCycles, &timeLimited);
        if (timeLimited != 0U)
        {
            return MMWDEMO_NAV_ETIME_BUDGET;
        }
        if (clusterAvailable > clusterPointCount)
        {
            dbscanLimited = 1U;
        }
        MmwDemo_NavPointGen_prepareCartesian(
            surfaceOutput, clusterPointCount, staticCfg);
        egoValid = MmwDemo_NavPointGen_compensateDoppler(
            surfaceOutput, clusterPointCount, cfg, staticCfg, &flags,
            &appliedEgoVelocityX, &appliedEgoVelocityY,
            &appliedEgoVelocityZ, &appliedEgoConfidence,
            deadlineStartCycles, deadlineOffsetCycles, &timeLimited);
        if (timeLimited != 0U)
        {
            return MMWDEMO_NAV_ETIME_BUDGET;
        }
        clusterCount = MmwDemo_NavPointGen_runDbscan(
            surfaceOutput, clusterPointCount, cfg, staticCfg,
            deadlineStartCycles, deadlineOffsetCycles,
            &dbscanLimited, &timeLimited);
        if (timeLimited != 0U)
        {
            return MMWDEMO_NAV_ETIME_BUDGET;
        }
        wallClusterCount = MmwDemo_NavPointGen_classifyWallClusters(
            clusterPointCount, clusterCount, cfg, egoValid,
            deadlineStartCycles, deadlineOffsetCycles, &timeLimited);
        if (timeLimited != 0U)
        {
            return MMWDEMO_NAV_ETIME_BUDGET;
        }
        navHeader->numDiscretePoints =
            MmwDemo_NavPointGen_emitClusterPoints(
                surfaceOutput, clusterPointCount, maxDiscrete,
                temporaryDiscrete, &discreteAvailable);
        gNavDbscanProcessCount++;
        gNavDbscanWallClusterCount += wallClusterCount;
        if (wallClusterCount != 0U)
        {
            /* Keep the legacy flag as a wire-compatible "wall rejected"
             * indication; the old six-bin heuristic is no longer used. */
            flags |= MMWDEMO_NAV_FLAG_EXTENDED_REJECTED;
        }
        if (dbscanLimited != 0U)
        {
            flags |= MMWDEMO_NAV_FLAG_DBSCAN_LIMITED;
            gNavDbscanLimitedCount++;
        }
        /* DBSCAN reused the nearest-map scratch. Rebuild it before producing
         * TLV12, then move the internal tracker points behind the actual
         * (possibly UART-limited) surface prefix. */
        retVal = MmwDemo_NavPointGen_buildNearestMap(
            samples, candidateHeader->numSamples,
            deadlineStartCycles, deadlineOffsetCycles);
        if (retVal != 0)
        {
            return retVal;
        }
        navHeader->numSurfacePoints = MmwDemo_NavPointGen_emitSurface(
            samples, candidateHeader->numSamples, maxSurface, surfaceOutput,
            &surfaceAvailable, deadlineStartCycles, deadlineOffsetCycles,
            &timeLimited);
        if (timeLimited != 0U)
        {
            return MMWDEMO_NAV_ETIME_BUDGET;
        }
        discreteOutput = surfaceOutput + navHeader->numSurfacePoints;
        memmove(discreteOutput, temporaryDiscrete,
                (uint32_t)navHeader->numDiscretePoints *
                sizeof(MmwDemo_NavPoint));
    }
    else
    {
        navHeader->numSurfacePoints = MmwDemo_NavPointGen_emitSurface(
            samples, candidateHeader->numSamples, maxSurface, surfaceOutput,
            &surfaceAvailable, deadlineStartCycles, deadlineOffsetCycles,
            &timeLimited);
        if (timeLimited != 0U)
        {
            return MMWDEMO_NAV_ETIME_BUDGET;
        }
        navHeader->numDiscretePoints = 0U;
        flags |= MMWDEMO_NAV_FLAG_NO_EGO_COMPENSATION;
    }

    if (surfaceAvailable > navHeader->numSurfacePoints)
    {
        flags |= MMWDEMO_NAV_FLAG_SURFACE_LIMITED;
    }
    if (discreteAvailable > navHeader->numDiscretePoints)
    {
        flags |= MMWDEMO_NAV_FLAG_DISCRETE_LIMITED;
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
    navHeader->egoVelocityXMps = appliedEgoVelocityX;
    navHeader->egoVelocityYMps = appliedEgoVelocityY;
    navHeader->egoVelocityZMps = appliedEgoVelocityZ;
    navHeader->egoConfidence = appliedEgoConfidence;
    navHeader->egoSequence =
        (cfg->egoMode == MMWDEMO_NAV_EGO_MODE_EXTERNAL) ?
        cfg->egoSequence : candidateHeader->radarFrameNumber;
    navHeader->processingCycles =
        Cycleprofiler_getTimeStamp() - processingStartCycles;
    if ((cycleBudget != 0U) &&
        (navHeader->processingCycles > cycleBudget))
    {
        flags |= MMWDEMO_NAV_FLAG_TIME_BUDGET_EXCEEDED;
    }
    navHeader->flags = flags;

    if (MmwDemo_NavPointGen_deadlineExpired(
            deadlineStartCycles, deadlineOffsetCycles) != 0U)
    {
        return MMWDEMO_NAV_ETIME_BUDGET;
    }

    *navSize = sizeof(*navHeader) +
        ((uint32_t)navHeader->numSurfacePoints +
         (uint32_t)navHeader->numDiscretePoints) *
        sizeof(MmwDemo_NavPoint);
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

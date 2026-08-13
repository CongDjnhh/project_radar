/**
 * @file nav_tracker.c
 *
 * @brief Converts DSS discrete navigation points to TI GTRACK 3-D input.
 *
 * Extended surfaces are removed by nav_point_gen on DSS. This module feeds
 * only the remaining moving/discrete points to GTRACK. Static nearest-surface
 * points are never offered to the tracker.
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#include <ti/drivers/osal/MemoryP.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/alg/gtrack/gtrack.h>

#include "nav_tracker.h"

#define MMWDEMO_NAV_TRACKER_MAX_POINTS MMWDEMO_NAV_MAX_DISCRETE_POINTS
#define MMWDEMO_NAV_TRACKER_PI         (3.14159265358979323846f)

static void *gNavTrackerHandle;
static uint32_t gNavTrackerLastFrame;
static uint8_t gNavTrackerHasLastFrame;
static uint16_t gNavTrackerSubFrame = 0xFFFFU;

static GTRACK_measurementPoint
    gNavTrackerMeasurements[MMWDEMO_NAV_TRACKER_MAX_POINTS];
static GTRACK_targetDesc
    gNavTrackerDescriptions[MMWDEMO_NAV_MAX_TARGETS];
static uint8_t gNavTrackerAssociations[MMWDEMO_NAV_TRACKER_MAX_POINTS];

static GTRACK_gatingParams gNavTrackerGating;
static GTRACK_allocationParams gNavTrackerAllocation;
static GTRACK_stateParams gNavTrackerState;
static GTRACK_advancedParameters gNavTrackerAdvanced;

volatile uint32_t gNavTrackerHeapBytes = 0U;
volatile int32_t gNavTrackerLastError = 0;
volatile uint32_t gNavTrackerProcessCount = 0U;
volatile uint32_t gNavTrackerResetCount = 0U;

void *gtrack_alloc(uint32_t numElements, uint32_t sizeInBytes)
{
    uint32_t bytes = numElements * sizeInBytes;
    void *memory = MemoryP_ctrlAlloc(bytes, 8U);
    if (memory != NULL)
    {
        gNavTrackerHeapBytes += bytes;
    }
    return memory;
}

void gtrack_free(void *pFree, uint32_t sizeInBytes)
{
    if (pFree != NULL)
    {
        MemoryP_ctrlFree(pFree, sizeInBytes);
        if (gNavTrackerHeapBytes >= sizeInBytes)
        {
            gNavTrackerHeapBytes -= sizeInBytes;
        }
        else
        {
            gNavTrackerHeapBytes = 0U;
        }
    }
}

void gtrack_log(GTRACK_VERBOSE_TYPE level, const char *format, ...)
{
    /* Tracking is configured with GTRACK_VERBOSE_NONE. Keep the required
     * dependency deterministic and independent of the CLI/UART task. */
    (void)level;
    (void)format;
}

static float MmwDemo_NavTracker_clamp(float value, float low, float high)
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

static void MmwDemo_NavTracker_delete(void)
{
    if (gNavTrackerHandle != NULL)
    {
        gtrack_delete(gNavTrackerHandle);
        gNavTrackerHandle = NULL;
    }
    gNavTrackerSubFrame = 0xFFFFU;
}

static int32_t MmwDemo_NavTracker_create(
    const MmwDemo_NavFrameHeader *navHeader)
{
    GTRACK_moduleConfig moduleCfg;
    int32_t errCode = 0;

    if (!(navHeader->framePeriodSec > 0.0f) ||
        !(navHeader->dopplerStepMps > 0.0f))
    {
        return MMWDEMO_NAV_TRACKER_EINVAL;
    }

    memset(&gNavTrackerGating, 0, sizeof(gNavTrackerGating));
    gNavTrackerGating.gain = 3.0f;
    gNavTrackerGating.limits.depth = 2.0f;
    gNavTrackerGating.limits.width = 2.0f;
    gNavTrackerGating.limits.height = 2.0f;
    gNavTrackerGating.limits.vel = 4.0f;

    memset(&gNavTrackerAllocation, 0, sizeof(gNavTrackerAllocation));
    gNavTrackerAllocation.snrThre = 60.0f;
    gNavTrackerAllocation.snrThreObscured = 200.0f;
    gNavTrackerAllocation.velocityThre = 0.10f;
    gNavTrackerAllocation.pointsThre = 3U;
    gNavTrackerAllocation.maxDistanceThre = 1.5f;
    gNavTrackerAllocation.maxVelThre = 2.0f;

    memset(&gNavTrackerState, 0, sizeof(gNavTrackerState));
    gNavTrackerState.det2actThre = 3U;
    gNavTrackerState.det2freeThre = 5U;
    gNavTrackerState.active2freeThre = 20U;
    gNavTrackerState.static2freeThre = 20U;
    gNavTrackerState.exit2freeThre = 5U;
    gNavTrackerState.sleep2freeThre = 20U;

    memset(&gNavTrackerAdvanced, 0, sizeof(gNavTrackerAdvanced));
    gNavTrackerAdvanced.gatingParams = &gNavTrackerGating;
    gNavTrackerAdvanced.allocationParams = &gNavTrackerAllocation;
    gNavTrackerAdvanced.stateParams = &gNavTrackerState;

    memset(&moduleCfg, 0, sizeof(moduleCfg));
    moduleCfg.stateVectorType = GTRACK_STATE_VECTORS_3DA;
    moduleCfg.verbose = GTRACK_VERBOSE_NONE;
    moduleCfg.maxNumPoints = MMWDEMO_NAV_TRACKER_MAX_POINTS;
    moduleCfg.maxNumTracks = MMWDEMO_NAV_MAX_TARGETS;
    moduleCfg.initialRadialVelocity = 0.0f;
    moduleCfg.maxRadialVelocity =
        0.5f * (float)navHeader->numDopplerBins *
        navHeader->dopplerStepMps;
    moduleCfg.radialVelocityResolution = navHeader->dopplerStepMps;
    moduleCfg.maxAcceleration[0] = 4.0f;
    moduleCfg.maxAcceleration[1] = 4.0f;
    moduleCfg.maxAcceleration[2] = 4.0f;
    /* Despite the old SDK comment saying ms, TI's reference app and library
     * use seconds here (for example 0.05 for a 50-ms frame). */
    moduleCfg.deltaT = navHeader->framePeriodSec;
    moduleCfg.advParams = &gNavTrackerAdvanced;

    MmwDemo_NavTracker_delete();
    gNavTrackerHandle = gtrack_create(&moduleCfg, &errCode);
    if ((gNavTrackerHandle == NULL) || (errCode != 0))
    {
        MmwDemo_NavTracker_delete();
        return MMWDEMO_NAV_TRACKER_ECREATE;
    }
    gNavTrackerSubFrame = navHeader->subFrameNumber;
    return 0;
}

static uint8_t MmwDemo_NavTracker_payloadValid(
    const MmwDemo_NavFrameHeader *header,
    uint32_t payloadSize)
{
    uint32_t expectedSize;

    if ((header == NULL) ||
        (payloadSize < sizeof(MmwDemo_NavFrameHeader)) ||
        (header->magic != MMWDEMO_NAV_MAGIC) ||
        (header->version != MMWDEMO_NAV_PROTOCOL_VERSION) ||
        (header->headerSize != sizeof(MmwDemo_NavFrameHeader)) ||
        (header->pointSize != sizeof(MmwDemo_NavPoint)) ||
        (header->numAzimuthBins != MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS) ||
        (header->numElevationBins != MMWDEMO_DENSE_ELEV_NUM_ELEVATION_BINS) ||
        (header->numSurfacePoints > MMWDEMO_NAV_MAX_SURFACE_POINTS) ||
        (header->numDiscretePoints > MMWDEMO_NAV_MAX_DISCRETE_POINTS) ||
        (!isfinite(header->rangeStepMeters)) ||
        (!isfinite(header->dopplerStepMps)) ||
        (!isfinite(header->framePeriodSec)) ||
        (!isfinite(header->rangeBiasMeters)) ||
        !(header->rangeStepMeters > 0.0f) ||
        !(header->dopplerStepMps > 0.0f) ||
        (header->framePeriodSec < 0.0f))
    {
        return 0U;
    }
    expectedSize = (uint32_t)header->headerSize +
        ((uint32_t)header->numSurfacePoints +
         (uint32_t)header->numDiscretePoints) *
        (uint32_t)header->pointSize;
    return (expectedSize == payloadSize) ? 1U : 0U;
}

static uint16_t MmwDemo_NavTracker_makeMeasurements(
    const MmwDemo_NavFrameHeader *header,
    const MmwDemo_NavPoint *points)
{
    const MmwDemo_NavPoint *discrete = points + header->numSurfacePoints;
    uint16_t inputCount = header->numDiscretePoints;
    uint16_t index;

    if (inputCount > MMWDEMO_NAV_TRACKER_MAX_POINTS)
    {
        inputCount = MMWDEMO_NAV_TRACKER_MAX_POINTS;
    }
    for (index = 0U; index < inputCount; index++)
    {
        float u = 2.0f * (float)discrete[index].azimuthIdx /
                  (float)header->numAzimuthBins;
        float v = (float)discrete[index].sourceVQ15 / 32767.0f;
        float horizontalSquared;
        float x;
        float y;
        float z;
        float range = (float)discrete[index].rangeIdx *
                      header->rangeStepMeters - header->rangeBiasMeters;

        if (range < 0.0f)
        {
            range = 0.0f;
        }

        u = MmwDemo_NavTracker_clamp(u, -1.0f, 1.0f);
        v = MmwDemo_NavTracker_clamp(v, -1.0f, 1.0f);
        horizontalSquared = 1.0f - u * u - v * v;
        if (horizontalSquared < 0.0f)
        {
            horizontalSquared = 0.0f;
        }
        x = range * u;
        z = range * v;
        y = range * sqrtf(horizontalSquared);

        gNavTrackerMeasurements[index].vector.range = range;
        gNavTrackerMeasurements[index].vector.azimuth = atan2f(x, y);
        gNavTrackerMeasurements[index].vector.elev =
            atan2f(z, sqrtf(x * x + y * y));
        gNavTrackerMeasurements[index].vector.doppler =
            (float)discrete[index].dopplerIdx * header->dopplerStepMps;
        /* snrDb10 stores 10*dB. GTRACK expects linear SNR. */
        gNavTrackerMeasurements[index].snr =
            powf(10.0f, (float)discrete[index].snrDb10 * 0.01f);
        gNavTrackerAssociations[index] = GTRACK_ID_POINT_NOT_ASSOCIATED;
    }
    return inputCount;
}

int32_t MmwDemo_NavTracker_process(
    const void *navPayload,
    uint32_t navPayloadSize,
    void *targetPayload,
    uint32_t targetPayloadCapacity,
    uint32_t *targetPayloadSize)
{
    const MmwDemo_NavFrameHeader *navHeader =
        (const MmwDemo_NavFrameHeader *)navPayload;
    const MmwDemo_NavPoint *navPoints;
    MmwDemo_NavTargetHeader *targetHeader;
    MmwDemo_NavTarget *target;
    uint32_t startCycles;
    uint16_t inputCount;
    uint16_t targetCount = 0U;
    uint16_t index;
    uint8_t presence = 0U;
    uint16_t flags = MMWDEMO_NAV_TARGET_FLAG_NO_EGO_COMP;
    int32_t retVal = 0;

    if (targetPayloadSize != NULL)
    {
        *targetPayloadSize = 0U;
    }
    if ((navPayload == NULL) || (targetPayload == NULL) ||
        (targetPayloadSize == NULL) ||
        (targetPayloadCapacity < sizeof(MmwDemo_NavTargetHeader) +
         MMWDEMO_NAV_MAX_TARGETS * sizeof(MmwDemo_NavTarget)))
    {
        gNavTrackerLastError = MMWDEMO_NAV_TRACKER_EINVAL;
        return MMWDEMO_NAV_TRACKER_EINVAL;
    }
    if (MmwDemo_NavTracker_payloadValid(navHeader, navPayloadSize) == 0U)
    {
        gNavTrackerLastError = MMWDEMO_NAV_TRACKER_EPAYLOAD;
        return MMWDEMO_NAV_TRACKER_EPAYLOAD;
    }

    startCycles = Cycleprofiler_getTimeStamp();
    memset(targetPayload, 0, sizeof(MmwDemo_NavTargetHeader));
    targetHeader = (MmwDemo_NavTargetHeader *)targetPayload;
    target = (MmwDemo_NavTarget *)((uint8_t *)targetPayload +
                                   sizeof(*targetHeader));

    /* A single GTRACK instance is deliberately used to stay within the MSS
     * control heap. Never mix measurements from two subframe geometries: a
     * dynamic target-subframe switch starts a fresh tracker explicitly. */
    if ((gNavTrackerHandle != NULL) &&
        (gNavTrackerSubFrame != navHeader->subFrameNumber))
    {
        MmwDemo_NavTracker_delete();
        gNavTrackerHasLastFrame = 0U;
        gNavTrackerSubFrame = 0xFFFFU;
        gNavTrackerResetCount++;
        flags |= MMWDEMO_NAV_TARGET_FLAG_RESET;
    }

    /* Frame zero identifies a fresh sensor run. A non-monotonic frame also
     * invalidates velocity state; do not let stale tracks survive a restart. */
    if ((navHeader->radarFrameNumber == 0U) ||
        ((gNavTrackerHasLastFrame != 0U) &&
         (navHeader->radarFrameNumber < gNavTrackerLastFrame)))
    {
        MmwDemo_NavTracker_delete();
        gNavTrackerHasLastFrame = 0U;
        gNavTrackerResetCount++;
        flags |= MMWDEMO_NAV_TARGET_FLAG_RESET;
    }

    if ((gNavTrackerHandle == NULL) &&
        (navHeader->protocolStatus == 0U) &&
        (navHeader->framePeriodSec > 0.0f))
    {
        retVal = MmwDemo_NavTracker_create(navHeader);
    }
    if (gNavTrackerHandle != NULL)
    {
        flags |= MMWDEMO_NAV_TARGET_FLAG_INITIALIZED;
    }

    if ((retVal == 0) && (gNavTrackerHandle != NULL) &&
        (navHeader->protocolStatus == 0U))
    {
        navPoints = (const MmwDemo_NavPoint *)
            ((const uint8_t *)navPayload + navHeader->headerSize);
        inputCount = MmwDemo_NavTracker_makeMeasurements(navHeader,
                                                          navPoints);
        if (navHeader->numDiscretePoints > inputCount)
        {
            flags |= MMWDEMO_NAV_TARGET_FLAG_INPUT_LIMITED;
        }
        gtrack_step(gNavTrackerHandle,
                    gNavTrackerMeasurements,
                    NULL,
                    inputCount,
                    gNavTrackerDescriptions,
                    &targetCount,
                    gNavTrackerAssociations,
                    NULL,
                    &presence,
                    NULL);
        if (targetCount > MMWDEMO_NAV_MAX_TARGETS)
        {
            targetCount = MMWDEMO_NAV_MAX_TARGETS;
            flags |= MMWDEMO_NAV_TARGET_FLAG_INPUT_LIMITED;
        }
        flags |= MMWDEMO_NAV_TARGET_FLAG_TRACKED;

        for (index = 0U; index < targetCount; index++)
        {
            uint16_t pointIndex;
            uint16_t associated = 0U;
            target[index].trackId = gNavTrackerDescriptions[index].tid;
            target[index].posX = gNavTrackerDescriptions[index].S[0];
            target[index].posY = gNavTrackerDescriptions[index].S[1];
            target[index].posZ = gNavTrackerDescriptions[index].S[2];
            target[index].velX = gNavTrackerDescriptions[index].S[3];
            target[index].velY = gNavTrackerDescriptions[index].S[4];
            target[index].velZ = gNavTrackerDescriptions[index].S[5];
            target[index].accX = gNavTrackerDescriptions[index].S[6];
            target[index].accY = gNavTrackerDescriptions[index].S[7];
            target[index].accZ = gNavTrackerDescriptions[index].S[8];
            /* GTRACK dimensions are depth, width, height, doppler. */
            target[index].dimX = gNavTrackerDescriptions[index].dim[1];
            target[index].dimY = gNavTrackerDescriptions[index].dim[0];
            target[index].dimZ = gNavTrackerDescriptions[index].dim[2];
            target[index].confidence =
                gNavTrackerDescriptions[index].confidenceLevel;
            for (pointIndex = 0U; pointIndex < inputCount; pointIndex++)
            {
                if (gNavTrackerAssociations[pointIndex] ==
                    gNavTrackerDescriptions[index].uid)
                {
                    associated++;
                }
            }
            target[index].numAssociatedPoints = associated;
            target[index].flags = (presence != 0U) ? 1U : 0U;
        }
        gNavTrackerProcessCount++;
        gNavTrackerLastFrame = navHeader->radarFrameNumber;
        gNavTrackerHasLastFrame = 1U;
    }

    targetHeader->magic = MMWDEMO_NAV_TARGET_MAGIC;
    targetHeader->version = MMWDEMO_NAV_TARGET_PROTOCOL_VERSION;
    targetHeader->headerSize = sizeof(*targetHeader);
    targetHeader->targetSize = sizeof(MmwDemo_NavTarget);
    targetHeader->flags = flags;
    targetHeader->numTargets = targetCount;
    targetHeader->subFrameNumber = navHeader->subFrameNumber;
    targetHeader->protocolStatus = (retVal < 0) ?
        (uint16_t)(-retVal) : navHeader->protocolStatus;
    targetHeader->radarFrameNumber = navHeader->radarFrameNumber;
    targetHeader->processingCycles =
        Cycleprofiler_getTimeStamp() - startCycles;
    targetHeader->trackerHeapBytes = gNavTrackerHeapBytes;
    *targetPayloadSize = sizeof(*targetHeader) +
        (uint32_t)targetCount * sizeof(MmwDemo_NavTarget);
    gNavTrackerLastError = retVal;
    return retVal;
}

void MmwDemo_NavTracker_reset(void)
{
    MmwDemo_NavTracker_delete();
    gNavTrackerLastFrame = 0U;
    gNavTrackerHasLastFrame = 0U;
    gNavTrackerSubFrame = 0xFFFFU;
    memset(gNavTrackerMeasurements, 0, sizeof(gNavTrackerMeasurements));
    memset(gNavTrackerDescriptions, 0, sizeof(gNavTrackerDescriptions));
    memset(gNavTrackerAssociations, GTRACK_ID_POINT_NOT_ASSOCIATED,
           sizeof(gNavTrackerAssociations));
    gNavTrackerLastError = 0;
    gNavTrackerResetCount++;
}

void MmwDemo_NavTracker_resetSubFrame(uint16_t subFrameNumber)
{
    if ((gNavTrackerHandle != NULL) &&
        (gNavTrackerSubFrame == subFrameNumber))
    {
        MmwDemo_NavTracker_reset();
    }
}

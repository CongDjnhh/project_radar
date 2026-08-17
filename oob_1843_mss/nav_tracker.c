/**
 * @file nav_tracker.c
 *
 * @brief Converts DSS discrete navigation points to TI GTRACK 3-D input.
 *
 * Wall-like extended surfaces and DBSCAN noise are removed by nav_point_gen
 * on DSS. Compact clusters, including stationary obstacles, are offered to
 * GTRACK; compensated Doppler supplies the static/moving/uncertain state.
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/alg/gtrack/gtrack.h>

#include "nav_tracker.h"

#define MMWDEMO_NAV_TRACKER_MAX_POINTS MMWDEMO_NAV_MAX_DISCRETE_POINTS
#define MMWDEMO_NAV_TRACKER_PI         (3.14159265358979323846f)
#define MMWDEMO_NAV_TRACKER_ARENA_BYTES (64U * 1024U)

static void *gNavTrackerHandle;
static uint32_t gNavTrackerLastFrame;
static uint8_t gNavTrackerHasLastFrame;
static uint16_t gNavTrackerSubFrame = 0xFFFFU;
static float gNavTrackerMotionThresholdMps = 0.20f;

static GTRACK_measurementPoint
    gNavTrackerMeasurements[MMWDEMO_NAV_TRACKER_MAX_POINTS];
/* GTRACK sees native radar-relative Doppler because its state is
 * radar-relative. Keep a separate frame-aligned compensated float for target
 * motion classification; never quantize that derived value into an FFT bin. */
static float
    gNavTrackerCompensatedDoppler[MMWDEMO_NAV_TRACKER_MAX_POINTS];
static GTRACK_targetDesc
    gNavTrackerDescriptions[MMWDEMO_NAV_MAX_TARGETS];
static uint8_t gNavTrackerAssociations[MMWDEMO_NAV_TRACKER_MAX_POINTS];

static GTRACK_gatingParams gNavTrackerGating;
static GTRACK_allocationParams gNavTrackerAllocation;
static GTRACK_stateParams gNavTrackerState;
static GTRACK_advancedParameters gNavTrackerAdvanced;

/* GTRACK obtains all storage through gtrack_alloc. A fixed arena removes
 * general-purpose heap latency and fragmentation from frame-gap reset/recreate
 * paths. Allocation is monotonic; GTRACK delete frees every block, then the
 * arena is reclaimed atomically when the live block count reaches zero. */
#pragma DATA_ALIGN(gNavTrackerArena, 8)
static uint8_t gNavTrackerArena[MMWDEMO_NAV_TRACKER_ARENA_BYTES];
static uint32_t gNavTrackerArenaOffset;
static uint32_t gNavTrackerArenaLiveBlocks;

volatile uint32_t gNavTrackerHeapBytes = 0U;
volatile uint32_t gNavTrackerArenaHighWaterBytes = 0U;
volatile int32_t gNavTrackerLastError = 0;
volatile uint32_t gNavTrackerProcessCount = 0U;
volatile uint32_t gNavTrackerResetCount = 0U;
volatile uint32_t gNavTrackerMaxProcessCycles = 0U;

void *gtrack_alloc(uint32_t numElements, uint32_t sizeInBytes)
{
    uint32_t bytes;
    uint32_t alignedOffset;
    void *memory;
    if ((sizeInBytes != 0U) &&
        (numElements > (0xFFFFFFFFU / sizeInBytes)))
    {
        return NULL;
    }
    bytes = numElements * sizeInBytes;
    if (bytes == 0U)
    {
        return NULL;
    }
    alignedOffset = (gNavTrackerArenaOffset + 7U) & ~7U;
    if ((alignedOffset > MMWDEMO_NAV_TRACKER_ARENA_BYTES) ||
        (bytes > (MMWDEMO_NAV_TRACKER_ARENA_BYTES - alignedOffset)))
    {
        return NULL;
    }
    memory = &gNavTrackerArena[alignedOffset];
    gNavTrackerArenaOffset = alignedOffset + bytes;
    gNavTrackerArenaLiveBlocks++;
    gNavTrackerHeapBytes += bytes;
    if (gNavTrackerArenaOffset > gNavTrackerArenaHighWaterBytes)
    {
        gNavTrackerArenaHighWaterBytes = gNavTrackerArenaOffset;
    }
    return memory;
}

void gtrack_free(void *pFree, uint32_t sizeInBytes)
{
    if ((pFree != NULL) &&
        ((uint8_t *)pFree >= &gNavTrackerArena[0]) &&
        ((uint8_t *)pFree <
         &gNavTrackerArena[MMWDEMO_NAV_TRACKER_ARENA_BYTES]) &&
        (gNavTrackerArenaLiveBlocks != 0U))
    {
        if (gNavTrackerHeapBytes >= sizeInBytes)
        {
            gNavTrackerHeapBytes -= sizeInBytes;
        }
        else
        {
            gNavTrackerHeapBytes = 0U;
        }
        gNavTrackerArenaLiveBlocks--;
        if (gNavTrackerArenaLiveBlocks == 0U)
        {
            gNavTrackerArenaOffset = 0U;
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

void MmwDemo_NavTracker_setMotionThreshold(float thresholdMps)
{
    if (isfinite(thresholdMps) &&
        (thresholdMps >= 0.0f) && (thresholdMps <= 20.0f))
    {
        gNavTrackerMotionThresholdMps = thresholdMps;
    }
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
    /* DBSCAN has already rejected isolated noise. Do not prevent compact
     * stationary obstacles (chair, pole, box) from allocating a track. */
    gNavTrackerAllocation.velocityThre = 0.0f;
    gNavTrackerAllocation.pointsThre = 3U;
    gNavTrackerAllocation.maxDistanceThre = 1.5f;
    gNavTrackerAllocation.maxVelThre = 2.0f;

    memset(&gNavTrackerState, 0, sizeof(gNavTrackerState));
    gNavTrackerState.det2actThre = 3U;
    gNavTrackerState.det2freeThre = 5U;
    gNavTrackerState.active2freeThre = 30U;
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
        (!isfinite(header->egoVelocityXMps)) ||
        (!isfinite(header->egoVelocityYMps)) ||
        (!isfinite(header->egoVelocityZMps)) ||
        (!isfinite(header->egoConfidence)) ||
        (header->egoConfidence < 0.0f) ||
        (header->egoConfidence > 1.0f) ||
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
        gNavTrackerCompensatedDoppler[index] =
            gNavTrackerMeasurements[index].vector.doppler;
        if (((header->flags & MMWDEMO_NAV_FLAG_EGO_VALID) != 0U) &&
            (range > 0.001f))
        {
            /* GTRACK retains native radar-relative Doppler. Motion-state
             * classification uses the separate frame-aligned compensated
             * float, without quantizing it back into an FFT bin. */
            gNavTrackerCompensatedDoppler[index] +=
                (x * header->egoVelocityXMps +
                 y * header->egoVelocityYMps +
                 z * header->egoVelocityZMps) / range;
        }
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
    uint16_t flags = 0U;
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

    if ((navHeader->flags & MMWDEMO_NAV_FLAG_NO_EGO_COMPENSATION) != 0U)
    {
        flags |= MMWDEMO_NAV_TARGET_FLAG_NO_EGO_COMP;
    }
    if ((navHeader->flags & MMWDEMO_NAV_FLAG_DBSCAN_ACTIVE) != 0U)
    {
        flags |= MMWDEMO_NAV_TARGET_FLAG_DBSCAN_INPUT;
    }
    if ((navHeader->flags & MMWDEMO_NAV_FLAG_EGO_EXTERNAL) != 0U)
    {
        flags |= MMWDEMO_NAV_TARGET_FLAG_EGO_EXTERNAL;
    }
    if ((navHeader->flags & MMWDEMO_NAV_FLAG_EGO_SCENE) != 0U)
    {
        flags |= MMWDEMO_NAV_TARGET_FLAG_EGO_SCENE;
    }
    if ((navHeader->flags & MMWDEMO_NAV_FLAG_EGO_VALID) != 0U)
    {
        flags |= MMWDEMO_NAV_TARGET_FLAG_EGO_VALID;
    }
    if ((navHeader->flags & MMWDEMO_NAV_FLAG_EGO_LOW_CONFIDENCE) != 0U)
    {
        flags |= MMWDEMO_NAV_TARGET_FLAG_EGO_LOW_CONF;
    }
    if ((navHeader->flags & MMWDEMO_NAV_FLAG_TRANSLATION_ONLY) != 0U)
    {
        flags |= MMWDEMO_NAV_TARGET_FLAG_TRANSLATION_ONLY;
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

    /* Frame zero identifies a fresh sensor run. Duplicate, non-monotonic, or
     * skipped navigation frames invalidate a fixed-deltaT tracker. Reset
     * deterministically instead of silently integrating with the wrong dt. */
    if ((navHeader->radarFrameNumber == 0U) ||
        ((gNavTrackerHasLastFrame != 0U) &&
         (navHeader->radarFrameNumber != (gNavTrackerLastFrame + 1U))))
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
            float associatedAbsDoppler = 0.0f;
            uint16_t objectFlags = (presence != 0U) ?
                MMWDEMO_NAV_TARGET_OBJECT_FLAG_PRESENCE : 0U;
            target[index].trackId = gNavTrackerDescriptions[index].tid;
            target[index].posX = gNavTrackerDescriptions[index].S[0];
            target[index].posY = gNavTrackerDescriptions[index].S[1];
            target[index].posZ = gNavTrackerDescriptions[index].S[2];
            target[index].velX = gNavTrackerDescriptions[index].S[3];
            target[index].velY = gNavTrackerDescriptions[index].S[4];
            target[index].velZ = gNavTrackerDescriptions[index].S[5];
            if ((navHeader->flags & MMWDEMO_NAV_FLAG_EGO_VALID) != 0U)
            {
                /* Report target velocity after translation compensation while
                 * retaining target position in the current radar frame. */
                target[index].velX += navHeader->egoVelocityXMps;
                target[index].velY += navHeader->egoVelocityYMps;
                target[index].velZ += navHeader->egoVelocityZMps;
            }
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
                    associatedAbsDoppler += fabsf(
                        gNavTrackerCompensatedDoppler[pointIndex]);
                }
            }
            target[index].numAssociatedPoints = associated;
            if (((navHeader->flags & MMWDEMO_NAV_FLAG_EGO_VALID) == 0U) ||
                (associated == 0U))
            {
                objectFlags |= MMWDEMO_NAV_TARGET_OBJECT_FLAG_UNCERTAIN;
            }
            else if ((associatedAbsDoppler / (float)associated) <=
                     gNavTrackerMotionThresholdMps)
            {
                float targetSpeed = sqrtf(
                    target[index].velX * target[index].velX +
                    target[index].velY * target[index].velY +
                    target[index].velZ * target[index].velZ);
                if (targetSpeed <= gNavTrackerMotionThresholdMps)
                {
                    objectFlags |= MMWDEMO_NAV_TARGET_OBJECT_FLAG_STATIC;
                }
                else
                {
                    objectFlags |= MMWDEMO_NAV_TARGET_OBJECT_FLAG_MOVING;
                }
            }
            else
            {
                objectFlags |= MMWDEMO_NAV_TARGET_OBJECT_FLAG_MOVING;
            }
            target[index].flags = objectFlags;
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
    if (targetHeader->processingCycles > gNavTrackerMaxProcessCycles)
    {
        gNavTrackerMaxProcessCycles = targetHeader->processingCycles;
    }
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
    memset(gNavTrackerCompensatedDoppler, 0,
           sizeof(gNavTrackerCompensatedDoppler));
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

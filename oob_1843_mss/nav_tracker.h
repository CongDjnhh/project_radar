/**
 * @file nav_tracker.h
 *
 * @brief MSS-side 3-D GTRACK adapter and TLV13 wire ABI.
 */

#ifndef MMWDEMO_NAV_TRACKER_H
#define MMWDEMO_NAV_TRACKER_H

#include <stdint.h>
#include "nav_point_gen.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MMWDEMO_NAV_TARGET_MAGIC              (0x4E415654U) /* NAVT */
#define MMWDEMO_NAV_TARGET_PROTOCOL_VERSION   (1U)
#define MMWDEMO_NAV_MAX_TARGETS               (16U)

#define MMWDEMO_NAV_TARGET_FLAG_INITIALIZED   (1U << 0)
#define MMWDEMO_NAV_TARGET_FLAG_TRACKED       (1U << 1)
#define MMWDEMO_NAV_TARGET_FLAG_INPUT_LIMITED (1U << 2)
#define MMWDEMO_NAV_TARGET_FLAG_NO_EGO_COMP   (1U << 3)
#define MMWDEMO_NAV_TARGET_FLAG_RESET          (1U << 4)
#define MMWDEMO_NAV_TARGET_FLAG_EGO_EXTERNAL   (1U << 5)
#define MMWDEMO_NAV_TARGET_FLAG_EGO_SCENE      (1U << 6)
#define MMWDEMO_NAV_TARGET_FLAG_EGO_VALID      (1U << 7)
#define MMWDEMO_NAV_TARGET_FLAG_DBSCAN_INPUT   (1U << 8)
#define MMWDEMO_NAV_TARGET_FLAG_EGO_LOW_CONF   (1U << 9)
#define MMWDEMO_NAV_TARGET_FLAG_TRANSLATION_ONLY (1U << 10)

/* Per-target flags. Bit zero retains the original presence indication. */
#define MMWDEMO_NAV_TARGET_OBJECT_FLAG_PRESENCE  (1U << 0)
#define MMWDEMO_NAV_TARGET_OBJECT_FLAG_STATIC    (1U << 1)
#define MMWDEMO_NAV_TARGET_OBJECT_FLAG_MOVING    (1U << 2)
#define MMWDEMO_NAV_TARGET_OBJECT_FLAG_UNCERTAIN (1U << 3)

#define MMWDEMO_NAV_TRACKER_EINVAL            (-201)
#define MMWDEMO_NAV_TRACKER_EPAYLOAD          (-202)
#define MMWDEMO_NAV_TRACKER_EOUTPUT           (-203)
#define MMWDEMO_NAV_TRACKER_ECREATE           (-204)

typedef struct MmwDemo_NavTarget_t
{
    uint32_t trackId;
    float posX;
    float posY;
    float posZ;
    float velX;
    float velY;
    float velZ;
    float accX;
    float accY;
    float accZ;
    float dimX;
    float dimY;
    float dimZ;
    float confidence;
    uint16_t numAssociatedPoints;
    uint16_t flags;
} MmwDemo_NavTarget;

typedef struct MmwDemo_NavTargetHeader_t
{
    uint32_t magic;
    uint16_t version;
    uint16_t headerSize;
    uint16_t targetSize;
    uint16_t flags;
    uint16_t numTargets;
    uint16_t subFrameNumber;
    uint16_t protocolStatus;
    uint16_t reserved16;
    uint32_t radarFrameNumber;
    uint32_t processingCycles;
    uint32_t trackerHeapBytes;
} MmwDemo_NavTargetHeader;

typedef char MmwDemo_NavTargetSizeCheck[
    (sizeof(MmwDemo_NavTarget) == 60U) ? 1 : -1];
typedef char MmwDemo_NavTargetHeaderSizeCheck[
    (sizeof(MmwDemo_NavTargetHeader) == 32U) ? 1 : -1];

int32_t MmwDemo_NavTracker_process(
    const void *navPayload,
    uint32_t navPayloadSize,
    void *targetPayload,
    uint32_t targetPayloadCapacity,
    uint32_t *targetPayloadSize);

void MmwDemo_NavTracker_setMotionThreshold(float thresholdMps);

void MmwDemo_NavTracker_reset(void);
void MmwDemo_NavTracker_resetSubFrame(uint16_t subFrameNumber);

extern volatile uint32_t gNavTrackerHeapBytes;
extern volatile uint32_t gNavTrackerArenaHighWaterBytes;
extern volatile int32_t gNavTrackerLastError;
extern volatile uint32_t gNavTrackerProcessCount;
extern volatile uint32_t gNavTrackerResetCount;
extern volatile uint32_t gNavTrackerMaxProcessCycles;

#ifdef __cplusplus
}
#endif

#endif /* MMWDEMO_NAV_TRACKER_H */

/**
 * @file nav_point_gen.h
 *
 * @brief MSS copy of the frame-synchronous navigation wire ABI.
 */

#ifndef MMWDEMO_NAV_POINT_GEN_H
#define MMWDEMO_NAV_POINT_GEN_H

#include <stdint.h>
#include <stddef.h>
#include "dense_elevation.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MMWDEMO_NAV_MAGIC                    (0x4E415646U)
#define MMWDEMO_NAV_PROTOCOL_VERSION         (2U)
#define MMWDEMO_NAV_MAX_SURFACE_POINTS       (1024U)
#define MMWDEMO_NAV_MAX_DISCRETE_POINTS      (128U)
#define MMWDEMO_NAV_NUM_AZIMUTH_SECTORS      (8U)
#define MMWDEMO_NAV_NUM_ELEVATION_SECTORS    (64U)

#define MMWDEMO_NAV_FLAG_FRAME_COMPLETE      (1U << 0)
#define MMWDEMO_NAV_FLAG_SURFACE_LIMITED     (1U << 1)
#define MMWDEMO_NAV_FLAG_DISCRETE_LIMITED    (1U << 2)
#define MMWDEMO_NAV_FLAG_EXTENDED_REJECTED   (1U << 3)
#define MMWDEMO_NAV_FLAG_NO_EGO_COMPENSATION (1U << 4)
#define MMWDEMO_NAV_FLAG_ELEVATION_FILTERED  (1U << 5)
#define MMWDEMO_NAV_FLAG_DEBUG_COPIED        (1U << 6)
#define MMWDEMO_NAV_FLAG_TIME_BUDGET_EXCEEDED (1U << 7)
#define MMWDEMO_NAV_FLAG_UART_LIMITED         (1U << 8)

#define MMWDEMO_NAV_EINVAL                   (-101)
#define MMWDEMO_NAV_EOUTPUT_CAPACITY         (-102)
#define MMWDEMO_NAV_ECANDIDATE_PAYLOAD       (-103)

typedef struct MmwDemo_NavPoint_t
{
    uint16_t rangeIdx;
    int8_t   azimuthIdx;
    int8_t   elevationIdx;
    int16_t  dopplerIdx;
    uint16_t snrDb10;
    int16_t  sourceVQ15;
} MmwDemo_NavPoint;

typedef struct MmwDemo_NavFrameHeader_t
{
    uint32_t magic;
    uint16_t version;
    uint16_t headerSize;
    uint16_t pointSize;
    uint16_t flags;
    uint16_t numSurfacePoints;
    uint16_t numDiscretePoints;
    uint16_t subFrameNumber;
    uint16_t numAzimuthBins;
    uint16_t numElevationBins;
    uint16_t numRangeBins;
    uint16_t numDopplerBins;
    uint16_t rangeStart;
    uint16_t rangeEnd;
    uint16_t protocolStatus;
    uint32_t radarFrameNumber;
    uint32_t processingCycles;
    float    rangeStepMeters;
    float    dopplerStepMps;
    float    framePeriodSec;
    /* low/high signed int16: applied min/max elevation in 0.1 degree. */
    uint32_t elevationFovDeg10;
    float    rangeBiasMeters;
    uint32_t cycleBudget;
} MmwDemo_NavFrameHeader;

typedef char MmwDemo_NavPointSizeCheck[
    (sizeof(MmwDemo_NavPoint) == 10U) ? 1 : -1];
typedef char MmwDemo_NavHeaderSizeCheck[
    (sizeof(MmwDemo_NavFrameHeader) == 64U) ? 1 : -1];

#ifdef __cplusplus
}
#endif

#endif /* MMWDEMO_NAV_POINT_GEN_H */

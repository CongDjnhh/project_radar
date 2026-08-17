/**
 * @file nav_point_gen.h
 *
 * @brief Frame-synchronous navigation point generation and wire ABI.
 */

#ifndef MMWDEMO_NAV_POINT_GEN_H
#define MMWDEMO_NAV_POINT_GEN_H

#include <stdint.h>
#include <stddef.h>
#include "dense_elevation.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MMWDEMO_NAV_MAGIC                    (0x4E415646U) /* NAVF */
#define MMWDEMO_NAV_PROTOCOL_VERSION         (4U)
#define MMWDEMO_NAV_MAX_SURFACE_POINTS       (1024U)
#define MMWDEMO_NAV_MAX_DISCRETE_POINTS      (128U)
#define MMWDEMO_NAV_NUM_AZIMUTH_SECTORS      (8U)
#define MMWDEMO_NAV_NUM_ELEVATION_SECTORS    (64U)
#define MMWDEMO_NAV_DBSCAN_MAX_POINTS         (384U)
#define MMWDEMO_NAV_DBSCAN_MAX_CLUSTERS       (32U)

#define MMWDEMO_NAV_EGO_MODE_OFF              (0U)
#define MMWDEMO_NAV_EGO_MODE_EXTERNAL         (1U)
#define MMWDEMO_NAV_EGO_MODE_SCENE            (2U)

#define MMWDEMO_NAV_FLAG_FRAME_COMPLETE      (1U << 0)
#define MMWDEMO_NAV_FLAG_SURFACE_LIMITED     (1U << 1)
#define MMWDEMO_NAV_FLAG_DISCRETE_LIMITED    (1U << 2)
#define MMWDEMO_NAV_FLAG_EXTENDED_REJECTED   (1U << 3)
#define MMWDEMO_NAV_FLAG_NO_EGO_COMPENSATION (1U << 4)
#define MMWDEMO_NAV_FLAG_ELEVATION_FILTERED  (1U << 5)
#define MMWDEMO_NAV_FLAG_DBSCAN_ACTIVE        (1U << 6)
#define MMWDEMO_NAV_FLAG_TIME_BUDGET_EXCEEDED (1U << 7)
#define MMWDEMO_NAV_FLAG_UART_LIMITED         (1U << 8)
#define MMWDEMO_NAV_FLAG_EGO_EXTERNAL          (1U << 9)
#define MMWDEMO_NAV_FLAG_EGO_SCENE             (1U << 10)
#define MMWDEMO_NAV_FLAG_EGO_VALID             (1U << 11)
#define MMWDEMO_NAV_FLAG_EGO_LOW_CONFIDENCE    (1U << 12)
#define MMWDEMO_NAV_FLAG_DBSCAN_LIMITED         (1U << 13)
#define MMWDEMO_NAV_FLAG_TRANSLATION_ONLY       (1U << 14)

#define MMWDEMO_NAV_EINVAL                   (-101)
#define MMWDEMO_NAV_EOUTPUT_CAPACITY         (-102)
#define MMWDEMO_NAV_ECANDIDATE_PAYLOAD       (-103)
#define MMWDEMO_NAV_ETIME_BUDGET             (-104)
#define MMWDEMO_NAV_EFRAME_SLACK              (-105)

/** Configuration is fully runtime supplied. Range bounds use [min,max]. */
typedef struct MmwDemo_NavPointGen_Cfg_t
{
    MmwDemo_DenseElevation_Cfg sourceCfg;
    float    minMovingSpeedMps;
    uint16_t surfaceRangeToleranceBins;
    uint16_t maxSurfacePoints;
    uint16_t uartMaxSurfacePoints;
    uint16_t maxDiscretePoints;
    uint8_t  surfaceMinRunLength;
    uint8_t  surfaceElevationNeighborBins;
    uint8_t  dbscanEnabled;
    uint8_t  dbscanMinPoints;
    uint16_t dbscanMaxPoints;
    uint8_t  dbscanMaxClusters;
    uint8_t  egoMode;
    uint16_t sceneMinInliers;
    uint16_t egoVelocityValidFrames;
    uint16_t reserved16;
    float    dbscanEpsilonM;
    float    dbscanRangeScale;
    float    wallMinLengthM;
    float    wallMaxThicknessM;
    float    wallMinAspectRatio;
    float    staticVelocityThresholdMps;
    float    egoVelocityXMps;
    float    egoVelocityYMps;
    float    egoVelocityZMps;
    float    egoConfidence;
    float    egoConfidenceThreshold;
    float    egoMaxSpeedMps;
    float    sceneResidualThresholdMps;
    uint32_t egoSequence;
} MmwDemo_NavPointGen_Cfg;

/** Compact point used in the DSS-to-MSS result and TLV 12. */
typedef struct MmwDemo_NavPoint_t
{
    uint16_t rangeIdx;
    int8_t   azimuthIdx;
    int8_t   elevationIdx;
    int16_t  dopplerIdx;
    uint16_t snrDb10;
    int16_t  sourceVQ15;
    int16_t  powerDb10;    /* Uncalibrated/profile-dependent raw beam power,
                            * 0.1 dB units; INT16_MIN is invalid. */
} MmwDemo_NavPoint;

/**
 * Header followed by numSurfacePoints surface samples, then
 * numDiscretePoints tracker-input samples. TLV 12 transmits this header and
 * only the surface portion; the discrete portion remains internal to radar.
 */
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
    /* Exact frame-aligned translation used for Doppler compensation. */
    float    egoVelocityXMps;
    float    egoVelocityYMps;
    float    egoVelocityZMps;
    float    egoConfidence;
    uint32_t egoSequence;
} MmwDemo_NavFrameHeader;

typedef struct MmwDemo_NavPointGen_StaticCfg_t
{
    MmwDemo_DenseElevation_StaticCfg sourceStaticCfg;
    float rangeStepMeters;
    float dopplerStepMps;
    float framePeriodSec;
    float rangeBiasMeters;
} MmwDemo_NavPointGen_StaticCfg;

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
    uint32_t *navSize);

void MmwDemo_NavPointGen_reset(void);

typedef char MmwDemo_NavPointSizeCheck[
    (sizeof(MmwDemo_NavPoint) == 12U) ? 1 : -1];
typedef char MmwDemo_NavHeaderSizeCheck[
    (sizeof(MmwDemo_NavFrameHeader) == 84U) ? 1 : -1];

#ifdef __cplusplus
}
#endif

#endif /* MMWDEMO_NAV_POINT_GEN_H */

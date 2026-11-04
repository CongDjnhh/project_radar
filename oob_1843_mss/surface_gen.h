/**
 * @file surface_gen.h
 *
 * @brief Frame-synchronous surface point generation and wire ABI.
 */

#ifndef SURFACE_GEN_H
#define SURFACE_GEN_H

#include <stdint.h>
#include <stddef.h>
#include "dense_elevation.h"
#include "local_inc/mmw_output.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MMWDEMO_NAV_MAX_SURFACE_POINTS       (MMWDEMO_SURFACE_MAX_BOXES)
#define MMWDEMO_NAV_MAX_OBSTACLE_BOXES       (MMWDEMO_SURFACE_MAX_BOXES)
#define MMWDEMO_NAV_NUM_AZIMUTH_SECTORS      (8U)
#define MMWDEMO_NAV_NUM_ELEVATION_SECTORS    (64U)

#define MMWDEMO_NAV_EINVAL                   (-101)
#define MMWDEMO_NAV_ECANDIDATE_PAYLOAD       (-103)

typedef struct MmwDemo_SurfaceClusterThresh_t
{
    int16_t  dopplerThreshBins;
    uint16_t rangeThreshBins;
    float    azimMargin;
    float    elevMargin;
} MmwDemo_SurfaceClusterThresh;

/** Configuration is fully runtime supplied. Range bounds use [min,max]. */
typedef struct MmwDemo_SurfaceGen_Cfg_t
{
    MmwDemo_DenseElevation_Cfg sourceCfg;
    float    minMovingSpeedMps;
    float    clusterVelSpreadMps;
    float    clusterRangeTolM;
    uint16_t maxSurfacePoints;
    uint16_t uartMaxSurfacePoints;
} MmwDemo_SurfaceGen_Cfg;

typedef struct MmwDemo_SurfaceGen_StaticCfg_t
{
    MmwDemo_DenseElevation_StaticCfg sourceStaticCfg;
    float rangeStepMeters;
    float dopplerStepMps;
    float framePeriodSec;
    float rangeBiasMeters;
    float minRangeMeters;
    float maxRangeMeters;
    float minMovingSpeedMps;
    float clusterVelSpreadMps;
    float clusterRangeTolM;
    uint32_t elapsedUsec;
    uint8_t motionMode;
    uint8_t numVirtualAntAzim;
    uint8_t reservedPad[2];
    MmwDemo_SurfaceStatus *status;
} MmwDemo_SurfaceGen_StaticCfg;

/* Backwards-compatibility aliases */
typedef MmwDemo_SurfaceGen_Cfg MmwDemo_NavPointGen_Cfg;
typedef MmwDemo_SurfaceGen_StaticCfg MmwDemo_NavPointGen_StaticCfg;

#include <ti/datapath/dpif/dpif_pointcloud.h>

int32_t MmwDemo_SurfaceGen_process(
    const MmwDemo_SurfaceGen_Cfg *cfg,
    const MmwDemo_SurfaceGen_StaticCfg *staticCfg,
    uint8_t *candidateBuffer,
    uint32_t candidateBufferSize,
    MmwDemo_CustomObstacle *outObstacles,
    uint16_t maxOutputPoints,
    uint32_t *numPointsOut);

void MmwDemo_SurfaceGen_reset(void);
void MmwDemo_SurfaceGen_resetSubFrame(uint16_t subFrame);

/* Pure post-processing entry point, also used by target fixtures.
 * Input samples remain unchanged; frame-local index scratch lives in L3. */
int32_t MmwDemo_SurfaceGen_cluster(
    const MmwDemo_DenseElevation_OutputSample *samples, uint16_t sampleCount,
    const MmwDemo_SurfaceGen_StaticCfg *staticCfg,
    ObstacleBox *outBoxes, uint16_t capacity, uint32_t *count);

#define MmwDemo_NavPointGen_process MmwDemo_SurfaceGen_process
#define MmwDemo_NavPointGen_reset   MmwDemo_SurfaceGen_reset

#ifdef __cplusplus
}
#endif

#endif /* SURFACE_GEN_H */

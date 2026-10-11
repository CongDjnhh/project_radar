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
#include "local_inc/mmw_output.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MMWDEMO_NAV_MAX_SURFACE_POINTS       (256U)
#define MMWDEMO_NAV_MAX_OBSTACLE_BOXES       (256U)
#define MMWDEMO_NAV_NUM_AZIMUTH_SECTORS      (8U)
#define MMWDEMO_NAV_NUM_ELEVATION_SECTORS    (64U)

#define MMWDEMO_NAV_EINVAL                   (-101)
#define MMWDEMO_NAV_ECANDIDATE_PAYLOAD       (-103)

/** Configuration is fully runtime supplied. Range bounds use [min,max]. */
typedef struct MmwDemo_NavPointGen_Cfg_t
{
    MmwDemo_DenseElevation_Cfg sourceCfg;
    float    minMovingSpeedMps;
    uint16_t maxSurfacePoints;
    uint16_t uartMaxSurfacePoints;
} MmwDemo_NavPointGen_Cfg;

typedef struct MmwDemo_NavPointGen_StaticCfg_t
{
    MmwDemo_DenseElevation_StaticCfg sourceStaticCfg;
    float rangeStepMeters;
    float dopplerStepMps;
    float framePeriodSec;
    float rangeBiasMeters;
} MmwDemo_NavPointGen_StaticCfg;

#include <ti/datapath/dpif/dpif_pointcloud.h>

int32_t MmwDemo_NavPointGen_process(
    const MmwDemo_NavPointGen_Cfg *cfg,
    const MmwDemo_NavPointGen_StaticCfg *staticCfg,
    uint8_t *candidateBuffer,
    uint32_t candidateBufferSize,
    MmwDemo_CustomObstacle *outObstacles,
    uint16_t maxOutputPoints,
    uint32_t *numPointsOut);

void MmwDemo_NavPointGen_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* MMWDEMO_NAV_POINT_GEN_H */

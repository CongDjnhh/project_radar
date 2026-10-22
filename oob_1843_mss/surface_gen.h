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

#define MMWDEMO_NAV_MAX_SURFACE_POINTS       (256U)
#define MMWDEMO_NAV_MAX_OBSTACLE_BOXES       (256U)
#define MMWDEMO_NAV_NUM_AZIMUTH_SECTORS      (8U)
#define MMWDEMO_NAV_NUM_ELEVATION_SECTORS    (64U)

#define MMWDEMO_NAV_EINVAL                   (-101)
#define MMWDEMO_NAV_ECANDIDATE_PAYLOAD       (-103)

/** Configuration is fully runtime supplied. Range bounds use [min,max]. */
typedef struct MmwDemo_SurfaceGen_Cfg_t
{
    MmwDemo_DenseElevation_Cfg sourceCfg;
    float    minMovingSpeedMps;
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

#define MmwDemo_NavPointGen_process MmwDemo_SurfaceGen_process
#define MmwDemo_NavPointGen_reset   MmwDemo_SurfaceGen_reset

#ifdef __cplusplus
}
#endif

#endif /* SURFACE_GEN_H */

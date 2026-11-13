/**
 * @file surface_cluster_3d.h
 *
 * @brief Pure-C hardware-agnostic 3D adaptive anisotropic ellipsoid DBSCAN,
 *        dynamic/static partitioning, temporal hold, and 32-byte ObstacleBox
 *        wire-ABI output generation for AWR2944P OOB-DDMA radar point clouds.
 */

#ifndef SURFACE_CLUSTER_3D_H
#define SURFACE_CLUSTER_3D_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Compile-time static assertion portable across C89, C99, and C11. */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#define SURFACE_CLUSTER_STATIC_ASSERT(cond, name) _Static_assert((cond), #name)
#else
#define SURFACE_CLUSTER_STATIC_ASSERT(cond, name) \
    typedef char surface_cluster_static_assert_##name[(cond) ? 1 : -1]
#endif

/** Capacity and structural limits (sized to CM4 finalMaxNumDetObjs = 793). */
#define SURFACE_CLUSTER_MAX_POINTS                (793U)
#define SURFACE_CLUSTER_MAX_BOXES                 (255U)
#define SURFACE_CLUSTER_HISTORY_MAX               (64U)
#define SURFACE_CLUSTER_MAX_SUBFRAMES             (4U)
#define SURFACE_CLUSTER_DEFAULT_MIN_PTS           (2U)

/** Motion modes matching MmwDemo_SurfaceStatus / CLI surfaceMotionCfg. */
#define SURFACE_CLUSTER_MOTION_MOBILE             (0U)
#define SURFACE_CLUSTER_MOTION_FIXED              (1U)

/** Telemetry version matching MMWDEMO_SURFACE_STATUS_VERSION. */
#define SURFACE_CLUSTER_STATUS_VERSION            (1U)

/** Explicit physics model multipliers (Rule-7 named constants). */
#define SURFACE_CLUSTER_DEFAULT_RANGE_RES_MULT    (3.0f)
#define SURFACE_CLUSTER_DEFAULT_VEL_RES_MULT      (2.0f)
#define SURFACE_CLUSTER_ELEV_INTERFEROMETER_SCALE (0.25f)
#define SURFACE_CLUSTER_MIN_BOX_RES_MULT          (0.5f)

/** Return status codes. */
#define SURFACE_CLUSTER_OK                        (0)
#define SURFACE_CLUSTER_ERR_EINVAL                (-101)
#define SURFACE_CLUSTER_ERR_INVALID_PHYSICS       (-102)
#define SURFACE_CLUSTER_ERR_CAPACITY              (-103)

/** Status flag bitmask matching MMWDEMO_SURFACE_* wire bits in mmw_output.h. */
#define SURFACE_CLUSTER_FLAG_CFAR_LIMIT           (1UL << 0)
#define SURFACE_CLUSTER_FLAG_DENSE_LIMIT          (1UL << 1)
#define SURFACE_CLUSTER_FLAG_CLUSTER_LIMIT        (1UL << 2)
#define SURFACE_CLUSTER_FLAG_BOX_LIMIT            (1UL << 3)
#define SURFACE_CLUSTER_FLAG_UART_LIMIT           (1UL << 4)
#define SURFACE_CLUSTER_FLAG_INVALID_GEOMETRY     (1UL << 5)
#define SURFACE_CLUSTER_FLAG_PROCESS_ERROR        (1UL << 6)
#define SURFACE_CLUSTER_FLAG_COVERAGE_UNVERIFIED  (1UL << 7)
#define SURFACE_CLUSTER_FLAG_PREVIOUS_DEADLINE    (1UL << 8)
#define SURFACE_CLUSTER_FLAG_TIMING_UNVALIDATED   (1UL << 9)
#define SURFACE_CLUSTER_FLAG_HOLD_ACTIVE          (1UL << 10)
#define SURFACE_CLUSTER_FLAG_HISTORY_LIMIT        (1UL << 11)
#define SURFACE_CLUSTER_FLAG_BUDGET_EXCEEDED      (1UL << 12)

/** ObstacleBox.boxId target class codes and dynamic bitflag (mmw_output.h). */
#define SURFACE_CLUSTER_CLASS_UNKNOWN             (0U)
#define SURFACE_CLUSTER_CLASS_DRONE               (1U)
#define SURFACE_CLUSTER_CLASS_PEDESTRIAN          (2U)
#define SURFACE_CLUSTER_CLASS_VEHICLE             (3U)
#define SURFACE_CLUSTER_CLASS_MOVING              (4U)
#define SURFACE_CLUSTER_DYNAMIC_FLAG              (0x8000U)

#define SURFACE_CLUSTER_PACK_BOX_ID(isDyn, classType, rank) \
    ((int16_t)(((isDyn) ? SURFACE_CLUSTER_DYNAMIC_FLAG : 0U) | \
               (((uint16_t)(classType) & 0x007FU) << 8U) | \
               ((uint16_t)(rank) & 0x00FFU)))

/** Input point cloud layout matching DPIF_PointCloudCartesian (16 bytes). */
typedef struct SurfaceCluster_InputPoint_t
{
    float x;
    float y;
    float z;
    float velocity;
} SurfaceCluster_InputPoint;

/** Input side-info layout matching DPIF_PointCloudSideInfo (4 bytes). */
typedef struct SurfaceCluster_SideInfo_t
{
    int16_t snr;
    int16_t noise;
} SurfaceCluster_SideInfo;

/** Wire-ABI 32-byte obstacle box matching ObstacleBox (TLV 20 / TLV 22). */
typedef struct SurfaceCluster_Box_t
{
    float   x;
    float   y;
    float   z;
    float   width_x;
    float   depth_y;
    float   height_z;
    float   velocity;
    int16_t snr;
    int16_t boxId;
} SurfaceCluster_Box;

/** Wire-ABI 48-byte telemetry status matching MmwDemo_SurfaceStatus (TLV 21). */
typedef struct SurfaceCluster_Status_t
{
    uint16_t version;
    uint16_t size;
    uint32_t flags;
    uint16_t cfarReturned;
    uint16_t cfarProcessed;
    uint16_t denseProduced;
    uint16_t denseClustered;
    uint16_t clusterCount;
    uint16_t boxCount;
    uint16_t heldCount;
    uint16_t rangeCount;
    uint32_t processingUs;
    uint32_t maxProcessingUs;
    uint32_t previousExportUs;
    uint32_t framePeriodUs;
    uint32_t uartEstimatedUs;
    uint32_t previousExportFrameNumber;
} SurfaceCluster_Status;

/** Derived mmWave physics contract matching MmwDemo_DerivedPhysicsParams (60 bytes). */
typedef struct SurfaceCluster_Physics_t
{
    float    rangeStep;
    float    dopplerStep;
    float    rangeResolution;
    float    dopplerResolution;
    float    wavelength;
    float    centerFreq;
    float    bandwidth;
    float    chirpInterval;
    float    framePeriodSec;
    float    vMax;
    float    rMax;
    float    azimuthBeamwidthRad;
    float    elevationBeamwidthRad;
    uint16_t numVirtualAntAzim;
    uint16_t numVirtualAntElev;
    uint16_t reserved[2];
} SurfaceCluster_Physics;

/** Explicit runtime clustering configuration. Zero thresholds derive from physics. */
typedef struct SurfaceCluster_Config_t
{
    float    minRangeMeters;      /**< Minimum valid radial range [m], >= 0.0f */
    float    maxRangeMeters;      /**< Maximum valid radial range [m], > minRangeMeters */
    float    minMovingSpeedMps;   /**< [m/s]; 0.0f derives 2.0 * dopplerResolution */
    float    clusterVelSpreadMps; /**< [m/s]; 0.0f derives 2.0 * dopplerResolution */
    float    clusterRangeTolM;    /**< [m];   0.0f derives 3.0 * rangeResolution */
    float    holdDurationSec;     /**< [s];   0.0f disables temporal hold (0 frames) */
    uint16_t maxBoxes;            /**< Shared dyn+stat cap in [1, 255] */
    uint16_t minClusterPts;       /**< Minimum core/cluster points, >= 2 */
    uint8_t  motionMode;          /**< SURFACE_CLUSTER_MOTION_MOBILE or _FIXED */
    uint8_t  reserved[3];
} SurfaceCluster_Config;

/** Caller-provided output buffers and counts for a single frame. */
typedef struct SurfaceCluster_Output_t
{
    SurfaceCluster_Box    *dynBoxes;     /**< Dynamic boxes output (TLV 20), len >= dynCap */
    uint16_t               dynCap;       /**< Capacity of dynBoxes in elements */
    uint16_t               numDynBoxes;  /**< Emitted dynamic box count */
    SurfaceCluster_Box    *statBoxes;    /**< Static boxes output (TLV 22), len >= statCap */
    uint16_t               statCap;      /**< Capacity of statBoxes in elements */
    uint16_t               numStatBoxes; /**< Emitted static box count */
    SurfaceCluster_Status *status;       /**< Required telemetry output (48B, TLV 21) */
} SurfaceCluster_Output;

/** Internal working point representation (caller treats state as opaque). */
typedef struct SurfaceCluster_WorkPt_t
{
    float    x;
    float    y;
    float    z;
    float    r;
    float    velocity;
    int16_t  snr;
    uint16_t pad;
} SurfaceCluster_WorkPt;

/** Internal cluster AABB accumulator (caller treats state as opaque). */
typedef struct SurfaceCluster_Accum_t
{
    float    minX;
    float    maxX;
    float    minY;
    float    maxY;
    float    minZ;
    float    maxZ;
    float    sumVel;
    int16_t  maxSnr;
    uint16_t numPoints;
} SurfaceCluster_Accum;

/** Internal temporal history slot (caller treats state as opaque). */
typedef struct SurfaceCluster_HistEntry_t
{
    SurfaceCluster_Box box;
    uint16_t           ageFrames;
    uint8_t            valid;
    uint8_t            isDynamic;
} SurfaceCluster_HistEntry;

/**
 * @brief Caller-owned clustering and temporal hold state (~43.9 KB).
 *        Place in static .bss / MSS_L2 RAM; never allocate on task stack.
 */
typedef struct SurfaceCluster_State_t
{
    SurfaceCluster_WorkPt    pts[SURFACE_CLUSTER_MAX_POINTS];
    SurfaceCluster_Accum     clusters[SURFACE_CLUSTER_MAX_BOXES];
    SurfaceCluster_HistEntry history[SURFACE_CLUSTER_MAX_SUBFRAMES][SURFACE_CLUSTER_HISTORY_MAX];
    int16_t                  labels[SURFACE_CLUSTER_MAX_POINTS];
    uint16_t                 bfsQueue[SURFACE_CLUSTER_MAX_POINTS];
    uint16_t                 nbrScratch[SURFACE_CLUSTER_MAX_POINTS];
    uint16_t                 winStart[SURFACE_CLUSTER_MAX_POINTS];
    uint16_t                 winEnd[SURFACE_CLUSTER_MAX_POINTS];
    uint8_t                  inQueue[SURFACE_CLUSTER_MAX_POINTS];
    uint8_t                  histMatched[SURFACE_CLUSTER_HISTORY_MAX];
    uint8_t                  currMatched[SURFACE_CLUSTER_MAX_BOXES];
    uint8_t                  lastMotionMode[SURFACE_CLUSTER_MAX_SUBFRAMES];
    uint8_t                  isInitialized;
    uint8_t                  reserved[3];
} SurfaceCluster_State;

/**
 * @brief Initialize caller-owned clustering state and clear all subframe histories.
 * @param state Pointer to caller-owned state buffer.
 * @return SURFACE_CLUSTER_OK or SURFACE_CLUSTER_ERR_EINVAL.
 */
int32_t SurfaceCluster_init(SurfaceCluster_State *state);

/**
 * @brief Reset temporal hold history for a single subframe.
 * @param state Pointer to initialized state buffer.
 * @param subFrameIdx Subframe index in [0, SURFACE_CLUSTER_MAX_SUBFRAMES - 1].
 * @return SURFACE_CLUSTER_OK or SURFACE_CLUSTER_ERR_EINVAL.
 */
int32_t SurfaceCluster_resetSubFrame(SurfaceCluster_State *state, uint8_t subFrameIdx);

/**
 * @brief Validate derived mmWave physical parameters (zero silent fallbacks).
 * @param physics Pointer to physics struct populated from RF parser.
 * @return SURFACE_CLUSTER_OK or SURFACE_CLUSTER_ERR_INVALID_PHYSICS.
 */
int32_t SurfaceCluster_validatePhysics(const SurfaceCluster_Physics *physics);

/**
 * @brief Execute 3D anisotropic ellipsoid DBSCAN, dynamic/static partitioning,
 *        optional temporal hold, and y-sorted wire-ABI box generation.
 *
 * @details Reads Cartesian points and side-info via byte-strided memcpy so callers
 *          may pass TI SDK DPIF_PointCloudCartesian / DPIF_PointCloudSideInfo arrays
 *          directly without strict-aliasing violations or TI SDK header coupling.
 *          Resets out->status clustering fields and flags at frame start; caller
 *          may populate timing/UART fields and OR timing/UART flags afterwards.
 *
 * @param state               Caller-owned state buffer (must be initialized via SurfaceCluster_init).
 * @param cfg                 Validated clustering configuration.
 * @param physics             Validated derived mmWave physics parameters.
 * @param cartesianPts        Pointer to input Cartesian point array (may be NULL iff numPoints == 0).
 * @param cartesianStrideBytes Stride in bytes between elements (>= sizeof(SurfaceCluster_InputPoint)).
 * @param sideInfoPts         Pointer to input side-info array (may be NULL iff numPoints == 0).
 * @param sideInfoStrideBytes Stride in bytes between elements (>= sizeof(SurfaceCluster_SideInfo)).
 * @param numPoints           Number of input detected points (values > 793 keep top-793 by SNR).
 * @param dopNumObjOut        Raw CFAR count prior to AoA filtering (reported as status->cfarReturned).
 * @param subFrameIdx         Active subframe index in [0, 3].
 * @param out                 Caller output struct containing dynBoxes/statBoxes/status.
 * @return SURFACE_CLUSTER_OK on success or negative SURFACE_CLUSTER_ERR_* on invalid inputs.
 */
int32_t SurfaceCluster_process(
    SurfaceCluster_State *state,
    const SurfaceCluster_Config *cfg,
    const SurfaceCluster_Physics *physics,
    const void *cartesianPts,
    size_t cartesianStrideBytes,
    const void *sideInfoPts,
    size_t sideInfoStrideBytes,
    uint16_t numPoints,
    uint16_t dopNumObjOut,
    uint8_t subFrameIdx,
    SurfaceCluster_Output *out);

#ifdef __cplusplus
}
#endif

#endif /* SURFACE_CLUSTER_3D_H */
